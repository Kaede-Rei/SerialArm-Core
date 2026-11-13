#!/usr/bin/env python3
"""Read-only audit of gain tasks and exported evidence, using the standard library.

The report distinguishes archived conclusions from evidence available in each
record.  It never recomputes a legacy ``validated`` flag or writes a Profile.
"""

from __future__ import annotations

import argparse
from bisect import bisect_right
from collections import Counter
import csv
import io
import json
import math
from pathlib import Path
import statistics
import sys
import tempfile
from typing import Any
import xml.etree.ElementTree as ET
import zipfile


class Evidence:
    def __init__(self, path: Path):
        self.path = path
        self.archive = zipfile.ZipFile(path) if path.is_file() else None
        if self.archive:
            self.names = {n.removeprefix('evidence/'): n for n in self.archive.namelist()
                          if n.startswith('evidence/') and not n.endswith('/')}
        elif path.is_dir():
            self.names = {str(p.relative_to(path)): str(p) for p in path.rglob('*') if p.is_file()}
        else:
            raise ValueError(f'No task directory or ZIP: {path}')

    def close(self):
        if self.archive:
            self.archive.close()

    def binary(self, name: str):
        return self.archive.open(self.names[name]) if self.archive else open(self.names[name], 'rb')

    def json(self, name: str) -> dict[str, Any]:
        if name not in self.names:
            return {}
        with self.binary(name) as stream:
            return json.load(stream)


def trajectory(source: Evidence, name: str, peak_target_rad: float,
               measurement_window_s: tuple[float, float] | None = None) -> dict[str, Any]:
    axes: dict[str, Any] = {}
    invalid = 0
    with source.binary(name) as stream:
        rows = csv.DictReader(io.TextIOWrapper(stream))
        required = {'time_s', 'joint', 'reference', 'actual'}
        if not required.issubset(rows.fieldnames or []):
            return {'status': 'not_a_tracking_record', 'columns': rows.fieldnames}
        for row in rows:
            try:
                reference, actual, stamp = (float(row[k]) for k in ('reference', 'actual', 'time_s'))
                if not all(math.isfinite(x) for x in (reference, actual, stamp)):
                    raise ValueError('nonfinite sample')
                fresh = int(row.get('new_measurement', '0'))
                limited = int(row.get('limited', '0'))
            except (ValueError, KeyError, TypeError):
                invalid += 1
                continue
            axis = axes.setdefault(row['joint'], {
                'samples': 0, 'fresh_samples': 0, 'limited_samples': 0,
                'first_time_s': stamp, 'last_time_s': stamp,
                'first_reference_rad': reference, 'last_reference_rad': reference,
                'first_actual_rad': actual, 'reference_min_rad': reference, 'reference_max_rad': reference,
                'actual_min_rad': actual, 'actual_max_rad': actual,
                'peak_error_rad': 0.0, '_sum_squared': 0.0,
                'maximum_sample_gap_s': 0.0, 'nonmonotonic_samples': 0,
                'fresh_feedback_flag_recorded': 'new_measurement' in row,
                'feedback_receive_time_recorded': 'feedback_time_s' in row,
                'feedback_sequence_recorded': 'feedback_sequence' in row,
                'known_feedback_sequence_samples': 0,
                'maximum_fresh_feedback_gap_s': 0.0, 'nonmonotonic_fresh_feedback': 0,
                'repeated_fresh_feedback_sequence': 0, '_previous_fresh_time': None,
                '_previous_fresh_sequence': None, '_actual_positions': set(),
                'independent_disturbance_input_peak_nm': None,
                '_window': [],
            })
            gap = stamp - axis['last_time_s']
            axis['maximum_sample_gap_s'] = max(axis['maximum_sample_gap_s'], gap)
            axis['nonmonotonic_samples'] += gap < 0
            axis['last_time_s'] = stamp
            axis['last_reference_rad'] = reference
            axis['samples'] += 1
            axis['fresh_samples'] += bool(fresh)
            axis['limited_samples'] += bool(limited)
            axis['reference_min_rad'] = min(axis['reference_min_rad'], reference)
            axis['reference_max_rad'] = max(axis['reference_max_rad'], reference)
            axis['actual_min_rad'] = min(axis['actual_min_rad'], actual)
            axis['actual_max_rad'] = max(axis['actual_max_rad'], actual)
            axis['_actual_positions'].add(actual)
            if 'independent_disturbance_nm' in row:
                disturbance = float(row['independent_disturbance_nm'])
                if not math.isfinite(disturbance):
                    raise ValueError('nonfinite recorded independent disturbance')
                axis['independent_disturbance_input_peak_nm'] = max(
                    axis['independent_disturbance_input_peak_nm'] or 0.0, abs(disturbance))
            if row.get('actual_gains_known') == '1':
                for term in ('kp', 'kd'):
                    value = float(row['actual_' + term])
                    key = 'recorded_actual_' + term
                    bounds = axis.setdefault(key, {'first': value, 'min': value, 'max': value, 'last': value})
                    bounds.update(last=value, min=min(bounds['min'], value), max=max(bounds['max'], value))
                axis['actual_gains_known_samples'] = axis.get('actual_gains_known_samples', 0) + 1
                if all(key in row for key in ('reference_velocity', 'velocity')):
                    pd = float(row['actual_kp'])*(reference-actual) + float(row['actual_kd'])*(
                        float(row['reference_velocity'])-float(row['velocity']))
                    values = [('reconstructed_pd_nm', pd)]
                    if 'feedforward_nm' in row:
                        values.append(('reconstructed_total_command_nm', pd+float(row['feedforward_nm'])
                                       +float(row.get('independent_disturbance_nm', 0))))
                    for key, value in values:
                        bounds = axis.setdefault(key, {'min':value, 'max':value, 'peak_abs':abs(value)})
                        bounds.update(min=min(bounds['min'], value), max=max(bounds['max'], value),
                                      peak_abs=max(bounds['peak_abs'], abs(value)))
            if fresh and measurement_window_s and measurement_window_s[0] <= stamp < measurement_window_s[1]:
                axis['_window'].append((reference, actual))
            if fresh:
                feedback_time = float(row.get('feedback_time_s', stamp))
                previous = axis['_previous_fresh_time']
                if math.isfinite(feedback_time):
                    if previous is not None:
                        axis['maximum_fresh_feedback_gap_s'] = max(axis['maximum_fresh_feedback_gap_s'], feedback_time - previous)
                        axis['nonmonotonic_fresh_feedback'] += feedback_time <= previous
                    axis['_previous_fresh_time'] = feedback_time
                else:
                    axis['nonmonotonic_fresh_feedback'] += 1
                sequence = row.get('feedback_sequence')
                if sequence is not None and int(sequence) > 0:
                    axis['known_feedback_sequence_samples'] += 1
                    if sequence == axis['_previous_fresh_sequence']:
                        axis['repeated_fresh_feedback_sequence'] += 1
                    axis['_previous_fresh_sequence'] = sequence
            error = reference - actual
            axis['peak_error_rad'] = max(axis['peak_error_rad'], abs(error))
            axis['_sum_squared'] += error * error
    for axis in axes.values():
        axis['rms_error_rad'] = math.sqrt(axis.pop('_sum_squared') / axis['samples'])
        axis['peak_error_deg'] = math.degrees(axis['peak_error_rad'])
        axis['peak_target_met'] = axis['peak_error_rad'] < peak_target_rad
        axis['reference_span_rad'] = axis['reference_max_rad'] - axis['reference_min_rad']
        axis['actual_span_rad'] = axis['actual_max_rad'] - axis['actual_min_rad']
        axis['actual_span_deg'] = math.degrees(axis['actual_span_rad'])
        axis['first_reference_minus_actual_rad'] = axis['first_reference_rad'] - axis['first_actual_rad']
        axis['reference_envelope_contains_initial_actual'] = axis['reference_min_rad'] <= axis['first_actual_rad'] <= axis['reference_max_rad']
        positions = sorted(axis.pop('_actual_positions'))
        axis['distinct_actual_positions'] = len(positions)
        axis['minimum_observed_position_step_rad'] = min((b - a for a, b in zip(positions, positions[1:])), default=None)
        window = axis.pop('_window')
        if measurement_window_s:
            def centered_rms(values):
                mean = statistics.mean(values)
                return math.sqrt(statistics.mean((value-mean)**2 for value in values))
            reference_rms = centered_rms([pair[0] for pair in window]) if window else None
            actual_rms = centered_rms([pair[1] for pair in window]) if window else None
            axis['response_window'] = {'start_s':measurement_window_s[0], 'end_exclusive_s':measurement_window_s[1],
                'planned_window_complete':axis['first_time_s'] <= measurement_window_s[0] and axis['last_time_s'] >= measurement_window_s[1],
                'fresh_samples':len(window), 'centered_reference_rms_rad':reference_rms,
                'centered_actual_rms_rad':actual_rms,
                'response_rms_ratio':actual_rms/reference_rms if reference_rms else None,
                'actual_rms_in_observed_codes':actual_rms/axis['minimum_observed_position_step_rad']
                if actual_rms is not None and axis['minimum_observed_position_step_rad'] else None}
        axis.pop('_previous_fresh_time')
        axis.pop('_previous_fresh_sequence')
    return {'status': 'measured' if axes else 'no_valid_samples', 'invalid_rows': invalid,
            'peak_scope': 'all_recorded_rows_without_plot_decimation', 'axes': axes}


def tracking_kinematics(rows: list[dict[str, float]]) -> dict[str, Any]:
    """Diagnose the executed record; timing comparisons never change acceptance."""
    if len(rows) < 3 or any(b['time_s'] <= a['time_s'] for a, b in zip(rows, rows[1:])):
        return {'status':'insufficient_or_nonmonotonic_record'}
    stamps = [row['time_s'] for row in rows]
    def reference_at(stamp):
        upper = min(max(1, bisect_right(stamps, stamp)), len(rows)-1)
        a, b = rows[upper-1], rows[upper]
        weight = min(1.0, max(0.0, (stamp-a['time_s'])/(b['time_s']-a['time_s'])))
        return a['reference']+(b['reference']-a['reference'])*weight
    intervals = []
    for a, b in zip(rows, rows[1:]):
        source_known = a.get('reference_sample_time_known') and b.get('reference_sample_time_known')
        basis = 'reference_sample_time_s' if source_known else 'time_s'
        dt = b[basis]-a[basis]
        predicted = .5*(a['reference_velocity']+b['reference_velocity'])*dt
        intervals.append({'start_s':a['time_s'], 'duration_s':dt,
            'reference_interval_time_basis':basis,
            'reference_delta_rad':b['reference']-a['reference'],
            'velocity_integral_rad':predicted,
            'position_velocity_mismatch_rad':b['reference']-a['reference']-predicted})
    worst = max(intervals, key=lambda row:abs(row['position_velocity_mismatch_rad']))
    ages = [row['time_s']-row['feedback_time_s'] for row in rows if 'feedback_time_s' in row]
    receive_errors = [row['actual']-reference_at(row['feedback_time_s']) for row in rows
                      if 'feedback_time_s' in row and stamps[0] <= row['feedback_time_s'] <= stamps[-1]]
    direction_errors = [[row['reference']-row['actual'] for row in rows
                        if direction*row['reference_velocity'] > .05] for direction in (1, -1)]
    # A finite 40 ms difference avoids treating one encoder code as a velocity
    # spike. It tests sign/scale, not a motor velocity observer or delay model.
    velocities = []
    for index, row in enumerate(rows):
        old = bisect_right(stamps, row['time_s']-.04)-1
        if old < 0:
            continue
        elapsed = row['time_s']-rows[old]['time_s']
        measured = (row['actual']-rows[old]['actual'])/elapsed
        reported = statistics.mean(item['velocity'] for item in rows[old:index+1])
        velocities.append((reported, measured))
    energy = sum(v*v for v, _ in velocities)
    slope = sum(v*dq for v, dq in velocities)/energy if energy > 1e-12 else None
    plateau_start = 0
    plateaus = []
    for index, row in enumerate(rows[1:], 1):
        if row['actual'] == rows[index-1]['actual']:
            continue
        segment = rows[plateau_start:index]
        elapsed = segment[-1]['time_s']-segment[0]['time_s']
        median_velocity = statistics.median(item['reference_velocity'] for item in segment)
        if elapsed >= .03 and abs(median_velocity) > .05:
            plateaus.append({'start_s':segment[0]['time_s'], 'duration_s':elapsed,
                'reference_travel_rad':segment[-1]['reference']-segment[0]['reference'],
                'median_reference_velocity_rad_s':median_velocity})
        plateau_start = index
    return {'status':'recorded_kinematics',
        'scope':'diagnostic_only_original_full_cycle_error_and_acceptance_unchanged',
        'known_reference_sample_times':sum(bool(row.get('reference_sample_time_known')) for row in rows),
        'nonincreasing_known_reference_intervals':sum(row['duration_s']<=0 and row['reference_interval_time_basis']=='reference_sample_time_s' for row in intervals),
        'peak_position_velocity_mismatch_rad':abs(worst['position_velocity_mismatch_rad']),
        'worst_reference_interval':worst,
        'reference_duplicate_intervals_with_nonzero_velocity':sum(
            row['reference_delta_rad']==0 and abs(row['velocity_integral_rad'])>1e-6 for row in intervals),
        'feedback_receive_age_s':{'median':statistics.median(ages), 'max':max(ages)} if ages else None,
        'receive_aligned_error_rad':{'rms':math.sqrt(statistics.mean(e*e for e in receive_errors)),
                                    'peak':max(abs(e) for e in receive_errors)} if receive_errors else None,
        'receive_alignment_scope':'host_receive_timestamp_diagnostic_not_known_device_sampling_or_motor_delay',
        'reported_velocity_to_40ms_position_difference_scale':slope,
        'direction_mean_error_rad':dict(zip(('positive','negative'),
            (statistics.mean(errors) if errors else None for errors in direction_errors))),
        'moving_reference_position_plateaus':len(plateaus),
        'longest_moving_reference_position_plateau':max(plateaus,key=lambda row:row['duration_s']) if plateaus else None,
        'plateau_scope':'recorded_position_plateaus_can_include_friction_quantization_or_feedback_effects_not_a_motor_fault_diagnosis'}


def full_path_execution_diagnostics(source: Evidence, records: dict[str, Any]) -> dict[str, Any]:
    diagnostics = {}
    for name, record in records.items():
        if record['status'] != 'measured' or name == 'gain-trajectory-noise.csv':
            continue
        axes: dict[str, list[dict[str, float]]] = {}
        with source.binary(name) as stream:
            reader = csv.DictReader(io.TextIOWrapper(stream))
            required = ('time_s','reference','reference_velocity','actual','velocity')
            if not set(required).issubset(reader.fieldnames or []):
                diagnostics[name] = {'status':'velocity_columns_not_recorded'}
                continue
            for row in reader:
                values = {key:float(row[key]) for key in required}
                if 'feedback_time_s' in row:
                    values['feedback_time_s'] = float(row['feedback_time_s'])
                if row.get('reference_sample_time_known') == '1':
                    values['reference_sample_time_s'] = float(row['reference_sample_time_s'])
                    values['reference_sample_time_known'] = True
                if all(math.isfinite(value) for value in values.values()):
                    axes.setdefault(row['joint'],[]).append(values)
        diagnostics[name] = {joint:tracking_kinematics(rows) for joint, rows in axes.items()}
    return diagnostics


def short_probe_diagnostics(source: Evidence, candidate: dict[str, Any], peak_target_deg: float) -> dict[str, Any]:
    """Describe observed motion and timing without inventing identification."""
    experiments = candidate.get('experiments') or []
    fits = {item.get('record_path'): item for item in experiments
            if item.get('kind') in ('reference_tracking', 'independent_torque_disturbance', 'motion_establishment_probe')}
    parameters = {item.get('record_path'): item for item in experiments
                  if item.get('kind') == 'short_experiment_parameters'}
    names = candidate.get('joint_names', [])
    records = {}
    for name in sorted(source.names):
        if not name.startswith('servo-probe-') or not name.endswith('.csv'):
            continue
        duration = parameters.get(name, {}).get('duration_s')
        window = (.7, duration-.7) if isinstance(duration, (int, float)) and duration > 1.4 else None
        record = trajectory(source, name, math.radians(peak_target_deg), window)
        fit = fits.get(name, {})
        models = [axis for axis in fit.get('axes', [])
                  if 'tracking_valid' in axis or 'disturbance_valid' in axis or 'model_definition' in axis]
        recoveries = [axis for axis in fit.get('axes', []) if axis.get('kind') == 'disturbance_recovery']
        record['experiment_kind'] = fit.get('kind', 'not_recorded')
        record['evidence_scope'] = ('motion_establishment_trial_consult_per_axis_plan'
                                   if record['experiment_kind'] == 'motion_establishment_probe' else 'recorded_experiment')
        if record['experiment_kind'] == 'independent_torque_disturbance':
            record['independent_input_scope'] = 'Recorded applied joint-torque command, not an independent physical motor-torque calibration'
        record['actual_trial_parameters'] = parameters.get(name)
        for joint, axis in record.get('axes', {}).items():
            index = names.index(joint) if joint in names else -1
            model = models[index] if 0 <= index < len(models) else {}
            recovery = recoveries[index] if 0 <= index < len(recoveries) else {}
            planned = parameters.get(name, {})
            plans = planned.get('plans', [])
            axis_plan = plans[index] if 0 <= index < len(plans) else {}
            motion_only = axis_plan.get('motion_probe', record['experiment_kind'] == 'motion_establishment_probe')
            axis['planned_motion_probe'] = motion_only
            axis['planned_reference_bias_rad'] = axis_plan.get('reference_bias_rad')
            axis['position_domain_source'] = planned.get('position_domain_source')
            domain = planned.get('working_domain', [])
            domain = domain[index] if 0 <= index < len(domain) else {}
            axis['planned_reference_domain_rad'] = [domain.get('reference_lower_rad'),domain.get('reference_upper_rad')]
            eligible = planned.get('disturbance_axis_eligible', [])
            axis['planned_disturbance_axis_eligible'] = eligible[index] if 0 <= index < len(eligible) else None
            axis['evidence_scope'] = ('symmetric_single_frequency_motion_only_not_FRF' if motion_only
                                      else 'recorded_experiment')
            center = planned.get('center_rad', [])
            recorded_center = center[index] if 0 <= index < len(center) else None
            axis['recorded_center_source'] = planned.get('center_source', 'not_recorded')
            axis['recorded_center_rad'] = recorded_center
            axis['first_reference_minus_recorded_center_rad'] = (axis['first_reference_rad'] - recorded_center
                                                               if recorded_center is not None else None)
            axis['archived_identification_reason'] = model.get('reason', 'not_recorded')
            axis['archived_model_definition'] = model.get('model_definition')
            axis['archived_tracking_valid'] = model.get('tracking_valid')
            axis['archived_disturbance_valid'] = model.get('disturbance_valid')
            axis['archived_disturbance_goal_evaluated'] = model.get('disturbance_goal_evaluated')
            axis['archived_disturbance_goal_met'] = model.get('disturbance_goal_met')
            axis['archived_static_measurement_complete'] = recovery.get('static_measurement_complete')
            axis['archived_motion_observed'] = recovery.get('motion_observed')
            axis['archived_disturbance_encoder_step_rad'] = recovery.get('encoder_step_rad')
            axis['archived_disturbance_motion_floor_rad'] = recovery.get('motion_floor_rad')
            axis['archived_disturbance_measurement_complete'] = recovery.get('measurement_complete')
            axis['archived_disturbance_assessment_reason'] = recovery.get('reason')
            axis['archived_identifiable'] = model.get('identifiable')
            axis['archived_missing_evidence'] = model.get('missing_evidence')
            if not axis['fresh_feedback_flag_recorded']:
                classification = 'feedback_evidence_not_recorded'
            elif (not axis['fresh_samples'] or axis['maximum_fresh_feedback_gap_s'] > .06
                  or axis['nonmonotonic_fresh_feedback'] or axis['repeated_fresh_feedback_sequence']):
                classification = 'feedback_missing_or_discontinuous'
            elif axis['limited_samples']:
                classification = 'limited_excitation'
            elif record['experiment_kind'] == 'independent_torque_disturbance':
                classification = ('independent_input_without_observed_displacement'
                                  if axis['independent_disturbance_input_peak_nm'] and axis['actual_span_rad'] <= 1e-12
                                  else 'independent_disturbance_measurement_requires_recorded_response_assessment')
            elif axis['reference_span_rad'] <= 1e-12:
                classification = 'no_reference_excitation'
            elif axis['actual_span_rad'] <= 1e-12:
                classification = 'fresh_feedback_no_position_response'
            elif axis['actual_span_rad'] < .2 * axis['reference_span_rad']:
                classification = 'fresh_feedback_weak_position_response'
            else:
                classification = 'position_response_observed'
            axis['motion_diagnostic'] = classification
        record['interpretation'] = 'Motion coverage and freshness diagnostics only; the 20% weak-response label is not gain acceptance, and zero displacement under torque is not an identified stiffness'
        records[name] = record
    return records


def measurement_center_diagnostics(source: Evidence, candidate: dict[str, Any], peak_target_deg: float) -> dict[str, Any]:
    """Keep positioning evidence separate from identification and purpose passes."""
    records = {}
    names = candidate.get('joint_names', [])
    for item in candidate.get('experiments', []):
        name = item.get('record_path')
        if item.get('kind') != 'demonstrated_measurement_center_entry' or name not in source.names:
            continue
        record = trajectory(source, name, math.radians(peak_target_deg))
        record['archived_experiment'] = item
        record['evidence_scope'] = 'positioning_only_no_identification_or_purpose_qualification'
        record['lead_comparison_scope'] = 'recorded_final_lead_vector_not_recomputed_Safety_or_dynamic_gain_history'
        previous = {}
        with source.binary(name) as stream:
            for line, row in enumerate(csv.DictReader(io.TextIOWrapper(stream)), 2):
                joint = row['joint']
                if joint not in names or not int(row.get('new_measurement', 0)):
                    continue
                axis = record['axes'][joint]
                index = names.index(joint)
                point = {'line':line, 'time_s':float(row['time_s']), 'reference_rad':float(row['reference']),
                         'actual_rad':float(row['actual']), 'kp':float(row['actual_kp']), 'kd':float(row['actual_kd']),
                         'feedforward_nm':float(row['feedforward_nm'])}
                point['reference_minus_actual_rad'] = point['reference_rad']-point['actual_rad']
                lead = item.get('feedback_lead_rad', [])
                uncertainty = item.get('position_uncertainty_rad', [])
                axis['recorded_feedback_lead_rad'] = lead[index] if index < len(lead) else None
                axis['recorded_position_uncertainty_rad'] = uncertainty[index] if index < len(uncertainty) else None
                before = previous.get(joint)
                if before:
                    transition = {'before':before, 'after':point,
                                  'actual_delta_rad':point['actual_rad']-before['actual_rad'],
                                  'reference_delta_rad':point['reference_rad']-before['reference_rad']}
                    axis['last_fresh_transition'] = transition
                    if transition['actual_delta_rad'] and 'first_nonzero_fresh_position_step' not in axis:
                        axis['first_nonzero_fresh_position_step'] = transition
                if axis['recorded_feedback_lead_rad'] is not None and abs(point['reference_minus_actual_rad']) > axis['recorded_feedback_lead_rad']+1e-12:
                    axis['recorded_hard_lead_exceeded_rows'] = axis.get('recorded_hard_lead_exceeded_rows', 0)+1
                    axis.setdefault('first_recorded_hard_lead_exceeded', point)
                axis['last_fresh_sample'] = point
                previous[joint] = point
        records[name] = record
    return records


def endpoint_transition_diagnostics(source: Evidence, candidate: dict[str, Any], peak_target_deg: float) -> dict[str, Any]:
    """Separate generated endpoint references from physical arrival and full paths."""
    records={}
    names=candidate.get('joint_names', [])
    for item in candidate.get('experiments', []):
        name=item.get('record_path')
        if item.get('kind')!='feedback_bounded_endpoint_transition' or name not in source.names:
            continue
        record=trajectory(source,name,math.radians(peak_target_deg))
        record['archived_experiment']=item
        record['evidence_scope']='endpoint_positioning_not_full_path_tracking_or_purpose_qualification'
        record['exact_reference_arrival_scope']='historical_1e-8_rad_1e-6_rad_per_s_command_thresholds_not_actual_arrival_acceptance'
        with source.binary(name) as stream:
            for line,row in enumerate(csv.DictReader(io.TextIOWrapper(stream)),2):
                joint=row['joint']
                if joint not in names:
                    continue
                index=names.index(joint);target=item['target_rad'][index]
                axis=record['axes'][joint]
                reference,actual,velocity=(float(row[key]) for key in ('reference','actual','reference_velocity'))
                axis.update(last_line=line,last_actual_rad=actual,last_reference_velocity_rad_s=velocity,
                            archived_target_rad=target,last_reference_minus_target_rad=reference-target,
                            last_actual_minus_target_rad=actual-target)
                cap=item.get('reference_error_cap_rad',[])
                axis['archived_reference_error_cap_rad']=cap[index] if index<len(cap) else None
                if abs(reference-target)<1e-8 and abs(velocity)<1e-6:
                    axis.setdefault('first_exact_stopped_reference_time_s',float(row['time_s']))
        records[name]=record
    return records


def establishment_progression(candidate: dict[str, Any], probes: dict[str, Any]) -> dict[str, Any]:
    """Expose the executed staircase and input changes; never infer breakaway friction."""
    decisions = [item for item in candidate.get('candidate_comparisons', [])
                 if item.get('kind') == 'motion_establishment']
    axes = {}
    for index, joint in enumerate(candidate.get('joint_names', [])):
        rounds = []
        for decision in decisions:
            name = decision.get('record_path')
            record = probes.get(name, {})
            measured = record.get('axes', {}).get(joint, {})
            parameters = record.get('actual_trial_parameters') or {}
            def at(key, default=None):
                values = parameters.get(key, [])
                return values[index] if index < len(values) else default
            planned = at('plans', {})
            domain = at('working_domain', {})
            coverage = decision.get('axes', [])
            motion = coverage[index] if index < len(coverage) else {}
            motion_only = planned.get('motion_probe', record.get('experiment_kind') == 'motion_establishment_probe')
            duration = planned.get('duration_s', parameters.get('duration_s'))
            frequencies = planned.get('frequencies_hz')
            executed_frequencies = frequencies[:1] if motion_only and frequencies else frequencies
            def recorded_gain(term):
                bounds = measured.get('recorded_actual_'+term, {})
                return bounds.get('first') if bounds and bounds.get('min') == bounds.get('max') else None
            row = {'record_path':name, 'kp':recorded_gain('kp'), 'kd':recorded_gain('kd'),
                'actual_gains_known_samples':measured.get('actual_gains_known_samples', 0),
                'amplitude_rad':planned.get('amplitude_rad'), 'frequencies_hz':executed_frequencies,
                'plan_frequency_array':frequencies, 'motion_probe':motion_only, 'duration_s':duration,
                'reference_bias_rad':planned.get('reference_bias_rad'),
                'reference_lower_rad':domain.get('reference_lower_rad'),
                'reference_upper_rad':domain.get('reference_upper_rad'),
                'planned_measurement_window_cycles':[frequency*(duration-1.4) for frequency in executed_frequencies]
                if executed_frequencies and isinstance(duration,(int,float)) and duration>1.4 else None,
                'feedback_effort_budget_nm':domain.get('feedback_effort_budget_nm'),
                'protocol_kp_max':domain.get('protocol_kp_max'),
                'reference_span_rad':measured.get('reference_span_rad'), 'actual_span_rad':measured.get('actual_span_rad'),
                'pd_command':measured.get('reconstructed_pd_nm'),
                'total_command':measured.get('reconstructed_total_command_nm'),
                'response_window':measured.get('response_window'),
                'positive_travel_rad':motion.get('positive_travel_rad'),
                'negative_travel_rad':motion.get('negative_travel_rad'),
                'required_travel_rad':motion.get('required_travel_rad'),
                'archived_motion_qualified':motion.get('qualified'),
                'input_response_progress':motion.get('input_response_progress'),
                'blocked':index in decision['blocked_axes'] if 'blocked_axes' in decision else None,
                'archived_tracking_valid':measured.get('archived_tracking_valid'),
                'archived_identification_reason':measured.get('archived_identification_reason'),
                'retry':decision.get('retry')}
            for field, output in (('reasons','decision_reason'), ('current_kp','recorded_current_kp'),
                                  ('current_kd','recorded_current_kd'), ('next_kp','next_kp'), ('next_kd','next_kd'),
                                  ('excitation_retests','excitation_retests'), ('boundary_retests','boundary_retests')):
                values = decision.get(field, [])
                row[output] = values[index] if index < len(values) else None
            if rounds:
                previous = rounds[-1]
                row['amplitude_reduced_while_kp_increased'] = bool(
                    row['kp'] is not None and previous['kp'] is not None and row['kp'] > previous['kp']
                    and row['amplitude_rad'] is not None and previous['amplitude_rad'] is not None
                    and row['amplitude_rad'] < previous['amplitude_rad'])
            rounds.append(row)
        if rounds:
            tail = rounds[-3:]
            spans = [r['actual_span_rad'] for r in tail if r['actual_span_rad'] is not None]
            peaks = [r['pd_command']['peak_abs'] for r in tail if r['pd_command']]
            frozen = 0
            for row in reversed(rounds):
                if row['frequencies_hz'] is None or row['frequencies_hz'] != rounds[-1]['frequencies_hz']:
                    break
                frozen += 1
            axes[joint] = {'rounds':rounds, 'trailing_same_frequency_rounds':frozen,
                'last_three_rounds_actual_span_range_rad':[min(spans), max(spans)] if spans else None,
                'last_three_rounds_pd_peak_range_nm':[min(peaks), max(peaks)] if peaks else None}
    return {'recorded_decisions':len(decisions), 'axes':axes,
            'interpretation':'Executed command/response progression only; PD command peak is not a measured friction threshold, and task completion does not attest the later persistent session state'}


def execution_summary(candidate: dict[str, Any], result: dict[str, Any], targets: dict[str, Any], probes: dict[str, Any]) -> dict[str, Any]:
    elapsed = result.get('gain_elapsed_s', candidate.get('actual_elapsed_s'))
    budget = candidate.get('time_budget_s', result.get('gain_time_budget_s'))
    exhausted = result.get('gain_budget_exhausted', candidate.get('budget_exhausted'))
    warnings = result.get('gain_warnings', candidate.get('warnings', []))
    unstarted = [name for name, target in targets.items()
                 if 'not_started' in target['warnings'] or target.get('measurement_state') == 'not_started']
    all_unstarted = bool(targets) and len(unstarted) == len(targets)
    status_termination, candidate_termination = result.get('termination'), candidate.get('termination')
    termination = status_termination if isinstance(status_termination, dict) else candidate_termination
    termination = termination if isinstance(termination, dict) else {}
    known_kinds = {'qualified', 'measured_not_qualified', 'insufficient_measurement', 'budget_exhausted',
                   'cancelled', 'execution_fault', 'running'}
    conflicts = []
    if isinstance(status_termination, dict) and isinstance(candidate_termination, dict) and status_termination != candidate_termination:
        conflicts.append('status_candidate_termination_mismatch')
    if result.get('phase') == 'cancelled' and candidate.get('outcome') == 'execution_fault' and not result.get('fault_interrupted'):
        conflicts.append('cancelled_status_with_legacy_execution_fault_candidate')
    if termination.get('kind') == 'qualified' and (candidate.get('available') is not True or candidate.get('validated') is not True):
        conflicts.append('qualified_termination_without_available_validated_evidence')
    if result.get('fault_interrupted'):
        outcome = 'execution_fault'
    elif termination.get('kind') in known_kinds:
        outcome = (termination['kind'] if 'qualified_termination_without_available_validated_evidence' not in conflicts
                   else 'archived_termination_requires_evidence')
    elif result.get('phase') == 'cancelled':
        outcome = 'cancelled'
    elif exhausted:
        outcome = 'budget_exhausted'
    elif all_unstarted and any('identification_insufficient' in value for value in warnings):
        outcome = 'startup_identification_insufficient_before_purpose_validation'
    elif result.get('phase') == 'failed':
        outcome = 'execution_fault'
    elif candidate.get('available') is False:
        outcome = 'no_complete_purpose_measurement'
    elif candidate.get('validated') is True:
        outcome = 'archived_validated_with_recorded_scope'
    else:
        outcome = 'archived_result_requires_warning_or_more_evidence'
    diagnostics = Counter(axis['motion_diagnostic'] for record in probes.values()
                          if record.get('experiment_kind') in ('reference_tracking', 'motion_establishment_probe')
                          for axis in record.get('axes', {}).values())
    return {'archived_phase': result.get('phase', 'not_in_artifact'), 'archived_outcome': candidate.get('outcome'), 'outcome': outcome,
            'archived_fault_interrupted':result.get('fault_interrupted'),
            'archived_status_termination':status_termination, 'archived_candidate_termination':candidate_termination,
            'termination_objects_match':status_termination == candidate_termination
            if isinstance(status_termination, dict) and isinstance(candidate_termination, dict) else None,
            'termination_consistency_issues':conflicts,
            'actual_elapsed_s': elapsed, 'hard_budget_s': budget,
            'target_budget_s': candidate.get('target_budget_s', result.get('gain_target_budget_s')),
            'budget_exhausted': exhausted, 'recorded_remaining_s': result.get('gain_remaining_s'),
            'finished_before_hard_budget': elapsed < budget if isinstance(elapsed, (int, float)) and isinstance(budget, (int, float)) else None,
            'full_path_passes': result.get('gain_path_pass_current', candidate.get('path_passes')),
            'short_probe_records': len(probes), 'unstarted_purposes': unstarted,
            'all_requested_purposes_unstarted': all_unstarted,
            'reference_probe_axis_diagnostics': dict(diagnostics),
            'last_probe_stage': result.get('gain_probe_stage'),
            'last_probe_kind': result.get('gain_probe_kind'),
            'warnings': warnings, 'error': result.get('error'),
            'status_policy': 'Preserve archived termination and conflicts; a complete phase or qualified label does not promote unavailable purpose measurements to passed'}


def position_reversals(points: list[tuple]) -> dict[str, Any]:
    """Summarize receive-stamped position turns without asserting a mechanical mode."""
    positions = sorted({p[2] for p in points})
    quantum = min((b-a for a, b in zip(positions, positions[1:]) if b > a), default=0.0)
    threshold = max(3*quantum, 1e-6)
    direction, turns = 0, []
    extreme = points[0] if points else None
    for point in points[1:]:
        change = point[2]-extreme[2]
        if direction == 0:
            if abs(change) >= threshold:
                direction = 1 if change > 0 else -1
                extreme = point
        elif change*direction >= 0:
            extreme = point
        elif abs(change) >= threshold:
            turns.append({'polarity':direction, 'extreme_line':extreme[3], 'confirmed_line':point[3],
                          'extreme_time_s':extreme[0], 'confirmed_time_s':point[0],
                          'reverse_position_delta_rad':change, 'reference_delta_rad':point[4]-extreme[4]})
            direction = -direction
            extreme = point
    positive = [t['extreme_time_s'] for t in turns if t['polarity'] == 1]
    periods = [b-a for a, b in zip(positive, positive[1:]) if b > a]
    result = {'basis':'fresh_receive_stamps_position_hysteresis_no_mode_identification',
              'observed_position_step_rad':quantum, 'reversal_threshold_rad':threshold,
              'confirmed_reversals':len(turns), 'first_reversal':turns[0] if turns else None,
              'last_reversals':turns[-4:],
              'observed_positive_peak_frequency_hz':1/statistics.median(periods) if periods else None}
    if points:
        for seconds in (.3, .1):
            tail = [p[2]-p[4] for p in points if p[0] >= points[-1][0]-seconds]
            result[f'last_{seconds:g}s_position_error_peak_to_peak_rad'] = max(tail)-min(tail)
    return result


def frame_phases(source: Evidence) -> dict[str, Any]:
    if 'frames.csv' not in source.names:
        return {'status': 'not_in_artifact'}
    phases: dict[str, Any] = {}
    stages: dict[str, Any] = {}
    segments = []
    previous_stamp = None
    maximum_cycle_gap_s = 0.0
    with source.binary('frames.csv') as stream:
        rows = csv.DictReader(io.TextIOWrapper(stream))
        kp_columns = [n for n in rows.fieldnames or [] if n.startswith('cmd_kp:')]
        kd_columns = [n for n in rows.fieldnames or [] if n.startswith('cmd_kd:')]
        joints = [n[2:] for n in rows.fieldnames or [] if n.startswith('q:')]
        for line, row in enumerate(rows, 2):
            phase = row.get('phase', 'unknown')
            stamp = int(row['monotonic_ns'])
            if previous_stamp is not None:
                maximum_cycle_gap_s = max(maximum_cycle_gap_s, (stamp-previous_stamp)/1e9)
            previous_stamp = stamp
            item = phases.setdefault(phase, {
                'samples': 0, 'first_ns': stamp, 'last_ns': stamp,
                '_directions': Counter(), '_gains': Counter(), 'position_ranges':{},
            })
            item['samples'] += 1
            item['last_ns'] = stamp
            item['_directions'][row.get('direction', 'unknown')] += 1
            item['_gains'][(tuple(row[c] for c in kp_columns),
                            tuple(row[c] for c in kd_columns))] += 1
            if row.get('valid', '1') == '1':
                for joint in joints:
                    positions = item['position_ranges'].setdefault(joint, {})
                    for layer in ('q','nominal_q','accepted_q'):
                        if layer+':'+joint not in row:
                            continue
                        value = float(row[layer+':'+joint])
                        bounds = positions.setdefault(layer, {'min':value,'max':value})
                        bounds.update(min=min(bounds['min'],value),max=max(bounds['max'],value))
            stage = row.get('gain_stage', '')
            key = (phase, stage)
            if not segments or (segments[-1]['phase'], segments[-1]['gain_stage']) != key:
                segments.append({'phase':phase, 'gain_stage':stage, 'first_line':line, 'first_ns':stamp, 'samples':0})
            segments[-1].update(last_line=line, last_ns=stamp, samples=segments[-1]['samples']+1)
            if stage and stage != 'none':
                record = stages.setdefault(stage, {'samples': 0, 'axes': {}, 'first_line':line, 'first_ns':stamp})
                record['samples'] += 1
                record.update(last_line=line, last_ns=stamp)
                for joint in joints:
                    axis = record['axes'].setdefault(joint, {'ranges': {}, 'limited_samples': 0,
                        'maximum_feedback_age_ms': 0.0, 'maximum_accepted_minus_nominal_rad': 0.0,
                        'actual_gain_change_samples':0, 'fresh_receive_samples':0, 'repeated_receive_samples':0,
                        'nonmonotonic_receive_samples':0, 'invalid_feedback_samples':0,
                        'maximum_fresh_receive_gap_s':0.0, '_previous_receive_ns':None, '_previous_gains':None, '_positions':[]})
                    for prefix in ('q', 'dq', 'ref_q', 'ref_dq', 'nominal_q', 'virtual_q', 'accepted_q', 'cmd_kp', 'cmd_kd', 'cmd_tor', 'model_ff'):
                        value = row.get(prefix + ':' + joint)
                        if value is None:
                            continue
                        number = float(value)
                        bounds = axis['ranges'].setdefault(prefix, {'first': number, 'min': number, 'max': number})
                        bounds['min'] = min(bounds['min'], number)
                        bounds['max'] = max(bounds['max'], number)
                        bounds['last'] = number
                    if all(key+':'+joint in row for key in ('cmd_kp','cmd_kd','ref_q','ref_dq','q','dq','cmd_tor')):
                        value = lambda key: float(row[key+':'+joint])
                        gains = (value('cmd_kp'), value('cmd_kd'))
                        if axis['_previous_gains'] is not None and gains != axis['_previous_gains']:
                            axis['actual_gain_change_samples'] += 1
                        axis['_previous_gains'] = gains
                        pd = gains[0]*(value('ref_q')-value('q'))+gains[1]*(value('ref_dq')-value('dq'))
                        for key, number in (('reconstructed_pd_nm',pd), ('reconstructed_pd_plus_command_torque_nm',pd+value('cmd_tor'))):
                            if abs(number) > axis.get(key+'_peak_abs', -1):
                                axis[key+'_peak_abs'] = abs(number)
                                axis[key+'_peak'] = {'line':line, 'monotonic_ns':stamp, 'signed_nm':number}
                    received = row.get('feedback_received_ns:'+joint)
                    timing_valid = row.get('feedback_timing_valid:'+joint, '1') == '1'
                    axis['invalid_feedback_samples'] += row.get('valid', '1') != '1'
                    if received is not None and timing_valid and int(received) > 0:
                        received_ns = int(received)
                        previous = axis['_previous_receive_ns']
                        if previous is not None and received_ns < previous:
                            axis['nonmonotonic_receive_samples'] += 1
                        elif received_ns != previous:
                            axis['fresh_receive_samples'] += 1
                            if previous is not None:
                                axis['maximum_fresh_receive_gap_s'] = max(axis['maximum_fresh_receive_gap_s'], (received_ns-previous)/1e9)
                            if row.get('valid', '1') == '1':
                                axis['_positions'].append((received_ns/1e9, stamp/1e9, float(row['q:'+joint]), line, float(row.get('ref_q:'+joint, row['q:'+joint]))))
                        else:
                            axis['repeated_receive_samples'] += 1
                        if previous is None or received_ns >= previous:
                            axis['_previous_receive_ns'] = received_ns
                    axis['maximum_feedback_age_ms'] = max(axis['maximum_feedback_age_ms'], float(row.get('feedback_age_ms', 0)))
                    axis['limited_samples'] += bool(int(row.get('ref_limited:' + joint, 0)) or int(row.get('tor_limited:' + joint, 0)))
                    if 'accepted_q:' + joint in row and 'nominal_q:' + joint in row:
                        axis['maximum_accepted_minus_nominal_rad'] = max(axis['maximum_accepted_minus_nominal_rad'],
                            abs(float(row['accepted_q:' + joint]) - float(row['nominal_q:' + joint])))
    for item in phases.values():
        item['duration_s'] = (item['last_ns'] - item['first_ns']) / 1e9
        item['directions'] = dict(item.pop('_directions'))
        gains = item.pop('_gains')
        item['distinct_gain_vectors_including_handoffs'] = len(gains)
        item['most_recorded_gain_vectors'] = [
            {'kp': list(map(float, kp)), 'kd': list(map(float, kd)), 'samples': count}
            for (kp, kd), count in gains.most_common(5)]
    for record in stages.values():
        record['duration_s'] = (record['last_ns']-record['first_ns'])/1e9
        for axis in record['axes'].values():
            for bounds in axis['ranges'].values():
                bounds['span'] = bounds['max'] - bounds['min']
            axis['position_reversals'] = position_reversals(axis.pop('_positions'))
            axis.pop('_previous_receive_ns')
            axis.pop('_previous_gains')
    return {'status': 'recorded', 'phases': phases, 'gain_stages': stages, 'segments':segments,
            'maximum_cycle_gap_s':maximum_cycle_gap_s,
            'torque_reconstruction_scope':'recorded_accepted_cycles_only_not_the_unrecorded_rejected_command'}


def direction_comparisons(records: dict[str, Any]) -> list[dict[str, Any]]:
    comparisons = []
    for kind in ('native', 'compliant'):
        first, second = (records.get(f'gain-trajectory-{kind}-train-{i}.csv', {}).get('axes', {})
                         for i in (0, 1))
        common = first.keys() & second.keys()
        if not common:
            continue
        same = max(max(abs(first[j][k] - second[j][k]) for k in
                       ('first_reference_rad', 'last_reference_rad')) for j in common)
        opposite = max(max(abs(first[j]['first_reference_rad'] - second[j]['last_reference_rad']),
                           abs(first[j]['last_reference_rad'] - second[j]['first_reference_rad']))
                       for j in common)
        comparisons.append({'kind': kind, 'same_endpoints_max_difference_rad': same,
                            'reversed_endpoints_max_difference_rad': opposite,
                            'opposite_path_endpoints': opposite < 0.005 and same > 0.005,
                            'interpretation': 'Endpoints diagnose comparison conditions; equal endpoints alone do not prove equal complete paths'})
    return comparisons


def nested_parameter_measurement_complete(candidate: dict[str, Any], purpose: str, target: dict[str, Any]) -> bool | None:
    """Describe modern nested measurements while retaining archived flags.

    This is evidence coverage, not a qualification or an application operation.
    Legacy local-probe formats keep their original scope instead of receiving
    an inferred modern full-path result.
    """
    if candidate.get('validation_scope') != 'independent_demonstration_trajectory':
        return None
    n = len(candidate.get('joint_names', []))
    def number(value):
        return isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value) and value >= 0
    def gains(value):
        return isinstance(value, dict) and all(isinstance(value.get(term), list) and len(value[term]) == n
            and all(number(v) for v in value[term]) for term in ('kp', 'kd'))
    def complete(value, kind):
        if not isinstance(value, dict) or value.get('completed') is not True or value.get('available') is not True:
            return False
        if kind == 'tracking':
            rows = value.get('metrics')
            return (value.get('independent') is True and number(value.get('duration_s')) and value['duration_s'] > 0
                and number(value.get('measured_duration_s')) and value['measured_duration_s'] >= value['duration_s']-.06
                and isinstance(rows, list) and len(rows) == n and all(isinstance(row, dict)
                and number(row.get('samples')) and row['samples'] > 0
                and number(row.get('measurement_samples')) and row['measurement_samples'] > 0
                and all(number(row.get(key)) for key in ('rms_error_rad','peak_error_rad','velocity_rms_rad_s','tail_error_rad','settling_s'))
                for row in rows))
        if not number(value.get('samples')) or value['samples'] <= 0:
            return False
        if kind == 'hold':
            fresh = value.get('fresh_samples')
            return (isinstance(fresh, list) and len(fresh) == n and all(number(v) and v > 0 for v in fresh)
                    and all(number(value.get(key)) for key in ('worst_error_rad','worst_velocity_rad_s')))
        axes = value.get('axes')
        return (isinstance(axes, list) and len(axes) == n and all(isinstance(axis, dict)
            and number(axis.get('samples')) and axis['samples'] > 0
            and all(number(axis.get(key)) for key in ('initial_velocity_rad_s','peak_velocity_rad_s','final_velocity_rad_s','decay_s'))
            for axis in axes))
    if n == 0 or not gains(target) or target.get('error') in ('not_started', 'measurement_in_progress'):
        return False
    if purpose == 'compliant_drag':
        return all(v == 0 for v in target['kp']) and complete(target.get('drag_validation'), 'drag')
    if not complete(target.get('native_validation'), 'tracking'):
        return False
    if purpose == 'servo_tracking':
        admittance = target.get('admittance_validation') or {}
        native_only = target.get('admittance_required') is False and admittance.get('required') is False and admittance.get('status') == 'not_requested'
        return native_only or complete(admittance, 'tracking')
    if purpose not in ('rigid_tracking', 'compliant_tracking') or not gains(target.get('hold_gains')):
        return False
    hold_key = ('hold_compensation_verification' if (target.get('hold_compensation') or {}).get('enabled') is True
                else 'hold_verification')
    return complete(target.get(hold_key), 'hold')


def saved_reference_envelope(source: Evidence, name: str = 'trajectory.csv') -> dict[str, Any]:
    if name not in source.names:
        return {'status':'not_in_artifact'}
    with source.binary(name) as stream:
        rows = csv.reader(io.TextIOWrapper(stream))
        first, header = next(rows, []), next(rows, [])
        if len(first)!=2 or first[0]!='sample_dt' or not header or header[0]!='index':
            return {'status':'unsupported_saved_path_format'}
        axes = {joint:{'min':math.inf,'max':-math.inf} for joint in header[1:]}
        count = 0
        for row in rows:
            if len(row)!=len(header):
                raise ValueError('Saved path dimensions changed in '+name)
            for joint, text in zip(header[1:],row[1:]):
                value = float(text)
                if not math.isfinite(value):
                    raise ValueError('Nonfinite saved path in '+name)
                axes[joint]['min'] = min(axes[joint]['min'],value)
                axes[joint]['max'] = max(axes[joint]['max'],value)
            count += 1
    return {'status':'recorded' if count else 'no_samples', 'samples':count,
            'duration_s':float(first[1])*max(0,count-1), 'axes':axes if count else {},
            'scope':'saved_demonstration_reference_not_a_measured_hardware_limit'}


def position_envelope_evidence(source: Evidence, candidate: dict[str, Any], probes: dict[str, Any], cycle: dict[str, Any]) -> dict[str, Any]:
    saved = saved_reference_envelope(source)
    limits = {}
    if 'source/model.urdf' in source.names:
        with source.binary('source/model.urdf') as stream:
            model = ET.parse(stream).getroot()
        for joint in model.findall('joint'):
            limit = joint.find('limit')
            if joint.get('name') not in candidate.get('joint_names',[]) or limit is None:
                continue
            if 'lower' in limit.attrib and 'upper' in limit.attrib:
                limits[joint.get('name')] = {'lower_rad':float(limit.get('lower')),'upper_rad':float(limit.get('upper'))}
    trials = {}
    for name, record in probes.items():
        parameters = record.get('actual_trial_parameters') or {}
        axes = {}
        for joint, measured in record.get('axes',{}).items():
            path = saved.get('axes',{}).get(joint)
            declared = limits.get(joint)
            axis = {}
            for label, lower, upper in (('reference',measured['reference_min_rad'],measured['reference_max_rad']),
                                        ('actual',measured['actual_min_rad'],measured['actual_max_rad'])):
                axis[label+'_range_rad'] = [lower,upper]
                if path:
                    axis[label+'_margin_to_saved_path_rad'] = [lower-path['min'],path['max']-upper]
                if declared:
                    axis[label+'_margin_to_urdf_rad'] = [lower-declared['lower_rad'],declared['upper_rad']-upper]
            axes[joint] = axis
        trials[name] = {'axes':axes, 'position_domain_source':parameters.get('position_domain_source'),
            'demonstrated_path_lower_rad':parameters.get('demonstrated_path_lower_rad'),
            'demonstrated_path_upper_rad':parameters.get('demonstrated_path_upper_rad'),
            'disturbance_axis_eligible':parameters.get('disturbance_axis_eligible')}
    teaching = cycle.get('phases',{}).get('teaching',{}).get('position_ranges',{})
    return {'saved_reference':saved, 'urdf_declared_joint_limits':limits,
            'recorded_teaching_position_ranges':teaching or None, 'short_probe_ranges':trials,
            'interpretation':'Negative margin means outside that recorded envelope; URDF declarations and demonstrated positions are distinct evidence, neither infers an unmeasured physical stop'}


def audit(source: Evidence, peak_target_deg: float) -> dict[str, Any]:
    candidate = source.json('gain-candidate.json')
    if not candidate:
        raise ValueError('gain-candidate.json is missing')
    result, metadata = source.json('result.json'), source.json('metadata.json')
    targets = {}
    recorded_targets = candidate.get('targets', {})
    if not recorded_targets and candidate.get('category') not in (None, 'all'):
        recorded_targets = {candidate['category']: candidate}
    for name, target in recorded_targets.items():
        scope = target.get('qualification_scope', {})
        response = target.get('response_model_status', 'not_recorded')
        holds = {}
        for key in ('hold_verification', 'hold_compensation_verification'):
            if key in target:
                holds[key] = target[key]
        parameter_complete = nested_parameter_measurement_complete(candidate, name, target)
        targets[name] = {
            'archived_available': target.get('available'), 'archived_validated': target.get('validated'),
            'archived_qualification_complete':target.get('qualification_complete'), 'archived_error':target.get('error'),
            'nested_parameter_measurement_complete':parameter_complete,
            'legacy_available_aggregation_conflict':parameter_complete is True and target.get('available') is False,
            'warnings': target.get('warnings', []), 'kp': target.get('kp'), 'kd': target.get('kd'),
            'hold_gains': target.get('hold_gains'), 'hold_trials': holds,
            'hold_compensation_saved': target.get('hold_compensation'),
            'response_model_status': response, 'qualification_scope': scope,
            'response_model': target.get('response_model'),
            'independent_disturbance_evidence': scope.get('disturbance_release', 'not_recorded'),
            'disturbance_verification': target.get('disturbance_verification'),
            'hold_disturbance_verification': target.get('hold_disturbance_verification'),
            'integral_trial_present_but_saved_disabled': bool(
                target.get('hold_compensation_verification', {}).get('integral_samples', 0)
                and target.get('hold_compensation', {}).get('enabled') is False),
            'native_validation_record': target.get('native_validation', {}).get('record_path'),
            'measurement_state': ('not_started' if 'not_started' in target.get('warnings', [])
                                  or target.get('error') == 'not_started' else
                                  'complete' if parameter_complete is True or target.get('available') is True else 'incomplete_or_not_recorded'),
        }
    records = {name: trajectory(source, name, math.radians(peak_target_deg))
               for name in sorted(source.names)
               if name.startswith('gain-trajectory-') and name.endswith('.csv')}
    duration = candidate.get('budget_source_duration_s', result.get('gain_budget_source_duration_s'))
    probes = short_probe_diagnostics(source, candidate, peak_target_deg)
    cycle = frame_phases(source)
    return {
        'task_id': candidate.get('task_id'),
        'policy': 'read_only_no_archived_status_promotion',
        'archived_available': candidate.get('available'), 'archived_validated': candidate.get('validated'),
        'seed': {'source': candidate.get('seed_source'), 'kp': candidate.get('seed_kp'),
                 'kd': candidate.get('seed_kd'), 'measurement_policy': metadata.get('measurement_policy')},
        'model_initialization': candidate.get('model_initialization'),
        'task_model_demand': candidate.get('task_model_demand'),
        'response_guard_events': candidate.get('response_guard_events', []),
        'unrun_disturbance_experiments':[item for item in candidate.get('experiments', [])
                                       if item.get('kind') == 'independent_disturbance_not_run'],
        'execution_context': {key: metadata.get(key) for key in (
            'joint_names', 'mapping', 'actuator_capabilities', 'runtime_provenance',
            'core_fingerprint', 'urdf_fingerprint', 'hardware_effective_fingerprint')},
        'experiments': candidate.get('experiments'),
        'candidate_comparisons': candidate.get('candidate_comparisons'),
        'continuation_review':{
            'archived_review_required':candidate.get('review_required'),
            'archived_nonfatal_stage_warnings':candidate.get('nonfatal_stage_warnings'),
            'archived_execution_policy':candidate.get('execution_policy'),
            'recorded_measurement_warnings':[item for item in candidate.get('experiments',[])
                                             if item.get('kind')=='measurement_warning'],
            'scope':'archived_policy_and_stage_evidence_not_automatic_result_promotion'},
        'gain_tuning_mode': candidate.get('gain_tuning_mode', 'not_recorded'),
        'execution_summary': execution_summary(candidate, result, targets, probes),
        'budget': {'recorded_source_duration_s': duration,
                   'source': candidate.get('budget_source'),
                   'source_task_id': candidate.get('budget_source_task_id'),
                   'archived_budget_s': candidate.get('time_budget_s'),
                   'archived_elapsed_s': result.get('gain_elapsed_s', candidate.get('actual_elapsed_s')),
                   'retimed_trajectory_duration_s': result.get('trajectory_duration_s'),
                   'requested_quick_target_s': duration * 12 if duration else None,
                   'requested_quick_hard_limit_s': duration * 15 if duration else None,
                   'requested_fine_hard_limit_s': duration * 24 if duration else None},
        'audit_peak_target_deg': peak_target_deg,
        'targets': targets, 'trajectory_records': records, 'short_probe_records': probes,
        'full_path_execution_diagnostics':full_path_execution_diagnostics(source,records),
        'measurement_center_records':measurement_center_diagnostics(source,candidate,peak_target_deg),
        'endpoint_transition_records':endpoint_transition_diagnostics(source,candidate,peak_target_deg),
        'trajectory_record_counts':{
            'records_with_position_samples':sum(record['status']=='measured' for record in records.values()),
            'header_only_records':[name for name,record in records.items() if record['status']=='no_valid_samples'],
            'scope':'raw_full_path_file_coverage_not_archived_started_pass_count_or_acceptance'},
        'establishment_progression':establishment_progression(candidate, probes),
        'candidate_path_comparisons': direction_comparisons(records), 'cycle_record':cycle,
        'position_envelope_evidence':position_envelope_evidence(source,candidate,probes,cycle),
        'missing_files': [n for n in ('result.json', 'frames.csv') if n not in source.names],
    }


def compare(source: Evidence, other: Evidence) -> dict[str, Any]:
    common = source.names.keys() & other.names.keys()
    differences = []
    for name in sorted(common):
        with source.binary(name) as left, other.binary(name) as right:
            while True:
                a, b = left.read(1024 * 1024), right.read(1024 * 1024)
                if a != b:
                    differences.append(name)
                    break
                if not a:
                    break
    return {'compared_files': len(common), 'different_files': differences,
            'only_in_source': sorted(source.names.keys() - other.names.keys()),
            'only_in_companion': sorted(other.names.keys() - source.names.keys()),
            'source_snapshot_files': sorted(name for name in common if name.startswith('source/')),
            'source_snapshot_files_identical': (not any(name.startswith('source/') for name in differences)
                                              if any(name.startswith('source/') for name in common) else None),
            'method': 'streamed_byte_comparison_without_extraction'}


def compact_summary(report: dict[str, Any]) -> dict[str, Any]:
    probes = {}
    keys = ('motion_diagnostic', 'samples', 'fresh_samples', 'limited_samples', 'maximum_fresh_feedback_gap_s',
            'reference_span_rad', 'actual_span_rad', 'first_reference_minus_actual_rad',
            'reference_envelope_contains_initial_actual', 'recorded_center_source', 'recorded_center_rad',
            'first_reference_minus_recorded_center_rad', 'independent_disturbance_input_peak_nm',
            'archived_identification_reason', 'archived_model_definition', 'planned_motion_probe', 'evidence_scope',
            'planned_reference_bias_rad',
            'position_domain_source', 'planned_reference_domain_rad', 'planned_disturbance_axis_eligible',
            'response_window', 'archived_disturbance_valid', 'archived_disturbance_goal_met',
            'archived_static_measurement_complete', 'archived_motion_observed',
            'archived_disturbance_encoder_step_rad', 'archived_disturbance_motion_floor_rad',
            'archived_disturbance_measurement_complete', 'archived_disturbance_assessment_reason')
    for name, record in report['short_probe_records'].items():
        probes[name] = {joint: {key: axis.get(key) for key in keys} for joint, axis in record.get('axes', {}).items()}
    result = {key: report[key] for key in ('task_id', 'policy', 'archived_available', 'archived_validated',
                                         'gain_tuning_mode', 'seed', 'execution_summary', 'response_guard_events')}
    result['short_probe_diagnostics'] = probes
    coverage_keys = ('archived_available','archived_validated','archived_qualification_complete','archived_error',
                     'nested_parameter_measurement_complete','legacy_available_aggregation_conflict','measurement_state')
    result['measurement_coverage'] = {name:{key:target.get(key) for key in coverage_keys}
                                      for name,target in report['targets'].items()}
    result['full_path_correction_plans'] = [item for item in report.get('candidate_comparisons') or []
                                            if item.get('kind') == 'full_path_correction_plan']
    result['full_path_execution_diagnostics'] = report['full_path_execution_diagnostics']
    result['position_envelope_evidence'] = report['position_envelope_evidence']
    result['unrun_disturbance_experiments'] = report['unrun_disturbance_experiments']
    result['measurement_center_records'] = report['measurement_center_records']
    result['endpoint_transition_records'] = report['endpoint_transition_records']
    result['trajectory_record_counts'] = report['trajectory_record_counts']
    result['continuation_review'] = report['continuation_review']
    result['establishment_progression'] = {joint:{
        'actual_kp_steps':[row['kp'] for row in axis['rounds']],
        'actual_kd_steps':[row['kd'] for row in axis['rounds']],
        'trailing_same_frequency_rounds':axis['trailing_same_frequency_rounds'],
        'last_three_rounds_actual_span_range_rad':axis['last_three_rounds_actual_span_range_rad'],
        'last_three_rounds_pd_peak_range_nm':axis['last_three_rounds_pd_peak_range_nm'],
        'last_round':axis['rounds'][-1]} for joint, axis in report['establishment_progression']['axes'].items()}
    if 'establishment_regression' in report:
        result['establishment_regression'] = report['establishment_regression']
    if 'artifact_comparison' in report:
        result['artifact_comparison'] = report['artifact_comparison']
    return result


def check_fifteen_probe_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Read-only regression against the original 15-probe task, not a new robot test."""
    checks = []
    def check(label, condition):
        if not condition:
            raise ValueError('15-probe reference regression failed: '+label)
        checks.append(label)
    check('original_task_identity', report['task_id'] == 'model-calibration-1794236700517')
    progression = report['establishment_progression']
    check('fifteen_original_reference_probes', progression['recorded_decisions'] == 15
          and len(report['short_probe_records']) == 15)
    joint = progression['axes']['joint3']
    rows = joint['rounds']
    expected = [16.23931623931624,16.23931623931624,22.832722832722833,31.99023199023199,
        44.81074481074481,62.75946275946276,87.91208791208791,123.07692307692308,172.4053724053724,
        241.39194139194137,337.97313797313797,473.2600732600732,500,500,500]
    check('executed_kp_staircase', all(math.isclose(r['kp'], value, abs_tol=1e-9) for r, value in zip(rows, expected)))
    check('executed_kd_unchanged', all(math.isclose(r['kd'],2.7887667887667886,abs_tol=1e-9) for r in rows))
    check('frequency_frozen_after_first_retest', joint['trailing_same_frequency_rounds'] == 14
          and rows[-1]['frequencies_hz'] == [.3825,.765,1.3114285714285714])
    check('effort_domain_reduces_excitation_while_kp_grows', sum(r.get('amplitude_reduced_while_kp_increased',False) for r in rows) == 3
          and all(r['feedback_effort_budget_nm'] == 7 for r in rows))
    check('late_motion_and_pd_platform_recorded', joint['last_three_rounds_actual_span_range_rad'] == [.002288818359375]*2
          and all(2.7 < p < 2.8 for p in joint['last_three_rounds_pd_peak_range_nm']))
    last = rows[-1]
    check('formal_window_ratio_above_point_two_not_full_record_ratio', last['response_window']['fresh_samples'] == 805
          and math.isclose(last['response_window']['response_rms_ratio'], .204692751140461, abs_tol=1e-9)
          and math.isclose(last['response_window']['centered_actual_rms_rad'], .0005053974285202635, abs_tol=1e-12))
    check('five_and_three_code_travel_below_six_code_requirement', last['positive_travel_rad'] == .0019073486328125
          and last['negative_travel_rad'] == .0011444091796875 and last['required_travel_rad'] == .002288818359375
          and last['boundary_retests'] == 0 and last['retry'] is False)
    check('every_axis_has_actual_gain_and_unlimited_fresh_data', all(
        axis.get('actual_gains_known_samples') == axis['samples'] == axis['fresh_samples']
        and axis['limited_samples'] == 0 for record in report['short_probe_records'].values() for axis in record['axes'].values()))
    summary = report['execution_summary']
    check('insufficient_measurement_not_an_archived_robot_fault', summary['archived_phase'] == 'complete'
          and summary['archived_outcome'] == 'insufficient_measurement' and summary['archived_fault_interrupted'] is False
          and not summary['error'] and not report['response_guard_events'])
    check('no_purpose_pass_or_budget_expiry', report['archived_available'] is False and report['archived_validated'] is False
          and summary['full_path_passes'] == 0 and summary['all_requested_purposes_unstarted'] and not summary['budget_exhausted'])
    return {'status':'passed', 'checks':checks, 'scope':'original_read_only_CSV_and_archived_decisions_not_hardware_reexecution'}


def check_establishment_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Check known raw task records without treating later fixes as re-execution."""
    task = report['task_id']
    if task == 'model-calibration-1794236700517':
        return check_fifteen_probe_regression(report)
    if task == 'model-calibration-1794241207035':
        return check_hold_entry_regression(report)
    if task == 'model-calibration-1794267281603':
        return check_demonstrated_boundary_regression(report)
    if task in ('model-calibration-1794278968578','model-calibration-1794278779311'):
        return check_low_input_plateau_regression(report)
    if task in ('model-calibration-1794284413104','model-calibration-1794284166755'):
        return check_fixed_endpoint_domain_regression(report)
    if task == 'model-calibration-1794288727413':
        return check_entry_quantization_regression(report)
    if task == 'model-calibration-1794291242687':
        return check_endpoint_timeout_regression(report)
    if task == 'model-calibration-1794295009141':
        return check_current_accuracy_regression(report)
    if task not in ('model-calibration-1794238166406', 'model-calibration-1794238336638'):
        raise ValueError('No establishment regression for task '+str(task))
    checks = []
    def check(label, condition):
        if not condition:
            raise ValueError(task+' reference regression failed: '+label)
        checks.append(label)
    summary = report['execution_summary']
    probes = report['short_probe_records']
    rows = report['establishment_progression']['axes']['joint3']['rounds']
    check('no_archived_control_fault_or_response_guard', summary['archived_fault_interrupted'] is False
          and not report['response_guard_events'])
    check('no_purpose_measurement_or_budget_expiry', report['archived_available'] is False
          and report['archived_validated'] is False and summary['all_requested_purposes_unstarted']
          and summary['full_path_passes'] == 0 and not summary['budget_exhausted'])
    check('fresh_actual_gain_evidence_without_limiting', all(
        axis.get('actual_gains_known_samples') == axis['samples'] == axis['fresh_samples']
        and axis['limited_samples'] == 0 for record in probes.values() for axis in record['axes'].values()))
    if task == 'model-calibration-1794238166406':
        check('twelve_original_multitone_probes', len(probes) == len(rows) == 12
              and all(not row['motion_probe'] for row in rows))
        check('same_complete_joint3_gain_for_slower_confirmation', all(
            math.isclose(row['kp'],241.39194139194137,abs_tol=1e-9)
            and math.isclose(row['kd'],2.7887667887667886,abs_tol=1e-9) for row in rows[-3:])
            and max(row['kp'] for row in rows) < 242)
        check('slower_frequency_extends_the_actual_measurement_window', rows[-2]['frequencies_hz'][0] == .26775
              and rows[-1]['frequencies_hz'][0] == .2 and rows[-1]['duration_s'] == 7.65
              and math.isclose(rows[-1]['planned_measurement_window_cycles'][0],1.25,abs_tol=1e-9)
              and rows[-1]['response_window']['planned_window_complete'])
        last = rows[-1]
        check('recorded_directional_and_rms_shortfall', last['positive_travel_rad'] == .002288818359375
              and last['negative_travel_rad'] == .0019073486328125
              and math.isclose(last['required_travel_rad'],.0030351324800064017,abs_tol=1e-12)
              and math.isclose(last['response_window']['response_rms_ratio'],.15108483747068616,abs_tol=1e-9)
              and last['response_window']['fresh_samples'] == 1230)
        check('planned_input_plateau_not_protocol_gain_exhaustion', last['blocked']
              and last['decision_reason'] == 'axis_excitation_input_budget_plateau_at_low_frequency_boundary'
              and last['input_response_progress']['input_budget_plateau']
              and last['kp'] < last['protocol_kp_max'] and last['feedback_effort_budget_nm'] == 7
              and math.isclose(last['pd_command']['peak_abs'],2.7746612771039456,abs_tol=1e-9))
        check('recorded_insufficient_measurement_exit', summary['archived_phase'] == 'complete'
              and summary['archived_outcome'] == 'insufficient_measurement' and not summary['error']
              and math.isclose(summary['actual_elapsed_s'],96.120621942,abs_tol=1e-9))
    else:
        partial = probes['servo-probe-native-establish-2.csv']['axes']['joint3']
        check('one_completed_decision_and_two_saved_probes', len(rows) == 1 and len(probes) == 2)
        check('second_probe_is_partial_without_fit_or_decision', partial['samples'] == 862
              and math.isclose(partial['last_time_s'],4.378096506,abs_tol=1e-9)
              and partial['response_window']['planned_window_complete'] is False
              and partial['archived_identification_reason'] == 'not_recorded')
        check('partial_probe_does_not_establish_motion', partial['actual_span_rad'] == 0
              and partial['motion_diagnostic'] == 'fresh_feedback_no_position_response')
        check('cancelled_status_retained_despite_old_candidate_fault_label', summary['archived_phase'] == 'cancelled'
              and summary['archived_outcome'] == 'execution_fault' and summary['outcome'] == 'cancelled'
              and summary['termination_consistency_issues'] == ['cancelled_status_with_legacy_execution_fault_candidate'])
        check('original_cancellation_elapsed_and_error_preserved', math.isclose(summary['actual_elapsed_s'],14.523766726,abs_tol=1e-9)
              and summary['error'] == 'short response experiment interrupted')
    return {'status':'passed', 'checks':checks,
            'scope':'original_read_only_CSV_and_archived_decisions_not_hardware_reexecution'}


def check_hold_entry_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Preserve the original complete path/hold and rejected HOLD entry."""
    checks = []
    def check(label, condition):
        if not condition:
            raise ValueError('HOLD-entry reference regression failed: '+label)
        checks.append(label)
    summary = report['execution_summary']
    check('original_software_execution_error_without_robot_fault', summary['archived_phase'] == 'failed'
          and summary['error'] == 'short response input rejected' and summary['archived_fault_interrupted'] is False
          and summary['termination_objects_match'] and not report['response_guard_events'])
    check('original_budget_and_elapsed_preserved', report['budget']['recorded_source_duration_s'] == 23
          and summary['hard_budget_s'] == 345 and not summary['budget_exhausted']
          and math.isclose(summary['actual_elapsed_s'],220.332346088,abs_tol=1e-9))
    check('hold_disturbance_rejected_before_any_sample', summary['last_probe_kind'] == 'rigid_tracking-hold-disturbance'
          and report['short_probe_records']['servo-probe-rigid_tracking-hold-disturbance.csv']['status'] == 'no_valid_samples')
    path = report['trajectory_records']['gain-trajectory-native-frozen.csv']['axes']
    expected = [1.6008196533219479,1.044294911826663,1.077334653372139,1.202442687731282,1.2151474508579305,1.9066086547415753]
    check('complete_native_path_retains_all_cycle_peaks', len(path) == 6 and all(
        axis['samples'] == axis['fresh_samples'] == 6347 and axis['limited_samples'] == 0
        and math.isclose(axis['peak_error_deg'],peak,abs_tol=1e-9)
        for axis,peak in zip(path.values(),expected)))
    check('rigid_and_servo_parameter_measurements_are_complete', all(
        report['targets'][key]['nested_parameter_measurement_complete'] is True
        and report['targets'][key]['legacy_available_aggregation_conflict']
        and report['targets'][key]['archived_available'] is False and report['targets'][key]['archived_validated'] is False
        for key in ('rigid_tracking','servo_tracking')))
    rigid = report['targets']['rigid_tracking']
    check('actual_second_baseline_hold_complete_without_integral', rigid['hold_trials']['hold_verification']['available'] is True
          and rigid['hold_trials']['hold_verification']['validated'] is True
          and rigid['hold_trials']['hold_verification']['integral_samples'] == 0
          and rigid['hold_compensation_saved']['enabled'] is False)
    check('unstarted_compliant_and_drag_remain_unmeasured', all(
        report['targets'][key]['nested_parameter_measurement_complete'] is False
        and report['targets'][key]['measurement_state'] == 'not_started'
        for key in ('compliant_tracking','compliant_drag')))
    one_code = report['short_probe_records']['servo-probe-native-established-disturbance.csv']['axes']['joint1']
    check('original_one_code_disturbance_misclassification_is_not_rewritten', one_code['actual_span_rad'] == .0003814697265625
          and one_code['archived_motion_observed'] is True
          and one_code['archived_disturbance_assessment_reason'] == 'sustained_oscillation_after_release')
    check('missing_original_correction_plan_is_not_invented', not compact_summary(report)['full_path_correction_plans'])
    return {'status':'passed', 'checks':checks,
            'scope':'original_read_only_path_hold_and_rejected_entry_not_new_hardware_execution'}


def check_demonstrated_boundary_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Keep the original 12 measured probes separate from the unexecuted proposal."""
    checks = []
    def check(label, condition):
        if not condition:
            raise ValueError('Demonstrated-boundary reference regression failed: '+label)
        checks.append(label)
    summary = report['execution_summary']
    probes = report['short_probe_records']
    progression = report['establishment_progression']
    rows = progression['axes']['joint3']['rounds']
    check('twelve_actual_probes_without_inventing_a_thirteenth', len(probes)==len(rows)==12
          and 'servo-probe-native-establish-13.csv' not in probes
          and all(not row['motion_probe'] for row in rows))
    check('insufficient_input_plan_not_a_robot_fault_or_budget_expiry', summary['outcome']=='insufficient_measurement'
          and summary['warnings'][0]=='identification_insufficient:independent disturbance plan exceeds current calibration effort/Safety allowance'
          and summary['archived_fault_interrupted'] is False and not summary['budget_exhausted']
          and not summary['error'] and not report['response_guard_events'] and summary['termination_objects_match'])
    check('trusted_source_and_original_elapsed', report['budget']['recorded_source_duration_s']==23
          and summary['hard_budget_s']==345 and math.isclose(summary['actual_elapsed_s'],95.817887322,abs_tol=1e-9)
          and summary['full_path_passes']==0 and summary['all_requested_purposes_unstarted'])
    check('unlimited_continuous_feedback_with_recorded_actual_gains', all(
        axis['samples']==axis['fresh_samples']==axis.get('actual_gains_known_samples')
        and axis['limited_samples']==0 and axis['maximum_fresh_feedback_gap_s']<.03
        for probe in probes.values() for axis in probe['axes'].values())
        and report['cycle_record']['maximum_cycle_gap_s']<.011)
    check('joint3_slowed_at_same_gains_with_an_actual_longer_window', all(
        math.isclose(row['kp'],241.39194139194137,abs_tol=1e-9) and row['kd']==5 for row in rows[-3:])
        and [row['frequencies_hz'][0] for row in rows[-3:]]==[.3825,.26775,.2]
        and rows[-1]['duration_s']==7.65 and rows[-1]['response_window']['planned_window_complete'])
    last=rows[-1]
    check('last_measured_joint3_response_shortfall_and_only_proposed_single_tone',
          math.isclose(last['response_window']['response_rms_ratio'],.153587125034818,abs_tol=1e-9)
          and last['positive_travel_rad']==.002288818359375 and last['negative_travel_rad']==.0019073486328125
          and math.isclose(last['required_travel_rad'],.002858624397879064,abs_tol=1e-12)
          and last['decision_reason']=='spectral_excitation_boundary_requires_symmetric_motion_probe_at_same_complete_gains'
          and math.isclose(last['pd_command']['peak_abs'],2.566106153554522,abs_tol=1e-9))
    envelope=report['position_envelope_evidence']
    final=envelope['short_probe_ranges'][last['record_path']]['axes']
    check('joint2_and_joint3_reference_outside_saved_path_but_inside_declared_URDF', all(
        final[joint]['reference_margin_to_saved_path_rad'][0]<-.006
        and final[joint]['reference_margin_to_urdf_rad'][0]>.08 for joint in ('joint2','joint3')))
    check('other_axes_frozen_while_joint3_alone_slowed', all(
        progression['axes'][joint]['rounds'][-1]['frequencies_hz']==[.3825,.765,1.3114285714285714]
        and len({row['kp'] for row in progression['axes'][joint]['rounds'][-3:]})==1
        for joint in ('joint1','joint2','joint4','joint5','joint6')))
    check('wrist_weak_motion_far_from_the_saved_path_boundary', all(
        final[joint]['actual_margin_to_saved_path_rad'][0]>.1
        and final[joint]['actual_margin_to_saved_path_rad'][1]>.1
        and progression['axes'][joint]['rounds'][-1]['actual_span_rad']<=.000762939453125
        for joint in ('joint5','joint6')))
    check('new_bias_and_mask_evidence_not_fabricated_for_original_record', all(
        axis['planned_reference_bias_rad'] is None and axis['planned_disturbance_axis_eligible'] is None
        for probe in probes.values() for axis in probe['axes'].values())
        and not report['unrun_disturbance_experiments'] and report['archived_validated'] is False)
    return {'status':'passed','checks':checks,
            'scope':'original_read_only_twelve_CSVs_not_reexecution_or_a_measured_physical_stop'}


def check_low_input_plateau_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Preserve a fully sampled, low-input plateau exit without asserting capacity."""
    checks=[]
    def check(label,condition):
        if not condition:
            raise ValueError('Low-input plateau reference regression failed: '+label)
        checks.append(label)
    latest=report['task_id']=='model-calibration-1794278968578'
    summary=report['execution_summary'];probes=report['short_probe_records']
    series=report['establishment_progression']['axes']
    decisions=[x for x in report['candidate_comparisons'] if x.get('kind')=='motion_establishment']
    last=decisions[-1];blocked='joint5' if latest else 'joint3';index=4 if latest else 2
    check('complete_insufficient_measurement_not_fault_or_budget_expiry',
          summary['archived_phase']=='complete' and summary['outcome']=='insufficient_measurement'
          and summary['archived_fault_interrupted'] is False and not summary['error']
          and not summary['budget_exhausted'] and not report['response_guard_events']
          and summary['termination_objects_match'] and summary['recorded_remaining_s']>200)
    check('all_actual_probes_complete_with_fresh_stable_gain_and_no_limiting',len(probes)==(11 if latest else 12)
          and sum(axis['samples']-axis['fresh_samples'] for record in probes.values() for axis in record['axes'].values())==(0 if latest else 2)
          and all(axis['samples']==axis['actual_gains_known_samples'] and axis['fresh_samples']>0
                  and axis['limited_samples']==0 and axis['response_window']['planned_window_complete']
                  and axis['maximum_fresh_feedback_gap_s']<.014
                  and axis['recorded_actual_kp']['min']==axis['recorded_actual_kp']['max']
                  and axis['recorded_actual_kd']['min']==axis['recorded_actual_kd']['max']
                  for record in probes.values() for axis in record['axes'].values()))
    check('every_recorded_reference_stays_in_its_recorded_working_domain',all(
        axis['planned_reference_domain_rad'][0]-1e-12<=axis['reference_min_rad']
        and axis['reference_max_rad']<=axis['planned_reference_domain_rad'][1]+1e-12
        for record in probes.values() for axis in record['axes'].values()))
    check('no_full_path_or_purpose_measurement_is_promoted',summary['full_path_passes']==0
          and summary['all_requested_purposes_unstarted'] and report['archived_available'] is False
          and report['archived_validated'] is False)
    tail=series[blocked]['rounds'][-1]
    check('single_tone_blocked_at_tested_input_with_original_reason',last['blocked_axes']==[index]
          and tail['motion_probe'] and tail['blocked'] and tail['archived_tracking_valid'] is False
          and tail['decision_reason']=='symmetric_motion_probe_insufficient_at_tested_input_not_hardware_capability'
          and tail['input_response_progress']['plateau_motion_confirmation'])
    check('blocked_axis_is_below_recorded_gain_and_effort_bounds',tail['kp']<tail['protocol_kp_max']
          and tail['pd_command']['peak_abs']<tail['feedback_effort_budget_nm']
          and tail['recorded_current_kp']==tail['kp'] and tail['recorded_current_kd']==tail['kd'])
    if latest:
        rows=series['joint5']['rounds']
        expected=[.3663003663003663]*4+[.6105006105006104,.8547008547008547,1.2210012210012209,
            1.7094017094017093,2.4420024420024418,2.4420024420024418,2.4420024420024418]
        check('actual_joint5_gain_staircase_and_unchanged_damping',
              [row['kp'] for row in rows]==expected and all(row['kd']==.1111111111111111 for row in rows))
        check('slower_multitone_then_single_tone_are_real_complete_trials',
              rows[-3]['motion_probe'] and not rows[-2]['motion_probe'] and rows[-1]['motion_probe']
              and rows[-2]['frequencies_hz'][0]==rows[-1]['frequencies_hz'][0]==.2
              and rows[-1]['duration_s']==11.4
              and rows[-1]['response_window']['fresh_samples']==1969)
        check('low_response_does_not_get_hidden_by_the_full_record_span',
              tail['positive_travel_rad']==.0003814697265625 and tail['negative_travel_rad']==.000762939453125
              and math.isclose(tail['required_travel_rad'],.01119997767141106,abs_tol=1e-12)
              and math.isclose(tail['response_window']['response_rms_ratio'],.008128583072283167,abs_tol=1e-10)
              and math.isclose(tail['pd_command']['peak_abs'],.0783786748736684,abs_tol=1e-10))
        check('early_small_direction_span_anchor_and_later_response_plateau_are_preserved',
              rows[6]['input_response_progress']['anchor_span_rad']==.0030517578125
              and rows[6]['input_response_progress']['anchor_minimum_travel_rad']==.0003814697265625
              and rows[8]['decision_reason']=='response_effort_plateau_requires_slower_complete_gain_set_confirmation'
              and rows[9]['decision_reason']=='confirmed_response_plateau_requires_symmetric_motion_probe_at_same_complete_gains')
        check('joint5_is_far_from_demonstration_edges_with_working_gain_headroom',
              min(report['position_envelope_evidence']['short_probe_ranges'][tail['record_path']]['axes']['joint5']['reference_margin_to_saved_path_rad'])>.65
              and probes[tail['record_path']]['actual_trial_parameters']['working_domain'][4]['working_kp_max']>44)
        check('other_axes_motion_and_unexecuted_next_steps_remain_distinct',
              last['established_axes']==[True,True,True,False,False,False]
              and last['next_kp'][3]>last['current_kp'][3] and last['next_kp'][5]>last['current_kp'][5]
              and math.isclose(summary['actual_elapsed_s'],126.999660274,abs_tol=1e-9))
    else:
        check('comparison_task_blocked_joint3_while_wrist_steps_remained_proposed',
              last['established_axes']==[True,True,False,True,False,False]
              and last['next_kp'][4]>last['current_kp'][4] and last['next_kp'][5]>last['current_kp'][5]
              and series['joint5']['rounds'][-1]['kp']==4.8840048840048835
              and math.isclose(summary['actual_elapsed_s'],111.36628776,abs_tol=1e-9))
    return {'status':'passed','checks':checks,
            'scope':'original_read_only_completed_CSVs_not_new_conditions_or_a_friction_capacity_measurement'}


def check_fixed_endpoint_domain_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Distinguish a fixed endpoint working-domain exit from a bus-write fault."""
    checks=[]
    def check(label,condition):
        if not condition:
            raise ValueError('Fixed-endpoint reference regression failed: '+label)
        checks.append(label)
    latest=report['task_id']=='model-calibration-1794284413104'
    summary=report['execution_summary'];probes=report['short_probe_records']
    rows=report['establishment_progression']['axes']['joint3']['rounds']
    centers=[record['actual_trial_parameters']['center_rad'][2] for record in probes.values()]
    center=.01125335693359375 if latest else .01049041748046875
    check('trusted_recording_budget_not_retimed_duration',report['budget']['recorded_source_duration_s']==28.14
          and summary['hard_budget_s']==422.1 and not summary['budget_exhausted'])
    check('all_probes_keep_initial_fresh_endpoint_without_recentering',len(probes)==(16 if latest else 12)
          and all(value==center for value in centers)
          and all(record['actual_trial_parameters']['center_source']=='fresh_measured_establishment_center' for record in probes.values()))
    check('endpoint_is_below_checked_demonstration_not_a_declared_hardware_stop',
          math.isclose(report['position_envelope_evidence']['saved_reference']['axes']['joint3']['min'],.016593933105468743,abs_tol=1e-12)
          and all(axis['planned_reference_domain_rad'][0]==center for record in probes.values() for joint,axis in record['axes'].items() if joint=='joint3')
          and report['position_envelope_evidence']['saved_reference']['axes']['joint3']['max']>1.5)
    check('recorded_commands_retain_known_gains_and_no_limiting',all(
        axis['actual_gains_known_samples']==axis['samples'] and axis['limited_samples']==0 and axis['fresh_samples']>=100
        and axis['maximum_fresh_feedback_gap_s']<.06 for record in probes.values() for axis in record['axes'].values()))
    check('no_full_path_or_purpose_validation_is_invented',summary['full_path_passes']==0
          and summary['all_requested_purposes_unstarted'] and report['archived_available'] is False and report['archived_validated'] is False)
    check('motion_was_measured_before_the_final_exit',all(row['archived_motion_qualified'] for row in rows[8:11])
          and all(row['motion_probe'] and math.isclose(row['frequencies_hz'][0],1/6,abs_tol=1e-12) for row in rows[8:11])
          and rows[8]['kp']==172.4053724053724)
    if latest:
        final=rows[-1]
        check('all_sixteen_actual_windows_are_complete',len(rows)==16 and all(
            axis['response_window']['planned_window_complete'] for record in probes.values() for axis in record['axes'].values())
            and probes[final['record_path']]['axes']['joint3']['samples']==2931)
        check('final_current_motion_miss_is_not_promoted_by_historical_axis_flags',
              final['archived_motion_qualified'] is False and final['blocked'] and not final['retry']
              and report['candidate_comparisons'][-1]['established_axes']==[True]*6
              and final['positive_travel_rad']==.003814697265625 and final['negative_travel_rad']==.002288818359375
              and math.isclose(final['required_travel_rad'],.0038411660187035335,abs_tol=1e-12))
        check('local_known_input_is_saturated_without_protocol_gain_or_robot_fault',
              final['decision_reason']=='excitation_amplitude_calibration_no_legal_input_growth_in_observed_domain'
              and final['kp']<final['protocol_kp_max'] and final['input_response_progress']['input_budget_plateau']
              and math.isclose(final['input_response_progress']['planned_input_nm'],3.3114276526031996,abs_tol=1e-9)
              and summary['archived_fault_interrupted'] is False and not report['response_guard_events'])
        check('original_insufficient_exit_and_remaining_time',summary['outcome']=='insufficient_measurement'
              and math.isclose(summary['actual_elapsed_s'],210.283872595,abs_tol=1e-9)
              and math.isclose(summary['recorded_remaining_s'],211.816127405,abs_tol=1e-9))
    else:
        partial=probes['servo-probe-native-establish-12.csv']['axes']['joint3']
        check('bus_write_fault_retains_partial_twelfth_window_without_a_decision',len(rows)==11
              and partial['samples']==1978 and partial['response_window']['planned_window_complete'] is False
              and math.isclose(partial['last_time_s'],10.087969692,abs_tol=1e-9)
              and summary['outcome']=='execution_fault' and summary['archived_fault_interrupted'] is True
              and 'MOTOR_BUS_WRITE_FAILED / WRITE_FAILED' in summary['error'])
        check('comparison_has_different_cause_and_elapsed_not_a_motion_block',
              not rows[-1]['blocked'] and rows[-1]['archived_motion_qualified']
              and math.isclose(summary['actual_elapsed_s'],136.422213997,abs_tol=1e-9))
    return {'status':'passed','checks':checks,
            'scope':'original_read_only_endpoint_motion_and_fault_evidence_not_relocation_or_physical_stop_identification'}


def check_entry_quantization_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Reproduce the archived unknown-step lead rejection, without replaying it."""
    checks=[]
    def check(label,condition):
        if not condition:
            raise ValueError('Measurement-entry quantization regression failed: '+label)
        checks.append(label)
    summary=report['execution_summary']
    record=report['measurement_center_records']['gain-transition-native-measurement-entry.csv']
    entry=record['archived_experiment'];axis=record['axes']['joint3']
    tail=axis['last_fresh_transition'];last=axis['last_fresh_sample']
    check('entry_ended_before_identification_and_every_purpose',entry['completed'] is False
          and entry['current_segment']==15 and entry['segment_count']==63
          and not report['short_probe_records'] and summary['full_path_passes']==0
          and summary['all_requested_purposes_unstarted'])
    check('archived_insufficient_exit_is_not_a_fault_or_budget_expiry',summary['outcome']=='insufficient_measurement'
          and summary['archived_fault_interrupted'] is False and not summary['budget_exhausted']
          and summary['termination_objects_match'] is True
          and summary['archived_status_termination']['reason']=='identification_insufficient:measurement_center_entry:path_reference_outside_feedback_lead'
          and math.isclose(summary['actual_elapsed_s'],17.527012053,abs_tol=1e-9)
          and math.isclose(summary['recorded_remaining_s'],404.572987947,abs_tol=1e-9))
    check('actual_partial_entry_cycles_retain_known_gains_and_record_the_single_repeat',all(
        a['samples']==a['actual_gains_known_samples']==3026
        and a['fresh_samples']==(3025 if joint=='joint6' else 3026)
        and a['limited_samples']==0 and a['maximum_fresh_feedback_gap_s']<.06
        and not a['nonmonotonic_fresh_feedback'] and not a['repeated_fresh_feedback_sequence']
        for joint,a in record['axes'].items()))
    noise=next(item for item in report['experiments'] if item.get('stage')=='trajectory_noise')['axes']
    check('initial_static_zero_step_is_not_the_step_observed_during_entry',noise[2]['encoder_step_rad']==0
          and noise[2]['noise_rms_rad']==0 and axis['recorded_position_uncertainty_rad']==1e-6
          and axis['minimum_observed_position_step_rad']==.0003814697265625
          and math.isclose(axis['first_nonzero_fresh_position_step']['after']['time_s'],10.622906473,abs_tol=1e-9))
    check('last_fresh_position_reverses_one_code_while_reference_advances',
          tail['actual_delta_rad']==-.0003814697265625
          and math.isclose(tail['reference_delta_rad'],2.275642993690152e-6,abs_tol=1e-12)
          and tail['before']['actual_rad']==.01239776611328125 and last['actual_rad']==.01201629638671875
          and math.isclose(last['reference_rad'],.02201757202971244,abs_tol=1e-12))
    check('only_joint3_last_row_exceeds_original_unchanged_hard_lead',axis['recorded_feedback_lead_rad']==.01
          and axis['recorded_hard_lead_exceeded_rows']==1
          and math.isclose(last['reference_minus_actual_rad']-.01,1.27564299369e-6,abs_tol=1e-12)
          and all(not a.get('recorded_hard_lead_exceeded_rows') for joint,a in record['axes'].items() if joint!='joint3')
          and entry['feedback_lead_rad']==[.01]*6)
    check('bounded_gain_steps_change_P_without_D_or_hard_lead_tightening',
          len(entry['gain_steps'])==17 and entry['kd']==entry['initial_kd']
          and all(step['kd']==entry['initial_kd'] for step in entry['gain_steps'])
          and math.isclose(axis['recorded_actual_kp']['last'],62.14896214896215,abs_tol=1e-10)
          and axis['recorded_actual_kd']['min']==axis['recorded_actual_kd']['max']
          and math.isclose(axis['reconstructed_pd_nm']['peak_abs'],.6446705585515932,abs_tol=1e-10))
    check('archived_parameters_do_not_become_available_or_validated',report['archived_available'] is False
          and report['archived_validated'] is False and not report['response_guard_events'])
    return {'status':'passed','checks':checks,
            'scope':'original_raw_entry_quantization_and_lead_rejection_not_new_transfer_success_or_physical_friction_identification'}


def check_endpoint_timeout_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Preserve the completed local evidence and premature endpoint timeout."""
    checks=[]
    def check(label,condition):
        if not condition:
            raise ValueError('Endpoint timeout reference regression failed: '+label)
        checks.append(label)
    summary=report['execution_summary'];trans=report['endpoint_transition_records']['gain-transition-native-frozen.csv']
    metadata=trans['archived_experiment'];axes=trans['axes']
    records=report['measurement_center_records'];entry=records['gain-transition-native-measurement-entry.csv']['archived_experiment']
    returning=records['gain-transition-native-measurement-return.csv']['archived_experiment']
    check('current_entry_and_return_completed_without_reusing_old_center_passes',entry['completed'] and returning['completed']
          and entry['direction']=='toward_measurement_region' and returning['direction']=='return_to_demonstrated_endpoint'
          and entry['actual_rad'][2]>.118 and entry['observed_position_step_rad'][2]==.0003814697265625)
    check('current_complete_group_motion_and_real_spectral_confirmation_exist',
          report['establishment_progression']['recorded_decisions']==15
          and report['candidate_comparisons'][14]['motion_established'] is True
          and all(axis['qualified'] for axis in report['candidate_comparisons'][14]['axes'])
          and 'servo-probe-native-spectral-confirmation.csv' in report['short_probe_records'])
    check('local_transition_timeout_not_total_budget_or_safety_fault',summary['outcome']=='insufficient_measurement'
          and summary['archived_fault_interrupted'] is False and not summary['budget_exhausted']
          and summary['archived_status_termination']['reason']=='identification_insufficient:gain_endpoint_transition_insufficient_response'
          and math.isclose(summary['actual_elapsed_s'],244.009482464,abs_tol=1e-9)
          and math.isclose(summary['recorded_remaining_s'],178.090517536,abs_tol=1e-9)
          and metadata['completed'] is False and math.isclose(metadata['elapsed_s'],.857375203,abs_tol=1e-9))
    legacy_cost=max(.8,max(abs(a-b)/v+v/acc+.5 for a,b,v,acc in zip(
        metadata['initial_reference_rad'],metadata['target_rad'],metadata['max_velocity_rad_s'],metadata['max_acceleration_rad_s2'])))
    check('old_geometric_timeout_can_be_recomputed_from_recorded_parameters',math.isclose(legacy_cost,.8568115234375,abs_tol=1e-12)
          and metadata['elapsed_s']>legacy_cost)
    check('actual_transition_has_known_unlimited_partial_cycles_within_original_error_caps',all(
        axis['samples']==axis['fresh_samples']==axis['actual_gains_known_samples']==169
        and axis['limited_samples']==0 and axis['peak_error_rad']<axis['archived_reference_error_cap_rad']
        for axis in axes.values()))
    wrist=axes['joint4']
    check('one_reference_is_subcode_close_but_not_exactly_stopped',
          'first_exact_stopped_reference_time_s' not in wrist
          and math.isclose(wrist['last_reference_minus_target_rad'],1.4126385845679781e-5,abs_tol=1e-12)
          and math.isclose(wrist['last_reference_velocity_rad_s'],.00020534020414798062,abs_tol=1e-12)
          and wrist['minimum_observed_position_step_rad']==.0003814697265625)
    check('another_axis_arrives_too_late_for_old_150ms_ready_window',
          math.isclose(axes['joint2']['first_exact_stopped_reference_time_s'],.82666382,abs_tol=1e-9)
          and axes['joint2']['first_exact_stopped_reference_time_s']+.15>legacy_cost)
    check('started_pass_counter_is_not_an_actual_full_path_measurement',summary['full_path_passes']==1
          and report['trajectory_record_counts']['records_with_position_samples']==0
          and report['trajectory_record_counts']['header_only_records']==['gain-trajectory-native-frozen.csv']
          and summary['all_requested_purposes_unstarted'] and report['archived_available'] is False and report['archived_validated'] is False)
    return {'status':'passed','checks':checks,
            'scope':'original_endpoint_reference_timeout_and_raw_coverage_not_new_trajectory_success_or_hardware_acceptance'}


def check_current_accuracy_regression(report: dict[str, Any]) -> dict[str, Any]:
    """Reproduce the current poor tracking without attributing it to a motor."""
    checks = []
    def check(label, condition):
        if not condition:
            raise ValueError('Current accuracy reference regression failed: '+label)
        checks.append(label)
    summary = report['execution_summary']
    name = 'gain-trajectory-native-frozen.csv'
    axes = report['trajectory_records'][name]['axes']
    motion = report['full_path_execution_diagnostics'][name]
    check('no_archived_safety_fault_limiting_or_budget_expiry',
          summary['archived_fault_interrupted'] is False and not summary['budget_exhausted']
          and not report['response_guard_events'] and all(not axis['limited_samples'] for axis in axes.values()))
    check('one_real_native_path_and_unrun_compliant_path_are_distinct',
          report['trajectory_record_counts']['records_with_position_samples']==1
          and report['trajectory_record_counts']['header_only_records']==['gain-trajectory-compliant-frozen.csv']
          and summary['full_path_passes']==1 and axes['joint1']['samples']==9166)
    check('all_axes_have_real_peak_error_above_original_point_one_degree',
          all(axis['peak_error_deg']>.9 and not axis['peak_target_met'] for axis in axes.values())
          and math.isclose(axes['joint6']['peak_error_deg'],2.3635198893295706,abs_tol=1e-9))
    check('receive_timestamp_alignment_does_not_remove_poor_tracking',
          motion['joint6']['receive_aligned_error_rad']['peak']>.03
          and motion['joint1']['receive_aligned_error_rad']['rms']>.006)
    check('reference_position_and_velocity_show_real_staircase_intervals',
          motion['joint5']['peak_position_velocity_mismatch_rad']>.01
          and motion['joint6']['peak_position_velocity_mismatch_rad']>.01
          and motion['joint3']['reference_duplicate_intervals_with_nonzero_velocity']>0)
    check('feedback_velocity_has_consistent_sign_and_scale',all(
          .90 < axis['reported_velocity_to_40ms_position_difference_scale'] < 1.1 for axis in motion.values()))
    progress = report['establishment_progression']['axes']['joint6']['rounds']
    check('wrist_P_rises_without_paired_D_during_thirteen_startup_windows',
          len(progress)==13 and progress[-1]['kp']>100*progress[0]['kp']
          and all(row['kd']==progress[0]['kd'] for row in progress)
          and axes['joint6']['recorded_actual_kd']['min']==axes['joint6']['recorded_actual_kd']['max'])
    check('moving_reference_has_position_plateaus_and_direction_error',
          motion['joint5']['moving_reference_position_plateaus']>20
          and motion['joint6']['longest_moving_reference_position_plateau']['duration_s']>.1
          and motion['joint6']['direction_mean_error_rad']['positive']>.01
          and motion['joint6']['direction_mean_error_rad']['negative']<-.01)
    return {'status':'passed','checks':checks,
            'scope':'original_full_cycle_reference_timing_gains_and_feedback_not_motor_fault_identification_or_new_hardware_validation'}


def self_test():
    linear = [{'time_s':i*.01,'reference':i*.002,'reference_velocity':.2,
               'actual':i*.002-.001,'velocity':.2,'feedback_time_s':i*.01-.005} for i in range(100)]
    kinematic = tracking_kinematics(linear)
    assert kinematic['peak_position_velocity_mismatch_rad'] < 1e-15
    assert kinematic['receive_aligned_error_rad']['peak'] < 1e-15
    assert math.isclose(kinematic['reported_velocity_to_40ms_position_difference_scale'],1,abs_tol=1e-12)
    timed = [dict(row, time_s=row['time_s']+.001*(index%3),
                  reference_sample_time_s=row['time_s'],reference_sample_time_known=True)
             for index,row in enumerate(linear)]
    sampled = tracking_kinematics(timed)
    assert sampled['known_reference_sample_times']==100
    assert sampled['worst_reference_interval']['reference_interval_time_basis']=='reference_sample_time_s'
    assert sampled['peak_position_velocity_mismatch_rad'] < 1e-15
    staircase = [dict(row) for row in linear]
    staircase[30]['reference'] = staircase[29]['reference']
    inconsistent = tracking_kinematics(staircase)
    assert inconsistent['peak_position_velocity_mismatch_rad'] > .0019
    assert inconsistent['reference_duplicate_intervals_with_nonzero_velocity'] == 1
    assert inconsistent['scope'] == 'diagnostic_only_original_full_cycle_error_and_acceptance_unchanged'
    stationary = [dict(row,actual=0,velocity=0) for row in linear[:20]]
    stationary.append(dict(linear[20],actual=.001,velocity=0))
    plateau = tracking_kinematics(stationary)
    assert plateau['longest_moving_reference_position_plateau']['duration_s'] == .19
    assert tracking_kinematics([linear[1],linear[0],linear[2]])['status']=='insufficient_or_nonmonotonic_record'
    with tempfile.TemporaryDirectory(prefix='serial-arm-gain-audit-') as directory:
        folder = Path(directory) / 'task'
        folder.mkdir()
        (folder / 'gain-candidate.json').write_text(json.dumps({
            'task_id': 'synthetic', 'seed_source': 'configured_servo', 'budget_source_duration_s': 25,
            'available': True, 'validated': False, 'targets': {'rigid_tracking': {
                'available': True, 'validated': False, 'response_model_status': 'not_estimated',
                'hold_compensation': {'enabled': False},
                'hold_compensation_verification': {'integral_samples': 1}}}}))
        for index, endpoints in enumerate(((0, 1), (1, 0))):
            (folder / f'gain-trajectory-native-train-{index}.csv').write_text(
                'time_s,joint,reference,actual,new_measurement,limited\n'
                f'0,j1,{endpoints[0]},{endpoints[0]},1,0\n'
                f'.01,j1,{endpoints[1]},{endpoints[1] - .01},0,1\n')
        source = Evidence(folder)
        report = audit(source, 0.1)
        assert report['budget']['requested_quick_hard_limit_s'] == 375
        assert report['budget']['requested_fine_hard_limit_s'] == 600
        assert report['archived_validated'] is False
        assert report['candidate_path_comparisons'][0]['opposite_path_endpoints']
        assert report['targets']['rigid_tracking']['integral_trial_present_but_saved_disabled']
        axis = report['trajectory_records']['gain-trajectory-native-train-0.csv']['axes']['j1']
        assert not axis['peak_target_met'] and axis['samples'] == 2 and axis['fresh_samples'] == 1
        artifact = Path(directory) / 'artifact.zip'
        with zipfile.ZipFile(artifact, 'w') as archive:
            for name in source.names:
                archive.write(folder / name, 'evidence/' + name)
        companion = Evidence(artifact)
        assert not compare(source, companion)['different_files']
        assert audit(companion, 0.1)['missing_files'] == ['result.json', 'frames.csv']
        companion.close()
        candidate = json.loads((folder / 'gain-candidate.json').read_text())
        candidate.update(candidate.pop('targets')['rigid_tracking'])
        candidate['category'] = 'rigid_tracking'
        (folder / 'gain-candidate.json').write_text(json.dumps(candidate))
        assert list(audit(source, 0.1)['targets']) == ['rigid_tracking']
        (folder / 'servo-probe-native-short-0.csv').write_text(
            'time_s,joint,reference,reference_velocity,actual,velocity,limited,new_measurement,feedback_time_s,feedback_sequence\n'
            '0,j1,0,0,.03,0,0,1,0,1\n.01,j1,.01,1,.03,0,0,1,.01,2\n'
            '0,j2,0,0,0,0,0,1,0,1\n.01,j2,.01,1,.001,0,0,1,.01,2\n'
            '0,j3,0,0,0,0,0,0,0,1\n.01,j3,.01,1,0,0,0,0,.01,1\n')
        (folder / 'servo-probe-native-short-disturbance-0.csv').write_text(
            'time_s,joint,reference,actual,independent_disturbance_nm,limited,new_measurement,feedback_time_s,feedback_sequence\n'
            '0,j1,0,0,.1,0,1,0,1\n.01,j1,0,0,-.1,0,1,.01,2\n')
        (folder / 'result.json').write_text(json.dumps({'phase': 'complete', 'gain_elapsed_s': 32.65740481,
            'gain_budget_exhausted': False, 'gain_path_pass_current': 0,
            'gain_warnings': ['identification_insufficient:native', 'rigid_tracking:not_started']}))
        candidate.update({'available': False, 'validated': False, 'time_budget_s': 423.3, 'warnings': ['not_started'],
                          'task_model_demand': {'executed':False, 'measured':False},
                          'response_guard_events':[{'joint':'j1','time_s':.9,'actual_kp':100,'actual_kd':2,
                              'reaction':'latched_existing_stop_hold','mechanical_resonance_identified':False}],
                          'joint_names': ['j1', 'j2', 'j3'], 'experiments': [{
                              'kind': 'reference_tracking', 'record_path': 'servo-probe-native-short-0.csv', 'axes': []}, {
                              'kind': 'independent_torque_disturbance', 'record_path': 'servo-probe-native-short-disturbance-0.csv',
                              'axes': [{'disturbance_valid': False, 'disturbance_goal_met': True},
                                       {'kind': 'disturbance_recovery', 'static_measurement_complete': True,
                                        'motion_observed': False, 'measurement_complete': False}]}]})
        (folder / 'gain-candidate.json').write_text(json.dumps(candidate))
        with (folder / 'frames.csv').open('w') as stream:
            writer = csv.writer(stream)
            writer.writerow(['monotonic_ns','phase','gain_stage','valid','q:j1','dq:j1','ref_q:j1','ref_dq:j1',
                             'cmd_kp:j1','cmd_kd:j1','cmd_tor:j1','feedback_received_ns:j1','feedback_timing_valid:j1'])
            for i, q in enumerate((0,.006,.001,.007,.001,.007,.007)):
                writer.writerow([int((1+i*.01)*1e9),'gain_adaptation','native-frozen',1,q,.2,.01,0,
                                 100 if i<3 else 200,2,.5,int((1+min(i,5)*.01)*1e9),1])
        source.close()
        source = Evidence(folder)
        report = audit(source, .1)
        axes = report['short_probe_records']['servo-probe-native-short-0.csv']['axes']
        assert axes['j1']['motion_diagnostic'] == 'fresh_feedback_no_position_response'
        assert axes['j2']['motion_diagnostic'] == 'fresh_feedback_weak_position_response'
        assert axes['j3']['motion_diagnostic'] == 'feedback_missing_or_discontinuous'
        assert not axes['j1']['reference_envelope_contains_initial_actual']
        assert math.isclose(axes['j1']['first_reference_minus_actual_rad'], -.03)
        summary = report['execution_summary']
        assert summary['actual_elapsed_s'] == 32.65740481 and summary['finished_before_hard_budget']
        assert summary['outcome'] == 'startup_identification_insufficient_before_purpose_validation'
        assert summary['all_requested_purposes_unstarted'] and report['archived_available'] is False
        assert compact_summary(report)['archived_validated'] is False
        locked = report['short_probe_records']['servo-probe-native-short-disturbance-0.csv']['axes']['j1']
        assert locked['motion_diagnostic'] == 'independent_input_without_observed_displacement'
        assert (locked['archived_disturbance_valid'] is False and locked['archived_disturbance_goal_met'] is True
                and locked['archived_static_measurement_complete'] is True
                and locked['archived_motion_observed'] is False and locked['archived_disturbance_measurement_complete'] is False)
        cycle = report['cycle_record']
        recorded = cycle['gain_stages']['native-frozen']['axes']['j1']
        assert cycle['segments'][0]['first_line'] == 2 and cycle['segments'][0]['last_line'] == 8
        assert math.isclose(cycle['maximum_cycle_gap_s'], .01)
        assert recorded['actual_gain_change_samples'] == 1
        assert recorded['fresh_receive_samples'] == 6 and recorded['repeated_receive_samples'] == 1
        assert math.isclose(recorded['reconstructed_pd_plus_command_torque_nm_peak_abs'], 1.9)
        assert recorded['position_reversals']['confirmed_reversals'] == 4
        assert math.isclose(recorded['position_reversals']['observed_positive_peak_frequency_hz'], 50)
        assert report['archived_available'] is False and report['archived_validated'] is False
        assert report['task_model_demand']['executed'] is False
        assert report['response_guard_events'][0]['mechanical_resonance_identified'] is False
        source.close()
        candidate['candidate_comparisons'] = []
        for number, kp, amplitude in ((1,100,.02),(2,140,.015)):
            name = f'servo-probe-native-establish-{number}.csv'
            with (folder/name).open('w') as stream:
                writer = csv.writer(stream)
                writer.writerow(['time_s','joint','reference','actual','reference_velocity','velocity','feedforward_nm',
                    'new_measurement','limited','feedback_time_s','feedback_sequence','actual_kp','actual_kd','actual_gains_known'])
                for sequence, (stamp, reference, actual) in enumerate(((0,0,0),(.7,0,0),(1,.01,.002),
                        (1.3,-.01,-.002),(2,0,0),(3,0,.01)),1):
                    writer.writerow([stamp,'j1',reference,actual,0,.1,.5,1,0,stamp,sequence,kp,2,1])
            candidate['experiments'].extend([{'kind':'short_experiment_parameters','record_path':name,'duration_s':3,
                'kp':[kp],'kd':[2],'plans':[{'amplitude_rad':amplitude,'frequencies_hz':[.4,.8,1.2]}]},
                {'kind':'reference_tracking','record_path':name,'axes':[]}])
            candidate['candidate_comparisons'].append({'kind':'motion_establishment','record_path':name,
                'axes':[{'positive_travel_rad':.002,'negative_travel_rad':.002,'required_travel_rad':.003,'qualified':False}],
                'next_kp':[140],'reasons':['insufficient_motion'],'excitation_retests':[1],'boundary_retests':[0]})
        (folder/'gain-candidate.json').write_text(json.dumps(candidate))
        source = Evidence(folder)
        report = audit(source,.1)
        series = report['establishment_progression']['axes']['j1']
        last = series['rounds'][-1]
        assert last['response_window']['fresh_samples'] == 4
        assert math.isclose(last['response_window']['response_rms_ratio'], .2)
        assert math.isclose(last['pd_command']['peak_abs'], 1.6) and math.isclose(last['total_command']['peak_abs'], 1.42)
        assert last['amplitude_reduced_while_kp_increased'] and series['trailing_same_frequency_rounds'] == 2
        assert series['last_three_rounds_actual_span_range_rad'] == [.012,.012]
        assert compact_summary(report)['establishment_progression']['j1']['actual_kp_steps'] == [100,140]
        source.close()
        cancelled = execution_summary({'outcome':'execution_fault'}, {'phase':'cancelled','fault_interrupted':False}, {}, {})
        assert cancelled['outcome'] == 'cancelled' and cancelled['termination_consistency_issues'] == ['cancelled_status_with_legacy_execution_fault_candidate']
        termination = {'kind':'cancelled','reason':'stop_requested','blocked_joint_names':[],'fault_interrupted':False}
        unified = execution_summary({'outcome':'cancelled','termination':termination}, {'phase':'cancelled','termination':dict(termination)}, {}, {})
        assert unified['outcome'] == 'cancelled' and unified['termination_objects_match'] and not unified['termination_consistency_issues']
        mismatch = execution_summary({'termination':termination}, {'termination':dict(termination,reason='different')}, {}, {})
        assert mismatch['termination_consistency_issues'] == ['status_candidate_termination_mismatch']
        unmeasured = execution_summary({'available':False,'validated':False,'termination':dict(termination,kind='qualified')}, {}, {}, {})
        assert unmeasured['outcome'] == 'archived_termination_requires_evidence' and 'qualified_termination_without_available_validated_evidence' in unmeasured['termination_consistency_issues']
        name = 'servo-probe-native-establish-2.csv'
        for item in candidate['experiments']:
            if item.get('record_path') != name:
                continue
            if item['kind'] == 'short_experiment_parameters':
                item['duration_s'] = 6.4
                item['plans'][0].update(motion_probe=True,duration_s=6.4)
            else:
                item.update(kind='motion_establishment_probe',axes=[{'tracking_valid':False,'identifiable':False,
                    'model_definition':'motion_establishment_only_no_frequency_response_fit',
                    'reason':'motion_establishment_only_no_spectral_fit'}])
        (folder/'gain-candidate.json').write_text(json.dumps(candidate))
        source = Evidence(folder)
        report = audit(source,.1)
        last = report['establishment_progression']['axes']['j1']['rounds'][-1]
        measured = report['short_probe_records'][name]['axes']['j1']
        assert last['motion_probe'] and last['frequencies_hz'] == [.4] and measured['archived_tracking_valid'] is False
        assert measured['response_window']['planned_window_complete'] is False
        assert last['plan_frequency_array'] == [.4,.8,1.2] and last['planned_measurement_window_cycles'] == [2.0]
        record = report['short_probe_records'][name]
        record['actual_trial_parameters']['plans'][0]['motion_probe'] = False
        mixed = establishment_progression(candidate,{name:record})['axes']['j1']['rounds'][-1]
        assert not mixed['motion_probe'] and mixed['frequencies_hz'] == [.4,.8,1.2]
        source.close()
        modern = {'validation_scope':'independent_demonstration_trajectory','joint_names':['j1']}
        measured = {'available':False,'validated':False,'kp':[10],'kd':[.2],'hold_gains':{'kp':[10],'kd':[.2]},
            'native_validation':{'available':True,'completed':True,'independent':True,'duration_s':1,'measured_duration_s':1,
                'metrics':[dict.fromkeys(('samples','measurement_samples','rms_error_rad','peak_error_rad','velocity_rms_rad_s','tail_error_rad','settling_s'),1)]},
            'hold_verification':{'available':True,'completed':True,'samples':100,'fresh_samples':[100],
                'worst_error_rad':.01,'worst_velocity_rad_s':0},
            'admittance_required':False,'admittance_validation':{'required':False,'status':'not_requested'}}
        assert nested_parameter_measurement_complete(modern,'rigid_tracking',measured) is True and measured['available'] is False and measured['validated'] is False
        assert nested_parameter_measurement_complete(modern,'servo_tracking',measured) is True
        incomplete = json.loads(json.dumps(measured));incomplete['native_validation']['metrics'][0]['measurement_samples'] = 0
        assert nested_parameter_measurement_complete(modern,'rigid_tracking',incomplete) is False
        incomplete = dict(measured,admittance_required=True)
        assert nested_parameter_measurement_complete(modern,'servo_tracking',incomplete) is False
        assert nested_parameter_measurement_complete({'joint_names':['j1']},'rigid_tracking',measured) is None
        assert compact_summary(report)['measurement_coverage']['rigid_tracking']['archived_validated'] is False
        name='servo-probe-native-establish-2.csv'
        parameter=next(item for item in candidate['experiments']
                       if item.get('kind')=='short_experiment_parameters' and item.get('record_path')==name)
        parameter.update(position_domain_source='checked_demonstration_and_fresh_feedback_intersect_explicit_safety',
            demonstrated_path_lower_rad=[0],demonstrated_path_upper_rad=[.02],disturbance_axis_eligible=[False],
            working_domain=[{'reference_lower_rad':0,'reference_upper_rad':.02}])
        parameter['plans'][0]['reference_bias_rad']=.01
        candidate['experiments'].append({'kind':'independent_disturbance_not_run','completed':False,
            'reason':'independent_disturbance_not_requested_without_bidirectional_demonstrated_room',
            'planned_kp':[140],'planned_kd':[2]})
        (folder/'gain-candidate.json').write_text(json.dumps(candidate))
        (folder/'trajectory.csv').write_text('sample_dt,.1\nindex,j1\n0,0\n1,.02\n')
        (folder/'source').mkdir()
        (folder/'source/model.urdf').write_text('<robot name="audit"><joint name="j1" type="revolute"><limit lower="-.1" upper=".1"/></joint></robot>')
        source=Evidence(folder)
        report=audit(source,.1)
        axis=report['short_probe_records'][name]['axes']['j1']
        assert axis['planned_reference_bias_rad']==.01 and axis['planned_reference_domain_rad']==[0,.02]
        assert axis['planned_disturbance_axis_eligible'] is False
        assert report['establishment_progression']['axes']['j1']['rounds'][-1]['reference_lower_rad']==0
        envelope=report['position_envelope_evidence']
        assert envelope['saved_reference']['axes']['j1']=={'min':0,'max':.02}
        assert envelope['short_probe_ranges'][name]['axes']['j1']['reference_margin_to_saved_path_rad']==[-.01,.01]
        assert envelope['short_probe_ranges'][name]['axes']['j1']['reference_margin_to_urdf_rad']==[.09000000000000001,.09000000000000001]
        unrun=compact_summary(report)['unrun_disturbance_experiments']
        assert len(unrun)==1 and unrun[0]['completed'] is False and 'record_path' not in unrun[0]
        assert report['cycle_record']['phases']['gain_adaptation']['position_ranges']['j1']['q']=={'min':0.0,'max':.007}
        source.close()
        entry_name='gain-transition-native-measurement-entry.csv'
        candidate['experiments'].append({'kind':'demonstrated_measurement_center_entry','record_path':entry_name,
            'completed':False,'feedback_lead_rad':[.01],'position_uncertainty_rad':[1e-6]})
        (folder/'gain-candidate.json').write_text(json.dumps(candidate))
        with (folder/entry_name).open('w') as stream:
            writer=csv.writer(stream,lineterminator='\n')
            writer.writerow(['time_s','joint','reference','reference_velocity','actual','velocity','new_measurement',
                             'limited','feedforward_nm','actual_kp','actual_kd','actual_gains_known'])
            for time,reference,actual in ((.01,.02201529638671875,.01201629638671875),
                                         (.02,.02201529638671875,.01239776611328125),
                                         (.03,.02201757202971244,.01201629638671875)):
                writer.writerow([time,'j1',reference,0,actual,0,1,0,6.897,62.14896214896215,2.78998778998779,1])
        source=Evidence(folder)
        report=audit(source,.1)
        entry=report['measurement_center_records'][entry_name]
        axis=entry['axes']['j1'];tail=axis['last_fresh_transition']
        assert entry['evidence_scope']=='positioning_only_no_identification_or_purpose_qualification'
        assert axis['minimum_observed_position_step_rad']==.0003814697265625
        assert axis['first_nonzero_fresh_position_step']['after']['time_s']==.02
        assert tail['actual_delta_rad']==-.0003814697265625 and tail['reference_delta_rad']>0
        assert axis['recorded_hard_lead_exceeded_rows']==1 and axis['first_recorded_hard_lead_exceeded']['line']==4
        assert compact_summary(report)['measurement_center_records'][entry_name]['axes']['j1']['recorded_position_uncertainty_rad']==1e-6
        source.close()
        endpoint_name='gain-transition-synthetic-frozen.csv'
        candidate['experiments'].append({'kind':'feedback_bounded_endpoint_transition','record_path':endpoint_name,
            'completed':False,'initial_reference_rad':[0],'target_rad':[.005],'reference_error_cap_rad':[.01]})
        (folder/'gain-candidate.json').write_text(json.dumps(candidate))
        with (folder/endpoint_name).open('w') as stream:
            writer=csv.writer(stream,lineterminator='\n')
            writer.writerow(['time_s','joint','reference','reference_velocity','actual','velocity','new_measurement',
                             'limited','feedforward_nm','actual_kp','actual_kd','actual_gains_known'])
            for time,reference,velocity in ((0,0,0),(.4,.004,0.001),(.8,.00501,.0002)):
                writer.writerow([time,'j1',reference,velocity,0,0,1,0,0,10,1,1])
        (folder/'gain-trajectory-synthetic-frozen.csv').write_text('time_s,joint,reference,actual,new_measurement,limited\n')
        source=Evidence(folder);report=audit(source,.1)
        endpoint=report['endpoint_transition_records'][endpoint_name];axis=endpoint['axes']['j1']
        assert endpoint['evidence_scope']=='endpoint_positioning_not_full_path_tracking_or_purpose_qualification'
        assert math.isclose(axis['last_reference_minus_target_rad'],1e-5,abs_tol=1e-12)
        assert axis['last_actual_minus_target_rad']==-.005
        assert axis['last_reference_velocity_rad_s']==.0002 and 'first_exact_stopped_reference_time_s' not in axis
        assert report['trajectory_record_counts']['header_only_records']==['gain-trajectory-synthetic-frozen.csv']
        assert endpoint_name in compact_summary(report)['endpoint_transition_records']
        source.close()
        assert report['continuation_review']['archived_review_required'] is None
        candidate.update(review_required=True,nonfatal_stage_warnings=1,
                         execution_policy='continue_available_measurements_until_stop_fault_or_original_budget')
        warning={'kind':'measurement_warning','phase':'native-training','reason':'synthetic_insufficient_response',
                 'continued':True,'completed':False,'validated':False,'exit_reference':'fresh_measured_position_hold',
                 'kp':[10],'kd':[1],'blocked_joint_names':['j1']}
        candidate['experiments'].append(warning)
        (folder/'gain-candidate.json').write_text(json.dumps(candidate))
        source=Evidence(folder);report=audit(source,.1);review=report['continuation_review']
        assert review['archived_review_required'] is True
        assert review['archived_nonfatal_stage_warnings']==1
        assert review['archived_execution_policy']==candidate['execution_policy']
        assert review['recorded_measurement_warnings']==[warning]
        assert compact_summary(report)['continuation_review']==review
        source.close()
    return {'self_test': 'passed', 'checks': 87}


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', nargs='?', type=Path, help='Task directory or gain artifact ZIP')
    parser.add_argument('--compare', type=Path, help='Compare shared evidence bytes with another directory or ZIP')
    parser.add_argument('--peak-target-deg', type=float, default=0.1)
    parser.add_argument('--summary', action='store_true', help='Print startup/exit, timing and short-response diagnostics without raw experiment details')
    parser.add_argument('--self-test', action='store_true')
    parser.add_argument('--check-establishment-regression', action='store_true',
                        help='Check a known original establishment, cancellation or rejected HOLD-entry task, read-only')
    args = parser.parse_args()
    if args.self_test:
        print(json.dumps(self_test(), indent=2))
        return 0
    if not args.source:
        parser.error('source is required unless --self-test is used')
    if not math.isfinite(args.peak_target_deg) or args.peak_target_deg <= 0:
        parser.error('--peak-target-deg must be finite and positive')
    source = companion = None
    try:
        source = Evidence(args.source)
        report = audit(source, args.peak_target_deg)
        if args.check_establishment_regression:
            report['establishment_regression'] = check_establishment_regression(report)
        if args.compare:
            companion = Evidence(args.compare)
            report['artifact_comparison'] = compare(source, companion)
        print(json.dumps(compact_summary(report) if args.summary else report, ensure_ascii=False, indent=2, allow_nan=False))
        return 0
    except (ValueError, OSError, KeyError, zipfile.BadZipFile) as error:
        print(f'Gain evidence audit failed: {error}', file=sys.stderr)
        return 1
    finally:
        if source:
            source.close()
        if companion:
            companion.close()


if __name__ == '__main__':
    raise SystemExit(main())
