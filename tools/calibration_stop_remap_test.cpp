// Ordered gravity stops must stay on their demonstrated visit through retiming.
// Pure header-only checks: no controller, hardware plugin or Profile is loaded.
#include "serial_arm/dynamics/calibration_retime.hpp"
#include <iostream>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace retime = serial_arm::calibration_retime;
using Path = std::vector<std::vector<double>>;

void check(bool value, const char* message) {
    if(!value) throw std::runtime_error(message);
}
template<class Action> void rejected(Action action, const char* message) {
    bool failed = false;
    try { action(); } catch(const std::exception&) { failed = true; }
    check(failed, message);
}

void first_visit_mapping() {
    for(const std::size_t joints : {1U, 3U, 7U}) {
        const std::vector<double> scalar{-1, .1 + 1e-14, .12, .2 + 1e-14, .3, .2, .1, -1};
        Path path;
        const auto pose = [joints](double value) {
            std::vector<double> q(joints);
            for(std::size_t j = 0; j < joints; ++j) q[j] = .2 * j + (j % 2 ? -1.0 : 1.0) * value;
            return q;
        };
        for(double value : scalar) path.push_back(pose(value));
        const Path stops{pose(.1), pose(.2), pose(.3), pose(.2), pose(.1)};
        const auto mapped = retime::ordered_stop_indices(path, stops, .025);
        check(mapped == std::vector<std::size_t>{1, 3, 4, 5, 6},
            "a slightly closer return visit must not steal the outbound ordered stop");
        check(retime::ordered_stop_indices(path, stops, .025) == mapped,
            "repeated import must preserve the same visit and target ordering");
        Path reversed(path.rbegin(), path.rend());
        const auto back = retime::ordered_stop_indices(reversed, stops, .025);
        check(back == std::vector<std::size_t>{1, 2, 3, 4, 6},
            "a reversed recorded path must preserve stops in its own supplied direction");
    }
    const Path path{{-.2}, {.06}, {.09}, {.1 + 1e-15}, {.1 - 1e-15}, {.1}, {.14}, {.1}, {0}};
    check(retime::ordered_stop_indices(path, {{.1}}, .025) == std::vector<std::size_t>{5},
        "mapping must finish the first local approach rather than stop at tolerance entry or a numeric plateau wiggle");
    check(retime::ordered_stop_indices({{0}, {.1}, {.2}}, {{0}, {.2}}, .025, false) ==
        std::vector<std::size_t>{0, 2}, "the planner may retain its original endpoint stop conditions");
}

void shared_retimed_node() {
    const Path sparse{{0}, {.1}, {.2}, {.3}, {.4}, {.3}, {.2}, {.1}, {0}};
    const Path stops{{.098}, {.1}, {.102}, {.2}, {.3}, {.4}, {.3}, {.2}, {.1}};
    check(retime::ordered_stop_indices(sparse, stops, .025) ==
        std::vector<std::size_t>{1, 1, 1, 2, 3, 4, 5, 6, 7},
        "nearby legal source stops may share one retimed knot without consuming a later return visit");
    check(retime::ordered_stop_indices({{0}, {.1}, {.2}}, {{.09}, {.1}, {.11}}, .025) ==
        std::vector<std::size_t>{1, 1, 1},
        "multiple ordered stops near the last interior knot must not become an artificial import failure");
    rejected([] { retime::ordered_stop_indices({{0}, {.1}, {.2}}, {{.1}, {.126}}, .025); },
        "shared knots must still satisfy the unchanged original tolerance for every target");
}

void original_tolerance_and_order() {
    rejected([] { retime::ordered_stop_indices({{0}, {.025000000001}, {.1}}, {{0}}, .025); },
        "the original 0.025rad stop tolerance must not gain a numerical acceptance margin");
    rejected([] { retime::ordered_stop_indices({{0,0}, {.01,.026}, {.1,.1}}, {{0,0}}, .025); },
        "every joint must remain inside the original maximum absolute error tolerance");
    rejected([] { retime::ordered_stop_indices({{0}, {.1}, {.2}, {.3}}, {{.2}, {.1}}, .025); },
        "unavailable ordered stops must fail rather than reorder targets or search backwards");
    rejected([] { retime::ordered_stop_indices({{0}, {.1}, {.2}}, {{0}}, .025); },
        "the terminal mapping must still exclude path endpoints");
    rejected([] { retime::ordered_stop_indices({{0}, {.1,.2}, {.2}}, {{.1}}, .025); },
        "stop mapping must reject dimension mismatches before indexing");
    rejected([] { retime::ordered_stop_indices({{0}, {std::numeric_limits<double>::quiet_NaN()}, {.2}}, {{.1}}, .025); },
        "non-finite path values must not look like a zero-distance match");
}

void repeated_retiming() {
    Path input;
    for(unsigned k = 0; k <= 200; ++k) {
        const double q = k <= 100 ? k * .01 : (200 - k) * .01;
        input.push_back({q, .02});
    }
    const Path stops{{.2,.02}, {.6,.02}, {.9,.02}};
    for(unsigned pass = 0; pass < 2; ++pass) {
        const auto planned = retime::plan(input, {.5,.5}, {1,1}, 200, .003, stops);
        const auto mapped = retime::ordered_stop_indices(planned.positions, stops, .025);
        const auto turn = static_cast<std::size_t>(std::max_element(planned.positions.begin(), planned.positions.end(),
            [](const auto& a, const auto& b) { return a[0] < b[0]; }) - planned.positions.begin());
        check(mapped.size() == stops.size() && mapped.front() < mapped[1] && mapped[1] < mapped.back() && mapped.back() < turn,
            "actual retiming and reimport must retain all outbound gravity stops before the return turn");
        for(std::size_t i = 0; i < mapped.size(); ++i)
            for(std::size_t j = 0; j < stops[i].size(); ++j)
                check(std::abs(planned.positions[mapped[i]][j] - stops[i][j]) <= .025,
                    "retiming must preserve the original per-axis stop tolerance");
        input = planned.positions;
    }
}

void verify_retimed_limits(const retime::Plan& planned,const std::vector<double>& max_velocity,
    const std::vector<double>& max_acceleration) {
    check(std::isfinite(planned.single_pass_s) && planned.single_pass_s>0,
        "adjacent legal static stops must have a finite positive retimed duration");
    for(std::size_t k=0;k+1<planned.positions.size();++k)for(unsigned quarter=0;quarter<4;++quarter) {
        const double progress=k+quarter*.25;
        const auto sample=retime::interpolate(planned.positions,progress,planned.sample_dt,1);
        for(std::size_t axis=0;axis<max_velocity.size();++axis)
            check(std::abs(sample.velocity[axis])<=max_velocity[axis]*(1+1e-6),
                "rest-to-rest intervals must retain the original actual Hermite velocity limits");
        if(progress>.0001 && progress<planned.positions.size()-1-.0001) {
            const auto before=retime::interpolate(planned.positions,progress-.0001,planned.sample_dt,1);
            const auto after=retime::interpolate(planned.positions,progress+.0001,planned.sample_dt,1);
            for(std::size_t axis=0;axis<max_velocity.size();++axis)
                check(std::abs(after.velocity[axis]-before.velocity[axis])/(.0002*planned.sample_dt)<=max_acceleration[axis]*(1+1e-6),
                    "adjacent stops must retain actual replay acceleration instead of bypassing the final derivative check");
        }
    }
}

void adjacent_static_stops() {
    Path source;
    for(unsigned k=0;k<=200;++k)source.push_back({k*.001,.02});
    // Dense original stops map to shared and adjacent geometry knots. Both
    // speed-limited trapezoids and acceleration-limited triangles are legal.
    for(double velocity:{.002,.3}) {
        const std::vector<double> max_v{velocity,.3},max_a{.2,.2};
        const auto planned=retime::plan(source,max_v,max_a,200,.003,source);
        verify_retimed_limits(planned,max_v,max_a);
        const auto indices=retime::ordered_stop_indices(planned.positions,source,.025);
        check(indices.size()==source.size() && std::is_sorted(indices.begin(),indices.end()),
            "all nearby demonstrated targets must survive shared/adjacent retimed stop nodes in original order");
        for(std::size_t i=0;i<source.size();++i)
            check(std::abs(planned.positions[indices[i]][0]-source[i][0])<=.025 && planned.positions[indices[i]][1]==.02,
                "dense stop preservation must not expand position tolerance or alter a fixed joint");
    }
}

void recorded_import_fixture(const char* filename) {
    std::ifstream stream(filename);check(static_cast<bool>(stream),"recorded import fixture must be readable");
    std::string line;std::getline(stream,line);std::getline(stream,line);
    Path source;
    while(std::getline(stream,line)) {
        if(line.empty())continue;
        std::istringstream row(line);std::string field;std::getline(row,field,',');
        std::vector<double> q;while(std::getline(row,field,','))q.push_back(std::stod(field));
        source.push_back(std::move(q));
    }
    check(source.size()>=20,"the unchanged recorded import source must contain its full trajectory");
    Path stops;
    for(std::size_t index=0;index<source.size();index+=4)stops.push_back(source[index]);
    if(stops.back()!=source.back())stops.push_back(source.back());
    const std::vector<double> max_v(source.front().size(),5.4),max_a(source.front().size(),10.8);
    const auto planned=retime::plan(source,max_v,max_a,200,.003,stops);
    const auto indices=retime::ordered_stop_indices(planned.positions,stops,.025);
    check(indices.size()==stops.size() && std::is_sorted(indices.begin(),indices.end()),
        "unchanged real import curves must preserve ordered stops through their actual retiming");
    for(std::size_t i=0;i<stops.size();++i)for(std::size_t axis=0;axis<max_v.size();++axis)
        check(std::abs(stops[i][axis]-planned.positions[indices[i]][axis])<=.025,
            "recorded import fixtures must keep the original per-axis stop tolerance");
    verify_retimed_limits(planned,max_v,max_a);
    std::cout<<"recorded import fixture: "<<source.size()<<" original points, "<<stops.size()
        <<" ordered stops, "<<planned.positions.size()<<" retimed points, "<<planned.single_pass_s<<" seconds\n";
}

int main(int argc,char** argv) {
    try {
        const std::vector<std::vector<double>> paused_path{{0.0},{.1},{.2}};
        const auto paused=serial_arm::calibration_retime::interpolate(paused_path,.6,.02,0);
        const auto running=serial_arm::calibration_retime::interpolate(paused_path,.6,.02,1);
        if(paused.position!=running.position || paused.velocity!=std::vector<double>{0.0})
            throw std::runtime_error("zero-rate interior identification dwell must preserve position and have zero velocity");
        first_visit_mapping();
        shared_retimed_node();
        original_tolerance_and_order();
        repeated_retiming();
        adjacent_static_stops();
        for(int index=1;index<argc;++index)recorded_import_fixture(argv[index]);
        std::cout << "calibration stop mapping checks passed: first local visit, outbound/return order, 1/3/7 axes, unchanged tolerance and repeated retiming\n";
    } catch(const std::exception& error) {
        std::cerr << error.what() << '\n';
        return 1;
    }
}
