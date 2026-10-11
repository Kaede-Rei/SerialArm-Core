// Pure candidate-COM geometry for the read-only calibration overlay
// Coordinates are in each URDF Link frame and must never be written back to the robot
export function gravityComMarkers(links, firstMoments, displayScale = 1, showUnchanged = false, sourceInertials = null, candidateMasses = null) {
  const index = new Map((links || []).map(link => [link.name, link]));
  const scale = Number(displayScale);
  if (![1, 12, 25].includes(scale)) throw new Error('invalid COM visualization scale');
  const rows = [];
  for (const item of firstMoments || []) {
    const link = index.get(item?.link_name);
    const mass = Number(link?.inertial?.mass);
    // Invalid tensor is a physics warning, not a reason to hide its existing COM
    if (!link?.inertial || !Number.isFinite(mass) || mass <= 0) continue;
    const sourceMass = sourceInertials === null ? mass : Number(sourceInertials?.[item?.link_name]?.mass);
    if (!Number.isFinite(sourceMass) || sourceMass <= 0) continue;
    // Historical comparisons must use the saved URDF's COM, not the currently
    // loaded model's COM; the geometry can match while inertials differ
    const original = sourceInertials?.[item?.link_name]?.com ?? link?.inertial?.origin?.xyz;
    const values = item?.value;
    if (!Array.isArray(original) || original.length !== 3 || !Array.isArray(values) || values.length !== 3) continue;
    const originalCom = original.map(Number);
    const firstMoment = values.map(Number);
    if (![...originalCom, ...firstMoment].every(Number.isFinite)) continue;
    const candidateMass = candidateMasses === null ? sourceMass : Number(candidateMasses[item.link_name]);
    if (!Number.isFinite(candidateMass) || candidateMass <= 0) continue;
    const candidateCom = firstMoment.map(v => v / candidateMass);
    const shift = candidateCom.map((v, i) => v - originalCom[i]);
    const offsetMm = 1000 * Math.hypot(...shift);
    if (!showUnchanged && offsetMm < 0.001) continue;
    rows.push({
      linkName: item.link_name,
      originalCom,
      candidateCom,
      displayCom: originalCom.map((v, i) => v + shift[i] * scale),
      offsetMm,
      displayScale: scale,
    });
  }
  return rows;
}
