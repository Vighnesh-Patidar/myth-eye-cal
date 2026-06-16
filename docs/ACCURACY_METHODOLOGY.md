# Accuracy Measurement Methodology — 4-device fused pose

Procedure to quantify fused 3D pose accuracy against ground truth as a function
of observer count (1–4) and geometry, including the through-wall (no-LOS) case.
Satisfies the §12 v1.0 item *"accuracy benchmark vs ground truth at 2/3/4
observers."*

## 1. Equipment
- 4 Android phones running the same APK, all on one Wi-Fi (UDP broadcast allowed).
- One subject; a logging laptop on the same Wi-Fi (runs the viewer / a WS recorder).
- Ground-truth source (see §3) and a tape measure / laser; optionally a surveyed
  GPS point or printed fiducials.

## 2. Shared frame & co-localization (§15.12)
1. Choose an **absolute origin**: a surveyed `lat/lon/alt`, *or* a local
   tape-measured origin marked on the floor as `(0,0,0)` with **+X = east,
   +Y = north, +Z = up** (matching the rotation-vector ENU orientation frame).
2. Enter the **same origin** on every phone (*Set origin*). Place each phone at a
   measured spot and either *GPS→pos* (outdoor) or type the measured `(x,y,z)`
   (indoor). Record every phone's position.
3. Orientation is automatic and shared (rotation-vector, gravity + north).

## 3. Ground truth (pick by environment, most rigorous first)
- **(A) Surveyed static poses** — subject holds scripted poses with specific
  joints at tape-measured world coordinates (e.g. wrists on marked stands). Best
  indoors, no extra hardware.
- **(B) Reference sensor** — depth camera / mocap (Azure Kinect, OptiTrack)
  co-registered to the same origin → per-frame GT skeleton. Highest fidelity.
- **(C) Known rigid object** of known geometry at a known pose.

Record GT keypoints in the **same world frame**. Because the origin is an
absolute known coordinate, GT taken in absolute coordinates needs only a fixed
offset correction — no per-point cross-verification.

## 4. Metrics
- **MPJPE** (Mean Per-Joint Position Error) — mean Euclidean error per keypoint,
  in mm. Primary metric.
- **PA-MPJPE** — Procrustes-aligned MPJPE; removes residual rigid frame
  misalignment, isolating *pose-shape* error from *co-localization* error. The
  gap (MPJPE − PA-MPJPE) quantifies co-localization error.
- **Per-axis error** (lateral vs along-ray) — exposes triangulation quality.
- **Through-wall recovery** — same metrics on a phone with its camera covered
  (0 LOS) fusing the others.
- **Jitter** — per-keypoint std over a static hold (temporal stability).
- Report per-keypoint and overall mean / median / 95th percentile.

## 5. Test matrix
| Variable | Values |
|---|---|
| Observers N | 1, 2, 3, 4 (drop subsets from one 4-phone capture) |
| Geometry spread | ~0°, 45°, 90° between observers |
| Subject distance | 2 m, 3 m, 4 m |
| Occlusion | full LOS; 1 phone no-LOS (through-wall) |
| Pose | T-pose, arms-forward, crouch, slow walk |

## 6. Procedure
1. Co-localize all 4 phones (§2); record positions + origin.
2. Launch on all; confirm each is broadcasting (status overlay) and the viewer
   shows `observers: N`.
3. Subject performs each scripted pose, holding ~5 s.
4. Record the fused pose stream on the logger and capture GT, timestamped.
5. Repeat across the test matrix.

## 7. Data capture
- **Fused poses**: connect a recorder to `ws://<phone-ip>:8080/pose`, log each
  `pose_frame` JSON with arrival time. *(A small recorder script is a to-do.)*
- **Per-N evaluation from one capture**: log the raw per-observer UDP beacons and
  re-run fusion offline to derive N = 1…4 without re-staging.
  *(Beacon logging hook is a to-do.)*
- **GT**: from the chosen method, timestamped to align with fused frames.

## 8. Analysis & correction
1. Temporally align fused vs GT (nearest timestamp within the fusion window).
2. Compute MPJPE and PA-MPJPE per configuration.
3. The Procrustes transform from PA-MPJPE *is* the residual co-localization
   correction (rotation + translation). With an absolute known origin it should
   be small — apply it as one global correction, not per-point.
4. Plot error vs N (expect the ~`1/√N` trend, §5.2) and vs geometry spread.

## 9. Acceptance targets (set after a baseline run; feed back into §12)
- MPJPE < _TBD_ cm at 4 observers, full LOS.
- Through-wall MPJPE < _TBD_ cm with ≥ 2 LOS observers.
- Error monotonically decreasing with N.

## 10. Known confounds
- **Co-localization error** (position pin / GPS coarseness, camera→body extrinsic)
  dominates *absolute* MPJPE → always report PA-MPJPE too.
- MediaPipe landmark error; grayscale-bitmap shortcut; degenerate single-phone
  depth (§15.11) — mitigated by multi-view triangulation (§15.3).
- Cross-device clock offset (handled by the receiver-stamped fusion window).
- Wi-Fi broadcast loss / client isolation.

## Outstanding tooling (to build before a real run)
- [ ] WS `pose_frame` recorder (JSON + timestamp) on the logger.
- [ ] Raw UDP-beacon logger + offline re-fusion for per-N subsets.
- [ ] Ground-truth capture/co-registration for the chosen method (§3).
