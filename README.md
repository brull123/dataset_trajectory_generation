# Two-UAV trajectory generator

Generates two synchronized, parallel line trajectories fully inside the safety
polygon and vertical limits from an MRS world configuration. Output trajectory
rows use the MRS trajectory loader format:

```text
x,y,z,heading
```

The actual files have no header because the loader expects numeric rows only.
The generator understands both the older top-level `safety_area` structure and
the newer nested `mrs_uav_managers.safety_area_manager.safety_area` structure.
For a `latlon_origin` polygon, coordinates are converted to local east/north
metres in `world_origin`.

## Install and run

```bash
python3 -m pip install -r requirements.txt
python3 generate_trajectories.py /path/to/world_config.yaml
```

By default this creates:

- `generated/uav1.txt`
- `generated/uav2.txt`
- `generated/loader_config.yaml`

Add `--plot` to also create `generated/trajectories.png`:

```bash
python3 generate_trajectories.py /path/to/world_config.yaml --plot
```

To place a completed trajectory near a particular safety-area border, add a
world-frame direction and a standoff. For example, this places it toward the
south border while retaining the normal 0.5 m margin plus 3 m of extra offset:

```bash
python3 generate_trajectories.py /path/to/world_config.yaml \
  --placement-direction south --boundary-offset 3
```

The trajectory map is placed in its own figure: it contains the horizontal
safety polygon, both paths, and heading arrows. A separate diagnostics figure
contains altitude profiles and vertical safety limits, synchronized inter-UAV
distance with the minimum highlighted, signed relative X/Y/Z velocity and its
3D magnitude, 3D velocity and acceleration magnitudes, and unwrapped heading profiles for
both UAVs. Acceleration uses the same finite
differences as the dynamic constraint verifier. `--plot overview.png` saves the
map as `overview.png` and diagnostics as `overview_diagnostics.png`; use
`--show-plot` to open both figures interactively.

Add `--plot-rrt` to a plotted random-walk trajectory to overlay the RRT
planning samples, highlighted spline-control points, and the planning camera's
horizontal FOV plus its minimum and maximum target-distance bounds.

When a constraints profile is active (`--constraints` or `--platform`), the
velocity diagnostics also show the applicable combined horizontal/vertical
speed envelope. When a path exceeds a dynamic limit, the generator preserves
its spatial path and extends its duration until every enabled speed,
acceleration, jerk, snap, and heading constraint is satisfied. When there is
headroom, it also resamples that same spatial path to the shortest duration
that still passes every active constraint.

## Command-line parameter reference

The command has the form:

```bash
python3 generate_trajectories.py CONFIG [OPTIONS]
```

Distances are in metres, angles are in degrees unless stated otherwise, and
times are in seconds.

| Parameter | Default | Applies to | Description |
| --- | --- | --- | --- |
| `CONFIG` | required | all patterns | Path to an MRS world or safety-area YAML file. Both the older top-level and newer `mrs_uav_managers` layouts are supported. |
| `-h`, `--help` | — | command line | Print the generated command-line help and exit. |
| `--platform PATH` | unset | all patterns | Load an MRS platform overlay such as `platforms/x500.yaml`, validate that the selected constraints profile is permitted, and enable checking with the bundled upstream MRS profiles when `--constraints` is omitted. |
| `--output-dir PATH` | `generated` | all patterns | Directory for `uav1.txt`, `uav2.txt`, `loader_config.yaml`, and a relative plot path. |
| `--dt SECONDS` | `0.2` | all patterns | Time between trajectory samples. It must be at least 0.01 s for the MRS MPC tracker. |
| `--duration SECONDS` | `500` | all patterns | Initial trajectory duration. The number of samples is `floor(duration / dt) + 1`. When constraint checking fails, duration is extended automatically without changing the spatial path. |
| `--pattern NAME` | `parallel` | all patterns | Select `parallel`, `straight-helix`, `dataset-weave`, `dataset-orbit`, `dataset-lissajous`, `dataset-random-walk-static`, or `dataset-random-walk-moving`. |
| `--separation METRES` | `2.0` | `parallel` | Perpendicular distance between the two parallel UAV paths. |
| `--lead-distance METRES` | `2.0` | `straight-helix` | Distance by which UAV 2 leads the straight-flying UAV 1 along the forward axis. |
| `--helix-radius METRES` | `0.75` | `straight-helix` | Radius of UAV 2's helix around the forward axis. |
| `--helix-turns COUNT` | `2.0` | `straight-helix` | Number of complete helix revolutions made by UAV 2. Fractional values are allowed. |
| `--minimum-distance METRES` | `5.0` | dataset patterns | Required minimum 3D centre-to-centre UAV distance. The generated pattern reaches this value exactly. |
| `--maximum-distance METRES` | three times the minimum | dataset patterns | Desired maximum 3D UAV distance. When omitted it can be reduced to fit; an explicitly supplied value is a hard requirement. |
| `--travel-distance METRES` | `40` | dataset patterns | Desired UAV 1 path length. For a straight path this is its end-to-end travel; for a circle this is its circumference. It is shortened when necessary to fit. Mutually exclusive with `--circle-radius`. |
| `--circle-radius METRES` | unset | circular dataset path | Exact radius of UAV 1's circle. Requires `--observer-path circle`, is mutually exclusive with `--travel-distance`, and causes an error if it cannot fit. |
| `--observer-path NAME` | `straight` | dataset patterns | Select `straight` or `circle` for the recording UAV. On a circle, its front camera points tangentially along the path. |
| `--camera-horizontal-fov DEGREES` | `90` | dataset patterns | Full horizontal field of view of UAV 1's front camera. Target motion uses at most 80% of it. Valid range: greater than 0 and less than 180. |
| `--camera-vertical-fov DEGREES` | `60` | dataset patterns | Full vertical field of view of UAV 1's front camera. Target motion uses at most 80% of it and is also limited by altitude clearance. Valid range: greater than 0 and less than 180. |
| `--relative-heading-turns COUNT` | `1.5` | dataset patterns | Number of rotations made by UAV 2's heading relative to UAV 1 during the trajectory. |
| `--random-seed INTEGER` | `1` | random-walk dataset patterns | Reproducible random seed. Change it to create different smooth paths. |
| `--random-waypoints COUNT` | `20` | random-walk dataset patterns | Number of spatially separated controls used by the cubic B-spline. Must be at least four; larger values increase path variety and typically require more duration. |
| `--static-camera-placement NAME` | `circle` | `dataset-random-walk-static` | Select `circle` for the maximum inscribed-circle placement or `edge` for the previous nearest-boundary placement. |
| `--static-camera-circle-clearance METRES` | `7.5` | circle static camera | Required clearance from the safety boundary when sizing the maximum inscribed camera circle. Must be at least `--horizontal-margin`. |
| `--static-camera-circle-point NAME` | `north` | circle static camera | Camera point on the circle: `north`, `northeast`, `east`, `southeast`, `south`, `southwest`, `west`, or `northwest`. |
| `--static-camera-inset METRES` | `5` | edge static camera | Distance from the nearest boundary into the safety area for `--static-camera-placement edge`. Must fit inside the area and be at least `--horizontal-margin`. |
| `--camera-heading DEGREES` | automatic | `dataset-random-walk-static` | World-frame look direction override. If omitted, the camera points to the inscribed-circle center or, for edge placement, toward the field interior. The generator rejects headings whose FOV cannot contain the requested target path. |
| `--moving-camera-radius METRES` | `1.0` | `dataset-random-walk-moving` | Maximum horizontal displacement of the filming UAV's smooth random walk. It is intentionally small relative to target motion. |
| `--moving-camera-heading-walk DEGREES` | `12` | `dataset-random-walk-moving` | Maximum deviation from the filming UAV's initial look direction during its smooth bounded heading random walk. |
| `--margin METRES` | `0.5` | all patterns | Backward-compatible shared safety clearance. It supplies both directional margins unless they are set separately. |
| `--horizontal-margin METRES` | `--margin` | all patterns | Minimum clearance from the horizontal safety polygon. This is also the clearance used for boundary placement. |
| `--vertical-margin METRES` | `--margin` | all patterns | Minimum clearance from `min_z` and `max_z`. |
| `--placement-direction NAME` | `center` | all patterns | After generation, center the combined trajectory mean in the largest inscribed safety-area circle. Select `north`, `northeast`, `east`, `southeast`, `south`, `southwest`, `west`, or `northwest` to then translate both trajectories toward that border. Directions use the world frame: east is +x and north is +y. |
| `--boundary-offset METRES` | `0` | with placement direction | Additional standoff from the selected boundary, beyond `--horizontal-margin`. The trajectories are shifted together, so their timing and relative geometry are unchanged. |
| `--constraints PATH` | unset | all patterns | MRS ConstraintManager YAML file used to check both generated trajectories before files are written. With `--platform`, the bundled `constraints/mrs_default.yaml` is used when this option is omitted. |
| `--constraint-profile NAME` | `fast` (`medium` for moving random walk) | constraint checking | Named constraints profile to read from `--constraints`. Has no effect unless a constraints file or platform is supplied. |
| `--plot [PATH]` | disabled | all patterns | Save a visualization. With no path, writes `trajectories.png` under `--output-dir`; a relative supplied path is also resolved under that directory. |
| `--plot-rrt` | disabled | random-walk plots | Overlay RRT samples, selected spline controls, planning FOV, and target-distance bounds. Requires `--plot` or `--show-plot`. |
| `--show-plot` | disabled | all patterns | Open the visualization interactively in addition to any file requested with `--plot`. |

Options for another pattern may be present on the command line but are ignored.
For example, `--helix-radius` has no effect on a dataset pattern.

## Camera-dataset trajectories

The dataset patterns assign UAV 1 as the recording UAV with a front-facing
camera. It normally flies a straight or circular path; the random-walk variant
instead holds it stationary at the field edge. UAV 2 remains in front of the
camera and moves throughout the trajectory. Its distance, bearing, elevation,
altitude, and heading vary to produce changes in apparent size, image position,
viewpoint, and background.

Four patterns are available:

- `dataset-weave`: smooth side-to-side and vertical passes.
- `dataset-orbit`: an ellipse in the camera image plane.
- `dataset-lissajous`: denser, non-repeating-looking image-plane coverage.
- `dataset-random-walk-static`: a stationary edge camera observing a smooth,
  randomized, non-looping target path.
- `dataset-random-walk-moving`: a gently moving camera observing the same kind
  of varied target path; it uses the `medium` constraint profile by default.

For example:

```bash
python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml \
  --pattern dataset-lissajous \
  --minimum-distance 5.0 --maximum-distance 15.0 \
  --travel-distance 40 \
  --camera-horizontal-fov 90 --camera-vertical-fov 60 \
  --relative-heading-turns 1.5 --duration 30 --plot
```

`--minimum-distance` is the closest permitted 3D centre-to-centre distance.
After generation, UAV 2 is shifted toward UAV 1 by a uniform camera-frame
scale so the closest sampled distance reaches this value exactly. The shifted
path is then checked again for safety-area clearance and collision-free
separation. The dataset defaults are sized from
`worlds/world_temesvar_field_1.yaml`, whose converted safety area is about
137 x 161 m horizontally and spans 1--15 m altitude. They use a 5 m minimum
distance, aim for a 15 m maximum distance, and move the recording UAV 40 m.
The default duration is 500 s, giving the target enough time to cover the
larger field-scale path with diverse maneuvers within the default fast limits.
Use `--minimum-distance`, `--maximum-distance`, and `--travel-distance` to
override them. When the far distance or travel does not fit, it is reduced
automatically so the same command remains usable for Temesvar fields 2 and 3
and for smaller world configs. An explicitly supplied maximum distance is
treated as a hard requirement and produces an error if it cannot fit.

Camera bearing and elevation use at most 80% of the supplied field of view.
Generated paths are fitted or validated inside the safety polygon and altitude
limits with `--margin` clearance. A clear error is returned when the requested
formation cannot fit.

The corresponding reusable functions are `generate_dataset_weave()`,
`generate_dataset_orbit()`, `generate_dataset_lissajous()`,
`generate_static_camera_random_walk()`, `generate_moving_camera_random_walk()`,
and the configurable
`generate_dataset_trajectories()`.

### Static edge camera with random walk

The `dataset-random-walk-static` pattern first finds the largest circle centered
at the safety area's safest point. By default it reduces that radius by 7.5 m,
places UAV 1 at the circle's north point, and points its front camera at the
circle center. Select any of the eight compass points with
`--static-camera-circle-point`; use `--static-camera-circle-clearance` to
change the boundary clearance. UAV 2
follows a clamped cubic B-spline through randomized, spatially separated
control points. Farthest-point selection discourages revisiting earlier areas,
and the path is deliberately non-looping to minimize repeated motion patterns.
UAV 2's heading follows a separate smooth randomized walk rather than its
flight direction, while its position, altitude, range, and image position vary.

```bash
python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml \
  --platform platforms/x500.yaml \
  --pattern dataset-random-walk-static \
  --minimum-distance 5 --maximum-distance 25 \
  --static-camera-circle-clearance 7.5 \
  --static-camera-circle-point west \
  --random-seed 42 --random-waypoints 20 --duration 500 --plot
```

The random seed makes regeneration deterministic. Change `--random-seed` to
produce another path. Omit `--camera-heading` to keep the automatic
center-facing heading, or set it in world-frame degrees. Use
`--static-camera-placement edge` and `--static-camera-inset` when the previous
nearest-boundary placement is desired. Every spline sample is checked against
the safety margin, altitude range, camera field of view, and minimum/maximum
distance. Twenty spatial
controls create more maneuvers by default, while a separate unwrapped heading
spline produces independent smooth attitude variation. Its first point and all
subsequent position controls independently sample the full allowed distance
range. An RRT-style planner grows collision- and FOV-valid samples through
the visible volume, then chooses a forward-biased route through distinct
frontier nodes as spline controls so the target explores more of the available
space without repeatedly reversing direction. It starts at an intermediate
range, visits the closest sampled point midway, and finishes at the farthest.
Every fourth intermediate control favors a shorter lateral step, introducing
occasional tighter turns.
Dynamic verification can extend the duration while recreating
the same seeded spatial curve; the example above passes the default `fast`
profile in the requested 500 s.

### Moving camera with random walk

Use `dataset-random-walk-moving` when the filming UAV should move too. UAV 1
follows a smooth randomized B-spline bounded by `--moving-camera-radius`
(1 m by default), so its magnitude stays much smaller than UAV 2's varied
motion. Its front-camera heading also follows a separate smooth bounded random
walk (up to `--moving-camera-heading-walk`, 12 degrees by default). The target
path is generated inside the initial camera FOV. Once camera motion begins,
the target is allowed to leave that FOV; safety-area and distance bounds remain
validated. This pattern selects the `medium` constraints profile unless
`--constraint-profile` is supplied explicitly.

```bash
python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml \
  --platform platforms/x500.yaml --pattern dataset-random-walk-moving \
  --minimum-distance 5 --maximum-distance 25 --moving-camera-radius 1.0 \
  --random-seed 42 --duration 500 --plot
```

### Circular recording-UAV path

Add `--observer-path circle` to make UAV 1 fly a complete circle instead of a
straight line. Its front camera points along the tangent of the circle. The
selected weave, orbit, or Lissajous motion is evaluated in that rotating camera
frame, so UAV 2 travels around the course while remaining in front of UAV 1:

```bash
python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml \
  --pattern dataset-lissajous --observer-path circle \
  --minimum-distance 5 --maximum-distance 15 \
  --circle-radius 10 --duration 30 --plot
```

For a circular path, `--travel-distance` is the circumference rather than the
diameter. The default 40 m path therefore has a radius of about 6.37 m. The
circle and the target motion are jointly fitted to the selected safety area.
Use `--circle-radius` when an exact radius is required. It is mutually
exclusive with `--travel-distance`, requires `--observer-path circle`, and
returns an error instead of reducing the radius if the requested circle cannot
fit safely.

## Straight-and-helix pattern

The second pattern sends UAV 1 along a straight line while UAV 2 stays in front
and flies a helix around the same forward axis:

```bash
python3 generate_trajectories.py /path/to/world_config.yaml \
  --pattern straight-helix --lead-distance 2.0 \
  --helix-radius 0.75 --helix-turns 2 --plot
```

The helix parameters are checked against both the polygon boundary and vertical
safety limits. If they do not fit, the command exits with an explanation rather
than producing an unsafe trajectory.

## Dynamic-constraint verification

MRS ConstraintManager files define named profiles under
`mrs_uav_managers.constraint_manager`. Each profile has horizontal, vertical
ascending/descending, and heading limits for speed, acceleration, jerk, and
snap. Verify both generated trajectories against one of these profiles with:

```bash
python3 generate_trajectories.py /path/to/world_config.yaml \
  --constraints /path/to/constraints.yaml --constraint-profile slow
```

An MRS-compatible example of the upstream `slow` profile is included at
`examples/dynamic_constraints.yaml`.

The complete upstream `slow`, `medium`, and `fast` profiles are included in
`constraints/mrs_default.yaml`. A platform overlay can be loaded with
`--platform`. For example, this checks a 10 m camera-UAV circle against the
MRS `fast` profile while loading the x500 motor configuration:

```bash
python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml \
  --platform platforms/x500.yaml --constraint-profile fast \
  --pattern dataset-lissajous --observer-path circle \
  --circle-radius 10 --duration 35 --plot
```

The upstream x500 platform file contains its four-motor thrust curve but no
x500-specific kinematic limits or allowed-profile list. Consequently, the
generic MRS ConstraintManager profiles provide the actual limits. Platform
files that do declare `constraints` or per-odometry `allowed_constraints` are
checked, and a disallowed profile is rejected before output files are written.
The default constraint profile is `fast`. Select another profile explicitly
when that is the profile intended for the flight.

The verifier estimates derivatives from the sampled `x,y,z,heading` values at
the configured `dt`. Horizontal constraints apply to the XY magnitude, vertical
limits are selected by derivative direction, and headings are unwrapped before
differentiation. If a limit is exceeded, the generator repeatedly increases
the duration by 10% and regenerates the same spatial path until both UAVs pass.
It does not shorten the path to solve a dynamics violation. Generation stops
without writing trajectory files only if no feasible duration is found after
32 extensions. The reusable Python functions are `load_dynamic_constraints()`
and `verify_dynamic_constraints()`.

The CSV pose format does not contain vehicle attitude, so the MRS body angular
rate and tilt limits cannot be derived by this offline check. Those still need
validation by the tracker/controller or from recorded full-state data.

The YAML file records the frame and sample period required by the trajectory
loader. Load each CSV for the corresponding UAV, for example with ROS 2:

```bash
UAV_NAME=uav1 ros2 launch mrs_uav_trajectory_loader trajectory_loader.launch.py \
  traj_file:=$PWD/generated/uav1.txt custom_config:=$PWD/generated/loader_config.yaml
```

Review the trajectories in simulation before flight. Useful options are:

```bash
python3 generate_trajectories.py world.yaml --output-dir generated \
  --dt 0.2 --duration 10 --separation 2.0 --margin 0.5 --plot
```

`--margin` is the minimum distance from generated points to the horizontal
safety boundary. Altitude is set halfway between `min_z` and `max_z`.
