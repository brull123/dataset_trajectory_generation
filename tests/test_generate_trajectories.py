import math
import tempfile
import unittest
from pathlib import Path

from generate_trajectories import (
    _clearance,
    _trajectory_length,
    RRTVisualization,
    generate,
    generate_dataset_lissajous,
    generate_dataset_orbit,
    generate_dataset_weave,
    generate_moving_camera_random_walk,
    generate_static_camera_random_walk,
    generate_straight_and_helix,
    load_dynamic_constraints,
    load_platform_config,
    load_safety_area,
    main,
    parse_args,
    safest_point,
    shift_trajectories_to_boundary,
    validate_platform_constraint_profile,
    verify_dynamic_constraints,
    visualize_trajectories,
)


class TrajectoryGeneratorTest(unittest.TestCase):
    def _config(self, contents: str) -> Path:
        temporary = tempfile.NamedTemporaryFile(mode="w", suffix=".yaml", delete=False)
        temporary.write(contents)
        temporary.close()
        self.addCleanup(Path(temporary.name).unlink)
        return Path(temporary.name)

    def test_loads_nested_flat_polygon_and_generates_two_trajectories(self):
        path = self._config(
            """
mrs_uav_managers:
  safety_area_manager:
    safety_area:
      enabled: true
      horizontal:
        frame_name: world_origin
        points: [-5, -4, 5, -4, 5, 4, -5, 4]
      vertical:
        frame_name: world_origin
        min_z: 1
        max_z: 5
"""
        )
        area = load_safety_area(path)
        trajectories = generate(area, dt=0.2, duration=1.0, separation=1.0, margin=0.5)

        self.assertEqual(area.frame_id, "world_origin")
        self.assertEqual(len(trajectories), 2)
        self.assertEqual(len(trajectories[0]), 6)
        self.assertTrue(all(point[2] == 3.0 for trajectory in trajectories for point in trajectory))

    def test_shifts_completed_trajectories_to_requested_boundary_with_offset(self):
        path = self._config(
            """
safety_area:
  horizontal:
    frame_name: world_origin
    points: [-5, -5, 5, -5, 5, 5, -5, 5]
  vertical: {min_z: 1, max_z: 5}
"""
        )
        area = load_safety_area(path)
        trajectories = (
            [(0.0, -1.0, 3.0, 0.0), (0.0, 1.0, 3.0, 0.2)],
            [(1.0, -1.0, 3.0, -0.1), (1.0, 1.0, 3.0, 0.3)],
        )

        shifted = shift_trajectories_to_boundary(
            area, trajectories, "south", boundary_offset=1.0, margin=0.5
        )
        centered = shift_trajectories_to_boundary(
            area, trajectories, "center", boundary_offset=0.0, margin=0.5
        )

        self.assertAlmostEqual(min(point[1] for trajectory in shifted for point in trajectory), -3.5)
        self.assertEqual([point[0] for point in shifted[0]], [-0.5, -0.5])
        centered_points = [point for trajectory in centered for point in trajectory]
        self.assertAlmostEqual(sum(point[0] for point in centered_points) / len(centered_points), 0.0)
        self.assertAlmostEqual(sum(point[1] for point in centered_points) / len(centered_points), 0.0)
        self.assertEqual(
            [point[2:] for point in shifted[1]], [point[2:] for point in trajectories[1]]
        )

    def test_boundary_placement_arguments_are_parsed(self):
        args = parse_args(
            ["world.yaml", "--placement-direction", "east", "--boundary-offset", "2"]
        )
        self.assertEqual(args.placement_direction, "east")
        self.assertEqual(args.boundary_offset, 2.0)

    def test_rrt_plot_overlay_is_opt_in(self):
        args = parse_args(["world.yaml"])
        self.assertFalse(args.plot_rrt)
        args = parse_args(["world.yaml", "--plot-rrt"])
        self.assertTrue(args.plot_rrt)

    def test_directional_margins_override_legacy_shared_margin(self):
        args = parse_args(
            [
                "world.yaml",
                "--margin", "1",
                "--horizontal-margin", "2",
                "--vertical-margin", "3",
            ]
        )
        self.assertEqual(args.horizontal_margin, 2.0)
        self.assertEqual(args.vertical_margin, 3.0)

    def test_converts_latlon_polygon_to_world_origin_metres(self):
        path = self._config(
            """
world_origin:
  units: LATLON
  origin_x: 50.0
  origin_y: 14.0
safety_area:
  horizontal:
    frame_name: latlon_origin
    points:
      - [49.9999, 13.9999]
      - [49.9999, 14.0001]
      - [50.0001, 14.0001]
      - [50.0001, 13.9999]
  vertical: {min_z: 0.5, max_z: 10}
"""
        )
        area = load_safety_area(path)

        self.assertEqual(area.frame_id, "world_origin")
        self.assertGreater(max(point[0] for point in area.polygon), 7.0)
        self.assertGreater(max(point[1] for point in area.polygon), 11.0)

    def test_saves_visualization(self):
        path = self._config(
            """
safety_area:
  horizontal:
    frame_name: world_origin
    points: [-5, -4, 5, -4, 5, 4, -5, 4]
  vertical: {min_z: 1, max_z: 5}
"""
        )
        area = load_safety_area(path)
        trajectories = generate(area, dt=0.2, duration=1.0, separation=2.0, margin=0.5)
        constraints = load_dynamic_constraints(
            Path(__file__).parents[1] / "constraints" / "mrs_default.yaml", "fast"
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "trajectory.png"
            figure = visualize_trajectories(
                area, trajectories, dt=0.2, output_path=output, constraints=constraints
            )
            diagnostics = figure.diagnostics_figure

            self.assertTrue(output.is_file())
            self.assertTrue((Path(directory) / "trajectory_diagnostics.png").is_file())
            self.assertGreater(output.stat().st_size, 1_000)
            self.assertEqual(len(figure.axes), 1)
            self.assertLess(figure.axes[0].get_xlim()[1] - figure.axes[0].get_xlim()[0], 10.0)
            self.assertEqual(len(diagnostics.axes), 6)
            distance_axes = diagnostics.axes[1]
            relative_axes = diagnostics.axes[2]
            velocity_axes = diagnostics.axes[3]
            acceleration_axes = diagnostics.axes[4]
            heading_axes = diagnostics.axes[5]
            self.assertEqual(distance_axes.get_title(), "Inter-UAV distance")
            self.assertIn("minimum = 2.00 m", distance_axes.get_legend_handles_labels()[1])
            self.assertEqual(relative_axes.get_title(), "Relative velocity (UAV 2 − UAV 1)")
            self.assertEqual(len(relative_axes.lines), 5)
            self.assertTrue(
                all(len(line.get_xdata()) == len(trajectories[0]) - 1 for line in relative_axes.lines[:3])
            )
            self.assertEqual(velocity_axes.get_title(), "Velocity magnitude")
            self.assertEqual(len(velocity_axes.lines), 4)
            self.assertTrue(
                all(len(line.get_ydata()) == len(trajectories[0]) - 1 for line in velocity_axes.lines)
            )
            self.assertIn(
                "UAV 1 combined speed limit", velocity_axes.get_legend_handles_labels()[1]
            )
            self.assertEqual(acceleration_axes.get_title(), "Acceleration magnitude")
            self.assertEqual(len(acceleration_axes.lines), 2)
            self.assertTrue(
                all(len(line.get_ydata()) == len(trajectories[0]) - 2 for line in acceleration_axes.lines)
            )
            self.assertEqual(heading_axes.get_title(), "UAV heading")
            self.assertEqual(len(heading_axes.lines), 2)
            self.assertTrue(
                all(len(line.get_ydata()) == len(trajectories[0]) for line in heading_axes.lines)
            )

    def test_straight_follower_and_helix_leader(self):
        path = self._config(
            """
safety_area:
  horizontal:
    frame_name: world_origin
    points: [-8, -6, 8, -6, 8, 6, -8, 6]
  vertical: {min_z: 0, max_z: 8}
"""
        )
        area = load_safety_area(path)
        straight, helix = generate_straight_and_helix(
            area,
            dt=0.2,
            duration=4.0,
            lead_distance=2.0,
            helix_radius=1.0,
            helix_turns=2.5,
            margin=0.5,
        )

        self.assertEqual(len(straight), 21)
        self.assertTrue(all(point[1] == 0.0 and point[2] == 4.0 for point in straight))
        self.assertTrue(all(leader[0] > follower[0] for follower, leader in zip(straight, helix)))
        self.assertAlmostEqual(helix[0][1], 1.0)
        self.assertAlmostEqual(helix[2][2], 5.0)
        self.assertGreater(len({round(point[1], 3) for point in helix}), 4)
        self.assertGreater(len({round(point[2], 3) for point in helix}), 4)

    def test_loads_mrs_constraints_and_detects_speed_violation(self):
        path = self._config(
            """
mrs_uav_managers:
  constraint_manager:
    test:
      horizontal: {speed: 1, acceleration: 10, jerk: 10, snap: 10}
      vertical:
        ascending: {speed: 1, acceleration: 10, jerk: 10, snap: 10}
        descending: {speed: 2, acceleration: 20, jerk: 20, snap: 20}
      heading: {speed: 1, acceleration: 10, jerk: 10, snap: 10}
"""
        )
        constraints = load_dynamic_constraints(path, "test")
        trajectory = [(float(index * 2), 0.0, 2.0, 0.0) for index in range(5)]
        verification = verify_dynamic_constraints(trajectory, dt=1.0, constraints=constraints)

        self.assertFalse(verification.passed)
        self.assertEqual(constraints.vertical_descending["speed"], 2.0)
        self.assertEqual(verification.maxima["horizontal.speed"], 2.0)
        self.assertTrue(
            all(violation.quantity == "horizontal.speed" for violation in verification.violations)
        )

    def test_constraint_verifier_unwraps_heading(self):
        path = self._config(
            """
constraint_manager:
  test:
    horizontal: {speed: 10, acceleration: 10, jerk: 10, snap: 10}
    vertical:
      ascending: {speed: 10, acceleration: 10, jerk: 10, snap: 10}
      descending: {speed: 10, acceleration: 10, jerk: 10, snap: 10}
    heading: {speed: 1, acceleration: 10, jerk: 10, snap: 10}
"""
        )
        constraints = load_dynamic_constraints(path, "test")
        trajectory = [
            (0.0, 0.0, 1.0, heading)
            for heading in (3.10, 3.12, -3.14, -3.12, -3.10)
        ]
        verification = verify_dynamic_constraints(trajectory, dt=1.0, constraints=constraints)

        self.assertTrue(verification.passed)
        self.assertLess(verification.maxima["heading.speed"], 0.1)

    def test_dataset_patterns_keep_target_visible_and_separated(self):
        path = self._config(
            """
safety_area:
  horizontal:
    frame_name: world_origin
    points: [-5, -4, 5, -4, 5, 4, -5, 4]
  vertical: {min_z: 1, max_z: 5}
"""
        )
        area = load_safety_area(path)
        generators = (
            generate_dataset_weave,
            generate_dataset_orbit,
            generate_dataset_lissajous,
        )
        for generator in generators:
            with self.subTest(generator=generator.__name__):
                observer, target = generator(
                    area,
                    dt=0.2,
                    duration=10.0,
                    minimum_distance=2.0,
                    camera_horizontal_fov=90.0,
                    camera_vertical_fov=60.0,
                    relative_heading_turns=1.5,
                    margin=0.5,
                )
                distances = []
                for observer_point, target_point in zip(observer, target):
                    dx = target_point[0] - observer_point[0]
                    dy = target_point[1] - observer_point[1]
                    dz = target_point[2] - observer_point[2]
                    distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))
                    self.assertGreater(dx, 0.0)
                    self.assertLessEqual(abs(math.degrees(math.atan2(dy, dx))), 36.0 + 1e-8)
                    self.assertLessEqual(
                        abs(math.degrees(math.atan2(dz, math.hypot(dx, dy)))),
                        24.0 + 1e-8,
                    )
                    self.assertGreaterEqual(observer_point[0], -4.5 - 1e-8)
                    self.assertLessEqual(observer_point[0], 4.5 + 1e-8)
                    self.assertGreaterEqual(target_point[2], 1.5)
                    self.assertLessEqual(target_point[2], 4.5)

                self.assertAlmostEqual(min(distances), 2.0)
                self.assertGreater(max(distances) - min(distances), 0.5)
                self.assertGreater(math.dist(observer[0][:3], observer[-1][:3]), 0.25)
                self.assertGreater(math.dist(target[0][:3], target[-1][:3]), 0.25)
                self.assertTrue(all(point[3] == 0.0 for point in observer))
                self.assertGreater(len({round(point[3], 2) for point in target}), 10)

    def test_temesvar_1_dataset_defaults_use_field_scale(self):
        area = load_safety_area(
            Path(__file__).parents[1] / "worlds" / "world_temesvar_field_1.yaml"
        )
        observer, target = generate_dataset_lissajous(
            area,
            dt=0.2,
            duration=30.0,
            minimum_distance=5.0,
        )
        distances = [
            math.dist(observer_point[:3], target_point[:3])
            for observer_point, target_point in zip(observer, target)
        ]

        self.assertAlmostEqual(math.dist(observer[0][:3], observer[-1][:3]), 40.0)
        self.assertAlmostEqual(min(distances), 5.0)
        self.assertAlmostEqual(max(distances), 15.0)

    def test_circular_camera_path_keeps_target_in_front(self):
        area = load_safety_area(
            Path(__file__).parents[1] / "worlds" / "world_temesvar_field_1.yaml"
        )
        observer, target = generate_dataset_lissajous(
            area,
            dt=0.2,
            duration=30.0,
            minimum_distance=5.0,
            observer_path="circle",
        )
        observer_length = sum(
            math.dist(previous[:3], current[:3])
            for previous, current in zip(observer, observer[1:])
        )
        distances = []
        for observer_point, target_point in zip(observer, target):
            dx = target_point[0] - observer_point[0]
            dy = target_point[1] - observer_point[1]
            dz = target_point[2] - observer_point[2]
            heading = observer_point[3]
            forward = dx * math.cos(heading) + dy * math.sin(heading)
            lateral = -dx * math.sin(heading) + dy * math.cos(heading)
            distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))

            self.assertGreater(forward, 0.0)
            self.assertLessEqual(
                abs(math.degrees(math.atan2(lateral, forward))), 36.0 + 1e-8
            )
            self.assertLessEqual(
                abs(math.degrees(math.atan2(dz, math.hypot(dx, dy)))),
                24.0 + 1e-8,
            )

        self.assertAlmostEqual(observer_length, 40.0, places=2)
        self.assertAlmostEqual(math.dist(observer[0][:3], observer[-1][:3]), 0.0)
        self.assertAlmostEqual(min(distances), 5.0)
        self.assertAlmostEqual(max(distances), 15.0)

    def test_explicit_camera_circle_radius(self):
        area = load_safety_area(
            Path(__file__).parents[1] / "worlds" / "world_temesvar_field_1.yaml"
        )
        observer, _ = generate_dataset_weave(
            area,
            dt=0.2,
            duration=30.0,
            minimum_distance=5.0,
            observer_path="circle",
            circle_radius=10.0,
        )

        diameter_x = max(point[0] for point in observer) - min(
            point[0] for point in observer
        )
        self.assertAlmostEqual(diameter_x, 20.0)

    def test_static_camera_random_walk_is_visible_smooth_and_reproducible(self):
        area = load_safety_area(
            Path(__file__).parents[1] / "worlds" / "world_temesvar_field_1.yaml"
        )
        arguments = dict(
            area=area,
            dt=0.2,
            duration=180.0,
            minimum_distance=5.0,
            maximum_distance=25.0,
            random_seed=42,
            random_waypoints=10,
        )
        rrt_visualization = RRTVisualization()
        observer, target = generate_static_camera_random_walk(
            **arguments, rrt_visualization=rrt_visualization
        )
        repeated_observer, repeated_target = generate_static_camera_random_walk(**arguments)
        _, different_target = generate_static_camera_random_walk(
            **{**arguments, "random_seed": 43}
        )

        self.assertEqual(observer, repeated_observer)
        self.assertEqual(target, repeated_target)
        self.assertNotEqual(target, different_target)
        self.assertGreater(len(rrt_visualization.points or []), 10)
        self.assertEqual(len(rrt_visualization.spline_controls or []), 10)
        self.assertEqual(len(set(observer)), 1)
        circle_center, circle_clearance = safest_point(area.polygon)
        self.assertAlmostEqual(observer[0][0], circle_center[0])
        self.assertAlmostEqual(observer[0][1], circle_center[1] + circle_clearance - 7.5)
        self.assertAlmostEqual(observer[0][3], -math.pi / 2.0)
        self.assertGreaterEqual(_clearance(observer[0][:2], area.polygon), 7.5 - 1e-8)
        self.assertGreater(math.dist(target[0][:3], target[-1][:3]), 1.0)
        self.assertGreater(len({tuple(round(value, 3) for value in point[:3]) for point in target}), 800)

        distances = []
        observer_point = observer[0]
        for target_point in target:
            dx = target_point[0] - observer_point[0]
            dy = target_point[1] - observer_point[1]
            dz = target_point[2] - observer_point[2]
            heading = observer_point[3]
            forward = dx * math.cos(heading) + dy * math.sin(heading)
            lateral = -dx * math.sin(heading) + dy * math.cos(heading)
            distances.append(math.sqrt(dx * dx + dy * dy + dz * dz))
            self.assertGreater(forward, 0.0)
            self.assertLessEqual(
                abs(math.degrees(math.atan2(lateral, forward))), 36.0 + 1e-8
            )
            self.assertLessEqual(
                abs(math.degrees(math.atan2(dz, math.hypot(dx, dy)))),
                24.0 + 1e-8,
            )
        # The route begins at an intermediate range, reaches the nearest sample
        # later, and ends at the farthest one.
        self.assertGreater(distances[0], 5.0 + 0.1)
        self.assertAlmostEqual(min(distances), 5.0)
        self.assertGreater(max(distances), 15.0)
        self.assertLessEqual(max(distances), 25.0 + 1e-8)
        distance_changes = [
            following - previous
            for previous, following in zip(distances, distances[1:])
        ]
        self.assertTrue(any(change > 0.01 for change in distance_changes))
        self.assertTrue(any(change < -0.01 for change in distance_changes))
        misaligned_headings = 0
        for index, target_point in enumerate(target):
            previous = target[max(0, index - 1)]
            following = target[min(len(target) - 1, index + 1)]
            direction_x = following[0] - previous[0]
            direction_y = following[1] - previous[1]
            alignment = (
                direction_x * math.cos(target_point[3])
                + direction_y * math.sin(target_point[3])
            )
            if alignment < 0.0:
                misaligned_headings += 1
        self.assertGreater(misaligned_headings, len(target) // 3)
        constraints = load_dynamic_constraints(
            Path(__file__).parents[1] / "constraints" / "mrs_default.yaml", "medium"
        )
        self.assertTrue(verify_dynamic_constraints(target, 0.2, constraints).passed)

        custom_observer, _ = generate_static_camera_random_walk(
            **{**arguments, "camera_heading": 0.0}
        )
        self.assertAlmostEqual(custom_observer[0][3], 0.0)
        west_observer, _ = generate_static_camera_random_walk(
            **{**arguments, "camera_circle_point": "west"}
        )
        self.assertAlmostEqual(west_observer[0][3], 0.0)

    def test_moving_camera_random_walk_starts_visible_and_meets_medium_constraints(self):
        root = Path(__file__).parents[1]
        area = load_safety_area(root / "worlds" / "world_temesvar_field_1.yaml")
        arguments = dict(
            area=area,
            dt=0.2,
            duration=500.0,
            minimum_distance=5.0,
            maximum_distance=25.0,
            random_seed=42,
            random_waypoints=20,
            camera_motion_radius=1.0,
        )
        observer, target = generate_moving_camera_random_walk(**arguments)
        repeated_observer, repeated_target = generate_moving_camera_random_walk(**arguments)

        self.assertEqual(observer, repeated_observer)
        self.assertEqual(target, repeated_target)
        self.assertGreater(_trajectory_length(observer), 0.1)
        self.assertLess(_trajectory_length(observer), _trajectory_length(target))
        heading_span = max(point[3] for point in observer) - min(
            point[3] for point in observer
        )
        self.assertGreater(heading_span, math.radians(1.0))
        self.assertLessEqual(heading_span, math.radians(24.0) + 1e-8)
        distances = [
            math.dist(camera[:3], target_point[:3])
            for camera, target_point in zip(observer, target)
        ]
        self.assertAlmostEqual(min(distances), 5.0)
        self.assertLessEqual(max(distances), 25.0 + 1e-8)
        initial_camera, initial_target = observer[0], target[0]
        initial_dx = initial_target[0] - initial_camera[0]
        initial_dy = initial_target[1] - initial_camera[1]
        initial_dz = initial_target[2] - initial_camera[2]
        initial_forward = (
            initial_dx * math.cos(initial_camera[3])
            + initial_dy * math.sin(initial_camera[3])
        )
        initial_left = (
            -initial_dx * math.sin(initial_camera[3])
            + initial_dy * math.cos(initial_camera[3])
        )
        self.assertGreater(initial_forward, 0.0)
        self.assertLessEqual(
            abs(math.degrees(math.atan2(initial_left, initial_forward))), 45.0 + 1e-8
        )
        self.assertLessEqual(
            abs(math.degrees(math.atan2(initial_dz, math.hypot(initial_dx, initial_dy)))),
            30.0 + 1e-8,
        )
        constraints = load_dynamic_constraints(root / "constraints" / "mrs_default.yaml", "medium")
        self.assertTrue(verify_dynamic_constraints(observer, 0.2, constraints).passed)
        self.assertTrue(verify_dynamic_constraints(target, 0.2, constraints).passed)
        self.assertEqual(
            parse_args([str(root / "worlds" / "world_temesvar_field_1.yaml"),
                        "--pattern", "dataset-random-walk-moving"]).constraint_profile,
            "medium",
        )

    def test_x500_circular_dataset_satisfies_fast_constraints(self):
        root = Path(__file__).parents[1]
        platform = load_platform_config(root / "platforms" / "x500.yaml")
        constraints = load_dynamic_constraints(
            root / "constraints" / "mrs_default.yaml", "fast"
        )
        validate_platform_constraint_profile(platform, constraints.name)
        area = load_safety_area(root / "worlds" / "world_temesvar_field_1.yaml")
        trajectories = generate_dataset_lissajous(
            area,
            dt=0.2,
            duration=35.0,
            minimum_distance=5.0,
            observer_path="circle",
            circle_radius=10.0,
        )

        self.assertEqual(platform.name, "x500")
        self.assertEqual(platform.motor_count, 4)
        self.assertEqual(platform.allowed_constraints, ())
        self.assertTrue(
            all(
                verify_dynamic_constraints(trajectory, 0.2, constraints).passed
                for trajectory in trajectories
            )
        )

    def test_fast_is_default_and_cli_extends_duration(self):
        root = Path(__file__).parents[1]
        self.assertEqual(
            parse_args([str(root / "worlds" / "world_temesvar_field_1.yaml")]).constraint_profile,
            "fast",
        )
        self.assertEqual(
            parse_args([str(root / "worlds" / "world_temesvar_field_1.yaml")]).duration,
            500.0,
        )
        with tempfile.TemporaryDirectory() as directory:
            output = Path(directory) / "trajectory"
            result = main(
                [
                    str(root / "worlds" / "world_temesvar_field_1.yaml"),
                    "--platform",
                    str(root / "platforms" / "x500.yaml"),
                    "--pattern",
                    "dataset-lissajous",
                    "--observer-path",
                    "circle",
                    "--circle-radius",
                    "10",
                    "--duration",
                    "30",
                    "--output-dir",
                    str(output),
                ]
            )

            self.assertEqual(result, 0)
            self.assertGreater(len((output / "uav1.txt").read_text().splitlines()), 151)


if __name__ == "__main__":
    unittest.main()
