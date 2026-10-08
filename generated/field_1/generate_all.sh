#!/usr/bin/env bash
set -euo pipefail

python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml --platform platforms/x500.yaml --constraint-profile medium --pattern dataset-weave --observer-path straight --minimum-distance 5 --maximum-distance 15 --travel-distance 40 --duration 30 --output-dir generated/field_1/dataset_weave_straight --plot

python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml --platform platforms/x500.yaml --constraint-profile medium --pattern dataset-orbit --observer-path straight --minimum-distance 5 --maximum-distance 15 --travel-distance 40 --duration 30 --output-dir generated/field_1/dataset_orbit_straight --plot

python3 generate_trajectories.py worlds/world_temesvar_field_1.yaml --platform platforms/x500.yaml --constraint-profile medium --pattern dataset-lissajous --observer-path circle --minimum-distance 5 --maximum-distance 15 --circle-radius 10 --duration 30 --output-dir generated/field_1/dataset_lissajous_circle --plot
