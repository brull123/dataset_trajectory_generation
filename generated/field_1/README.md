# Temesvar field 1 generated trajectories

All three trajectory pairs use the x500 platform configuration, the MRS
`medium` constraints profile, a 0.2 s sample period, a 5--15 m inter-UAV range,
and an initial requested duration of 30 s. Dynamic verification extended only
the duration; the requested spatial paths were preserved.

| Dataset | Observer path | Final duration | Samples per UAV | Reproduction command |
| --- | --- | ---: | ---: | --- |
| `dataset_weave_straight` | 40 m straight path | 36.4 s | 183 | `dataset_weave_straight/generation_command.sh` |
| `dataset_orbit_straight` | 40 m straight path | 40.2 s | 202 | `dataset_orbit_straight/generation_command.sh` |
| `dataset_lissajous_circle` | 10 m-radius circle | 65.4 s | 328 | `dataset_lissajous_circle/generation_command.sh` |

Each dataset directory contains `uav1.txt`, `uav2.txt`,
`loader_config.yaml`, `trajectories.png`, and the exact generation command.
Run all three again from the repository root with:

```bash
bash generated/field_1/generate_all.sh
```
