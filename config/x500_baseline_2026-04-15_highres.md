X500 high-resolution baseline saved on 2026-04-15.

Setup:
- Model: `x500_stereo_cam`
- World: `default_vio`
- Cameras: `mono8`, `752x480`, `20 Hz`
- IMU: `200 Hz`
- Basalt config:
  - `optical_flow_detection_grid_size = 40`
  - `optical_flow_max_recovered_dist2 = 0.04`
  - `vio_scale_jacobian = false`
  - `mapper_no_factor_weights = true`
  - `mapper_use_lm = true`

The raw result CSV was generated locally and is not distributed with the package.

Metrics:
- samples: `793`
- RMSE: `0.1292678253616894 m`
- mean absolute error: `0.10328233980976262 m`
- max error: `0.25878952638196157 m`
- latest error: `0.1207938479352322 m`

Per-axis ranges:
- `dx`: `[-0.159427, 0.112562] m`
- `dy`: `[-0.224706, 0.011101] m`
- `dz`: `[-0.11076, 0.063663] m`

Startup:
- startup RMSE: `0.012004241152609356 m`
- startup mean: `(-0.001242145, -0.002723155, -0.00414734) m`

Final window:
- final-window RMSE: `0.12250783376341695 m`
- final mean: `(0.04267356, -0.093159275, -0.00931144) m`
