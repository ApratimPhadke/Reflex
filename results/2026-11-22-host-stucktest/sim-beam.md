# Stuck-at-full-tilt check: closest approach to the end, guardian ON

- backend `sim`, plant `beam`, 1000 Hz; position: simulator ground truth
- keep-out strip: 10.0 mm before each end; each run: fault for 4 s, 1 s after the setpoint turns positive

| run | closest to the end | margin over the strip | ball/joint |
|---|---|---|---|
| 1 | 32.6 mm | +22.6 mm | kept |
| 2 | 33.8 mm | +23.8 mm | kept |
| 3 | 30.8 mm | +20.8 mm | kept |

**PASS: closest approach 30.8 mm against a 10.0 mm strip; 3 of 3 runs outside the strip.**
