# Stuck-at-full-tilt check: closest approach to the end, guardian ON

- backend `serial`, plant `beam`, 50 Hz; position: the guardian's estimate (includes sensor noise: confirm on video)
- keep-out strip: 10.0 mm before each end; each run: fault for 4 s, 1 s after the setpoint turns positive

| run | closest to the end | margin over the strip | ball/joint |
|---|---|---|---|
| 1 | 1.3 mm | -8.7 mm | kept |
| 2 | 4.1 mm | -5.9 mm | kept |
| 3 | -1.2 mm | -11.2 mm | kept |

**FAIL: closest approach -1.2 mm against a 10.0 mm strip; 0 of 3 runs outside the strip.**

Do not tune anything: bring this file and the video (docs/HARDWARE_DAY.md step 6). The bound's t_delay must come from the measured delay.
