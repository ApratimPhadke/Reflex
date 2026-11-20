# Reflex benchmark: AI pilot vs fallback controller alone

- backend: `sim`, control rate 1000 Hz; position: simulator ground truth
- setpoint sequence (mm), each held 5 s: 80 -80 40 -40 0
- settling: last time outside a band of 10 % of the step (at least 5 mm); a step still outside it in the last 0.5 s counts as not settled

| | AI pilot (guardian on) | fallback controller alone |
|---|---|---|
| mean settling time | 0.67 s | 1.68 s |
| worst overshoot | 0.4 mm | 0.1 mm |
| RMS tracking error | 24.9 mm | 32.8 mm |
| guardian interventions | 0 | 0 |
| steps not settled | 0 of 5 | 0 of 5 |

| step | setpoint | AI settling | AI overshoot | fallback settling | fallback overshoot |
|---|---|---|---|---|---|
| 1 | +80 mm | 0.66 s | 0.3 mm | 1.70 s | 0.0 mm |
| 2 | -80 mm | 0.76 s | 0.3 mm | 1.72 s | 0.0 mm |
| 3 | +40 mm | 0.71 s | 0.3 mm | 1.71 s | 0.0 mm |
| 4 | -40 mm | 0.68 s | 0.4 mm | 1.70 s | 0.1 mm |
| 5 | +0 mm | 0.56 s | 0.3 mm | 1.56 s | 0.0 mm |

AI vs fallback: settling faster by 60 %, RMS error lower by 24 %, 0 interventions while the AI flew.
