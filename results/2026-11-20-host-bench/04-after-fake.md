# Reflex benchmark: AI pilot vs fallback controller alone

- backend: `serial`, control rate 50 Hz; position: the guardian's estimate (no ground truth on this backend)
- setpoint sequence (mm), each held 5 s: 80 -80 40 -40 0
- settling: last time outside a band of 10 % of the step (at least 5 mm); a step still outside it in the last 0.5 s counts as not settled

| | AI pilot (guardian on) | fallback controller alone |
|---|---|---|
| mean settling time | 1.44 s | 1.87 s |
| worst overshoot | 3.2 mm | 3.6 mm |
| RMS tracking error | 29.2 mm | 31.8 mm |
| guardian interventions | 0 | 0 |
| steps not settled | 0 of 5 | 0 of 5 |

| step | setpoint | AI settling | AI overshoot | fallback settling | fallback overshoot |
|---|---|---|---|---|---|
| 1 | +80 mm | 1.54 s | 3.2 mm | 2.04 s | 1.7 mm |
| 2 | -80 mm | 1.57 s | 2.7 mm | 1.82 s | 0.9 mm |
| 3 | +40 mm | 1.57 s | 2.9 mm | 1.75 s | 2.3 mm |
| 4 | -40 mm | 1.66 s | 1.2 mm | 2.11 s | 2.5 mm |
| 5 | +0 mm | 0.85 s | 2.8 mm | 1.64 s | 3.6 mm |

AI vs fallback: settling faster by 23 %, RMS error lower by 8 %, 0 interventions while the AI flew.
