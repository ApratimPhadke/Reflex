# Reflex benchmark: AI pilot vs fallback controller alone

- backend: `serial`, control rate 50 Hz; position: the guardian's estimate (no ground truth on this backend)
- setpoint sequence (mm), each held 5 s: 80 -80 40 -40 0
- settling: last time outside a band of 10 % of the step (at least 5 mm); a step still outside it in the last 0.5 s counts as not settled

| | AI pilot (guardian on) | fallback controller alone |
|---|---|---|
| mean settling time | 2.77 s | 1.82 s |
| worst overshoot | 3.2 mm | 3.7 mm |
| RMS tracking error | 28.8 mm | 31.9 mm |
| guardian interventions | 0 | 0 |
| steps not settled | 3 of 5 | 0 of 5 |

| step | setpoint | AI settling | AI overshoot | fallback settling | fallback overshoot |
|---|---|---|---|---|---|
| 1 | +80 mm | not settled, 4.98 s | 0.0 mm | 1.78 s | 1.6 mm |
| 2 | -80 mm | not settled, 4.99 s | 0.0 mm | 1.75 s | 0.9 mm |
| 3 | +40 mm | 4.24 s | 0.0 mm | 1.77 s | 2.3 mm |
| 4 | -40 mm | not settled, 4.98 s | 0.0 mm | 2.13 s | 2.5 mm |
| 5 | +0 mm | 1.30 s | 3.2 mm | 1.66 s | 3.7 mm |

AI vs fallback: settling SLOWER by 53 %, RMS error lower by 10 %, 0 interventions while the AI flew.
