# Reflex benchmark: AI pilot vs fallback controller alone

- backend: `sim`, control rate 1000 Hz; position: simulator ground truth
- plant: robot joint (simulated); setpoint sequence (mrad), each held 5 s: 600 -600 300 -300 0
- settling: last time outside a band of 10 % of the step (at least 5 mrad); a step still outside it in the last 0.5 s counts as not settled

| | AI pilot (guardian on) | fallback controller alone |
|---|---|---|
| mean settling time | 0.63 s | 0.84 s |
| worst overshoot | 0.4 mrad | 0.3 mrad |
| RMS tracking error | 164.7 mrad | 177.6 mrad |
| guardian interventions | 0 | 0 |
| steps not settled | 0 of 5 | 0 of 5 |

| step | setpoint | AI settling | AI overshoot | fallback settling | fallback overshoot |
|---|---|---|---|---|---|
| 1 | +600 mrad | 0.61 s | 0.4 mrad | 0.84 s | 0.3 mrad |
| 2 | -600 mrad | 0.66 s | 0.3 mrad | 0.85 s | 0.3 mrad |
| 3 | +300 mrad | 0.62 s | 0.3 mrad | 0.84 s | 0.1 mrad |
| 4 | -300 mrad | 0.62 s | 0.2 mrad | 0.84 s | 0.3 mrad |
| 5 | +0 mrad | 0.63 s | 0.3 mrad | 0.83 s | 0.3 mrad |

AI vs fallback: settling faster by 25 %, RMS error lower by 7 %, 0 interventions while the AI flew.
