# 2026-11-22: second plant, simulated robot joint (Linux host)

The same guardian, campaign and benchmark on `--plant joint`: one joint lifting a link against gravity
(I = 0.20 kg m^2, mgl = 1.5 N m, torque limit 4 N m, hard stops +-1.2 rad, keep-out 0.05 rad). Parameters
are chosen (`rx_cfg_joint`), not measured on a real joint.

| File | What | Command |
|---|---|---|
| `campaign.md/.log` | joint kept off its stops in **8/8** faults with the guardian; **limit hit in 3/8** without (40 N m command, stuck at full torque, sign-inverted); closest approach 25.4 deg from a stop (stuck) | `reflex-guardian --plant joint --fresh &; rxctl reset; rxctl campaign campaign.md` |
| `bench.md/.csv/.png` | setpoints +600, -600, +300, -300, 0 mrad: AI 0.63 s vs fallback 0.84 s mean settling (-25 %), RMS 164.6 vs 177.6 mrad (-7 %), 0 interventions, all steps settled | `rxctl bench bench.md --csv bench.csv` |
| `status-*.txt`, `guardian.log`, `env.txt` | status before and after, host and git | |

`make test` (`rx_simtest`, joint section) adds every fault at 1 kHz and 250 Hz and a Monte Carlo envelope test
on the joint: 0 violations for the nominal joint, inertia -15 %/+15 %, gravity load +15 %, torque rise 20 % slower,
and all three combined (informational).
