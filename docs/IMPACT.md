# Impact: who this protects, who would pay, and what a product would need

**Reflex is a prototype, not a certified product.** It runs on the standard QNX SDP 8.0 (non-commercial
licence), not on QNX's separately certified safety variant, and no part of it has been assessed against any
safety standard. What it shows is an architecture and evidence that the architecture works on a small plant.

## The problem it addresses

Learned controllers (neural networks, reinforcement-learning policies) often outperform hand-tuned ones, and
they keep improving with data. But nobody can say what a neural network will output in every situation, so
they are kept away from machines that move near people. Reflex puts a small, fixed, analysable guardian
between the learned controller and the actuator: every command is checked one control period ahead against a
bound on what the fallback controller can still recover from, and the fallback takes over before that bound
is crossed. In this repository the AI pilot settled 60 % faster than the fallback controller on the simulated
beam, and the guardian kept the plant safe in every injected fault (`results/2026-11-20-host-bench/`,
`results/2026-11-19-pi4-sim/`, `results/2026-11-22-pi4-joint/`).

## Machines it is meant for (robotics and medical first)

| Machine | What the learned controller would do | What the guardian would enforce |
|---|---|---|
| Lower-limb exoskeleton | adapt assistance torque to the wearer's gait | joint angle and velocity stay inside the range the brake/fallback can stop |
| Rehabilitation robot | personalise therapy trajectories | force and range limits that hold whatever the policy outputs |
| Surgical-assist arm | smoother, learned motion near tissue | no-go zones and stopping distance before them |
| Autonomous mobile robot | learned navigation in crowds | speed versus braking distance to the nearest obstacle |

The second plant in this repository, a simulated robot joint lifting a load against gravity between hard
stops (`--plant joint`), is a stand-in for the first two rows. Its parameters are chosen, not measured on a
real exoskeleton.

## Who would use it, and who would pay

- **Robot and medical-device makers** who want learned control in a product but must show a regulator or
  customer that a bad output cannot cause harm. They would pay for a supported guardian runtime, the tooling
  (campaign, benchmark, Monte Carlo envelope test, flight recorder) and help with certification evidence.
- **Research labs and integrators** who want to train controllers on real hardware without breaking it. The
  open-source version serves them; it is also how a product would earn trust.

A plausible model is open core: the kernel stays open, and a commercial edition adds a certified-RTOS port,
plant-specific bound derivations, qualification evidence and support. This is an idea, not a validated
business: no customer interviews or pricing work have been done.

## How it relates to safety standards (not certified)

- **IEC 61508** (functional safety of electrical/electronic/programmable systems): the guardian is shaped like
  a safety function, small, separated from the untrusted part, with defined failure behaviour (fallback, I/O
  node watchdog). Certification would need a hazard analysis, a safety requirements spec, a systematic
  development process and independent assessment, none of which exist here.
- **IEC 62304** (medical device software life cycle) and **ISO 14971** (medical risk management): relevant for
  the exoskeleton, rehabilitation and surgical cases. The learned pilot would be treated as untrusted (SOUP-like)
  software whose risk is controlled by the guardian; the guardian itself would need the full 62304 process.
- Robot-specific standards such as **ISO 13482** (personal care robots) and **ISO 10218** (industrial robots) would
  set the limits the guardian enforces.

## What a path to product would need

1. A plant model with stated uncertainty bounds, and a recoverable-set bound proved for them (today it is
   analytic and checked by Monte Carlo simulation and fault injection, not formally proved).
2. Independent, diagnosed sensing (the guardian trusts its sensor today) and a hardware-enforced last line
   (the I/O node's 150 ms watchdog is a start).
3. A certified RTOS (QNX offers a safety-certified variant) and worst-case timing analysis on the target.
4. The pilot isolated by a dedicated user and a QNX security policy, not only by interface.
5. A development process, traceability and independent assessment for the chosen standard.
