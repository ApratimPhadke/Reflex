# Codelab: build a real-time safety guardian with QNX message passing

**Time:** about 30 minutes. **You need:** QNX SDP 8.0 on a Linux host (`source ~/qnx800/qnxsdp-env.sh`) and a
QNX 8.0 target (a Raspberry Pi 4 with the quick-start image is enough). **You build:** `guardian_lab.c`, one
small program that contains the whole pattern Reflex uses to let an untrusted AI drive a machine.

The problem: an untrusted process (the "pilot", for example a neural network) proposes actuator commands.
A trusted process (the "guardian") must run every control period on time, whatever the pilot does: hang,
crash, answer late, flood it, or be impersonated by someone else. On Linux you would reach for threads,
mutexes and timeouts. On QNX, six kernel features do it with one thread and no locks.

Every output below is the real output of this program on a Raspberry Pi 4 running QNX 8.0
(`results/2026-11-22-pi4-codelab/run.txt`); PIDs will differ on your target.

---

## Step 0. Build and run it once

```sh
source ~/qnx800/qnxsdp-env.sh
qcc -Vgcc_ntoaarch64le -std=gnu11 -Wall -Wextra -O2 -o guardian_lab docs/codelab/guardian_lab.c
scp guardian_lab qnxuser@<pi>:/tmp/
ssh qnxuser@<pi> /tmp/guardian_lab
```

The program is its own pilot: the guardian starts a well-behaved pilot at once and a spoofer at tick 50.
Read the steps below against the code; each one is marked `Step N` in `guardian_lab.c`.

Tip: do not call your message type `msg_t`. QNX already defines `msg_t` in `<sys/types.h>`, and the compiler
error is confusing (`conflicting types for 'msg_t'`).

## Step 1. A channel with fixed priority and disconnect notification

```c
int chid = ChannelCreate(_NTO_CHF_FIXED_PRIORITY | _NTO_CHF_DISCONNECT);
```

A channel is where messages and pulses arrive. Two flags matter:

- `_NTO_CHF_FIXED_PRIORITY`: by default a QNX server **inherits the priority of the client** whose message it
  is handling (good for client/server fairness, bad here). A priority-10 pilot would drag the priority-30
  guardian down to 10 while it handles the message. With the flag the guardian keeps its own priority.
- `_NTO_CHF_DISCONNECT`: when a client process goes away, the kernel sends a pulse (step 6).

**Try it.** Run once normally and once with `--no-fixed`:

```
$ ./guardian_lab
[guardian] handling a message from a priority-10 client, the guardian runs at priority 30
$ ./guardian_lab --no-fixed
[guardian] handling a message from a priority-10 client, the guardian runs at priority 10
```

In Reflex: `rx_server_create()` in `src/ipc/rx_ipc_qnx.c`.

## Step 2. A timer that delivers pulses to the same channel

```c
int self = ConnectAttach(ND_LOCAL_NODE, 0, chid, _NTO_SIDE_CHANNEL, 0);
struct sigevent ev;
SIGEV_PULSE_INIT(&ev, self, SERVER_PRIO, PULSE_TICK, 0);
timer_create(CLOCK_MONOTONIC, &ev, &tid);
timer_settime(tid, 0, &its, NULL);            /* every 10 ms */
```

The timer does not call a handler or raise a signal: it puts a small **pulse** (a non-blocking message with a
code) on the channel, at the priority you choose. The control period now arrives through the same door as
everything else.

```
[guardian pid 4001836] channel 1 (_NTO_CHF_FIXED_PRIORITY), timer pulse every 10 ms, running at priority 30
```

In Reflex the timer is armed with `TIMER_ABSTIME` on the clock tick boundary so the period does not drift.

## Step 3. One blocking call receives everything

```c
int rcvid = MsgReceive(chid, &m, sizeof m, &info);
if (rcvid == 0)  { /* a pulse: the tick, or a disconnect */ }
else             { /* a message from a client */ }
```

This is the guardian's **only** blocking call. It wakes for the timer pulse or for a pilot message, whichever
comes first, so there is no second thread, no mutex and no condition variable to get wrong. In Reflex this is
`rx_server_wait()`, and invariant 1 in `CLAUDE.md` forbids any other blocking call in the control loop.

## Step 4. Reply on the tick: the client is paced by the server

```c
pending = rcvid;                              /* message received: do not reply yet */
...
if (m.pulse.code == PULSE_TICK && pending) {  /* next tick: now reply with fresh state */
    MsgReply(pending, 0, &st, sizeof st); pending = 0;
}
```

`MsgSend()` blocks the **client** until the server replies; `MsgReply()` never blocks the server. By holding
the reply until the next tick, the guardian decides when the pilot runs: once per period, with the newest
state. A slow or stuck pilot only delays itself; the guardian never waits for it.

```
[pilot   pid 4005934] sent 150 proposals, last reply was tick 150; exiting
```

150 proposals, last reply at tick 150: exactly one exchange per tick.

## Step 5. The kernel says who sent the message

```c
if (info.pid != pilot) { MsgError(rcvid, EPERM); refused++; continue; }
```

`_msg_info.pid` is filled in by the kernel, so a process cannot claim to be someone else. The guardian only
accepts proposals from the pilot it started; anyone else, even a process that found the channel id, gets
`EPERM`.

```
[guardian] tick  50: message from pid 4005937 refused (only pid 4005934 may propose)
[spoofer pid 4005937] MsgSend refused: Operation not permitted
```

In Reflex this shows up as "spoofed sender refused" in the event log and in `rxctl status`.

## Step 6. The kernel says when a client dies

```c
} else if (m.pulse.code == _PULSE_CODE_DISCONNECT) {
    if (m.pulse.scoid == pilot_scoid) { /* the pilot is gone: fall back */ }
    ConnectDetach(m.pulse.scoid);
}
```

When any connected client goes away (exit, crash, `kill -9`), the kernel delivers a disconnect pulse with that
client's server connection id. **Check which client it was.** The first version of this program treated every
disconnect as the pilot's, so the spoofer's exit made the guardian give up on a healthy pilot. Comparing
`scoid` with the pilot's (remembered from `info.scoid` of its messages) fixes that:

```
[guardian] tick  50: disconnect pulse from another client (scoid 65539) - ignored
[guardian] tick 150: disconnect pulse from the pilot - fall back to the safe controller
[guardian] done after 201 ticks: 150 proposals accepted, 1 refused
```

Reflex does the same in `rx_server_wait()` (`pending_scoid`), and its supervisor also reaps the pilot with
`waitpid()`, so a death is noticed even when no message was pending.

---

## What you have built

| Requirement | Kernel feature | Step |
|---|---|---|
| guardian keeps its priority whatever the pilot's | `_NTO_CHF_FIXED_PRIORITY` | 1 |
| on-time control period without signals or extra threads | timer + `SIGEV_PULSE_INIT` | 2 |
| one thread, one blocking call, no locks | `MsgReceive` on one channel | 3 |
| pilot paced by the guardian, never the reverse | `MsgSend` blocks the client; reply on the tick | 4 |
| no impersonation | `_msg_info.pid`, `MsgError(EPERM)` | 5 |
| instant notice of a dead pilot | `_NTO_CHF_DISCONNECT`, `scoid` check | 6 |

The full guardian (`src/guardian/main.c`, `src/ipc/rx_ipc_qnx.c`) adds the safety decision itself (a one-step
lookahead against a recoverable-set bound, `src/core/rx_core.c`), a fallback controller, supervision and
restart of the pilot, shared-memory telemetry, a resource manager (`/dev/reflex`) and HAM restart of the
guardian itself. See `docs/ARCHITECTURE.md`.

## Exercises

1. Make the pilot hang: add `if (i == 80) for (;;);` in `client()`. The guardian keeps ticking; what should it
   do when no proposal arrives for several periods? (Reflex: the `stale` verdict.)
2. Run the pilot at priority 40 instead of 10, with and without `--no-fixed`. What happens to the guardian's
   priority in each case?
3. Kill the pilot from another shell (`slay -s KILL guardian_lab` kills all three; use the pilot's pid) and
   watch for the disconnect pulse.
