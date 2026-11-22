# 30-second social clip (Community Choice voting)

One idea only: an AI fails on purpose, the ball survives; the guardian is switched off, the ball falls.
Vertical 9:16 for phones. The captions come from `demo/social.rxs`, so they appear on the dashboard in the same
take. Record on the real beam if it exists; otherwise on the simulated plant, and say so on screen.

| Time | Shot | On-screen text (from `demo/social.rxs`) | Voice-over (optional, or none with music) |
|---|---|---|---|
| 0:00-0:03 | Close-up of the ball balancing; dashboard in presentation mode behind it, badge "AI IN CONTROL" | This AI is about to fail on purpose. | "This neural network is flying a real machine." |
| 0:03-0:09 | Flip injected. Badge turns green "GUARDIAN IN CONTROL"; the ball wobbles and stays | Guardian ON: the ball stays. | "We just made it output the exact wrong command. The guardian caught it before the ball could fall." |
| 0:09-0:15 | Guardian bypassed, same flip. The ball rolls off; the big "balls lost" number turns red | Guardian OFF: | "Same failure, guardian off." |
| 0:15-0:22 | Guardian back on, AI regains control; quick cut to `pidin` or `cat /dev/reflex` on the Pi | Reflex on QNX: AI you can let near a real machine. | "Reflex runs on QNX 8.0 on a Raspberry Pi. Every AI command is checked one control period ahead." |
| 0:22-0:30 | Wide shot of the rig; repo URL card | github.com/ApratimPhadke/Reflex | "Open source. Vote for Reflex." |

How to record:

```sh
./reflex-demo.sh start                    # on the Pi; open http://<pi>:8080/?present on the laptop
./rxctl script demo/social.rxs --check
./rxctl script demo/social.rxs            # start the phone when it says START RECORDING NOW
```

Dry run on the simulated plant (`results/2026-11-22-host-video/`): with the guardian on, the flipped AI was
overruled 5 times and the ball stayed; with it off, the ball fell 1 s later. Rules for the post: no copyrighted
music, no third-party logos, label it "simulated plant" if the real beam is not in the shot.
