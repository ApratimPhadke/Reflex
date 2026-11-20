#!/bin/sh
# Phase 3 step 4: 10 minutes of --learn-from-sim against the Black Pill emulator over /dev/serusb1.
cd ~/reflex
slay -f reflex-guardian 2>/dev/null; sleep 0.5
rm -f reflex-policy-serial.bin reflex-policy-serial.bin.tmp
echo "\$ ./reflex-guardian --backend serial --dev /dev/serusb1 --learn-from-sim --weights ./reflex-policy-serial.bin --fresh &"
nohup ./reflex-guardian --backend serial --dev /dev/serusb1 --learn-from-sim --weights ./reflex-policy-serial.bin --fresh > guardian-learn-serial.log 2>&1 < /dev/null &
sleep 2
echo "\$ ./rxctl period 3000"; ./rxctl period 3000
./rxctl reset
i=0
while [ $i -le 10 ]; do
  echo "=== t=$((i*60))s"
  ./rxctl status | grep -E "^mode|wake|interventions|no-pilot|stale|envelope|non-finite|range|^pilot|balls lost|learning"
  i=$((i+1)); [ $i -le 10 ] && sleep 60
done
echo "=== events"; ./rxctl events
echo "=== guardian-learn-serial.log"; cat guardian-learn-serial.log
echo "=== latency"; ./rxctl latency
slay -s INT reflex-guardian
echo DONE
