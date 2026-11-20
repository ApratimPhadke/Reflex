#!/bin/sh
# drop --hardware: fly the simulator weights (comparison only)
a=""; for x in "$@"; do [ "$x" = "--hardware" ] || a="$a $x"; done
exec /home/abc/A.I.M/hackathonproject/v4/reflex/build/linux/reflex-pilot $a
