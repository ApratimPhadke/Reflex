#!/usr/bin/env python3
"""Turn web/dashboard.html into a C header so rxdash needs no files at run time."""
import sys

src, dst = sys.argv[1], sys.argv[2]
data = open(src, "rb").read()
with open(dst, "w") as f:
    f.write("/* GENERATED from web/dashboard.html by tools/embed.py - do not edit */\n")
    f.write("static const unsigned char dash_html[] = {\n")
    for i in range(0, len(data), 20):
        f.write("  " + ",".join(str(b) for b in data[i:i + 20]) + ",\n")
    f.write("};\n")
    f.write(f"static const unsigned long dash_html_len = {len(data)};\n")
