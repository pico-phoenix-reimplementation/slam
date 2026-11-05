#!/usr/bin/env python3
import subprocess, re, sys

p = subprocess.run(
    ["adb", "shell", "su", "-c", "/vendor/bin/getserial"],
    capture_output=True, text=True
)
out = p.stdout + "\n" + p.stderr

m = re.search(r"0x[0-9A-Fa-f]+", out)
if not m:
    print("no 0x… pattern found", file=sys.stderr)
    sys.exit(1)

print(m.group(0))