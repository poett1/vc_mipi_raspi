#!/bin/bash
# vblank_margin_check.sh -- is the IMX900 vertical blanking floor large enough?
#
# The sensor test pattern cannot answer this (it bypasses the pixel array). The oracle is a
# static scene at fixed exposure and gain: a too-small VBLANK truncates the frame (zero rows at
# the bottom) and drops the whole-frame level. This captures a slow reference frame and one
# frame per requested margin, then counts zero rows (the truncation signal; the level columns
# are informational, they pick up mains-flicker banding at short exposures).
#
#   usage: vblank_margin_check.sh <bitdepth> <exposure_us> <margin_lines>...
#   e.g.   vblank_margin_check.sh 8 2000 220 235 258
#
# Margins below the driver's minimum cannot be probed (libcamera clamps to the mode minimum);
# the requested frame duration is line_time * (1536 + margin) with the Pi 5 padded line time
# (5.39 us for 8/10-bit, 8.215 us for 12-bit). Run inside the libcamera devenv or with the
# installed prefix's pkg-config on PKG_CONFIG_PATH; the mode-test binary is built if missing.

set -euo pipefail

BD=$1; EXP=$2; shift 2
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=${OUT:-/tmp/vblank_margin_check}
mkdir -p "$OUT"
BIN=$OUT/libcamera_mode_test
if [ ! -x "$BIN" ]; then
  g++ -std=c++20 -O1 "$HERE/libcamera_mode_test.cpp" -o "$BIN" $(pkg-config --cflags --libs libcamera)
fi

case $BD in
  8|10) LINE_US=5.3895 ;;
  12)   LINE_US=8.2155 ;;
  *) echo "bitdepth must be 8, 10 or 12" >&2; exit 1 ;;
esac

run() { # name frame_duration_us
  "$BIN" "$BD" 60 "$2" "$OUT/$1.pgm" "$EXP" 2>&1 | grep -E 'frames|ERROR' | grep -v DeviceEnumerator | sed "s/^/  $1: /"
}

echo "reference: 30 fps"
run ref 33333
for M in "$@"; do
  FD=$(python3 -c "import math; print(math.ceil($LINE_US*(1536+$M)))")
  echo "margin $M lines -> request $FD us"
  run "m$M" "$FD"
done

python3 - "$OUT" ref "$@" <<'PY'
import sys, numpy as np
out, ref, *margins = sys.argv[1:]
def load(n):
    d = open(f"{out}/{n}.pgm", "rb").read()
    hdr = b"P5\n2048 1536\n65535\n"
    return np.frombuffer(d[len(hdr):], dtype=">u2").reshape(1536, 2048).astype(float)
r = load(ref); rmean = r.mean()
def tail(a):  # last 32 rows relative to a band well above them; truncation attenuates or zeroes the tail
    return a[1504:1536].mean() / a[1408:1472].mean()
rtail = tail(r)
print(f"{'frame':>8} {'mean':>8} {'vs ref':>8} {'tail32':>7} {'zero rows':>10} {'first zero row':>15}")
print(f"{ref:>8} {rmean:8.0f} {'1.000':>8} {'1.000':>7} {(r.mean(axis=1)==0).sum():>10} {'-':>15}")
for m in margins:
    a = load(f"m{m}"); rows = a.mean(axis=1); z = np.where(rows == 0)[0]
    t = tail(a) / rtail
    # Truncation shows as zero rows or as an attenuated tail (a step at a fixed row, e.g. 76 %
    # from row 1504 on). Whole-frame level is informational: short exposures under mains
    # lighting carry a few percent of flicker banding.
    verdict = "OK" if len(z) == 0 and abs(t - 1) < 0.03 else "TRUNCATED"
    print(f"{'m'+m:>8} {a.mean():8.0f} {a.mean()/rmean:8.3f} {t:7.3f} {len(z):>10} {z[0] if len(z) else '-':>15}  {verdict}")
PY
