#!/bin/sh
# Record the DSP extractor's output (hexagon-sim) for every frame of a EuRoC-layout
# stereo sequence, for linux/lynx_stereo with LYNX_REPLAY_DIR (whole SLAM on DSP output).
#
#   run_sequence.sh <mav0 dir> <out dir> [parallel jobs]
#
# Needs build-host/orb_dump (places the PNG pair in the 640x400 eyes exactly as
# lynx_stereo does) and orbslam_sim (make). OpenCV and ncurses5 library paths
# via ORB_DUMP_LIBS / SIM_LIBS.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
seq=$1
out=$2
jobs=${3:-4}
dump=${ORB_DUMP:-$here/../../build-host/orb_dump}
sim=${HEXAGON_SIM:-$HOME/Qualcomm/HEXAGON_Tools/8.4.06/Tools/bin/hexagon-sim}
mkdir -p "$out"
export here seq out dump sim
ls "$seq/cam0/data" | grep '\.png$' | xargs -P "$jobs" -I{} sh -c '
	name={}; ts=${name%.png}
	[ -s "$out/$ts.orbd" ] && exit 0
	raw=$(mktemp "${TMPDIR:-/tmp}/orbsim.XXXXXX")
	LD_LIBRARY_PATH=${ORB_DUMP_LIBS:-} "$dump" "$seq/cam0/data/$name" "$seq/cam1/data/$name" "$raw" /dev/null >/dev/null
	LD_LIBRARY_PATH=${SIM_LIBS:-} "$sim" --mv66g_1024 --quiet "$here/orbslam_sim" -- "$raw" "$out/$ts.orbd" 1 2>/dev/null | grep pcycles | sed "s/^/$ts /" >> "$out/cycles.txt"
	rm -f "$raw"
'
echo "$(ls "$out" | grep -c orbd) frames in $out"
