#!/bin/sh
# Build liborbslam3_skel.so for the SM8250 / XR2 CDSP (Hexagon v66, HVX 128 B)
# without the proprietary CVP "HWA" library (see src/orbslam_dsp.cpp).
#
#   dsp/build.sh [out dir]          (default: dsp/build)
#
# The Hexagon SDK that targets v66 (5.x, what Lynx used) is not public any more;
# SDK 6.x starts at v68 (no v66 build files, QuRT or runtime) and ships neither
# QuRT headers nor qaic. So:
#   - compiler/linker and the v66 G0/pic runtime: Hexagon tools 8.4.x
#     (HEXAGON_TOOLS, e.g. ~/Qualcomm/HEXAGON_Tools/8.4.06/Tools);
#   - HAP/AEE headers: SDK 6.7 incs/ (HEXAGON_SDK); HAP_compute_res and FARF are
#     weak-linked or resolved from the DSP's fastrpc shell at load time;
#   - no QuRT: the extractor runs both eyes on the FastRPC thread
#     (ORBSLAM_DSP_SINGLE_THREAD, ~5.1 M cycles per stereo frame in hexagon-sim);
#   - no qaic: the skel (src/orbslam3_skel_hrpc.c) and the CPU stub
#     (app/.../hexagonrpc_client.c) are hand-written against
#     app/src/main/cpp/inc/orbslam3_rpc.h;
#   - no DT_NEEDED (-nostdlib): the DSP has no libc.so/libgcc.so to load, every
#     symbol must come from fastrpc_shell_unsigned_3, which exports the
#     allocator only as __wrap_malloc/free/memalign/calloc/realloc.
set -eu
here=$(cd "$(dirname "$0")" && pwd)
out=${1:-$here/build}
tools=${HEXAGON_TOOLS:-$HOME/Qualcomm/HEXAGON_Tools/8.4.06/Tools}
sdk=${HEXAGON_SDK:-$HOME/Qualcomm/Hexagon_SDK/6.7.0.0}
[ -f "$sdk/incs/HAP_farf.h" ] || { echo "no SDK headers in $sdk/incs" >&2; exit 1; }

mkdir -p "$out"
CC="$tools/bin/hexagon-clang"
CXX="$tools/bin/hexagon-clang++"
flags="-mv66 -mhvx -mhvx-length=128B -G0 -fpic -O2 -fno-zero-initialized-in-bss -fdata-sections -ffunction-sections
	-DORBSLAM_DSP_SINGLE_THREAD -Dmalloc=__wrap_malloc -Dfree=__wrap_free -Dmemalign=__wrap_memalign
	-Dcalloc=__wrap_calloc -Drealloc=__wrap_realloc -I$here/include -I$here/src -I$sdk/incs -I$sdk/incs/stddef
	-Wno-unused-variable -Wno-unknown-warning-option -Wno-format"
"$CC" $flags -c "$here/src/orbslam3_skel_hrpc.c" -o "$out/orbslam3_skel_hrpc.o"
objs="$out/orbslam3_skel_hrpc.o"
for f in orbslam_dsp orbslam_dsp_pyramid orbslam_dsp_fast orbslam_dsp_sort orbslam_dsp_rb orbslam_dsp_matcher; do
	"$CXX" $flags -std=c++14 -fno-exceptions -fno-rtti -c "$here/src/$f.cpp" -o "$out/$f.o"
	objs="$objs $out/$f.o"
done
"$CC" -mv66 -G0 -fpic -Wl,-Bsymbolic -Wl,-L"$tools/target/hexagon/lib/v66/G0/pic" -Wl,--no-threads \
	-Wl,--build-id -shared -o "$out/liborbslam3_skel.so" -Wl,-soname,liborbslam3_skel.so \
	-nostdlib -Wl,--start-group $objs -Wl,--end-group
echo "built $out/liborbslam3_skel.so"
echo "undefined symbols (resolved by the DSP's fastrpc shell):"
"$tools/bin/hexagon-nm" -u "$out/liborbslam3_skel.so" | awk '{print $2}' | tr '\n' ' '
echo
echo "DT_NEEDED:"
"$tools/bin/hexagon-llvm-readelf" -d "$out/liborbslam3_skel.so" | grep NEEDED || true
