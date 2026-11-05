#!/bin/sh
# 6DoF head tracking on the PICO 4: power and route the OV7251 tracking cameras
# (as pico4-camview.sh: CSIPHY5/4/3/2(proxy for the CSIPHY3 combo) -> CSID0-3 ->
# VFE0-3 RDI0, RAW10 640x481), start the Nordic MCU so they stream, start
# pico4-slam (ORB-SLAM3, features on the CDSP through hexagonrpcd) and run an
# OpenXR app under Monado (pico4-xr). pico4-slam sends head poses to the pico4
# driver's socket; orientation stays on the IMU, position comes from SLAM.
#
#   sudo pico4-slam.sh hello_xr -g Vulkan2 -ff Hmd -vc Stereo
#   sudo pico4-slam.sh                       # Monado service only, Ctrl-C to stop
#
# Environment (defaults in brackets):
#   PICO4_SLAM_BACKEND   cdsp | cpu [cdsp]; cpu runs the same binary without hexagonrpcd
#                        (no FastRPC fd: the hexagonrpc build falls back to the CPU extractor)
#   PICO4_SLAM_BIN       pico4-slam binary [/opt/pico4-slam/bin/pico4-slam]
#   PICO4_SLAM_VOC       [/opt/pico4-slam/share/ORBvoc.bin]
#   PICO4_SLAM_SETTINGS  [/var/lib/pico4/pico4-lynx.yaml, else /opt/pico4-slam/share/pico4-lynx.yaml]
#   CDSP_ROOT            hexagonrpcd root with dsp/cdsp (CDSP libraries + liborbslam3_skel.so)
#                        [/home/user/cdsprpc/root]
#   HEXAGONRPCD          patched hexagonrpcd (tools/fastrpc) [/home/user/cdsprpc/hexagonrpcd-v]
#   PICO_MCU_DIR         pico-mcu.py directory [/home/user/pico]
#   PICO4_SLAM_NO_XR     run only the tracker (no Monado/app) for this many seconds
#   PICO4_LED_OFF, PICO4_LED_THRESHOLD, PICO4_LED_DUMP: controller LED tracking (pico4_slam.cc)
#   PICO_MCU_EXPOSURE, PICO4_SLAM_GAIN, PICO4_SLAM_RECORD, PICO4_SLAM_TRAJ: see pico-mcu.py / pico4_slam.cc
# pico4-slam's log: journalctl -u pico4-slam
set -eu
app_args="$*"
backend=${PICO4_SLAM_BACKEND:-cdsp}
bin=${PICO4_SLAM_BIN:-/opt/pico4-slam/bin/pico4-slam}
voc=${PICO4_SLAM_VOC:-/opt/pico4-slam/share/ORBvoc.bin}
settings=${PICO4_SLAM_SETTINGS:-/var/lib/pico4/pico4-lynx.yaml}
[ -f "$settings" ] || settings=/opt/pico4-slam/share/pico4-lynx.yaml
cdsp_root=${CDSP_ROOT:-/home/user/cdsprpc/root}
hexagonrpcd=${HEXAGONRPCD:-/home/user/cdsprpc/hexagonrpcd-v}
mcu_dir=${PICO_MCU_DIR:-/home/user/pico}

# Cameras, same sequence as pico4-camview.sh.
echo 14 > /sys/module/qcom_camss/parameters/settle_cnt_override
echo 3 > /sys/module/qcom_camss/parameters/csiphy_combo
echo 2 > /sys/module/qcom_camss/parameters/csiphy_combo_proxy
modprobe pico-ov7251-mcu
D=/sys/bus/platform/drivers/pico-ov7251-mcu
for n in 0 1 2 3; do echo camera-tracking-$n > $D/unbind 2>/dev/null || true; done
echo 0 > /sys/devices/platform/mcu-vdd/state; sleep 0.1
echo 1 > /sys/devices/platform/mcu-vdd/state; sleep 1.2
for n in 0 1 2 3; do echo camera-tracking-$n > $D/bind; done; sleep 1

M="media-ctl -d /dev/media0"
F=Y10_1X10/640x481
$M -r
i=0
for pair in "5 tracking-0" "4 tracking-3" "3 tracking-1" "2 tracking-2"; do
	set -- $pair
	$M -l "\"msm_csiphy$1\":1->\"msm_csid$i\":0[1],\"msm_csid$i\":1->\"msm_vfe${i}_rdi0\":0[1]"
	$M -V "\"ov7251-mcu camera-$2\":0[fmt:$F],\"msm_csiphy$1\":0[fmt:$F],\"msm_csid$i\":0[fmt:$F],\"msm_vfe${i}_rdi0\":0[fmt:$F]"
	i=$((i + 1))
done
# vfe1 = CSIPHY4 = mav0 cam3 (bottom-left), vfe2 = CSIPHY3 = mav0 cam2 (bottom-right).

for u in pico4-slam pico-mcu; do
	systemctl stop $u 2>/dev/null || true
	systemctl reset-failed $u 2>/dev/null || true
done

# pico4-slam opens the V4L2 nodes and waits for frames; start it before the MCU.
slam_env="-E PICO4_SLAM_VOC=$voc -E PICO4_SLAM_SETTINGS=$settings"
# ORB-SLAM3 and hexagonrpcd -v print several lines per frame to stdout; through
# journald that costs CPU on the cores SLAM needs (lost tracking when anything
# else ran). Keep stderr (pico4-slam's status lines), run on the big cores
# (4-7) with priority.
slam_props="-p StandardOutput=null -p StandardError=journal -p Nice=-10 -p CPUAffinity=4-7"
for v in PICO4_SLAM_GAIN PICO4_SLAM_RECORD PICO4_SLAM_TRAJ PICO4_SLAM_HEAD PICO4_SLAM_VFE PICO4_SOCKET \
	PICO4_LED_OFF PICO4_LED_THRESHOLD PICO4_LED_DUMP PICO4_LED_LOG PICO4_LED_CPUS PICO4_CTRL_CALIB_DIR PICO4_EMBED_LOG; do
	eval "val=\${$v:-}"
	[ -n "$val" ] && slam_env="$slam_env -E $v=$val"
done
if [ "$backend" = cdsp ]; then
	# hexagonrpcd creates the unsigned CDSP PD, serves liborbslam3_skel.so from
	# $cdsp_root/dsp/cdsp and hands the FastRPC fd to pico4-slam (HEXAGONRPC_FD).
	systemd-run --unit pico4-slam $slam_props $slam_env -E PATH="$(dirname "$bin"):$PATH" \
		"$hexagonrpcd" -f /dev/fastrpc-cdsp -d cdsp -u \
		-c "$cdsp_root/dsp/cdsp/fastrpc_shell_unsigned_3" -R "$cdsp_root" -p "$(basename "$bin")" >/dev/null
else
	systemd-run --unit pico4-slam $slam_props $slam_env "$bin" >/dev/null
fi

(
	sleep 4
	systemd-run --unit pico-mcu -E PICO_MCU_NO_CYCLE=1 \
		${PICO_MCU_EXPOSURE:+-E PICO_MCU_EXPOSURE=$PICO_MCU_EXPOSURE} \
		${PICO4_CTRL_CALIB_DIR:+-E PICO4_CTRL_CALIB_DIR=$PICO4_CTRL_CALIB_DIR} \
		${PICO_MCU_NO_CALIB:+-E PICO_MCU_NO_CALIB=$PICO_MCU_NO_CALIB} \
		--working-directory="$mcu_dir" \
		sh -c "python3 -u pico-mcu.py start 86400 > $mcu_dir/mcu.log 2>&1" >/dev/null
) &

stop_all() {
	systemctl stop pico4-slam 2>/dev/null || true
	systemctl stop pico-mcu 2>/dev/null || true
}
trap stop_all EXIT
if [ -n "${PICO4_SLAM_NO_XR:-}" ]; then
	# Tracker only (no Monado), for PICO4_SLAM_NO_XR seconds: use with
	# PICO4_SLAM_TRAJ / PICO4_SLAM_RECORD to test the camera and DSP path.
	sleep "$PICO4_SLAM_NO_XR"
	exit 0
fi
# Same signal handling as pico4-camview.sh: pico4-xr stops monado-service.
set -- $app_args
# Keep stdin: sh gives background jobs /dev/null unless redirected explicitly
# (busybox ash ignores "0<&0", so go through fd 3), and hello_xr treats EOF on
# stdin as "press any key" and quits.
exec 3<&0
pico4-xr "$@" <&3 &
xr=$!
trap 'kill -INT $xr 2>/dev/null || true' INT TERM
wait $xr || wait $xr || true
