call setupEnv.bat
adb shell su -mm -c magic_remount_rw

adb push %HEXAGON_SDK_ROOT%\tools\elfsigner\getserial\CDSP\android_Release_aarch64\getserial /sdcard/getserial
adb shell su -c cp /sdcard/getserial /vendor/bin/getserial
adb shell su -c chmod 777 /vendor/bin/getserial
for /f %%A in ('python get_serial.py') do set "SERIAL=%%A"
echo SERIAL=%SERIAL%
python %HEXAGON_SDK_ROOT%/tools/elfsigner/elfsigner.py -t %SERIAL% -o ./testsigs --no_disclaimer
adb push testsigs/ /sdcard/
adb wait-for-device shell su -c mkdir /vendor/lib/rfsa/adsp_testsig_backup
adb wait-for-device shell su -c cp /sdcard/testsigs/testsig-* /vendor/lib/rfsa/adsp_testsig_backup/
adb wait-for-device shell su -c chmod 777 /vendor/lib/rfsa/adsp_testsig_backup/testsig-*
adb wait-for-device push .\trinity.txt /sdcard/trinity.txt
adb shell su -c cp /sdcard/trinity.txt /vendor/etc/qvr/cfg/356/1/65536/trinity.txt
adb reboot
