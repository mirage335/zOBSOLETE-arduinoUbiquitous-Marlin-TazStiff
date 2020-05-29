

# WARNING: Board type may not be correctly set in arduinoide .
# WARNING: May not be appropriate for all relevant projects. Intended as sane defaults ONLY.
_declare_arduino_device_mega2560
_declare_arduino_installation_1.8.5


#Enable search if "vm.img" and related files are missing.
export ubVirtImageLocal="false"

# # ATTENTION: Add to ops!
_refresh_anchors_arduino_rewrite() {
	cp -a "$scriptAbsoluteFolder"/_anchor "$scriptAbsoluteFolder"/_rewrite_arduinoide
}


_rewrite_arduinoide() {
	find . -maxdepth 1 -type f -name "*.h" -exec sed -i 's/utility\/u8g.h/clib\/u8g.h/g' {} \;
	find . -maxdepth 1 -type f -name "*.cpp" -exec sed -i 's/utility\/u8g.h/clib\/u8g.h/g' {} \;

	find . -maxdepth 1 -type f -name "*SdBaseFile.h" -exec sed -i 's/fpos_t/fpos_t1/g' {} \;
	find . -maxdepth 1 -type f -name "*SdBaseFile.cpp" -exec sed -i 's/fpos_t/fpos_t1/g' {} \;
}




_task_upload_legacy() {
	_messagePlain_nominal 'Upload.'
	
	export au_arduinoFirmware_hex="$scriptLib"/Marlin-TazStiff/binary/Marlin.ino.hex
	
	! _check_arduino_firmware_hex && _messagePlain_bad 'fail: missing: binary' && return 1
	
	_arduino_upload_serial_avrdude_mega2560
}


_refresh_anchors_task() {
	cp -a "$scriptAbsoluteFolder"/_anchor "$scriptAbsoluteFolder"/_task_upload_legacy
}
