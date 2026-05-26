arduino-cli compile --fqbn esp32-bluepad32:esp32:esp32 --build-path ./build
arduino-cli upload -p /dev/ttyUSB0 --fqbn esp32-bluepad32:esp32:esp32 --input-dir ./build
