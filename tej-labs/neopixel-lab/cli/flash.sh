arduino-cli compile --fqbn arduino:avr:uno --build-path ./build
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:uno --input-dir ./build
