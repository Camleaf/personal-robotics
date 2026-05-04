arduino-cli compile --fqbn arduino:avr:uno --build-path ./build
arduino-cli upload -p /dev/ttyACM1 --fqbn arduino:avr:uno --input-dir ./build
