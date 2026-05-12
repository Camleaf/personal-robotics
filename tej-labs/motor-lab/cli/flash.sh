arduino-cli compile --fqbn arduino:avr:mega --build-path ./build
arduino-cli upload -p /dev/ttyACM0 --fqbn arduino:avr:mega --input-dir ./build
