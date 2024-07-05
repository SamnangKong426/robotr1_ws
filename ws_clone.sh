cd ~
rm -rf ~/r1_ws
git clone https://github.com/SamnangKong426/robotr1_ws.git
cd ~/r1_ws
rm -rf build/ log/ install/

# colcon build
# source ~/r1_ws/install/setup.bash

# compile, 
arduino-cli compile --fqbn arduino:avr:mega src/r1/arduino_mega/

# upload
arduino-cli upload -p /dev/ttyUSB0 --fqbn arduino:avr:mega  src/r1/arduino_mega/

arduino-cli monitor -p /dev/ttyUSB0 --fqbn arduino:avr:mega --config baudrate=115200