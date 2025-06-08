#!/bin/bash

# Start and then close a Pyfirmata2 connection
# Don't ask me why, but if we don't do this after the Arduino is plugged in, then the Firmata connection code from
# openFrameworks doesn't work...
python -c "import pyfirmata2; pyfirmata2.Arduino('/dev/umrt-arm').exit()"

docker run \
  --rm \
  --name umrt-arm \
  --device=/dev/umrt-arm \
  -p 50000:22 \
  -v /etc/arm-firmware:/releases/arm-firmware \
  --entrypoint /bin/bash \
  umrt-arm-image \
  -c "cd /releases/arm-firmware/arm-ros-firmware && source install/setup.bash && ros2 launch umrt-arm-ros-firmware prairie_pioneer.launch.py"
