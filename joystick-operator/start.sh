#!/bin/bash

# The target of /dev/input/umrt-joystick (/dev/input/eventXX) is substituted into --device to get around an issue with
#     SDL not recognizing device symlinks for some reason
docker run \
  --rm \
  -it \
  --name umrt-arm-controller \
  -v /usr/local/bin/arm-firmware:/releases/arm-firmware \
  --net=host \
  --device `readlink -f /dev/input/arm_joystick` \
  --entrypoint /bin/bash \
  umrt-arm-image \
  -c "cd /releases/arm-firmware/joystick-operator && source install/setup.bash && ros2 launch joystick_operator joystick.launch.py"
