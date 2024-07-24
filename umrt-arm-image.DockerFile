FROM ros-humble-desktop

RUN sudo apt update && sudo apt install -y \
    ros-humble-ros2-control \
	ros-humble-ros2-controllers \
	gdb \
	gdbserver \
	libboost-all-dev \
    ros-humble-xacro \
    less \
    python3-curtsies \
    python-is-python3 \
 && rm -rf /var/lib/apt/lists/*