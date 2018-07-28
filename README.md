# tr1_xbox_teleop
This is a repository for the `tr1_xbox_teleop` package, which provides a simple program for controlling the TR1 using an XBOX controller.

### Dependncies
This package will require that you install the following dependencies:
- `joy` package. Instructions for installing `joy` can be found at [http://wiki.ros.org/joy](http://wiki.ros.org/joy).
- `xboxdrv`. Install with `sudo apt-get install xboxdrv`. Repository found at [https://gitlab.com/xboxdrv/xboxdrv](https://gitlab.com/xboxdrv/xboxdrv).


### Controls
Start tr1_xbox_teleop with `roslaunch tr1_xbox_teleop tr1_xbox_teleop.launch`. Press select to cycle through the basic control settings (Base, Left Arm, Head, Right Arm). Each joystick, axis, and button controls a different joint depending on the mode selected.

Happy hacking!

Team @ Slate Robotics
