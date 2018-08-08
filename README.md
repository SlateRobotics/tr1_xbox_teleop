## Introduction
The `tr1_xbox_teleop` package provides a simple program for controlling the TR1 using an XBOX controller. This guide tells you everything you need to know to get started!

## Basic hardware needs
In order for you to be able to use this package, you must first have a wireless XBOX controller as well as a wireless USB receiver for the controller. Below are the exact links for the products that we use.
 - [XBOX Controller](https://www.amazon.com/Microsoft-Wireless-Controller-Windows-Console/dp/B004QRKWKQ/ref=pd_sim_147_6?_encoding=UTF8&pd_rd_i=B004QRKWKQ&pd_rd_r=65e69851-9b2b-11e8-bbda-63260424ec2e&pd_rd_w=p1ljm&pd_rd_wg=C96Kf&pf_rd_i=desktop-dp-sims&pf_rd_m=ATVPDKIKX0DER&pf_rd_p=eb8198c1-8248-4314-940d-f60f1fec7e75&pf_rd_r=6HK7EMBK47GNRKYW44GG&pf_rd_s=desktop-dp-sims&pf_rd_t=40701&psc=1&refRID=6HK7EMBK47GNRKYW44GG)
 - [XBOX USB Receiver](https://www.amazon.com/Microsoft-Authentic-Wireless-Receiver-Windows/dp/B00FAS1WDG/ref=pd_bxgy_147_2?_encoding=UTF8&pd_rd_i=B00FAS1WDG&pd_rd_r=7f20b9ae-9b2b-11e8-af6e-0720243a5fcc&pd_rd_w=Za2U2&pd_rd_wg=40bau&pf_rd_i=desktop-dp-sims&pf_rd_m=ATVPDKIKX0DER&pf_rd_p=7ca3846a-7fcf-4568-9727-1bc2d7b4d5e0&pf_rd_r=8FM2AY0MQJKGGVR5VZET&pf_rd_s=desktop-dp-sims&pf_rd_t=40701&psc=1&refRID=8FM2AY0MQJKGGVR5VZET)

## Installation
There are two basic, non-TR1 dependencies required to get tr1_xbox_teleop to work on your machine: `joy` and `xboxdrv`. `joy` is a ROS package that contains `joy_node`, a node that interfaces a generic Linux joystick to ROS. `xboxdrv` is the linux driver required to read the joystick data from the USB receiver.
 - joy package. Instructions for installing joy can be found at [http://wiki.ros.org/joy](http://wiki.ros.org/joy).
 - xboxdrv. Install with `sudo apt-get install xboxdrv`. Git repository found at [https://gitlab.com/xboxdrv/xboxdrv](https://gitlab.com/xboxdrv/xboxdrv).

Once you have these installed, you will need to install the TR1-specific dependencies if you have not done so already. First, start by `cd`-ing into your `src/` folder within your ROS workspace.
```bash
cd ~/ros_ws/src
git clone https://github.com/slaterobotics/tr1_description
git clone https://github.com/slaterobotics/tr1_hardware_interface
git clone https://github.com/slaterobotics/tr1cpp
git clone https://github.com/slaterobotics/tr1_xbox_teleop
```

Next, `cd` back into the root ROS workspace folder, and `catkin_make` in order to install.
```bash
cd ..
catkin_make
source devel/setup.bash
```

You should now be ready to go!

## Running the node
Before you can startup the tr1_xbox_teleop node, you must now open up the terminal and start the XBOX drvier.
```bash
sudo xboxdrv
```

Next, turn on your XBOX controller and, in a second terminal tab, launch the ROS package
```bash
roslaunch tr1_xbox_teleop tr1_xbox_teleop.launch
```

## Controls
Now that the node is properly running, you are ready to start controlling your robot. Press select to cycle through the basic control settings (Base, Left Arm, Head, Right Arm). Each joystick, axis, and button controls a different joint depending on the mode selected.

Base Control Mode:
 - Left joystick controls translation across the ground plan.
 - Right joystick controls rotation

Right/Left Arm Control Modes:
 - Left joystick left/right: Shoulder Pan
 - Left joystick up/down: Shoulder Tilt
 - Right joystick left/right: Upper Arm Roll
 - Right joystick up/down: Elbow Flex
 - D-pad left/right: Forearm Roll
 - D-pad up/down: Wrist Flex
 - Button A/ Button B: Wrist Roll
 - Button X/ Button Y: Gripper

Head Control Mode:
 - Left joystick left/right: Head Pan
 - Left joystick up/down: Head Tilt

Happy hacking!

Team @ Slate Robotics
