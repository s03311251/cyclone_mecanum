Cyclone Mecanum
===============
サイクロンメカナム

![Cyclone Mecanum](docs/cyclone_mecanum.jpg)

## Copy From
`include/devices/PS4.*` is copied from [omni_control](https://github.com/lycpaul/omni_control)

## Dependencies
`umd_boards_ws/mecanum_top` from [umd_boards_ws](https://github.com/gaudat/umd_boards_ws) in branch wahaha

## Startup
1. `roscore`
2. `sudo ds4drv --hidraw --led 00ff00`
3. (according to the output from 2)
   `rosparam set joy_node/dev "/dev/input/js1"`
4. `rosrun joy joy_node`
5. (optional)
   `rostopic echo /joy`
6. `rosrun mecanum_top node /dev/ttyACM0`
7. `rosrun cyclone_mecanum mecanum_command`
