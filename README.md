Cyclone Mecanum
===============
サイクロンメ

![Cyclone Mecanum](docs/cyclone_mecanum.jpg)

## Copy From
`include/devices/PS4.*` is copied from [omni_control](https://github.com/lycpaul/omni_control)

Slighty different as the constructor as ask for the topic name from joy node

## Dependencies
`umd_boards_ws/mecanum_top` from [umd_boards_ws](https://github.com/gaudat/umd_boards_ws) in branch wahaha

## Startup

### Standalone
1. `roscore`
2. `sudo ds4drv --hidraw --led 00ff00`
3. (according to the output from 2) </br>
   `rosparam set joy_node/dev "/dev/input/js1"`
4. `rosrun joy joy_node`
5. (optional) </br>
   `rostopic echo /joy`
6. `rosrun mecanum_top node /dev/ttyACM0`
7. `rosparam set /mecanum_command/ps4_topic /joy`
8. `rosrun cyclone_mecanum mecanum_command`

### With omni_control
1. `roscore`
2. `sudo ds4drv --hidraw --led 00ff00`
3. (according to the output from 2) </br>
   `roslaunch omni_control mecanum_controller.launch joy:=/dev/input/jsX umd:=/dev/ttyACMX`
4. `rosrun mecanum_top node /dev/ttyACMX`
5. `rosparam set /mecanum_command/ps4_topic /omni_base/joy`
6. `rosrun cyclone_mecanum mecanum_command`

### Distinguish between /dev/ttyACMX
`udevadm info --name=/dev/ttyACMX`

UART Board `E:ID_MODEL=54FF70065185545318341887`

STM32F407Discovery `ID_MODEL=50002C001951353235373431`

## Control
|           |  Function Key                     | Precise Mode    |
| --------- | --------------------------------- | --------------- |
| Options   | Lift Both Rack                    | N/A             |
| Triangle  | Elevator Up                       | Move 20x slower |
| Cross     | Elevator Down                     | Move 20x slower |
| Rectangle | Rotate Rack anti-closewise by 18° | Rotate 1.8°     |
| Circle    | Rotate Rack closewise by 18°      | Rotate 1.8°     | 

|       | Modification Key |
| ----- | ---------------- |
| L1    | Golden Rack      |
| Share | Precise          |
