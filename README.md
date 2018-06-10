Cyclone Mecanum
===============
サイクロンメカナム

![Cyclone Mecanum](docs/cyclone_mecanum.jpg)

Known Bug: No GP chip inside

This package contain 2 nodes: `mecanum_command` and `mecanum_umd`.

## mecanum_command
This node reads several commands and take corrosponding actions.

### Topics & Services
- `/elevator_cont`
	- message: `cyclone_mecanum/ElevatorCont.h`
	    - int32 elevator_id
	    - int16 vel_setpoint, v_setpoint passing to motor of elevator (around 140 is ok)
- `/em_cont`
	- message: `cyclone_mecanum/EmCont.h`
	    - int32 em_id
	    - int16 release, true to release the electromagnetic, don't provide power for too long
- `/disk_spin`
	- service: `std_srvs/SetBool.h`
	    - bool data, true to spin clockwise
- `/elevator_oneshot`
	- service: `cyclone_mecanum/ElevatorOneshot.h`
	    - int32 elevator_id
	    - bool up
	- NOT implemented!

### Parameters
- `disk_spin_angle` (default: 36)
	- The Angle of disk spin once `/disk_spin` is called
- `light_gate_IO_port` (default: {1, 2})
- `em_IO_port`(default: {3})
	- The ports plugged to IO board

### Runtime Dependencies
- `mecanum_umd`
- `stepper_control` from package mechaduino_stepper

## mecanum_umd
This node is copied from the package `mecanum_top`, which handles the UMD boards.

### Parameters
- `umd_port` (default: /dev/ttyACM1)
- `umd_baudrate` (default: 921600)









# Outdated Information

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
