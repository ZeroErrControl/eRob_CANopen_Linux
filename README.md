# eRob_CANopen_Linux

```bash

mkdir erob_project
cd erob_project
colcon build
source ./install/setup.bash
```

After the compilation is complete, replace the `master.dcf` file in the src directory with the master.dcf file located at `install/canopen_tests/share/canopen_tests/config/cia_system/master.dcf`.
Then, run the following commands:

```bash
ros2 launch canopen_tests cia402_system.launch.py
```
Followed by these CANopen commands:
```bash
cansend can0 602#2B40600080000000
cansend can0 602#2B40600006000000
cansend can0 602#2B40600007000000
cansend can0 602#2B4060000F000000
```

If an error like the following occurs:


```bash
[cia402_slave_node-4] [INFO] [1732698978.901164570] [cia402_slave]: Received Quick Stop.
[cia402_slave_node-4] [INFO] [1732698978.901249863] [cia402_slave]: Quick_Stop_Active
```
Please check if the speed error of the eRob motor exceeds the allowable limit.
