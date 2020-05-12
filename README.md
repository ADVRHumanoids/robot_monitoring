# robot_monitoring
Packages such as UI tools and loggers for robot monitoring

### compiling
Follow the usual cmake workflow, and install to a location visible to the ROS package system.
Compilation could fail due to a **dependency on Qt5 version >= 5.9** (it must include the `QtCharts`
component). Depending on your system, the default version could be earlier. In this case, you must
install a recent version of Qt5, and set the `QT5_PATH` cmake variable to the `gcc_64` subfolder of the
Qt install dir. On my system, this value is `/opt/qt512/5.12.3/gcc_64`.

### running
 - `XBotCore -D` (or gazebo equivalent)
 - `rosrun robot_monitoring joint_state_gui`
