# robot_monitoring

## Usage
The Xbot2 GUI is implemented as a pair of processes, i.e., a server (backend) and a client (UI, frontend). 

The server is written in python and can easily pip-installed inside the preferred python environment. 

The client is written in Qt 6.8.0, and it is more convenient to download the pre-built binary from the [releases](https://github.com/ADVRHumanoids/robot_monitoring/releases) page.

The `xbot2_gui` executable takes care to download the client if needed, and to run both the client and the server.

```
cd server
pip install .
xbot2_gui
```
