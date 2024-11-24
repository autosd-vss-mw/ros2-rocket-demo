# ROS2 Rocket Demo

Welcome to the Rocket Launch Simulator based on CentOS, AutoSD and ROS2!  
This project allows you to simulate rocket launches, including specific mission operations.
The simulation includes detailed stages, realistic delays, and even custom mission operations for a truly engaging experience!


![Curiosity Rover](https://d2pn8kiwq2w21t.cloudfront.net/original_images/spaceimagesimageslargesizePIA14156_hires.jpg)
<small>Launched on a NASA rocket (NASA Atlas V), Curiosity touched down on Mars and now roams the Red Planet, uncovering its ancient secrets.</small>

## Development Environment

We encourage to use toolbox with container images provided by the Fedora Robotics SIG:

```sh
$ toolbox create \
--image quay.io/fedora-sig-robotics/ros2:jazzy-cs9 \
ros2-jazzy-cs9-box
$ toolbox enter ros2-jazzy-cs9-box
```

This image leverages the pre-built ROS2 packages for RHEL while using a CentOS Stream 9 base container.

## Building

To build all packages run:

```sh
colcon build --symlink-install 
```

Using `--symlink-install` will create symbolic links for your project's source code
so changes can be tested without running `colcon build` everytime one is made.

## Running

### Generic Instructions

Run a ROS2 by specifying its package and node name:

```
ros2 run $pkg_name $node_name
```

### rhover_control

This section describes how to run nodes within the `rhover_control`.

#### rhover_control.engine_manager

This node subscribes to the `rhover_control_engine_erver` to handle engine operations from client requests.

Running the node:

```sh
ros2 run rhover_control engine_manager
```

You can now open another tab and run client requests using ros2.

Do not forget to  run `source install/setup.bash` for each new tab you open to run `ros2` commands.

##### Services/Methods

###### GetEngineState

Return the engine's current state.

```sh
$ ros2 service call /get_engine_state rhover_control_interfaces/srv/GetEngineState
requester: making request: rhover_control_interfaces.srv.GetEngineState_Request()

response:
rhover_control_interfaces.srv.GetEngineState_Response(state=rhover_control_interfaces.msg.EngineState(on=False, speed=0.0, battery=100))
```

###### AdjustSpeed

Ajust the Rhover speed, up or down.

```sh
ros2 service call /adjust_speed rhover_control_interfaces/srv/AdjustSpeed '{"speed": 10.0}'
requester: making request: rhover_control_interfaces.srv.AdjustSpeed_Request(speed=10.0)

response:
rhover_control_interfaces.srv.AdjustSpeed_Response(err=rhover_control_interfaces.msg.ErrorStatus(err=False, errcode=0, errmsg=''))
```

It will return an error in case you try to reduce its speed and the rhover is already stopped or it does not have enough battery:

```sh
$ ros2 service call /adjust_speed rhover_control_interfaces/srv/AdjustSpeed '{"speed": -50.0}'
requester: making request: rhover_control_interfaces.srv.AdjustSpeed_Request(speed=-50.0)

response:
rhover_control_interfaces.srv.AdjustSpeed_Response(err=rhover_control_interfaces.msg.ErrorStatus(err=True, errcode=1, errmsg='rhover is stopped'))
```

###### Battery State

Checks the battery state as "ok" (high power), "warn" (somewhere above 50%) or "danger" (bellow 33%):

```sh
$ ros2 service call /battery_state rhover_control_interfaces/srv/BatteryState
requester: making request: rhover_control_interfaces.srv.BatteryState_Request()

response:
rhover_control_interfaces.srv.BatteryState_Response(ok=True, warn=False, danger=False)
```

Each "AdjustSpeed" call reduces its battery power by 10.

###### Stop

Stops the rhover, reducing its speed to 0.

```sh
ros2 service call /stop rhover_control_interfaces/srv/Stop
requester: making request: rhover_control_interfaces.srv.Stop_Request()

response:
rhover_control_interfaces.srv.Stop_Response(err=rhover_control_interfaces.msg.ErrorStatus(err=False, errcode=0, errmsg=''))
```

## License

[Apache-2.0](./LICENSE)
