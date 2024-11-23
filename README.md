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

This node subscribes to the `rhover_control_engine_manager` to abstarct engine operations.

Running the node:

```sh
ros2 run rhover_control engine_manager
```

Send data using the `ros2` cli:

```sh
ros2 topic pub /rhover_engine example_interfaces/msg/String "data: 'turn on'"
```

## License

[Apache-2.0](./LICENSE)
