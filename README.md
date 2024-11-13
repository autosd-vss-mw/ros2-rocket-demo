# ROS2 Rocket Demo

Welcome to the Rocket Launch Simulator based on CentOS, AutoSD and ROS2!  
This project allows you to simulate rocket launches, including specific mission operations.
The simulation includes detailed stages, realistic delays, and even custom mission operations for a truly engaging experience!


![Curiosity Rover](https://d2pn8kiwq2w21t.cloudfront.net/original_images/spaceimagesimageslargesizePIA14156_hires.jpg)
Launched on a NASA rocket (NASA Atlas V), Curiosity touched down on Mars and now roams the Red Planet, uncovering its ancient secrets.


## Building

To build all packages run:

```sh
colcon build
```

## Running

### Generic Instructions

Run a ROS2 by specifying its package and node name:

```
ros2 launch $pkg_name $node_name
```

### rocket_pkg

This section describes how to run nodes within the `rocket_pkg`.

#### 

This packages launches a rocket

## Example

``` python
from rocket_launcher import RocketLaunch
from rocket_launcher.nasa import curiosity_mission_operations

def main():
    # Initialize a RocketLaunch instance for the NASA Curiosity mission
    rocket = RocketLaunch(
        rocket_name='Curiosity',
        payload='Mars Rover',
        mission_type='curiosity'
    )

    # Simulate the entire launch sequence for the Curiosity mission
    rocket.simulate_launch()

    # Perform NASA-specific operations for the Curiosity rover
    curiosity_mission_operations(rocket)

if __name__ == "__main__":
    main()
```

## License

[Apache-2.0](./LICENSE)
