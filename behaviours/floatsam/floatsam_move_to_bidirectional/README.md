# FloatSam Move-To Bidirectional

Action server that commands FloatSam to navigate to a goal waypoint with bidirectional movement capability.

## Features

- **Waypoint Navigation**: Sends yaw and velocity setpoints to the Captain controller node
- **Bidirectional Motion**: If the goal waypoint is more than 90° away from the robot's current heading, the robot reverses direction and travels backwards. To be used in the loiter with heading action server

## Usage

Send a goal action request with waypoint coordinates, tolerance, and desired speed:

```bash
ros2 action send_goal /floatsam_usv_0/move_to_bidirectional smarc_msgs/action/BaseAction "{goal: {data: '{\"waypoint\": {\"latitude\": 58.8403874648212, \"longitude\": 17.6518276777968, \"tolerance\": 1.0}, \"speed\": \"standard\", \"constant_speed\": false}'}}"
```
