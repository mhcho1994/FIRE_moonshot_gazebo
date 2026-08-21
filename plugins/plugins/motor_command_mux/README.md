# FIRE Motor Command Mux

Converts one `gz.msgs.Double` topic per motor into the `gz.msgs.Actuators`
message expected by Gazebo's `MulticopterMotorModel`. It allows
`ArduPilotPlugin` controls of type `COMMAND` to use exactly the same propulsion
model and parameters as PX4.

The plugin clamps every input to `[0, max_velocity]` and publishes zero until
a command is received.

```xml
<plugin filename="FireMotorCommandMuxSystem"
        name="fire::gz::systems::MotorCommandMuxSystem">
  <input_topic>/model/px4vision/ardupilot/motor/0</input_topic>
  <input_topic>/model/px4vision/ardupilot/motor/1</input_topic>
  <input_topic>/model/px4vision/ardupilot/motor/2</input_topic>
  <input_topic>/model/px4vision/ardupilot/motor/3</input_topic>
  <output_sub_topic>command/motor_speed</output_sub_topic>
  <max_velocity>1000</max_velocity>
</plugin>
```
