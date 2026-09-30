# FIRE Motor Command Mux

Converts one `gz.msgs.Double` topic per motor into the `gz.msgs.Actuators`
message expected by Gazebo's `MulticopterMotorModel`. It allows
`ArduPilotPlugin` controls of type `COMMAND` to use exactly the same propulsion
model and parameters as PX4.

The plugin clamps every input to `[0, max_velocity]` and publishes zero until
a command is received. After reception, it holds the last command; it has no
command timeout.

```xml
<plugin filename="FireMotorCommandMuxSystem"
        name="fire::gz::systems::MotorCommandMuxSystem">
  <input_topic>/model/fire_px4vision/ardupilot/motor/0</input_topic>
  <input_topic>/model/fire_px4vision/ardupilot/motor/1</input_topic>
  <input_topic>/model/fire_px4vision/ardupilot/motor/2</input_topic>
  <input_topic>/model/fire_px4vision/ardupilot/motor/3</input_topic>
  <output_sub_topic>command/motor_speed</output_sub_topic>
  <max_velocity>1000</max_velocity>
</plugin>
```

## Build and connect

From the flightstack repository root:

```bash
cmake -S gz/FIRE_moonshot_gazebo/plugins/plugins -B build/fire_gz_plugins
cmake --build build/fire_gz_plugins -j
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build/fire_gz_plugins/lib${GZ_SIM_SYSTEM_PLUGIN_PATH:+:$GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

`install/autopilot.sh --with-ardupilot` also builds these plugins during the
`build` and `all` phases and registers their library path in the generated
`ardupilot_gz_env.sh` during the `env` and `all` phases.

`worlds/default_fire_px4vision.sdf` attaches both `ArduPilotPlugin` and this mux
through the vehicle include. It maps channels 0..3 to rotor_0..3, converting
PWM 1000..2000 into speed commands 0..1000 rad/s. All multipliers are positive;
the motor models determine CW / CCW rotation. The final output topic is
`/fire_px4vision/command/motor_speed` (without a `/model` prefix).

Use ArduPilot frame `gazebo-px4vision`, model `JSON`, and instance 0 (UDP port
9002). The shared vehicle model itself remains usable by PX4 without these
ArduPilot-specific plugins.

The ArduPilot world runs 1 ms physics steps with a 1000 Hz IMU to support
`SCHED_LOOP_RATE=400`. A 4 ms world step limits JSON updates to 250 Hz and can
fail ArduCopter's main-loop and gyro-rate pre-arm checks. Increasing only the
IMU update rate does not remove that physics-step limit.

The shared IMU retains PX4's FLU axes. This world sets the local
`ArduPilotPlugin` option `imuXYZToAirplaneXForwardZDown` to a 180-degree roll,
rotating acceleration and angular velocity into FRD before sending JSON.
The option defaults to identity for existing ArduPilot models whose sensor
pose already produces FRD. Rebuild `gz/ardupilot_gazebo/build` after updating
the plugin source. `modelXYZToAirplaneXForwardZDown` alone only converts the
vehicle pose and does not rotate IMU measurements.
