# FIRE Body Drag System

Gazebo Sim 8 system plugin that applies axis-dependent quadratic drag to a
link. The force is evaluated in the link frame:

```text
F_i = -0.5 * rho * Cd_i * A_i * abs(v_i) * v_i
```

The relative air velocity includes Gazebo's global wind velocity when a wind
entity is available. Velocity is evaluated at `center_of_pressure`, so a
nonzero offset also produces the corresponding aerodynamic moment.

## Build

From the repository root:

```bash
cmake -S gz/plugins -B build/gz_plugins
cmake --build build/gz_plugins -j
export GZ_SIM_SYSTEM_PLUGIN_PATH="$PWD/build/gz_plugins/lib:${GZ_SIM_SYSTEM_PLUGIN_PATH}"
```

## SDF configuration

```xml
<plugin filename="FireBodyDragSystem"
        name="fire::gz::systems::BodyDragSystem">
  <link_name>base_link</link_name>
  <air_density>1.225</air_density>
  <drag_coefficient>0.8 0.9 1.1</drag_coefficient>
  <reference_area>0.025 0.030 0.090</reference_area>
  <center_of_pressure>0 0 0</center_of_pressure>
  <use_wind>true</use_wind>
</plugin>
```

Parameters:

- `link_name` (required): Link receiving the force.
- `drag_coefficient`: Dimensionless `Cd_x Cd_y Cd_z`; default `0 0 0`.
- `reference_area`: Projected areas `A_x A_y A_z` in m^2; default `0 0 0`.
- `air_density`: Fluid density in kg/m^3; default `1.225`.
- `center_of_pressure`: Force application point in the link frame, in metres;
  default `0 0 0`.
- `use_wind`: Use the Gazebo wind entity when present; default `true`.

The plugin models parasitic body drag only. Rotor drag from
`MulticopterMotorModel` is a separate effect and may be enabled at the same
time.
