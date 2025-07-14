# Simulation Parameter Reference

| Name | Type | Default | Description |
|------|------|---------|-------------|
| `cmd_topic` | str | /drone/inner_propeller_cmd |  |
| `fan1_topic` | str | /drone/servo/fan1_tilt |  |
| `fan2_topic` | str | /drone/servo/fan2_tilt |  |
| `input_topic` | str | /drone/command |  |
| `output_topic` | str | /drone/px4/actuator_motors |  |
| `state_input_topic` | str | /drone/px4/odom |  |
| `state_output_topic` | str | /drone/state |  |
| `outer_motor_input_topic` | str | /drone/px4/actuator_motors |  |
| `outer_motor_output_topic` | str | /drone/outer_motor_pwm |  |
| `physics_engine` | str | bullet |  |
| `model_path` | str | /models/drone_model |  |
| `gz_world` | str | src/sim_launch/resource/empty_custom.sdf |  |
| `reward.ori` | float | -1.0 | 姿勢誤差に対するペナルティ重み |
| `reward.pos` | float | -1.0 | 位置誤差に対するペナルティ重み |
| `reward.smooth` | float | -0.02 | 行動変化（スムーズさ）に対するペナルティ重み |
| `physics.mass` | float | 1.0 | 機体質量[kg] |
| `physics.inertia` | float | 0.1 | 慣性モーメント（簡易） |
