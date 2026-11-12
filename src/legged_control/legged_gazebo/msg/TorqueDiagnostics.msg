# Updated internally on EVERY writeSim/readSim cycle, published at up to 100 Hz.
# Efforts are actuator torques in N m. Passive/contact loads are not motor commands.
std_msgs/Header header
uint64 samples
string[] name
float64[] limit
float64[] position_term
float64[] velocity_term
float64[] feedforward
float64[] requested_effort
float64[] applied_effort
float64[] measured_effort
uint8[] phase
float64[] peak_requested_effort
float64[] peak_applied_effort
float64[] peak_measured_effort
uint64[] saturation_count
# Context of each joint's largest absolute pre-limit request since initialization.
float64[] peak_time
float64[] peak_position_term
float64[] peak_velocity_term
float64[] peak_feedforward
float64[] peak_measured_velocity
# 0: unpowered/stopped, 1: position PD (stand-up), 2: MPC/WBC
uint8[] peak_phase
