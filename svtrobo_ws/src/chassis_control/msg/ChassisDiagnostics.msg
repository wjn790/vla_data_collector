std_msgs/Header header

# Bus voltage from motor driver (V)
float32 vbus

# Steering motor temperatures (degrees C): FL, FR, RL, RR
float32[4] motor_temperatures

# Steering motor error codes: FL, FR, RL, RR
uint8[4] motor_error_codes

# Wheel actual speeds from ZLAC8015D (RPM): FL, FR, RL, RR
float32[4] wheel_speeds_actual
