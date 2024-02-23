builtin_interfaces/Time stamp

uint8 device_id  # Robot Number
uint32 sw_rev    # Software revision
uint32 hw_rev    # Hardware revision

uint8 state      # 0 = no mission, 2 = user mission
float64 voltage  # Battery voltage in %

uint8 load       # Teensy load in %
uint8 m_left     # Motor 1
uint8 m_right    # Motor 2