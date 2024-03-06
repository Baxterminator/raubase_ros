# Possibilities for commands 
#   0 = command in position (x, y, heading)
#   1 = command in velocity + heading angle (v, heading)
#   2 = command in velocity + turn rate (v, turn rate)
char CMD_POS=0
char CMD_V_ANGLE=1
char CMD_V_TR=2

char move_type 1

# Command in position
float64 pos_x
float64 pos_y

# Heading command (Position + Velocity with heading command)
float64 heading
float64 turn_rate
float64 velocity