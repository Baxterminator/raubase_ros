# This message represent the state of the whole GPIO array

# Mode possibilities
int32 UNSET = 0
int32 READ = 1
int32 WRITE = 2


int32[] channels
int32[] modes
string[] names
bool[] values