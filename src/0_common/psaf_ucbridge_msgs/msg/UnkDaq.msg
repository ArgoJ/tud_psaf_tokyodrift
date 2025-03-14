# This msg is used for create a unknown DAQ group
# The group number default 0
uint8 grp_no 0

# How many channels should the group have in range[1,3] default 3
uint8 channel_num 3

# Should the channel be signed or unsigned? default unsigned
bool sign 0

# Set true to delete the group default false
bool delete_grp 0

# channels and options for the group
string options