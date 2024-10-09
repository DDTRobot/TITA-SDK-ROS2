# Command manager send msg to locomotion unit.
std_msgs/Header header
# "stand" "jump" "die"
string fsm_mode 

# Robot body pose
geometry_msgs/Pose pose

# Speed control of robot chassis 
geometry_msgs/Twist twist