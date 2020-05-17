#mission type definition
uint8 POINT=0
uint8 CLINE=1
uint8 CIRCLE=2

Header header

# mission type
int64 mission_type

# target point
geometry_msgs/PoseStamped goal

# followed common line 
CommonLine[] lines

# followed circle
CommonCircle circle
