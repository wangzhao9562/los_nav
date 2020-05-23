# los_nav  
Implementation of line-of-sight guidance for robot to execute some missions like point following, line following and circle following.  
  
# Compile     
cd [catkin work space]    
git clone https://github.com/wangzhao9562/los_nav.git    
catkin_make    
  
# Run    
## Point follow    
rosrun los_nav los_nav_node    
rosrun los_nav los_nav_test -p -60.0 30.0    
## Common line follow  
rosrun los_nav los_nav_node  
rosrun los_nav los_nav_test -cl 40.0 20.0 -60.0 100.0  
## Circle line follow  
rosrun los_nav los_nav_node  
rosrun los_nav los_nav_test -c 40.0 40.0 5.0  
    
