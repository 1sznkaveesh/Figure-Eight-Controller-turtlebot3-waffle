import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/kaveesh/turtlebot3_ws/install/my_turtlebot3_control'
