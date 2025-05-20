import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/workspace/ros_ws/src/install/nav2_gps_waypoint_follower_demo'
