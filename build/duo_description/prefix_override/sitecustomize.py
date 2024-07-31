import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/deadcat/ros2_ws/src/duo_description/install/duo_description'
