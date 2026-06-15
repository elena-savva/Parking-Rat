import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jetson/EVC/workshops/final_project_map_v3/final_project_map_v2/final_project_v4/install/aruco_parking'
