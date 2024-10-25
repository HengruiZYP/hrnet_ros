import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/media/edgeboard/15535b00-bc44-4cd4-b797-882073cf89e2/hrnet_python_ros/install/hrnet_pkg'
