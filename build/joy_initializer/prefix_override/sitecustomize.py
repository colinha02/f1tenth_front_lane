import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/autopilot03/Desktop/f1tenth_test0804/f1tenth_project_repo/install/joy_initializer'
