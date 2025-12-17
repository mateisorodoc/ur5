import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/matei/ur_yt_ws/install/ur5_keyboard_teleop'
