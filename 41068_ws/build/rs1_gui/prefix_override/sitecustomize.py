import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/jun/git/Robotics-Studio-1/41068_ws/install/rs1_gui'
