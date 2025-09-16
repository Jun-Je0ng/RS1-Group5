import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/student/RS1-Group5/41068_ws/src/install/rs1_gui'
