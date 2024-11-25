import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/manual_control/src/arduino_pkg/install/arduino_pkg'
