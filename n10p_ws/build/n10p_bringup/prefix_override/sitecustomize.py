import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/ylz/n10p_leishen/n10p_ws/install/n10p_bringup'
