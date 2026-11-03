import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/gomezluis2907/rovert-o-system/install/roverto_autonomy'
