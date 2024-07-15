import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/dalv204/sandbox/f1tenth_ws/install/particle_filter'
