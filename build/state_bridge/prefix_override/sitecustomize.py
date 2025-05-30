import sys
if sys.prefix == '/Users/koizumihinata/.pyenv/versions/3.12.0':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/Users/koizumihinata/drone_avoidance_rl/install/state_bridge'
