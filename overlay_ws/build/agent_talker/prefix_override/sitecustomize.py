import sys
if sys.prefix == '/usr':
    sys.real_prefix = sys.prefix
    sys.prefix = sys.exec_prefix = '/home/hiwi/workspaces/kinova_ws/kinova-ros2/overlay_ws/install/agent_talker'
