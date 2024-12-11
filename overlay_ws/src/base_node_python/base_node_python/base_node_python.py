import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
import rosidl_runtime_py
import sys


class MapManager(Node):

    def __init__(self):

        super().__init__('base_node_python')
        print('Started base_node_python node')

        # Read params
        self.declare_parameter('test_param_for_base_node', 'not_defined')
        test_param_for_base_node = self.get_parameter('test_param_for_base_node').get_parameter_value().string_value
        print("I got the test parameter: " + test_param_for_base_node)


def main(args=None):

    rclpy.init(args=args)
    try:
        base_node_python = MapManager()
        rclpy.spin(base_node_python)
    except KeyboardInterrupt:
        pass
    except ExternalShutdownException:
        sys.exit(1)
    finally:
        rclpy.try_shutdown()
        base_node_python.destroy_node()


if __name__ == '__main__':
    main()
