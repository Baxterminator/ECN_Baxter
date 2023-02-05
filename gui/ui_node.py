#!/usr/bin/env python3
import rclpy
from ecn_baxter import GUINode


def main(args=None):
    rclpy.init(args=args)

    gui = GUINode()

    rclpy.spin(gui)

    gui.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
