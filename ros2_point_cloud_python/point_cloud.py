import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs.msg as sensor_msgs
import std_msgs
import numpy as np

import ros2_numpy as rnp # extra ros2 package!


class PointCloud(Node):

    def __init__(self):
        super().__init__('point_cloud_node')
        self.publisher_ = self.create_publisher(PointCloud2, '/point_cloud', 10)
        self.timer = self.create_timer(0.5, self.timer_callback)
        self.i = 0

        self.subscription = self.create_subscription(PointCloud2, '/point_cloud', self.listener_callback, 10)


    def timer_callback(self):
        data = np.ones(100, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32),
        ])
        data['x'] = np.arange(100)
        data['y'] = data['x']*2
        data['z'] = data['x']*2
        data['intensity'] = data['x']*2

        msg = rnp.msgify(PointCloud2, data)
        self.PublishPointCloud(msg,self.publisher_,"velodyne")

    def listener_callback(self, msg):
        data = rnp.numpify(msg)
        print(type(data))
        print(data['x'].shape)
        print(data['y'].shape)
        print(data['z'].shape)

        # access the point:
        x_value = data[0][0]
        y_value = data[0][1]
        z_value = data[0][2]
        intensity_value = data[0][3]

        print(data[0])
        print(x_value,y_value,z_value,intensity_value)

        print("******************************")



    def PublishPointCloud(self, msg, publisher, frame_id):
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.header.stamp = self.get_clock().now().to_msg()
        publisher.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    node_obj = PointCloud()
    rclpy.spin(node_obj)
    node_obj.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
