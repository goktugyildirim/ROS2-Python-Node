from sensor_msgs.msg import PointCloud2
import numpy as np
import ros2_numpy as rnp  #Extra ROS2 Python package!

class PointCloudHelper(object):
    def __init__(self):
        pass

    def pushPoint(self,x, y, z, intensity, point_cloud):
        np.insert(point_cloud['x'], -1, x)
        np.insert(point_cloud['y'], -1, y)
        np.insert(point_cloud['z'], -1, z)
        np.insert(point_cloud['intensity'], -1, intensity)
        return point_cloud

    def givePointCloud(self,width):
        return np.empty(width, dtype=[
            ('x', np.float32),
            ('y', np.float32),
            ('z', np.float32),
            ('intensity', np.float32)])

    def givePointCloud2Msg(self,data):
        return rnp.msgify(PointCloud2, data)

    def publishPointCloud(self, msg, publisher, frame_id, time_stamp):
        msg.header.frame_id = frame_id
        msg.height = 1
        msg.header.stamp = time_stamp
        publisher.publish(msg)
