"""End-to-end exporter test with a deliberately labeled SYNTHETIC ROS bag."""
import importlib.util
import json
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest


@unittest.skipUnless(importlib.util.find_spec('rosbag2_py'), 'Source ROS 2 Humble for rosbag test')
class BagExportTest(unittest.TestCase):
    def test_synthetic_rosbag_export_and_experiment_rejection(self):
        import rosbag2_py
        from rclpy.serialization import serialize_message
        from nav_msgs.msg import OccupancyGrid, Odometry
        from sensor_msgs.msg import Imu, LaserScan
        from geometry_msgs.msg import TransformStamped
        from tf2_msgs.msg import TFMessage
        import pipeline
        with tempfile.TemporaryDirectory() as temporary:
            root = Path(temporary);bag = root/'fixture_bag'
            writer = rosbag2_py.SequentialWriter()
            writer.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id='sqlite3'),
                        rosbag2_py.ConverterOptions('', ''))
            definitions = {'/map':'nav_msgs/msg/OccupancyGrid', '/tf_static':'tf2_msgs/msg/TFMessage',
                           '/sensors/imu':'sensor_msgs/msg/Imu', '/sensors/wheel_odom':'nav_msgs/msg/Odometry',
                           '/sensors/scan':'sensor_msgs/msg/LaserScan', '/evaluation/ground_truth':'nav_msgs/msg/Odometry'}
            for topic, kind in definitions.items():
                writer.create_topic(rosbag2_py.TopicMetadata(name=topic, type=kind, serialization_format='cdr'))
            grid=OccupancyGrid();grid.header.frame_id='map';grid.info.width=2;grid.info.height=2
            grid.info.resolution=.1;grid.info.origin.orientation.w=1.;grid.data=[0,0,0,100]
            writer.write('/map', serialize_message(grid), 1)
            tf=TransformStamped();tf.header.frame_id='base_link';tf.child_frame_id='lidar_link';tf.transform.rotation.w=1.
            writer.write('/tf_static', serialize_message(TFMessage(transforms=[tf])), 2)
            for k in (0, 1):
                imu=Imu();imu.header.frame_id='imu_link';imu.header.stamp.sec=1;imu.header.stamp.nanosec=k*100000000
                odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link';odom.header.stamp=imu.header.stamp
                scan=LaserScan();scan.header.frame_id='lidar_link';scan.header.stamp=imu.header.stamp
                scan.angle_min=0.;scan.angle_increment=.1;scan.range_min=.05;scan.range_max=10.;scan.ranges=[1.,2.]
                truth=Odometry();truth.header.frame_id='map';truth.header.stamp=imu.header.stamp;truth.pose.pose.orientation.w=1.
                for j,(topic,msg) in enumerate([('/sensors/imu',imu),('/sensors/wheel_odom',odom),('/sensors/scan',scan),('/evaluation/ground_truth',truth)]):
                    writer.write(topic,serialize_message(msg),1000000000+k*100000000+j)
            del writer
            world=root/'fixture.wbt';world.write_text('# SYNTHETIC TEST FIXTURE, not a simulator world\n')
            output=root/'export'
            subprocess.run([sys.executable, str(Path(__file__).with_name('export_bag.py')),str(bag),str(output),
                            '--world','fixture','--world-file',str(world),'--trajectory','fixture','--scenario','unit-test',
                            '--split','train','--initial','0','0','0','--gt-to-map','0','0','0','--synthetic-fixture'],check=True)
            m=json.loads((output/'manifest.json').read_text())
            self.assertEqual(m['source'],'synthetic_fixture');self.assertEqual(m['counts']['scan'],2)
            self.assertTrue((output/'events.txt').read_text().startswith('W '))
            with self.assertRaisesRegex(ValueError,'recorded_rosbag'):
                pipeline.load_dataset(output)

if __name__=='__main__':unittest.main()
