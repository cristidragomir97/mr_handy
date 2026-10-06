#!/usr/bin/env python3
"""Verify native simulation sensor samples and conventional velocity routing."""
import time,math
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image,CameraInfo,LaserScan,Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

rclpy.init();node=Node('handy101_sensor_check');samples={};counts={};subscriptions=[]
def subscribe(kind,topic):
 def receive(message):samples[topic]=message;counts[topic]=counts.get(topic,0)+1
 subscriptions.append(node.create_subscription(kind,topic,receive,qos_profile_sensor_data))
subscribe(LaserScan,'/scan');subscribe(Imu,'/sensors/imu');subscribe(Odometry,'/diff_drive_controller/odom')
frames={'base_camera':'camera_optical_frame','head_camera':'head_camera_optical_frame','left_arm_camera':'left_arm_camera_tool_optical_frame'}
for camera in frames:
 subscribe(Image,f'/{camera}/color/image_raw');subscribe(CameraInfo,f'/{camera}/camera_info')
 if camera!='left_arm_gopro':subscribe(Image,f'/{camera}/depth/image_raw')
def wait(predicate,seconds=40):
 end=time.monotonic()+seconds
 while time.monotonic()<end:
  rclpy.spin_once(node,timeout_sec=.05)
  if predicate():return
 raise AssertionError(('Missing sensor samples', expected - samples.keys(), counts))
expected={'/scan','/sensors/imu','/diff_drive_controller/odom'}
for camera in frames:
 expected|={f'/{camera}/color/image_raw',f'/{camera}/camera_info'}
 if camera!='left_arm_gopro':expected.add(f'/{camera}/depth/image_raw')
assert not any(topic.startswith(('/right_wrist_camera/', '/left_arm_gopro/')) for topic, _ in node.get_topic_names_and_types())
wait(lambda:expected<=samples.keys() and all(counts.get(t,0)>1 for t in expected))
scan=samples['/scan'];assert scan.header.frame_id=='lidar_frame' and len(scan.ranges)==360
assert all(not math.isnan(v) for v in scan.ranges)
valid=[v for v in scan.ranges if scan.range_min<=v<=scan.range_max]
assert len(valid)>180,('Lidar has no usable room returns',len(valid),min(scan.ranges),max(scan.ranges))
imu=samples['/sensors/imu'];assert imu.header.frame_id=='imu_link'
assert abs(sum(getattr(imu.orientation,a)**2 for a in ('x','y','z','w'))-1)<1e-5
assert math.sqrt(sum(getattr(imu.linear_acceleration,a)**2 for a in ('x','y','z')))>8
print('PASS: 360-ray lidar and gravity-aware IMU publish live stamped samples.',flush=True)
for camera,frame in frames.items():
 image=samples[f'/{camera}/color/image_raw'];info=samples[f'/{camera}/camera_info']
 assert image.header.frame_id==info.header.frame_id==frame
 assert image.width==info.width==640 and image.height==info.height==480
 assert len(image.data)==image.height*image.step and info.k[0]>0
 assert image.header.stamp.sec or image.header.stamp.nanosec
 if camera!='left_arm_gopro':
  depth=samples[f'/{camera}/depth/image_raw'];assert depth.encoding=='32FC1' and depth.header.frame_id==frame
 print(f'PASS: {camera} RGB/calibration'+(' + depth' if camera!='left_arm_gopro' else '')+' stream.',flush=True)
# Test the public input, not the internal controller topic.
publisher=node.create_publisher(Twist,'/cmd_vel',10)
wait(lambda:publisher.get_subscription_count()>0)
start=samples['/diff_drive_controller/odom'].pose.pose.position;before=(start.x,start.y)
end=time.monotonic()+2
try:
 while time.monotonic()<end:
  command=Twist();command.linear.x=.08;publisher.publish(command);rclpy.spin_once(node,timeout_sec=.05)
finally:publisher.publish(Twist())
after=samples['/diff_drive_controller/odom'].pose.pose.position
assert math.hypot(after.x-before[0],after.y-before[1])>.02
print('PASS: /cmd_vel Twist -> base101 mux -> wheel controller -> measured odometry.',flush=True)
node.destroy_node();rclpy.shutdown()
