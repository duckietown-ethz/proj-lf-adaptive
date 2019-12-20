import rosbag
import numpy as np
import matplotlib.pyplot as plt
import rospy

bag_name = "debug_bag"
bag_name = bag_name + ".bag"
bag_read = rosbag.Bag(name_bag)

bot_name = "<YOUR_DUCKIEBOT_NAME>"

topic_names = ["/"+bot_name+"/lane_filter_node/lane_pose", "/"+bot_name+"/lane_controller_node/car_cmd"]

t_pose = np.asarray([])
t_cmd = np.asarray([])

h_pose = np.asarray([])
h_cmd = np.asarray([])

for topic, msg, t in bag_read.read_messages(topics = topic_name):

    if topic == topic_names[0]:
		t_pose = np.append(t_pose,np.asarray(rospy.Time.to_sec(t)))
        h_pose = np.append(t_pose,np.asarray(rospy.Time.to_sec(msg.header.stamp)))

	elif topic == topic_names[1]:
		t_cmd = np.append(t_cmd,np.asarray(rospy.Time.to_sec(t)))
        h_cmd = np.append(t_pose,np.asarray(rospy.Time.to_sec(msg.header.stamp)))

bag_read.close()
