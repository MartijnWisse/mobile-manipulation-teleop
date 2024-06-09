#!/usr/bin/env python

import rospy
from darknet_ros_msgs.msg import BoundingBoxes
from std_msgs.msg import String

class VisionToSpeech:
    def __init__(self):
        rospy.init_node('vision_to_speech', anonymous=True)

        self.object_classes = []
        self.gpt_publisher = rospy.Publisher('/gpt', String, queue_size=10)

        rospy.Subscriber('/speech_command', String, self.publish_object_classes)
        rospy.Subscriber('/darknet_ros/bounding_boxes', BoundingBoxes, self.bounding_boxes_callback)

        rospy.spin()

    def bounding_boxes_callback(self, data):
        self.object_classes = [box.Class for box in data.bounding_boxes]

    def publish_object_classes(self, data):
        # data is not yet used, but it is a string which could later be added to the specific speech command
        if self.object_classes:
            object_list = ','.join(self.object_classes)
            message = f"en/ nothing; {object_list} ; await further remote control signals"
            rospy.loginfo(f"Publishing to /gpt: {message}")
            self.gpt_publisher.publish(String(data=message))
        else:
            rospy.loginfo("No objects detected to publish.")

if __name__ == '__main__':
    try:
        VisionToSpeech()
    except rospy.ROSInterruptException:
        pass
