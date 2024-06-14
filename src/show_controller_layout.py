#!/usr/bin/env python
 
import rospy
from std_msgs.msg import String
from threading import Timer, Lock
import cv2
import os
import rospkg

class ImageDisplayNode:
    def __init__(self):
        rospy.init_node('image_display_node', anonymous=True)
        
        # Directory containing the images
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('mobile_manipulation_teleop')
        self.image_dir = package_path + '/images'
        
        # Subscriber for the /show_game_controller_layout topic
        self.subscriber = rospy.Subscriber('/show_game_controller_layout', String, self.callback)
        
        # Timer for hiding the image
        self.timer = None
        self.image_window_name = "Controller Layout"
        
        self.image_showing = ' '   # there are 5 windows that can be showing. ' ' means no window showing
        self.image_name = ' '
        self.last_msg_type = ' '
        self.last_msg_time = rospy.Time.now()
        self.show_image_flag = False
        self.hide_image_flag = False


    def callback(self, msg):
        # process all new incoming messages into an averaged message over 0.5 seconds

        # check if in last 0.5 second different messages have been posted
        elapsed_time = (rospy.Time.now() - self.last_msg_time)
        # update the last_msg_time
        self.last_msg_time = rospy.Time.now()
        if (elapsed_time.to_sec() < 0.3) and (self.last_msg_type != msg.data):
            if self.image_showing != "multiple_enable_buttons":
                self.show_image("multiple_enable_buttons")
        elif self.image_showing != msg.data: 
            self.show_image(msg.data)

        self.last_msg_type = msg.data

    def show_image(self, image_name):
        # Determine the file name based on the message
        file_name = f"{image_name}.jpg"
        self.image_name = image_name
        # Construct the full file path
        file_path = os.path.join(self.image_dir, file_name)
        
        # Load and display the image
        if os.path.isfile(file_path):
            #print('showing', file_name)
            self.image = cv2.imread(file_path)
            # Instead of showing the image here, set a flag
            self.show_image_flag = True
        else:
            rospy.logwarn("Image file not found: %s", file_path)

    def hide_image(self):
        #print('hiding the image')
        self.image_name = ' '
        # Instead of destroying the window here, set a flag
        self.hide_image_flag = True

if __name__ == '__main__':
    try:
        node = ImageDisplayNode()
    except rospy.ROSInterruptException:
        pass
    finally:
        cv2.destroyAllWindows()


rospy.sleep(0.6)
rate = rospy.Rate(10)   


# This part runs in the main thread, ensuring all OpenCV functions are called from here.
while not rospy.is_shutdown():

    elapsed_time = (rospy.Time.now() - node.last_msg_time)
    #print(elapsed_time.to_sec())
    if (elapsed_time.to_sec() > 0.3) and (node.image_showing != ' '):
        node.hide_image()


    # Show image if flag is set
    #print("imageflag",node.show_image_flag)
    if node.show_image_flag:
        if node.image_showing != ' ':
            # first close old image if necessary
            cv2.destroyWindow(node.image_window_name)
            cv2.waitKey(1)
        cv2.imshow(node.image_window_name, node.image)
        # Move the window to the bottom left corner of the screen
        # Ensure OpenCV handles GUI events
        cv2.waitKey(1)
        cv2.moveWindow(node.image_window_name, 0, 500)   #TODO: make screen size dependent
        cv2.setWindowProperty(node.image_window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
        node.show_image_flag = False
        node.image_showing = node.image_name

    # Hide image if flag is set
    if node.hide_image_flag:
        cv2.destroyWindow(node.image_window_name)
        cv2.waitKey(1)
        node.hide_image_flag = False
        node.image_showing = node.image_name

    cv2.waitKey(1)    
    # Sleep to maintain the loop rate
    rate.sleep()






# #!/usr/bin/env python
 
# import rospy
# from std_msgs.msg import String
# from threading import Timer, Lock
# import cv2
# import os
# import rospkg

# class ImageDisplayNode:
#     def __init__(self):
#         rospy.init_node('image_display_node', anonymous=True)
        
#         # Directory containing the images
#         rospack = rospkg.RosPack()
#         package_path = rospack.get_path('mobile_manipulation_teleop')
#         self.image_dir = package_path + '/images'
        
#         # Subscriber for the /show_game_controller_layout topic
#         self.subscriber = rospy.Subscriber('/show_game_controller_layout', String, self.callback)
        
#         # Timer for hiding the image
#         self.timer = None
#         self.image_window_name = "Controller Layout"
        
#         self.image_showing = ' '   # there are 5 windows that can be showing. ' ' means no window showing
#         self.last_msg_type = ' '
#         self.last_msg_time = rospy.Time.now()
#         rospy.sleep(0.6)

#         self.rate = rospy.Rate(10)   
#         # Start the ROS spin loop
#         while not rospy.is_shutdown():
#             elapsed_time = (rospy.Time.now()-self.last_msg_time)
#             print(elapsed_time.to_sec())
#             if (elapsed_time.to_sec()>0.5) and (self.image_showing != ' '): self.hide_image()
#             cv2.waitKey(1)
#             # Sleep to maintain the loop rate
#             self.rate.sleep()

#     def callback(self, msg):
#         # process all new incoming messages into an averaged message over 0.5 seconds

#         # check if in last 0.5 second different messages have been posted
#         elapsed_time = (rospy.Time.now()-self.last_msg_time)
#         #update the last_msg_time
#         self.last_msg_time = rospy.Time.now()
#         if (elapsed_time.to_sec()<0.5) and (self.last_msg_type != msg.data):
#             if self.image_showing != "multiple_enable_buttons":
#                 self.show_image("multiple_enable_buttons")
#         elif self.image_showing != msg.data: 
#             print(self.image_showing,msg.data, 'what the heck ', )
#             self.show_image(msg.data)


#         self.last_msg_type = msg.data


#     def show_image(self, image_name):
#         # Determine the file name based on the message
#         file_name = f"{image_name}.jpg"
#         self.image_showing = image_name
        
#         # Construct the full file path
#         file_path = os.path.join(self.image_dir, file_name)
        
#         # Load and display the image
#         if os.path.isfile(file_path):
#             print('showing ',file_name)
#             image = cv2.imread(file_path)
#             cv2.imshow(self.image_window_name, image)
#             cv2.waitKey(1)
#             # Move the window to the bottom left corner of the screen
#             #cv2.moveWindow(self.image_window_name, 0, cv2.getWindowImageRect(self.image_window_name)[3] - 100)
#         else:
#             rospy.logwarn("Image file not found: %s", file_path)

#     def hide_image(self):
#         print('hiding the image')
#         cv2.destroyWindow(self.image_window_name)
#         self.image_showing = ' '

# if __name__ == '__main__':
#     try:
#         ImageDisplayNode()
#     except rospy.ROSInterruptException:
#         pass
#     finally:
#         cv2.destroyAllWindows()
