#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64MultiArray, String
from mirte_msgs.msg import ServoPosition, WavCommand
import yaml
import copy
from std_srvs.srv import Trigger
import actionlib
from actionlib_msgs.msg import GoalID
from visual_servoing_action.msg import VisualServoAction, VisualServoGoal
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal, MoveBaseActionResult
from std_msgs.msg import Empty
from map_interaction.srv import SwitchMapInteractively, SwitchMapInteractivelyResponse
from map_interaction.msg import InspectHolonomicAction, InspectHolonomicGoal
import tf
import math

class MobileManipulationTeleop:
    def __init__(self):
        rospy.init_node('mobile_manipulation_teleop', anonymous=False)
        
        # Load configuration
        config_file = rospy.get_param('~config_file')
        with open(config_file, 'r') as f:
            self.config = yaml.safe_load(f)

        self.button_map = self.config['buttons']
        self.axis_map = self.config['axes']
        
        self.latched_states = {button: False for button in self.button_map.values()}

        self.wav_command = WavCommand()
        self.wav_file_name_publisher = rospy.Publisher('/wavfile', WavCommand, queue_size=1)

        self.speech_command_publisher = rospy.Publisher('/speech_command', String, queue_size=1)

        self.base_cmd_vel = Twist()
        self.base_cmd_vel_publisher = rospy.Publisher('/mobile_base_controller/cmd_vel', Twist, queue_size=1)

        self.arm_cur_angles = Float64MultiArray()
        self.arm_ref_angles = Float64MultiArray()
        self.arm_cur_angles.data = [0,0,0,0]
        self.arm_ref_angles.data = [0,0,0,0]
        self.arm_ref_angles_publisher = rospy.Publisher('/arm/joint_position_controller/command', Float64MultiArray, queue_size=1)

        # Visual servoing client initialization
        self.visual_servo_client = actionlib.SimpleActionClient('visual_servo', VisualServoAction)
        self.visual_servo_client.wait_for_server()
        self.visual_servo_running = False

        # Inspect_holonomic client initialization
        self.inspect_holonomic_client = actionlib.SimpleActionClient('inspect_holonomic', InspectHolonomicAction)
        self.inspect_holonomic_running = False
        self.listener = tf.TransformListener()

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        self.move_base_cancel_pub = rospy.Publisher('move_base/cancel', GoalID, queue_size=1)
     
        # FSM Idle/Search toggle publisher
        self.fsm_idle_search_toggle_publisher = rospy.Publisher('/FSM_idle_search_toggle', Empty, queue_size=1)


        # subscribe to actual angle values and update arm_cur_angles immediately every time a new servo angle comes in
        rospy.Subscriber("/mirte/servos/servoRot/position",      ServoPosition, lambda messg: self.update_arm_cur_angles(messg,0))   # the number (0) is the joint number
        rospy.Subscriber("/mirte/servos/servoShoulder/position", ServoPosition, lambda messg: self.update_arm_cur_angles(messg,1))
        rospy.Subscriber("/mirte/servos/servoElbow/position",    ServoPosition, lambda messg: self.update_arm_cur_angles(messg,2))
        rospy.Subscriber("/mirte/servos/servoWrist/position",    ServoPosition, lambda messg: self.update_arm_cur_angles(messg,3))

        rospy.Subscriber('/joy', Joy, self.joy_callback)
        
        rospy.spin()

    def button_triggered(self, button_name):
        button_id = self.button_map[button_name]
        if self.joydata.buttons[button_id] and not self.latched_states[button_id]:
            self.latched_states[button_id] = True
            return True
        elif not self.joydata.buttons[button_id]:
            self.latched_states[button_id] = False
        return False

    def is_button_pressed(self, button_name):
        button_id = self.button_map[button_name]
        return self.joydata.buttons[button_id]

    def axis(self, axis_name):
        axis_id = self.axis_map[axis_name]
        return self.joydata.axes[axis_id]

    def joy_callback(self, joydata):
        self.joydata = joydata
  
        # Define here what each button and axis does. 
        # All functionality should be outside this function, so that this function serves as a configuration function (keep the overview)
        # Buttons can be triggers (execute action once, re-execute only after button has first been released) 
        # or they can execute actions repeatedly, or both.
        # Button names are 'PlayStationName/XBoxName'


        # Base and arm motions, controllable when left shoulder button (L1) is pressed 
        if self.button_triggered('L1'):
            self.speak()
            self.cancel_all_goals()  # emergency stop
            self.arm_ref_angles = copy.deepcopy(self.arm_cur_angles)  # to prevent sudden arm motions when enabling motion
            rospy.loginfo(f"Resetting angles to {self.arm_ref_angles.data}")
 
        if self.is_button_pressed('L1'):    # enable motion
            if self.button_triggered('square/X'):    gripper_success = self.call_gripper_service('/gripper_close')
            if self.button_triggered('circle/B'):    gripper_success = self.call_gripper_service('/gripper_open')
            if self.is_button_pressed('triangle/Y'): self.tilt_wrist('down')
            if self.is_button_pressed('cross/A'):    self.tilt_wrist('up')
            if self.is_button_pressed('D_pad_up'):   self.extend_arm('extend')
            if self.is_button_pressed('D_pad_down'): self.extend_arm('contract')
            if self.is_button_pressed('D_pad_left'): pass # not yet assigned
            if self.is_button_pressed('D_pad_right'): pass # not yet assigned
            if self.button_triggered('R3'):         self.arm_to_home()

            self.base_cmd_vel.linear.x = self.axis('left_vertical')
            if self.is_button_pressed('L3'):  # enable strafing
                self.base_cmd_vel.linear.y = self.axis('left_horizontal')
                self.base_cmd_vel.angular.z = 0
            else:
                self.base_cmd_vel.angular.z = self.axis('left_horizontal')
                self.base_cmd_vel.linear.y = 0
            self.arm_rotation = self.axis('right_horizontal')
            self.arm_shoulder = self.axis('right_vertical')

            self.issue_motor_commands()


        # Sounds, controllable when left trigger button (L2) is pressed 
        elif self.is_button_pressed('L2'):  # enable sound commands
            if self.button_triggered('square/X'):    self.playsound('rekenen.wav', 1)  #the .wav files must exist in the robot for this to work
            if self.button_triggered('circle/B'):    self.playsound('scheids.wav', 1)
            if self.button_triggered('triangle/Y'):  self.playsound('toeter.wav',  1)
            if self.button_triggered('cross/A'):     self.playsound('fietsbel.wav',1)
            if self.button_triggered('D_pad_up'):    self.playsound('fiewiet.wav',1)
            if self.button_triggered('D_pad_down'):  self.playsound('hoei.wav',1)
            if self.button_triggered('D_pad_left'):  self.playsound('buzzer.wav',1)
            if self.button_triggered('D_pad_right'): self.playsound('correct.wav',1)
            if self.button_triggered('start/play'):  self.speak_what_you_see()


        # Atomic actions and services, controllable when right shoulder button (R1) is pressed
        elif self.is_button_pressed('R1'):  # enable atomic actions and services
            if self.button_triggered('circle/B'): self.pick_up_service()
            if self.button_triggered('square/X'): self.toggle_visual_servoing()
            if self.button_triggered('D_pad_left'): self.toggle_FSM_idle_state()
            if self.button_triggered('D_pad_right'): self.toggle_inspect_holonomic()


        # Mapping and vision commands, controllable when right trigger button (R2) is pressed 
        elif self.is_button_pressed('R2'):  # enable mapping and vision commands
            if self.button_triggered('select/back'): self.switch_map_interactively()
            if self.button_triggered('start/play'):  self.save_map_interactively()
            if self.button_triggered('cross/A'):     self.store_map_pose()

    def issue_motor_commands(self):
        # preserve bandwidth: only send arm commands when servo references have changed

        shoulder_scale = 0.02  #TODO: get scale from parameter file
        rotation_scale = 0.02  #TODO: get scale from parameter file

        if self.arm_rotation==0 and self.arm_shoulder==0:
            pass
        else:
            arm_ref_angles_list = list(self.arm_ref_angles.data)
            deviation = [abs(a - b) for a, b in zip(arm_ref_angles_list, self.arm_cur_angles.data)]  # make sure not to accept jumps that are too large, in case of network delays

            if deviation[0]<0.1: arm_ref_angles_list[0] += rotation_scale * -self.arm_rotation
            if deviation[1]<0.1: 
                arm_ref_angles_list[1] += shoulder_scale * -self.arm_shoulder
                arm_ref_angles_list[3] += shoulder_scale *  self.arm_shoulder  # keep absolute gripper orientation the same

            self.arm_ref_angles.data = arm_ref_angles_list 
            self.arm_ref_angles_publisher.publish(self.arm_ref_angles)
            #rospy.loginfo(f"arm ref angles are {self.arm_ref_angles.data}")

        # but always send drive commands to override any other drive commands, to be able to force the robot to stand still
        self.base_cmd_vel_publisher.publish(self.base_cmd_vel)


    def call_gripper_service(self,service_name):
        rospy.wait_for_service(service_name)
        try:
            service = rospy.ServiceProxy(service_name, Trigger)
            response = service()
            #rospy.loginfo(f"Service call to {service_name} succeeded: {response.success}, message: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to {service_name} failed: {e}")
            return False

    def switch_map_interactively(self):
        rospy.wait_for_service('/switch_map_interactively')
        try:
            service = rospy.ServiceProxy('/switch_map_interactively', SwitchMapInteractively)
            response = service()
            #rospy.loginfo(f"Service call to {service_name} succeeded: {response.success}, message: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to '/switch_map_interactively' failed: {e}")
            return False

    def save_map_interactively(self):
        rospy.wait_for_service('/save_map_interactively')
        try:
            service = rospy.ServiceProxy('/save_map_interactively', Trigger)
            response = service()
            #rospy.loginfo(f"Service call to {service_name} succeeded: {response.success}, message: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to '/save_map_interactively' failed: {e}")
            return False

    def store_map_pose(self):
        rospy.wait_for_service('/store_map_pose')
        try:
            service = rospy.ServiceProxy('/store_map_pose', Trigger)
            response = service()
            #rospy.loginfo(f"Service call to {service_name} succeeded: {response.success}, message: {response.message}")
            return response.success
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call to '/store_map_pose' failed: {e}")
            return False

    def playsound(self, wavfilename, volume):
        rospy.loginfo(f"Playing sound: {wavfilename}")
        self.wav_command.wavfile = wavfilename
        self.wav_command.sound_level = volume
        self.wav_file_name_publisher.publish(self.wav_command)

    def speak(self):
        rospy.loginfo("Speaking")

    def cancel_all_goals(self):
        rospy.loginfo("Canceling all goals")
        self.visual_servo_client.cancel_all_goals()
        self.visual_servo_running = False
        self.move_base_cancel_pub.publish(GoalID())

    def tilt_wrist(self, directionstring):
        scale = 0.05  #TODO: get scale from parameter file
        if directionstring=='up':
            direction = 1
            #rospy.loginfo("Tilting up wrist")
        elif directionstring=='down':
            direction = -1
            #rospy.loginfo("Tilting down wrist")
        else:
            rospy.error('non-existing wrist direction')

        arm_ref_angles_list = list(self.arm_ref_angles.data)
        arm_ref_angles_list[3] += scale * direction
        self.arm_ref_angles.data = arm_ref_angles_list 
        self.arm_ref_angles_publisher.publish(self.arm_ref_angles)
        #rospy.loginfo(f"arm ref angles are {self.arm_ref_angles.data}")

    def extend_arm(self, directionstring):
        scale = 0.05  #TODO: get scale from parameter file
        if directionstring=='extend':
            direction = 1
            #rospy.loginfo("Extending arm")
        elif directionstring=='contract':
            direction = -1
            #rospy.loginfo("Contracting arm")
        else:
            rospy.error('neither extendin nor contracting')

        arm_ref_angles_list = list(self.arm_ref_angles.data)
        arm_ref_angles_list[2] += scale *  direction
        arm_ref_angles_list[1] += scale * -direction * 0.5  # keep shoulder-to-wrist line in the same direction
        arm_ref_angles_list[3] += scale * -direction * 0.5  # keep absolute gripper angle the same
        self.arm_ref_angles.data = arm_ref_angles_list 
        self.arm_ref_angles_publisher.publish(self.arm_ref_angles)
        #rospy.loginfo(f"arm ref angles are {self.arm_ref_angles.data}")

    def arm_to_home(self):
        rospy.loginfo("Moving arm to home position")
        home = [0,-0.7,-1.4,0.7]
        self.arm_ref_angles.data = home
        self.arm_ref_angles_publisher.publish(self.arm_ref_angles)


    def speak_what_you_see(self):
        rospy.loginfo("Speaking what I see")
        self.speech_command_publisher.publish(String(data="this string is not used yet, only as trigger. Could become specific info to guide ChatGPT"))

    def pick_up_service(self):
        rospy.loginfo("Calling pick_up_service")

    def toggle_visual_servoing(self):
        if self.visual_servo_running:
            rospy.loginfo("Cancelling visual servoing action...")
            self.visual_servo_client.cancel_goal()
            self.visual_servo_running = False
        else:
            rospy.loginfo("Starting visual servoing action...")
            goal = VisualServoGoal()
            self.visual_servo_client.send_goal(goal, done_cb=self.visual_servo_done_callback)
            self.visual_servo_running = True

    def visual_servo_done_callback(self, state, result):
        rospy.loginfo("Visual servoing action finished with state: %d, result: %d", state, result.result)
        self.visual_servo_running = False

    def toggle_inspect_holonomic(self):
        self.inspect_holonomic_client.wait_for_server()
        if self.inspect_holonomic_running:
            rospy.loginfo("Cancelling inspect holonomic action...")
            self.inspect_holonomic_client.cancel_goal()
            self.inspect_holonomic_running = False
        else:
            rospy.loginfo("Starting inspect holonomic action...")
            target_pose = self.get_target_pose()
            if target_pose is not None:
                goal = InspectHolonomicGoal(target_pose=target_pose)
                self.inspect_holonomic_client.send_goal(goal, done_cb=self.inspect_holonomic_done_callback)
                self.inspect_holonomic_running = True

    def get_target_pose(self):
        try:
            now = rospy.Time.now()
            self.listener.waitForTransform("/map", "/base_link", now, rospy.Duration(4.0))
            (trans, rot) = self.listener.lookupTransform("/map", "/base_link", now)
            
            target_pose = PoseStamped()
            target_pose.header.frame_id = "map"
            target_pose.header.stamp = rospy.Time.now()
            
            # 0.7 meters in front of the robot
            yaw = tf.transformations.euler_from_quaternion(rot)[2]
            target_pose.pose.position.x = trans[0] + 0.7 * math.cos(yaw)
            target_pose.pose.position.y = trans[1] + 0.7 * math.sin(yaw)
            target_pose.pose.position.z = trans[2]
            target_pose.pose.orientation.x = rot[0]
            target_pose.pose.orientation.y = rot[1]
            target_pose.pose.orientation.z = rot[2]
            target_pose.pose.orientation.w = rot[3]
            
            return target_pose
        except (tf.LookupException, tf.ConnectivityException, tf.ExtrapolationException):
            rospy.logerr("Failed to get current robot pose")
            return None

    def inspect_holonomic_done_callback(self, state, result):
        rospy.loginfo("Inspect holonomic action finished with state: %d, result: %d", state, result.success)
        self.inspect_holonomic_running = False


    def toggle_FSM_idle_state(self):
        rospy.loginfo("Toggle FSM to idle state or to search state")
        self.fsm_idle_search_toggle_publisher.publish(Empty())
 
    def update_arm_cur_angles(self, data, joint_index):
        arm_angles_list = list(self.arm_cur_angles.data)
        arm_angles_list[joint_index] = data.angle
        self.arm_cur_angles.data = arm_angles_list 


if __name__ == "__main__":
    try:
        MobileManipulationTeleop()
    except rospy.ROSInterruptException:
        pass