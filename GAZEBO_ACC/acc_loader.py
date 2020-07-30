#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped 
from geometry_msgs.msg import PoseStamped 


EGO1_DATA = PoseStamped()
LEAD1_DATA = PoseStamped()

def callback_ego(data):
   #rospy.loginfo(rospy.get_caller_id())
   #rospy.loginfo(data)
   global EGO1_DATA
   EGO1_DATA = data
  

def callback_lead(data):
   #rospy.loginfo(rospy.get_caller_id())
   #rospy.loginfo(data)
   global LEAD1_DATA
   LEAD1_DATA = data
   

def launch(pub, vehicle):
    command_1 = "control create vehicle "+vehicle+" 0 -3 0 0 0.0 0.0 1 0"        
    pub.publish(command_1)
    rospy.sleep(1)
    
    command_4 = "control create controller "+vehicle+" SpeedController"
    pub.publish(command_4)
    rospy.sleep(1)

    command_5 = "control modify controller "+vehicle+" SpeedController vx 15.0 ax 2.0"
    pub.publish(command_5)
    rospy.sleep(1)

    command_6 = "control modify controller "+vehicle+" SpeedController activate"
    pub.publish(command_6)
    rospy.sleep(1)
   
def talker():
    
    pub = rospy.Publisher('/gazebo/scenario/command', String, queue_size=20)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    rospy.sleep(2)
    launch(pub,"lead1")
    launch(pub,"ego1") 
    
    pose_lead = "publisher ros_pose create lead1" 
    pub.publish(pose_lead)
    rospy.sleep(1)

    pose_ego = "publisher ros_pose create ego1" 
    pub.publish(pose_ego)
    rospy.sleep(1)

    lead_pose_sub = rospy.Subscriber('/lead1/gazebo/pose', PoseStamped, callback_lead)
    ego_pose_sub = rospy.Subscriber('/ego1/gazebo/pose', PoseStamped, callback_ego)

    rospy.sleep(2)

    while not rospy.is_shutdown():

        print("Reentered")
        command_9 = "control modify controller lead1 SpeedController vx 5.0 ax 2.0"
        pub.publish(command_9)
        rospy.sleep(1)

        command_10 = "control modify controller lead1 SpeedController activate"
        pub.publish(command_10)
        rospy.sleep(1)

        
        while ((LEAD1_DATA.pose.position.x - EGO1_DATA.pose.position.x) < 32.0 ) :
            

            command_11 = "control modify controller ego1 SpeedController vx 5.0 ax 10.0"
            pub.publish(command_11)
            rate.sleep()

            command_12 = "control modify controller ego1 SpeedController activate"
            pub.publish(command_12)
            rate.sleep()

           

            while((LEAD1_DATA.pose.position.x - EGO1_DATA.pose.position.x) > 25.0 ) :

                command_13 = "control modify controller lead1 SpeedController vx 15.0 ax 10.0"
                pub.publish(command_13)
                rate.sleep()

                command_14 = "control modify controller lead1 SpeedController activate"
                pub.publish(command_14)
                rate.sleep()

                command_15 = "control modify controller ego1 SpeedController vx 20.0 ax 3.0"
                pub.publish(command_15)
                rate.sleep()

                command_16 = "control modify controller ego1 SpeedController activate"
                pub.publish(command_16)
                rate.sleep()

                break
                
                



if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
