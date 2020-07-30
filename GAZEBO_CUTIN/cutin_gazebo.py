#!/usr/bin/env python
# license removed for brevity
import rospy
from std_msgs.msg import String
from geometry_msgs.msg import Vector3Stamped 
from geometry_msgs.msg import PoseStamped 
import matplotlib.pyplot as plt

#Initialization of variables

EGO1_DATA = PoseStamped()
LEAD1_DATA = PoseStamped()
CUT1_DATA = PoseStamped()

ego_velocity = []
lead_velocity = []
cutin_velocity = []
ego_time = []
lead_time = []
cutin_time = []

#call back functions for the rosdata 
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


def callback_cut(data):
   #rospy.loginfo(rospy.get_caller_id())
   #rospy.loginfo(data)
   global CUT1_DATA
   CUT1_DATA = data

def callback_ego_vel(data):
   #rospy.loginfo(rospy.get_caller_id())
   #rospy.loginfo(data)
   global EGO1_VELDATA
   EGO1_VELDATA = data
   ego_velocity.append(data.vector.x)
   ego_time.append(data.header.stamp.secs)

def callback_lead_vel(data):
   #rospy.loginfo(rospy.get_caller_id())
   #rospy.loginfo(data)
   global LEAD1_DATA
   LEAD1_VELDATA = data
   lead_velocity.append(data.vector.x)
   lead_time.append(data.header.stamp.secs)


def callback_cut_vel(data):
   #rospy.loginfo(rospy.get_caller_id())
   #rospy.loginfo(data)
   global CUT1_DATA
   CUT1_VELDATA = data
   cutin_velocity.append(data.vector.x)
   cutin_time.append(data.header.stamp.secs)


#Launch the vehicles
def launch(pub, vehicle):
    if(vehicle == "cut1"):
        command_1 = "control create vehicle "+vehicle+" 0 -3.0 0 0 0.0 0.0 1 0"        
        pub.publish(command_1)
        rospy.sleep(1)

        command_2 = "control create controller "+vehicle+" SpeedController"
        pub.publish(command_2)
        rospy.sleep(1)

        command_3 = "control modify controller "+vehicle+" SpeedController vx 15.0 ax 2.0"
        pub.publish(command_3)
        rospy.sleep(1)

        command_4 = "control modify controller "+vehicle+" SpeedController activate"
        pub.publish(command_4)
        rospy.sleep(1)    
    
    else:

        command_1 = "control create vehicle "+vehicle+" 0 -6.5 0 0 0.0 0.0 1 0"        
        pub.publish(command_1)
        rospy.sleep(1)
    
        command_2 = "control create controller "+vehicle+" SpeedController"
        pub.publish(command_2)
        rospy.sleep(1)

        command_3 = "control modify controller "+vehicle+" SpeedController vx 10.0 ax 2.0"
        pub.publish(command_3)
        rospy.sleep(1)

        command_4 = "control modify controller "+vehicle+" SpeedController activate"
        pub.publish(command_4)
        rospy.sleep(1)    

#Function to continuously check ACC
def updateACC(pub, rate):

    command_9 = "control modify controller lead1 SpeedController vx 10.0 ax 2.0"
    pub.publish(command_9)
    rospy.sleep(1)

    command_10 = "control modify controller lead1 SpeedController activate"
    pub.publish(command_10)
    rospy.sleep(1)

    command_9 = "control modify controller ego1 SpeedController vx 10.0 ax 2.0"
    pub.publish(command_9)
    rospy.sleep(1)

    command_10 = "control modify controller ego1 SpeedController activate"
    pub.publish(command_10)
    rospy.sleep(1)

    command_9 = "control modify controller cut1 SpeedController vx 15.0 ax 2.0"
    pub.publish(command_9)
    rospy.sleep(1)

    command_10 = "control modify controller cut1 SpeedController activate"
    pub.publish(command_10)
    rospy.sleep(1)

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
   
def talker():
    #create rosnode
    pub = rospy.Publisher('/gazebo/scenario/command', String, queue_size=20)
    rospy.init_node('talker', anonymous=True)
    rate = rospy.Rate(1) # 10hz

    rospy.sleep(2)
    
    launch(pub, "lead1")
    launch(pub, "ego1")
    launch(pub, "cut1")

    #Publish the ros velocity and pose for the vehicles    
    pose_cut = "publisher ros_pose create cut1" 
    pub.publish(pose_cut)
    rospy.sleep(1)

    pose_ego = "publisher ros_pose create ego1" 
    pub.publish(pose_ego)
    rospy.sleep(1)

    pose_lead = "publisher ros_pose create lead1" 
    pub.publish(pose_lead)
    rospy.sleep(1)


    speed_cut = "publisher ros_velocity create cut1"
    pub.publish(speed_cut)
    rospy.sleep(1)

    speed_lead = "publisher ros_velocity create lead1"
    pub.publish(speed_lead)
    rospy.sleep(1)

    speed_ego = "publisher ros_velocity create ego1"
    pub.publish(speed_ego)
    rospy.sleep(1)
    
    #Subscribe to the ros topics of velocity and pose
    cut_pose_sub = rospy.Subscriber('/cut1/gazebo/pose', PoseStamped, callback_cut)
    ego_pose_sub = rospy.Subscriber('/ego1/gazebo/pose', PoseStamped, callback_ego)
    lead_pose_sub = rospy.Subscriber('/lead1/gazebo/pose', PoseStamped, callback_lead)

    cut_velocity_sub = rospy.Subscriber('/cut1/gazebo/velocity', Vector3Stamped, callback_cut_vel)
    ego_velocity_sub = rospy.Subscriber('/ego1/gazebo/velocity', Vector3Stamped, callback_ego_vel)
    lead_velocity_sub = rospy.Subscriber('/lead1/gazebo/velocity', Vector3Stamped, callback_lead_vel)




    rospy.sleep(2)

    command_7 = "control modify controller ego1 SpeedController vx 11.0 ax 2.0"
    pub.publish(command_7)
    rospy.sleep(1)

    command_8 = "control modify controller ego1 SpeedController activate"
    pub.publish(command_8)
    rospy.sleep(1)

    CUTIN_SET = False

    # Perform Cut-In 
    while not CUTIN_SET:
        
        
        if((CUT1_DATA.pose.position.x - LEAD1_DATA.pose.position.x) > -5.0 ) :

            command_5 = "control create controller cut1 LaneChange tF 5.0 yF -3.5"
            pub.publish(command_5)
            rospy.sleep(1)

            command_6 = "control modify controller cut1 LaneChange activate"
            pub.publish(command_6)
            rospy.sleep(1)

            command_7 = "control modify controller cut1 SpeedController vx 10.0 ax 2.0"
            pub.publish(command_7)
            rospy.sleep(1)

            command_8 = "control modify controller cut1 SpeedController activate"
            pub.publish(command_8)
            rospy.sleep(1)

            CUTIN_SET = True

    while not rospy.is_shutdown():

        # Modify the behavior of other vehicles based on Cut-in

        while((CUT1_DATA.pose.position.x - LEAD1_DATA.pose.position.x) < 15.0 ) :
            print("Lead control")
            command_9 = "control modify controller lead1 SpeedController vx 5.0 ax 2.0"
            pub.publish(command_9)
            

            command_10 = "control modify controller lead1 SpeedController activate"
            pub.publish(command_10)
       

            if ((LEAD1_DATA.pose.position.x - EGO1_DATA.pose.position.x) < 20.0 ) :
                print("Ego Control")
                command_21 = "control modify controller ego1 SpeedController vx 3.0 ax 2.0"
                pub.publish(command_21)
                rospy.sleep(1)

                command_22 = "control modify controller ego1 SpeedController activate"
                pub.publish(command_22)
                rospy.sleep(1)

        
        # Continuous enabling of ACC
        updateACC(pub, rate)
        


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        pass
    #Plot the velocity profile
    finally:

        fig, axs = plt.subplots(3)
        fig.suptitle('velocity profile of vehicles')
        axs[0].plot(cutin_time, cutin_velocity)
        axs[0].set_title("Cut-in Vehicle")
        axs[1].plot(lead_time, lead_velocity)
        axs[1].set_title("lead Vehicle")
        axs[2].plot(ego_time, ego_velocity)
        axs[2].set_title("Ego Vehicle")

        for ax in axs.flat:
            ax.set(xlabel = 'Time', ylabel = 'velocity')

        plt.show()
        plt.savefig('foo.png')
