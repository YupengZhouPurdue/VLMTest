import os
import rospy
from std_msgs.msg import String

def find_trajectory_behaviors(directory):
    behaviors = []
    if not os.path.exists(directory):
        rospy.logwarn("Directory %s does not exist.", directory)
        return behaviors
    for fname in os.listdir(directory):
        if fname.endswith('.csv'):
            behaviors.append(os.path.splitext(fname)[0])
    return behaviors

def behavior_callback(event):
    # Repace the dirctory for your own where saved trajectories
    behaviors = find_trajectory_behaviors('Path')  
    pub.publish(str(behaviors))

if __name__ == '__main__':
    rospy.init_node('possible_behaviors_publisher')
    pub = rospy.Publisher('/possible_behaviors', String, queue_size=10)
    # Publish every 5 seconds
    timer = rospy.Timer(rospy.Duration(5), behavior_callback)
    rospy.loginfo("Publishing possible behaviors to /possible_behaviors topic.")
    rospy.spin()