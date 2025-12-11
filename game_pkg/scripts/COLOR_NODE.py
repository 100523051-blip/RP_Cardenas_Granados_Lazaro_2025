#!/usr/bin/env python3
import rospy
import time
from game_pkg.msg import user_msg

class Get_Color(object):
    def __init__(self,msg):
        self.msg=msg
        self.__pub = rospy.Publisher("color_information", user_msg, queue_size=10)
        time.sleep(2)
        self.main()
    
    def main(self):
        self.__pub.publish(msg)

if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            color = int(input("Enter color (1 Purple, 2 Green and 3 Orange):"))

            msg = user_msg()
            msg.color = color
            
            
            rospy.init_node("COLOR_NODE")
            rospy.loginfo("Publisher started...")
            node = Get_Color(msg)

            
        except rospy.ROSInterruptException:
            pass