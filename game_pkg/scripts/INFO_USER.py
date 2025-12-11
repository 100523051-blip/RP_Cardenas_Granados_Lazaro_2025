#!/usr/bin/python3

import rospy
import time
from game_pkg.msg import user_msg

class Get_Info(object):
    def __init__(self,msg):
        self.msg=msg
        self.__pub = rospy.Publisher("user_information", user_msg, queue_size=10)
        time.sleep(2)
        self.main()
    
    def main(self):
        self.__pub.publish(msg)
        
if __name__ == "__main__":
    while not rospy.is_shutdown():
        try:
            name = input("Enter your name: ")
            username = input("Enter your username: ")
            age = int(input("Enter your age: "))

            msg = user_msg()
            msg.name = name
            msg.username = username
            msg.age = age
            
            
            rospy.init_node("INFO_USER_NODE")
            rospy.loginfo("Publisher started...")
            node = Get_Info(msg)

            
        except rospy.ROSInterruptException:
            pass