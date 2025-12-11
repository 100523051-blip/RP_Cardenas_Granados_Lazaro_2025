#!/usr/bin/env python3

from __future__ import print_function
import rospy
from game_pkg.srv import GetUserScore, GetUserScoreRequest
from game_pkg.msg import user_msg
from std_msgs.msg import Int64

class ResultNode:
    def __init__(self):
        rospy.init_node("RESULT_NODE")
        
        # Subscribers
        self.__sub_user_info = rospy.Subscriber("user_information", user_msg, self.__callback_user_info) #Username
        self.__sub_result_info = rospy.Subscriber("result_information", Int64, self.__callback_result_info) #Score
        rospy.loginfo("Subscribers initialized...")
        
        # Store received data
        self.user_info = None
        self.last_score = None
        
    def __callback_user_info(self, msg):
        info = {
            "name": msg.name,
            "username": msg.username,
            "age": msg.age
        }
        self.user_info = info
        rospy.loginfo("Received user info:")
        print("Name: ", info["name"])
        print("Username: ", info["username"])
        print("Age: ", info["age"])
        
    def __callback_result_info(self, msg):
        score = msg.data
        self.last_score = score
        rospy.loginfo("Received score info:")
        print("Score: ", score)
        
    def get_score(self, username):
        rospy.wait_for_service("score")
        try:
            score_srv = rospy.ServiceProxy("score", GetUserScore)
            req = GetUserScoreRequest()
            req.username = username
            resp = score_srv(req)
            return resp.score
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None
    
    def run(self):
        while not rospy.is_shutdown():
            username = input("\nEnter username: ")
            if username.lower() == 'quit' or username.lower() == 'exit':
                break
                
            score = self.get_score(username)
            if score == -1:
                print("Username incorrect or not found.")
            elif score is None:
                print("Failed to get score from service.")
            else:
                print(f"Score for {username}: {score}")
            
            cont = input("Check another username? (y/n): ")
            if cont.lower() != 'y':
                break

if __name__ == "__main__":
    try:
        node = ResultNode()
        
        # Start rospy.spin in a separate thread so callbacks can run
        import threading
        
        def spin_thread():
            rospy.spin()
        
        spin_thread_obj = threading.Thread(target=spin_thread, daemon=True)
        spin_thread_obj.start()
        
        # Give ROS a moment to initialize
        rospy.sleep(0.5)
        
        # Run the interactive loop
        print("Node started. Listening for user info and scores...")
        node.run()
        
    except rospy.ROSInterruptException:
        pass
    except KeyboardInterrupt:
        print("\nShutting down...")

