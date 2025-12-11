#!/usr/bin/env python3
import rospy
from game_pkg.srv import SetGameDifficulty, SetGameDifficultyRequest

class DifficultyClient:
    def __init__(self):
        rospy.init_node("difficulty_client_node")

    def get_dif(self, difficulty):
        rospy.wait_for_service("difficulty")
        try:
            score_srv = rospy.ServiceProxy("difficulty", SetGameDifficulty)
            req = SetGameDifficultyRequest()
            req.change_difficulty = difficulty
            resp = score_srv(req)
            return resp.success
        except rospy.ServiceException as e:
            rospy.logerr("Service call failed: %s", e)
            return None

if __name__ == "__main__":
    client = DifficultyClient()
    
    while not rospy.is_shutdown():
        difficulty = input("Enter difficulty easy, medium, hard: ")
        success = client.get_dif(difficulty)
        if not success:
            print("Incorrect difficulty level.")
        else:
            print(f"Difficulty set to {difficulty}")
        
        cont = input("Try again? (y/n): ")
        if cont.lower() != 'y':
            break
