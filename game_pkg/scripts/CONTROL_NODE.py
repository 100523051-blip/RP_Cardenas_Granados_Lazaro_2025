#!/usr/bin/python3

import sys
import tty
import termios
import rospy
from std_msgs.msg import String


# Key mapping
KEYS = {
    '\x1b[D': "LEFT",      # Arrow left
    '\x1b[C': "RIGHT",     # Arrow right
    ' ': "SHOOT",          # Space Bar
    'q': "QUIT",
    'r': "RESTART",
    '\r': "RESUME",        # Enter
    'm': "MENU",
    'p': "PAUSE"
}


def get_key():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        ch1 = sys.stdin.read(1)

        # Arrow keys begin with ESC sequence: ESC + [ + A/B/C/D
        if ch1 == '\x1b':
            ch2 = sys.stdin.read(1)
            ch3 = sys.stdin.read(1)
            return ch1 + ch2 + ch3

        return ch1

    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def control_node():
    rospy.init_node("CONTROL_NODE")
    pub = rospy.Publisher("keyboard_control", String, queue_size=10)

    rospy.loginfo("CONTROL_NODE started.")
    rospy.loginfo(" ")
    rospy.loginfo("KEY CONTROL INSTRUCTIONS:")
    rospy.loginfo("Use ARROW KEYS to move ← →")
    rospy.loginfo("Use ASPACE BAR to shoot")
    rospy.loginfo("Use P to pause mid-gamne")
    rospy.loginfo("Use Q to quit game")

    rate = rospy.Rate(150)  # Frequency ro read valies

    while not rospy.is_shutdown():

        key = get_key()

        if key in KEYS:
            action = KEYS[key]
            msg = String(data=action)

            rospy.loginfo(f"CONTROL_NODE: {action}")
            pub.publish(msg)

            if action == "QUIT":
                rospy.signal_shutdown("User quit.")
                return

        rate.sleep()


if __name__ == "__main__":
    try:
        control_node()
    except rospy.ROSInterruptException:
        pass
