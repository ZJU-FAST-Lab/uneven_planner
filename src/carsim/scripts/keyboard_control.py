#!/usr/bin/env python

import atexit
import os
import signal
from threading import Lock
from tkinter import Frame, Label, Tk
import time
import rospy
from ackermann_msgs.msg import AckermannDriveStamped
from geometry_msgs.msg import Twist

UP = "w"
LEFT = "a"
DOWN = "s"
RIGHT = "d"
QUIT = "q"

state = [False, False, False, False]
state_lock = Lock()
state_pub = None
root = None
control = False


def keyeq(e, c):
    return e.char == c or e.keysym == c


def keyup(e):
    global state
    global control

    with state_lock:
        if keyeq(e, UP):
            state[0] = False
        elif keyeq(e, LEFT):
            state[1] = False
        elif keyeq(e, DOWN):
            state[2] = False
        elif keyeq(e, RIGHT):
            state[3] = False
        control = sum(state) > 0


def keydown(e):
    global state
    global control

    with state_lock:
        if keyeq(e, QUIT):
            shutdown()
        elif keyeq(e, UP):
            state[0] = True
            state[2] = False
        elif keyeq(e, LEFT):
            state[1] = True
            state[3] = False
        elif keyeq(e, DOWN):
            state[2] = True
            state[0] = False
        elif keyeq(e, RIGHT):
            state[3] = True
            state[1] = False
        control = sum(state) > 0

def publish_cb(_):
    with state_lock:
        if not control:
            return
        ack = AckermannDriveStamped()
        if state[0]:
            command.linear.x = max_velocity
        elif state[2]:
            command.linear.x = -max_velocity

        if state[1]:
            command.angular.z = max_steering_angle
        elif state[3]:
            command.angular.z = -max_steering_angle

        if state_pub is not None:
            state_pub.publish(command)

def exit_func():
    os.system("xset r on")


def shutdown():
    root.destroy()
    rospy.signal_shutdown("shutdown")


def main():
    global state_pub
    global root
    global command
    global max_velocity
    global max_steering_angle
    
    max_velocity = 1
    max_steering_angle = 0.5

    state_pub = rospy.Publisher(
        "/racebot/cmd_vel", Twist, queue_size=1
    )
    
    command = Twist()
    
    rospy.Timer(rospy.Duration(0.05), publish_cb)
    
    atexit.register(exit_func)
    os.system("xset r off")

    root = Tk()
    frame = Frame(root, width=100, height=100)
    frame.bind("<KeyPress>", keydown)
    frame.bind("<KeyRelease>", keyup)
    frame.pack()
    frame.focus_set()
    lab = Label(
        frame,
        height=10,
        width=30,
        text="Focus on this window\nand use the WASD keys\nto drive the car.\n\nPress Q to quit",
    )
    lab.pack()
    print("Press %c to quit" % QUIT)
    root.mainloop()


if __name__ == "__main__":
    rospy.init_node("keyboard_control", disable_signals=True)

    signal.signal(signal.SIGINT, lambda s, f: shutdown())
    time.sleep(5) #sleep for 5 seconds to wait for spawning model
    main()
