#! /usr/bin/env python3
# -*- coding: utf-8 -*-

import actionlib
import os
import rospy

from gx_sound_msgs.msg import SoundRequestAction, SoundRequestGoal


def main():
    rospy.init_node('sound_request_client_node')
    client = actionlib.SimpleActionClient('/gx_sound_player/sound_player/sound_request', SoundRequestAction)
    client.wait_for_server()
    rospy.loginfo("connected to actionlib server")

    base_dir = os.path.dirname(os.path.abspath(__file__))
    now = rospy.get_rostime()

    # 逆順に送信しても時刻順に再生される
    goal1 = SoundRequestGoal(stamp=rospy.Time(secs=now.to_sec() + 3.0), file=os.path.join(base_dir, "../data/audio_robot.wav"))

    rospy.loginfo("send request")

    rate = rospy.Rate(0.65)
    while not rospy.is_shutdown():
        # Fill in the goal here
        client.send_goal(goal1)
        rate.sleep()



if __name__ == '__main__':
    main()
