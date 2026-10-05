#!/usr/bin/env python
# -*- coding: utf-8 -*-

import rospy
from std_msgs.msg import String


def callback(msg):
    """
    回调函数：处理接收到的触觉数据
    """
    rospy.loginfo(f"接收到触觉数据：\n{msg.data}")


def main():
    rospy.init_node("handcontrol_subscriber", anonymous=True)

    # 创建订阅者
    rospy.Subscriber("touch_data", String, callback)

    try:
        rospy.spin()  # 保持节点运行
    except rospy.ROSInterruptException:
        rospy.loginfo("触觉订阅节点被手动停止")


if __name__ == "__main__":
    main()

