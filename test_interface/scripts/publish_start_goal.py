#!/usr/bin/env python3

import rospy
from geometry_msgs.msg import PoseStamped


def main():
    rospy.init_node("publish_start_goal", anonymous=False)

    topic = rospy.get_param("~topic")
    delay = float(rospy.get_param("~delay", 5.0))
    hold_duration = float(rospy.get_param("~hold_duration", 4.0))
    publish_period = float(rospy.get_param("~publish_period", 0.5))
    x = float(rospy.get_param("~x"))
    y = float(rospy.get_param("~y"))
    z = float(rospy.get_param("~z", 0.3))
    yaw_w = float(rospy.get_param("~w", 1.0))

    pub = rospy.Publisher(topic, PoseStamped, queue_size=1, latch=True)
    rospy.sleep(delay)

    end_time = rospy.Time.now() + rospy.Duration.from_sec(hold_duration)
    rate = rospy.Rate(max(1.0 / max(publish_period, 1e-3), 1.0))
    publish_count = 0
    while not rospy.is_shutdown() and rospy.Time.now() < end_time:
        msg = PoseStamped()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = "world"
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.position.z = z
        msg.pose.orientation.w = yaw_w
        pub.publish(msg)
        publish_count += 1
        rate.sleep()

    rospy.loginfo(
        "Published startup goal %d times to %s: (%.2f, %.2f, %.2f)",
        publish_count,
        topic,
        x,
        y,
        z,
    )
    rospy.sleep(0.2)


if __name__ == "__main__":
    main()
