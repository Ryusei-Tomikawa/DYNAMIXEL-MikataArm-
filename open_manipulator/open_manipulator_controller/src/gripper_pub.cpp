
#include <ros/ros.h>
#include <std_msgs/Float64.h>

ros::Publisher gripper_joint_sub_pub;

void gripperJointCallback(const std_msgs::Float64::ConstPtr& msg)
{
  std_msgs::Float64 flip;
  flip.data = -msg->data;
  gripper_joint_sub_pub.publish(flip);
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "gripper_pub");
    ros::NodeHandle nh;

    ros::Publisher gripper_joint_pub;

    gripper_joint_pub = nh.advertise<std_msgs::Float64>("gripper_position/command", 10);

    std_msgs::Float64 gripper_joint_;

    float count = 0;
    ros::Rate rate(1.0);
    while (ros::ok())
    {
        count += 0.05;
        gripper_joint_.data = count; 
        gripper_joint_pub.publish(gripper_joint_);
        rate.sleep();

        if (count == 3.0)
        {
            count = 0;
        }

        ROS_INFO("count:=%3f", count);
        
    }
  

  return 0;
}
