#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/Joy.h"

class TeleopJoy
{
public:
    TeleopJoy(ros::NodeHandle &nh) : nh_(nh)
    {
        teleop_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 10);
        joy_sub_ = nh_.subscribe("joy", 10, &TeleopJoy::joyCallback, this);

        robot_vel_.linear.y = 0.0;
        robot_vel_.linear.z = 0.0;
        robot_vel_.angular.x = 0.0;
        robot_vel_.angular.y - 0.0;
    }

    void joyCallback(const sensor_msgs::Joy::ConstPtr &xbox)
    {
        // if (xbox->buttons[5])
        // {

        // }
        // else
        // {
        //     robot_vel_.linear.x = 0.0;
        //     robot_vel_.angular.z = 0.0;
        // }

        robot_vel_.linear.x = xbox->axes[7] * robot_linear_vel;
        robot_vel_.angular.z = xbox->axes[6] * robot_angular_vel;

        if (xbox->buttons[0] || xbox->buttons[3])
        {
            linear_flag = true;
        }

        if (linear_flag)
        {
            robot_linear_vel += (xbox->buttons[3] - xbox->buttons[0]) * increment;
            linear_flag = false;
            std::cout << "currently:\tspeed: " << std::fixed << std::setprecision(2) << robot_linear_vel << "\turn: " << std::setprecision(2) << robot_angular_vel << std::endl;
        }

        if (xbox->buttons[1] || xbox->buttons[2])
        {
            angular_flag = true;
        }

        if (angular_flag)
        {
            robot_angular_vel += (xbox->buttons[1] - xbox->buttons[2]) * increment * 5;
            angular_flag = false;
            std::cout << "currently:\tspeed: " << std::fixed << std::setprecision(2) << robot_linear_vel << "\turn: " << std::setprecision(2) << robot_angular_vel << std::endl;
        }
    }

    void update()
    {

        teleop_pub_.publish(robot_vel_);
    }

private:
    ros::NodeHandle nh_;
    ros::Publisher teleop_pub_;
    ros::Subscriber joy_sub_;
    geometry_msgs::Twist robot_vel_;

    float robot_linear_vel = 0.08;
    float robot_angular_vel = 0.4;
    float increment = 0.01;

    bool linear_flag = false;
    bool angular_flag = false;
};

int main(int argc, char **argv)
{
    std::cout << R"(
        +----------------------------------------------------------+
        | Reading from the JoyStick: XBOX and Publishing to Twist! |
        |                                                          |
        |                     Moving around:                       |
        |                            ↑                             |
        |                        ←       →                         |
        |                            ↓                             |
        |                                                          |
        |         Y : increase linear speed by 0.01 m/s            |
        |         A : decrease linear speed by 0.01 rad/s          |
        |         B : increase angular speed by 0.05 m/s           |
        |         X : decrease angular speed by 0.05 rad/s         |
        |                                                          |
        |                     CTRL-C to quit                       |
        +----------------------------------------------------------+
    )" << std::endl;
    ros::init(argc, argv, "teleop_joy");
    ros::NodeHandle nh;
    ros::Rate loop_rate(10);

    TeleopJoy teleop_joy(nh);

    while (ros::ok())
    {

        teleop_joy.update();
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}
