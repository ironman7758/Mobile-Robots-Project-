/*  
*   A basic node for ros2 that runs with ariaCoda
*   To run use 'ros2 run ariaNode ariaNode -rp /dev/ttyUSB0'
*
*   Author: Kieran Quirke-Brown
*   Date: 12/01/2024
*/

#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>

# include "Aria/Aria.h"

//used with signal handler as signal handler function doesn't accept parameters
bool stopRunning = false;

using namespace std::chrono_literals;
/*
*   Basic ROS node that updates velocity of pioneer robot, Aria doesn't like
*   being spun as a node therefore we just use a single subscriber
*   parameters:
*       forward and ratation speeds are float that are bound to the node
*       but point at the same location as the aria velocities
*/
class ariaNode : public rclcpp::Node {
    public:
        ariaNode(float* forwardSpeed, float* rotationSpeed) : Node("Aria_node") {
            currentForwardSpeed = forwardSpeed;
            currentRotationSpeed = rotationSpeed;

            cmdVelSub = create_subscription<geometry_msgs::msg::Twist> (
                "cmd_vel", 10, std::bind(&ariaNode::cmdVelCallback, this, std::placeholders::_1)
            );    
            odom_pub_ = create_publisher<nav_msgs::msg::Odometry>("odom", 10);
            tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
            robot_ = nullptr;  // This will be set from main()
        }
        void publish_odometry() {
            if (!robot_) return;
       
            double x = robot_->getX() / 1000.0;  // mm to m
            double y = robot_->getY() / 1000.0;
            double theta = ArMath::degToRad(robot_->getTh());
       
            double vx = robot_->getVel() / 1000.0;
            double vtheta = ArMath::degToRad(robot_->getRotVel());
       
            rclcpp::Time now = get_clock()->now();
       
            // TF
            geometry_msgs::msg::TransformStamped tf_msg;
            tf_msg.header.stamp = now;
            tf_msg.header.frame_id = "odom";
            tf_msg.child_frame_id = "base_link";
            tf_msg.transform.translation.x = x;
            tf_msg.transform.translation.y = y;
            tf_msg.transform.translation.z = 0.0;
            tf_msg.transform.rotation = tf2::toMsg(tf2::Quaternion(0, 0, sin(theta / 2), cos(theta / 2)));
            tf_broadcaster_->sendTransform(tf_msg);
           
            // Odometry
            nav_msgs::msg::Odometry odom;
            odom.header.stamp = now;
            odom.header.frame_id = "odom";
            odom.child_frame_id = "base_link";
            odom.pose.pose.position.x = x;
            odom.pose.pose.position.y = y;
            odom.pose.pose.orientation = tf_msg.transform.rotation;
            odom.twist.twist.linear.x = vx;
            odom.twist.twist.angular.z = vtheta;
       
            odom_pub_->publish(odom);
        }
        void setRobot(ArRobot* robot) {
            robot_ = robot;
        }

    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg) {
           
            double linearSpeed = msg->linear.x;
            double angularSpeed = msg->angular.z;

            *currentForwardSpeed = linearSpeed;
            *currentRotationSpeed = angularSpeed;

            RCLCPP_DEBUG(this->get_logger(), "message received.");

        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
        float* currentForwardSpeed;
        float* currentRotationSpeed;

        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
        std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
        ArRobot* robot_;
   
};

// Deals with ctl+c handling to stop the motors correctly.
void my_handler(int s){
           printf("Caught signal %d\n",s);
           stopRunning = true;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    Aria::init();
    ArArgumentParser parser(&argc, argv);
    parser.loadDefaultArguments();
    ArRobot* robot;
    robot = new ArRobot();

    signal(SIGINT, my_handler);
   
    // RCLCPP_DEBUG(this->get_logger(),"Trying to connect to robot...");
    ArRobotConnector robotConnector(&parser, robot);
    if(!robotConnector.connectRobot()) {
        ArLog::log(ArLog::Terse, "simpleConnect: Could not connect to the robot.");
        if(parser.checkHelpAndWarnUnparsed()) {
            Aria::logOptions();
            Aria::exit(1);
        }
    }

    robot->setAbsoluteMaxTransVel(400);

    float forwardSpeed = 0.0;
    float rotationSpeed = 0.0;
   
   
    // RCLCPP_DEBUG(aNode->get_logger(),"Run Async");
    robot->runAsync(true);
    // RCLCPP_DEBUG(aNode->get_logger(),"Enable Motors");
    robot->enableMotors();

    auto aNode = std::make_shared<ariaNode>(&forwardSpeed, &rotationSpeed);
    aNode->setRobot(robot);
    RCLCPP_DEBUG(aNode->get_logger(),"Before Spin!...");

    /*
     *   Aria does not like to run in a ros node therefore we run a while loop
     *   that continuously spins the node to update velocities which are
     *   then sent using the normal Aria commands.
    */
    while (!stopRunning) {
        rclcpp::spin_some(aNode);
        // RCLCPP_DEBUG(aNode->get_logger(), "sending motor command.");
            robot->lock();
            robot->setVel(forwardSpeed * 500);
            robot->setRotVel(rotationSpeed * 50);
            aNode->publish_odometry();
            robot->unlock();
            // RCLCPP_DEBUG(aNode->get_logger(), "motor command sent.");
            // RCLCPP_DEBUG(aNode->get_logger(), "forward speed is now %f.", forwardSpeed);
            // RCLCPP_DEBUG(aNode->get_logger(), "rotational speed is now %f.", rotationSpeed);
    }
    RCLCPP_DEBUG(aNode->get_logger(), "After Spin");

    robot->disableMotors();
    robot->stopRunning();
    // wait for the thread to stop
    robot->waitForRunExit();

    // exit
    RCLCPP_DEBUG(aNode->get_logger(), "ending Aria node");
    Aria::exit(0);
    return 0;
}
