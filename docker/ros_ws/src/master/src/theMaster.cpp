#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>

class vMaster : public rclcpp:Node
{
    public:
    vMaster(float* fSpeed, float* rSpeed) : Node("vMaster")
        {
          currentFspeed = fSpeed;
          currentRspeed = rSpeed;

          cmdVelSub = create_subscription<geometry_msgs::msg::Twist> (
            "cmd_vel", 10, std::bind(&theMaster::cmdVelCallback, this, std::placeholders::_1)
          );
        }
    private:
        void cmdVelCallback(const geometry_msgs:msg::Twist::SharedPtr msg)
        {
          double linearSpeed = msg->linear.x;
          double angularSpeed = msg->angular.z;

          *currentFspeed = linearSpeed;
          *currentRspeed = angularSpeed;
        }

        rclcpp:Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
        float* currentFspeed;
        float* currentRspeed;
}

// Deals with ctl+c handling to stop the motors correctly.
void my_handler(int s){
  printf("Caught signal %d\n",s);
  stopRunning = true;
} //maybe need this

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto vel_out = std::make_shared<geometry_msgs::msg::Twist>("robot_vels");
  rclcpp::spin(vel_out);

  float fS = 0.0;
  float rS = 0.0;

  auto VNode = std::make_shared<vMaster>(&fS, &rS);
  RCLCPP_DEBUG(aNode->get_logger(),"Before Spin!...");
  rclcpp::spin(VNode);
  
  printf("hello world theMaster is up!\n");
  return 0;
}
