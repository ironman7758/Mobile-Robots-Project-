#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
using namespace std::chrono_literals;


class vMaster : public rclcpp::Node
{
    public:
    vMaster(float* fSpeed, float* rSpeed) : Node("vMaster")
        {
          currentFspeed = fSpeed;
          currentRspeed = rSpeed;

          cmdVelSub = create_subscription<geometry_msgs::msg::Twist> (
            "joy_vel", 10, std::bind(&vMaster::cmdVelCallback, this, std::placeholders::_1)
          );
        }
    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
        {
          double linearSpeed = msg->linear.x;
          double angularSpeed = msg->angular.z;

          *currentFspeed = linearSpeed;
          *currentRspeed = angularSpeed;
        }

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;
        float* currentFspeed;
        float* currentRspeed;
};

class reTwist : public rclcpp::Node
{
  public:
  reTwist(bool* active, float* fspeed, float* rspeed): Node("reTwist"), count_(0)
  {
    a = active;
    f = fspeed;
    r = rspeed;
    publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    auto timer_callback = 
      [this]() -> void 
      {
        auto message = geometry_msgs::msg::Twist();
        message.linear.x = *f;
        message.angular.z = *r;
        this->publisher_ ->publish(message);
      };
    timer_ = this->create_wall_timer(500ms, timer_callback);
  }
  private:
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ ;
  size_t count_;


  bool* a;
  float* f;
  float* r;
};

// Deals with ctl+c handling to stop the motors correctly.
// void my_handler(int s){
//   printf("Caught signal %d\n",s);
//   stopRunning = true;
// } //maybe need this

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  bool activebutton = true;

  float fS = 0.0;
  float rS = 0.0;

  auto VNode = std::make_shared<vMaster>(&fS, &rS);
  RCLCPP_DEBUG(VNode->get_logger(),"Before Spin!...");
  rclcpp::spin(VNode);

  auto moveNode = std::make_shared<reTwist>(&activebutton, &fS, &rS);
  //rclcpp::spin(moveNode);

  rclcpp::shutdown();
  printf("hello world theMaster is DOWN!\n");
  return 0;
}
