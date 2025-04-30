#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
//#include <sensor_msgs>
using namespace std::chrono_literals;

bool active = true;

class joyreader : public rclcpp::Node
{
  public:
  joyreader(bool* manualbutt , bool* autobutt, bool* deadswitch) : Node("joyreader_node")
      {
        mbutt = manualbutt; //O button
        abutt = autobutt; //X button
        dswch = deadswitch; //A trigger button -- L1 bumper

        joy_sub = create_subscription<sensor_msgs::msg::Joy> (
          "joy", 10, std::bind(&joyreader::joyCallback, this, std::placeholders::_1)
        );
        
      }
  private:
    void joyCallback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
      bool obutton = (bool) msg->buttons[1];
      bool xbutton = (bool) msg->buttons[0];
      bool lbumper = (bool) msg->buttons[4];

      *mbutt = obutton;
      *abutt = xbutton;
      *dswch = lbumper;
    }


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    bool* mbutt;
    bool* abutt;
    bool* dswch; 
};

class reTwist : public rclcpp::Node
{
  public:
    reTwist(float* fspeed, float* rspeed) : Node("reTwist"), count_(0)
    {
      f = fspeed;
      r = rspeed;

      cmdVelSub = create_subscription<geometry_msgs::msg::Twist> (
        "joy_vel", 10, std::bind(&reTwist::cmdVelCallback, this, std::placeholders::_1)
      );

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
    void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double linearSpeed = msg->linear.x;
      double angularSpeed = msg->angular.z;
      
      *f = linearSpeed;
      *r = angularSpeed;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmdVelSub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ ;
    size_t count_;


    float* f;
    float* r;
};

//Deals with ctl+c handling to stop the motors correctly.
void my_handler(int s)
{
  printf("Caught signal %d\n",s);
  active = false;
} //maybe need this

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  
  bool manualbutton;
  bool autobutton;

  bool manualmode = false;
  bool automode = false;
  bool trigger;

  float fS = 0.0;
  float rS = 0.0;

  //rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

  auto jNode = std::make_shared<joyreader>(&manualbutton, &autobutton, &trigger);
  // printf("Vmaster is up\n");
  // RCLCPP_DEBUG(VNode->get_logger(),"Before Spin!...");

  //Make sure auto doesn't turn off when button is not depressed
  

  auto moveNode = std::make_shared<reTwist>(&fS, &rS);
  printf("reTwist is up\n");

  //rclcpp::spin(VNode);
  while(active)
  {
    if (manualmode)
    {
      rclcpp::spin_some(moveNode);
    }
    else if (automode)
    {
      if (trigger)
      {
        //spin the node to pass automatic cmd_vels
        //check if the trigger is active
        //rclcpp::spin_some();
      }
    }
    else
    {
      // Do Nothing
    }

    //check the buttons again
    rclcpp::spin_some(jNode);
    if (manualbutton && !manualmode)
    {
      manualmode = true;
      automode = false;
      printf("manual mode activated!!\n");
    }
    else if (autobutton && !automode)
    {
      automode = true;
      manualmode = false;
      printf("automatic mode activated!!\n");
    }
  }


  rclcpp::shutdown();
  printf("hello world theMaster is DOWN!\n");
  return 0;
}
