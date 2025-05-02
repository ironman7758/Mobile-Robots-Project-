#include <cstdio>
#include <chrono>
#include <functional>
#include <memory>
#include <string>
#include <signal.h>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
//#include <sensor_msgs>
using namespace std::chrono_literals;

bool active = true;

class joyreader : public rclcpp::Node
{
  public:
  joyreader(bool* manualbutt , bool* autobutt, bool* deadswitch, bool* cancel) : Node("joyreader_node")
      {
        mbutt = manualbutt; //O button
        abutt = autobutt; //X button
        dswch = deadswitch; //A trigger button -- L1 bumper
        sbutt = cancel; //triangle button

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
      bool tributton = (bool) msg->buttons[2];

      *mbutt = obutton;
      *abutt = xbutton;
      *dswch = lbumper;
      *sbutt = tributton;
    }


    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;
    bool* mbutt;
    bool* abutt;
    bool* dswch; 
    bool* sbutt;
};

class reTwist : public rclcpp::Node
{
  public:
    reTwist(float* fspeed, float* rspeed, int* mode, bool* deadswitch) : Node("reTwist"), count_(0)
    {
      m = mode;
      retf = fspeed;
      reta = rspeed;
      trigger = deadswitch;

      twistVelSub = create_subscription<geometry_msgs::msg::Twist> (
        "joy_vel", 10, std::bind(&reTwist::twistCallback, this, std::placeholders::_1)
      );
      navVelSub = create_subscription<geometry_msgs::msg::Twist> (
        "nav2_vel", 10, std::bind(&reTwist::navCallback, this, std::placeholders::_1)
      );
      publisher_  = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
        //Modes:
        //      0 deactivated
        //      1 manual mode
        //      2 automatic mode 
        //
        //  start in deactivated mode
        //  once a mode is selected on the controller
        //  allow movements to be passed through
        //  if the cancel button is pressed then
        //  stop all movements
      auto timer_callback = 
        [this]() -> void 
        {
          float empty = 0.0; //very important that this is defined
          auto message = geometry_msgs::msg::Twist();
          switch(*m)
          {
            case 1: //manual mode
              message.linear.x = *twistf; 
              message.angular.z = *twistr;
              retf = twistf; 
              reta = twistr;
              break;
            case 2: //automatic mode
              if(*trigger)
              {
                message.linear.x = *navf;
                message.angular.z = *navr;
                retf = navf; 
                reta = navr;
                break;
              }
            case 0: //deactivated
              message.linear.x = empty;
              message.angular.z = empty;
              *retf = empty;
              *reta = empty;
              break; 
          }
          this->publisher_ ->publish(message);
        };
      timer_ = this->create_wall_timer(500ms, timer_callback);
    }
  private:
    void twistCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double linearSpeed = msg->linear.x;
      double angularSpeed = msg->angular.z;
      
      *twistf = linearSpeed;
      *twistr = angularSpeed;
    }
    void navCallback(const geometry_msgs::msg::Twist::SharedPtr msg)
    {
      double linearSpeed = msg->linear.x;
      double angularSpeed = msg->angular.z;
      
      *twistf = linearSpeed;
      *twistr = angularSpeed;
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twistVelSub;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr navVelSub;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_ ;
    size_t count_;
    
    float* retf;
    float* reta;
    bool* trigger;

    float* twistf;
    float* twistr;
    float* navf;
    float* navr;
    int* m;
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

  int mode;

  bool manualmode = false;
  bool automode = false;
  bool trigger = false;
  bool stop = false;

  float fS = 0.0;
  float rS = 0.0;


  //rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub;

  auto jNode = std::make_shared<joyreader>(&manualbutton, &autobutton, &trigger, &stop);
  // printf("Vmaster is up\n");
  // RCLCPP_DEBUG(VNode->get_logger(),"Before Spin!...");

  //Make sure auto doesn't turn off when button is not depressed
  

  auto MoveNode = std::make_shared<reTwist>(&fS, &rS, &mode, &trigger);
  printf("reTwist is up\n");

  //rclcpp::spin(VNode);
  while(active)
  {

    rclcpp::spin_some(MoveNode);

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

    if (stop)
    {
      printf("emergency stop!!! please restart the node to regain functionality");
      break;
    }
  }


  rclcpp::shutdown();
  printf("hello world theMaster is DOWN!\n");
  return 0;
}
