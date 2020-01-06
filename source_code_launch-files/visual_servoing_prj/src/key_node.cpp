#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include <nav_msgs/Odometry.h>
#include <visualization_msgs/Marker.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseArray.h>

 using namespace ros;
 using namespace nav_msgs;
 using namespace std;

#define KEYCODE_R 0x43
#define KEYCODE_L 0x44
#define KEYCODE_UP 0x41
#define KEYCODE_DOWN 0x42
#define KEYCODE_A 0x61
#define KEYCODE_D 0x64
#define KEYCODE_S 0x73
#define KEYCODE_W 0x77
#define KEYCODE_Q 0x71
#define KEYCODE_E 0x65
#define KEYCODE_SPACE 0x20

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

using namespace std;

class MoveTurtlebot
{
public:
  MoveTurtlebot();
  void keyLoop();
  void VisPoseCallback(const geometry_msgs::PoseArrayConstPtr& msg);

private:
  ros::NodeHandle nh_;
  double linear_, angular_;
  double l_scale_, a_scale_;
  ros::Publisher pubTwist;
  ros::Publisher pubGoal;
  ros::Subscriber pose_sub;
  ros::Publisher marker_pub;
};

MoveTurtlebot::MoveTurtlebot():
  linear_(0),
  angular_(0),
  l_scale_(1.0),
  a_scale_(1.0)
{
  nh_.param("scale_angular", a_scale_, a_scale_);
  nh_.param("scale_linear", l_scale_, l_scale_);
  pubGoal = nh_.advertise<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10);
  pubTwist = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 10);
  marker_pub = nh_.advertise<visualization_msgs::Marker>("visualization_marker", 10);
  pose_sub = nh_.subscribe<geometry_msgs::PoseArray>("/whycon/poses", 1, &MoveTurtlebot::VisPoseCallback, this);
}



int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  (void)sig;
  tcsetattr(kfd, TCSANOW, &cooked);
  ros::shutdown();
  exit(0);
}

void MoveTurtlebot::keyLoop()
{
  char key;
  bool move = false;

  tcgetattr(kfd, &cooked);

  memcpy(&raw, &cooked, sizeof(struct termios));

  raw.c_lflag &=~ (ICANON | ECHO);
  raw.c_cc[VEOL] = 1;
  raw.c_cc[VEOF] = 2;
  tcsetattr(kfd, TCSANOW, &raw);

  puts("Reading from keyboard");
  puts("---------------------------");
  puts("Use arrow keys to move the turtle.");

  int i;
  for(;;){
    if(read(kfd, &key, 1) < 0) {
      perror("read():");
      exit(-1);
    }

    linear_=angular_=0;
    ROS_DEBUG("value: 0x%02X\n", key);

	
    geometry_msgs::Twist twist;
    //geometry_msgs::Vector3 zero;
    //zero.x = zero.y = zero.z = 0.0;


	move_base_msgs::MoveBaseGoal goal;
	MoveBaseClient ac("move_base", true);

	while(!ac.waitForServer(ros::Duration(5.0))){
	ROS_INFO("Waiting for the move_base action server to come up");
	}

	//we'll send a goal to the robot to move 1 meter forward
	goal.target_pose.header.frame_id = "map";
	goal.target_pose.header.stamp = ros::Time::now();

	goal.target_pose.pose.position.x = -2.66862463951;
	goal.target_pose.pose.position.y = 0.0448093414307;
	goal.target_pose.pose.position.z = 0;

	goal.target_pose.pose.orientation.x = 0;
	goal.target_pose.pose.orientation.y = 0;
	goal.target_pose.pose.orientation.z = 0.857130809864;
	goal.target_pose.pose.orientation.w = 0.51509880099;

   

switch(key) {

      case KEYCODE_UP:
        ROS_DEBUG("scale-up");
	//l_scale_ += 0.1;
        twist.linear.x = l_scale_*1.0;
        cout << "scale-up: " << l_scale_ << endl;
        move = true;
        break;

      case KEYCODE_DOWN:
        ROS_DEBUG("scale-down");
        //l_scale_ -= 0.1;
        twist.linear.x = l_scale_*-1.0;
	cout << "scale-down" << l_scale_ << endl;
        move = true;
        break;

      case KEYCODE_W:
        ROS_DEBUG("FRONT");
        twist.angular.z = a_scale_*-1.0;
	cout << "FRONT" << twist.linear.x << endl;
        move = true;
        break;

      case KEYCODE_S:
        ROS_DEBUG("BACK");
        twist.angular.z = a_scale_*1.0;
	cout << "BACK" << twist.linear.x <<  endl;
        move = true;
        break;
     
      case KEYCODE_SPACE:
	ROS_INFO("Sending goal");
	ac.sendGoal(goal);
	ac.waitForResult();
        //move = false;
        break;
    }

     

    if(move == true) {

      pubTwist.publish(twist);
      move=false;
    }
  }

  return;
}


void MoveTurtlebot::VisPoseCallback(const geometry_msgs::PoseArrayConstPtr& msg)
{
	const std::vector<geometry_msgs::Pose>& ps = msg->poses;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "/whycon";

	marker.header.stamp = ros::Time::now();
	marker.ns = "lines";
	marker.pose.orientation.w = 1.0;
	marker.id = 0; //tag id
	marker.type = visualization_msgs::Marker::LINE_STRIP;  //tag type
	marker.scale.x = 0.05;
	marker.color.b = 1.0;
	marker.color.a = 1.0;

	geometry_msgs::Point p;
	p.x = ps[0].position.x;
	p.y = ps[0].position.y;
	p.z = ps[0].position.z;
	marker.points.push_back(p);
        marker_pub.publish(marker);
}



int main(int argc, char** argv)
{
  ros::init(argc, argv, "key_teleop");
  MoveTurtlebot move_turtlebot;

  signal(SIGINT,quit);

  move_turtlebot.keyLoop();

  return(0);
}



