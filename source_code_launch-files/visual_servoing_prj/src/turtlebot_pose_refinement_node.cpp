#include <ros/ros.h>
#include <ros/console.h>
#include <std_msgs/Int8.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>

#include <nav_msgs/Path.h>
#include <signal.h>
#include <termios.h>
#include <stdio.h>

#include <visp_bridge/3dpose.h>
#include <kobuki_msgs/Led.h>
#include <kobuki_msgs/Sound.h>
#include <visp/vpAdaptiveGain.h>
#include <visp/vpCameraParameters.h>
#include <visp/vpDot.h>
#include <visp/vpDot2.h>
#include <visp/vpFeatureBuilder.h>
#include <visp/vpFeatureDepth.h>
#include <visp/vpFeaturePoint.h>
#include <visp/vpHomogeneousMatrix.h>
#include <visp/vpPioneerPan.h>
#include <visp/vpRobot.h>
#include <visp/vpServo.h>
#include <visp/vpVelocityTwistMatrix.h>
#include <actionlib/server/simple_action_server.h>

class VS
{
private:
  ros::NodeHandle n;
  ros::Publisher  pubTwistRobot; // cmd_vel
  ros::Subscriber subPoseTarget_;  // pose_stamped
  ros::Subscriber subStatusTarget_;  // pose_stamped
  ros::Publisher mobile_base_led2; // led2 indicator
  ros::Publisher mobile_base_led1; // led1 indicator
  ros::Publisher mobile_base_sound; // sound indicator
  ros::Subscriber move_base_status_sub;



  vpServo task;
  // Current and desired visual feature associated to the x coordinate of the point
  vpFeaturePoint s_x, s_xd;
  vpFeatureDepth s_Z, s_Zd;
  kobuki_msgs::LedPtr led_msg_ptr;
  kobuki_msgs::SoundPtr sound_msg_ptr;
  vpCameraParameters cam;
  double depth;
  double Z, Zd;
  double lambda;
  
  bool valid_pose;
  bool valid_pose_prev;
  bool nav_status;

  double t_start_loop;
  double tinit;
  bool onSound;
  bool offSound;
  
  vpColVector v;
  vpColVector vi;
  double qm_pan; // Measured pan position (tilt is not handled in that example)
  double mu;
  vpAdaptiveGain lambda_adapt;
  vpPioneerPan robot;

public:
  void init_vs();
  void poseCallback(const geometry_msgs::PoseStampedConstPtr& msg);
  void move_base_statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg);
  void statusCallback(const std_msgs::Int8ConstPtr& msg);
  void navGoal();
  VS(int argc, char**argv);
  virtual ~VS() {
    task.kill();
  };
};

VS::VS(int argc, char**argv)
{
  init_vs();

  subPoseTarget_   = n.subscribe("/visp_auto_tracker/object_position", 10, &VS::poseCallback, this);
  move_base_status_sub = n.subscribe<actionlib_msgs::GoalStatusArray>("/move_base/status", 10, &VS::move_base_statusCallback, this);
  subStatusTarget_ = n.subscribe("/visp_auto_tracker/status", 10, &VS::statusCallback, this);

  pubTwistRobot  = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/navi", 10);
  mobile_base_led2 = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led2", 10);
  mobile_base_led1 = n.advertise<kobuki_msgs::Led>("/mobile_base/commands/led1", 10);
  mobile_base_sound = n.advertise<kobuki_msgs::Sound>("/mobile_base/commands/sound", 10);
}


void VS::init_vs()
{
  nav_status = false;
  depth = 0.225;
  lambda = 1.;
  valid_pose = false;
  valid_pose_prev = false;

  Z = Zd = depth;

  v.resize(2);
  vi.resize(2);
  v = 0; vi = 0;
  mu = 4;
  qm_pan = 0;

  //lambda_adapt.initStandard(4, 0.5, 40);
  lambda_adapt.initStandard(3.5, 1.5, 15);

  cam.initPersProjWithoutDistortion(800, 795, 320, 216);

  task.setServo(vpServo::EYEINHAND_L_cVe_eJe);
  task.setInteractionMatrixType(vpServo::DESIRED, vpServo::PSEUDO_INVERSE) ;
  task.setLambda(lambda_adapt) ;

  vpVelocityTwistMatrix cVe = robot.get_cVe();
  // Update the robot jacobian that depends on the pan position
  robot.set_eJe(qm_pan);
  // Get the robot jacobian
  vpMatrix eJe = robot.get_eJe();
  task.set_eJe( eJe );

  vpImagePoint ip(0,0);

  // Create the current x visual feature
  vpFeatureBuilder::create(s_x, cam, ip);

  // Create the desired x* visual feature
  s_xd.buildFrom(0, 0, Zd);

  // Add the feature
  task.addFeature(s_x, s_xd, vpFeaturePoint::selectX()) ;

  s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z , 0); // log(Z/Z*) = 0 that's why the last parameter is 0
  s_Zd.buildFrom(s_x.get_x(), s_x.get_y(), Zd , 0); // log(Z/Z*) = 0 that's why the last parameter is 0

  // Add thegoalPub feature
  task.addFeature(s_Z, s_Zd) ;

}

void VS::move_base_statusCallback(const actionlib_msgs::GoalStatusArrayConstPtr& msg){
 if (!msg->status_list.empty()){
        
	if (msg->status_list[0].status == 3)
              nav_status = true;
              
	else
	      nav_status = false;
        
 }
}

void VS::statusCallback(const std_msgs::Int8ConstPtr& msg) {
led_msg_ptr.reset(new kobuki_msgs::Led());
sound_msg_ptr.reset(new kobuki_msgs::Sound());

 if (msg->data == 3){
    valid_pose = true;
    led_msg_ptr->value = kobuki_msgs::Led::GREEN; //kobuki_msgs::Led::GREEN;
    mobile_base_led1.publish(led_msg_ptr);
 
	if(onSound && nav_status){
		sound_msg_ptr->value = kobuki_msgs::Sound::ON;
                mobile_base_sound.publish(sound_msg_ptr);
		onSound = false;
		offSound = true;
	}

    }
  else if(!(msg->data == 3) && (nav_status)) {
    
    led_msg_ptr->value = kobuki_msgs::Led::ORANGE; //kobuki_msgs::Led::ORANGE;
    mobile_base_led1.publish(led_msg_ptr);
	if(offSound){
		sound_msg_ptr->value = kobuki_msgs::Sound::OFF;
                mobile_base_sound.publish(sound_msg_ptr);
		offSound = false;
		onSound = true;
	}

  }
  else{
      
      valid_pose = false;
      led_msg_ptr->value = kobuki_msgs::Led::BLACK;
      mobile_base_led1.publish(led_msg_ptr);
      }
}


void VS::poseCallback(const geometry_msgs::PoseStampedConstPtr& msg)
{
  geometry_msgs::Twist out_cmd_vel;
  geometry_msgs::Twist robot_cmd_vel;

  led_msg_ptr.reset(new kobuki_msgs::Led());
  sound_msg_ptr.reset(new kobuki_msgs::Sound());
  
  
  try {
    t_start_loop = vpTime::measureTimeMs();

    std::ostringstream strs;
    strs << "Receive a new pose" << std::endl;
    std::string str;
    str = strs.str();
    ROS_DEBUG("%s", str.c_str());
    vpHomogeneousMatrix cMo = visp_bridge::toVispHomogeneousMatrix(msg->pose);

    vpPoint origin;
    origin.setWorldCoordinates(0,0,0);
    origin.project(cMo);
    Z = origin.get_Z();

    if (Z <= 0)
      ROS_DEBUG("Z <= 0");

    if (!valid_pose || Z <= 0) {
      ROS_DEBUG("not valid pose");

      out_cmd_vel.linear.x = 0;
      out_cmd_vel.linear.y = 0;
      out_cmd_vel.linear.z = 0;
      out_cmd_vel.angular.x = 0;
      out_cmd_vel.angular.y = 0;
      out_cmd_vel.angular.z = 0.1;
      pubTwistRobot.publish(out_cmd_vel);

      valid_pose = false;
      valid_pose_prev = valid_pose;

      return;
    }

    // Update the current x feature
    s_x.set_xyZ(origin.p[0], origin.p[1], Z);

    // Update log(Z/Z*) feature. Since the depth Z change, we need to update the intection matrix
    s_Z.buildFrom(s_x.get_x(), s_x.get_y(), Z, log(Z/Zd)) ;

    vpVelocityTwistMatrix cVe = robot.get_cVe();
    task.set_cVe( cVe );
    
      // Update the robot jacobian that depends on the pan position
      robot.set_eJe(qm_pan);
      // Get the robot jacobian
      vpMatrix eJe = robot.get_eJe();
      // Update the jacobian that will be used to compute the control law
      task.set_eJe(eJe);

    // Compute the control law. Velocities are computed in the mobile robot reference frame
    v = task.computeControlLaw() ;

    static unsigned long iter = 0;
       
    if (iter == 0) {
      // Start a new visual servovpColVector
      ROS_INFO("Reinit visual servo");

      tinit = t_start_loop;vpColVector
      vi = v;
    }
    iter ++;


    //v = v - vi*exp(-mu*(t_start_loop - tinit)/1000.);
    double max_linear_vel = 0.3;
    double max_angular_vel = vpMath::rad(50);
    vpColVector v_max(3);
    v_max[0] = max_linear_vel;
    v_max[1] = max_angular_vel;
    v_max[2] = max_angular_vel;

    vpColVector v_sat = vpRobot::saturateVelocities(v, v_max);

 
    robot_cmd_vel.linear.x = v_sat[0];
    robot_cmd_vel.linear.y = 0;
    robot_cmd_vel.linear.z = 0;
    robot_cmd_vel.angular.x = 0;
    robot_cmd_vel.angular.y = 0;
    robot_cmd_vel.angular.z = v_sat[1];

    pubTwistRobot.publish(robot_cmd_vel);
  
       if((iter % 15) == 0){
            led_msg_ptr->value = kobuki_msgs::Led::GREEN; //kobuki_msgs::Led::GREEN;
            mobile_base_led2.publish(led_msg_ptr);
		
       }else {
	    led_msg_ptr->value = kobuki_msgs::Led::BLACK;
            mobile_base_led2.publish(led_msg_ptr);
            } 

    valid_pose_prev = valid_pose;
    valid_pose = false;
 }
  catch(...) {
    ROS_INFO("Catch an exception: set vel to 0");
    out_cmd_vel.linear.x = 0;
    out_cmd_vel.linear.y = 0;
    out_cmd_vel.linear.z = 0;
    out_cmd_vel.angular.x = 0;
    out_cmd_vel.angular.y = 0;
    out_cmd_vel.angular.z = 0;
    pubTwistRobot.publish(out_cmd_vel);
    led_msg_ptr->value = kobuki_msgs::Led::BLACK;
    mobile_base_led2.publish(led_msg_ptr);
    
  }
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "PioneerPan");

  VS vs(argc, argv);

  ros::spin();
      
}



