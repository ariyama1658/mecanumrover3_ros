#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>

// グローバル変数を定義
std::vector<double> translation_values;
std::vector<double> rotation_values;

void callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
    if (!msg->transforms.empty()) {
        // 最初のfiducialのtranslation値を取得
        const geometry_msgs::Vector3& translation = msg->transforms[0].transform.translation;
        translation_values = {translation.x, translation.y, translation.z};
        ROS_INFO("Translation: [%f, %f, %f]", translation.x, translation.y, translation.z);

        // 最初のfiducialのrotation値を取得
        const geometry_msgs::Quaternion& rotation = msg->transforms[0].transform.rotation;
        rotation_values = {rotation.x, rotation.y, rotation.z, rotation.w};
        ROS_INFO("Rotation: [%f, %f, %f, %f]", rotation.x, rotation.y, rotation.z, rotation.w);
    }
}

class JoyCtrlRover
{
public:
  JoyCtrlRover();
  void joyPub();

private:
  void joyCallback(const sensor_msgs::Joy::ConstPtr& joy);
  

  ros::NodeHandle n_;

  int linear_x_,linear_y_, angular_, safety_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
  ros::Subscriber joy_sub_;
  geometry_msgs::Twist twist_;

};


JoyCtrlRover::JoyCtrlRover():
  linear_x_(1),
  linear_y_(0),
  angular_(3),
  a_scale_(0.8),
  l_scale_(0.6),
  safety_(7)
{

  n_.param("/joycon/axis_linear_x", linear_x_, linear_x_);
  n_.param("/joycon/axis_linear_y", linear_y_, linear_y_);
  n_.param("/joycon/axis_angular", angular_, angular_);
  n_.param("/joycon/scale_angular", a_scale_, a_scale_);
  n_.param("/joycon/scale_linear", l_scale_, l_scale_);
  n_.param("/joycon/safety_button", safety_, safety_);


  vel_pub_ = n_.advertise<geometry_msgs::Twist>("rover_twist", 1);


  joy_sub_ = n_.subscribe<sensor_msgs::Joy>("joy", 1, &JoyCtrlRover::joyCallback, this);

}

void JoyCtrlRover::joyCallback(const sensor_msgs::Joy::ConstPtr& joy)
{
  if(joy->buttons[safety_]){
    twist_.angular.z = a_scale_*joy->axes[angular_];
    twist_.linear.x = l_scale_*joy->axes[linear_x_];
    twist_.linear.y = l_scale_*joy->axes[linear_y_];
  }else{
    twist_.angular.z =0;
    twist_.linear.x = 0;
    twist_.linear.y = 0;
  }
  
}

void JoyCtrlRover::joyPub()
{
  vel_pub_.publish(twist_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "fiducial_transform_listener");
    ros::init(argc, argv, "joycon");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/fiducial_transforms", 10, callback);
    JoyCtrlRover joy_ctrl_mecanumrover;
    ros::Rate r(10);
    while(n.ok()){
      ros::spinOnce();
      joy_ctrl_mecanumrover.joyPub();
      r.sleep();
    }
}

