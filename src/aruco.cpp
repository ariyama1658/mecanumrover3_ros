//2つのコードマージ
#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <vector>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/Twist.h>





class ArucoNode {
public:
    ArucoNode() {
        // ノードハンドルの初期化
        nh_ = ros::NodeHandle();
        // パブリッシャの初期化
        vel_pub_ = nh_.advertise<geometry_msgs::Twist>("rover_twist", 1);
        // サブスクライバの初期化
        sub_ = nh_.subscribe("/fiducial_transforms", 10, &ArucoNode::callback, this);
    }

    void spin() {
        ros::Rate r(10);
        while (ros::ok()) {
            ros::spinOnce();
            joyPub();
            r.sleep();
        }
    }

private:
    void callback(const fiducial_msgs::FiducialTransformArray::ConstPtr& msg) {
        // コールバック関数の実装
        if (!msg->transforms.empty()) {
            const geometry_msgs::Vector3& translation = msg->transforms[0].transform.translation;
            const geometry_msgs::Quaternion& rotation = msg->transforms[0].transform.rotation;
            twist_.linear.x = 0;
            twist_.linear.y = translation.x;
            twist_.angular.z = rotation.z; // 必要に応じて設定
        }
    }

    void joyPub() {
        vel_pub_.publish(twist_);
    }

    ros::NodeHandle nh_;
    ros::Publisher vel_pub_;
    ros::Subscriber sub_;
    geometry_msgs::Twist twist_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco");
    ArucoNode aruco_node;
    aruco_node.spin();
    return 0;
}
/*
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

  int linear_x_,linear_y_, angular_, safety_;
  double l_scale_, a_scale_;
  ros::Publisher vel_pub_;
 // ros::Subscriber joy_sub_;
  geometry_msgs::Twist twist_;

  

void sendCallback()
{
  //if(\){
    twist_.angular.z = a_scale_*rotation_values[2];
    twist_.linear.x = l_scale_*translation_values[0];
    //twist_.linear.x = l_scale_*0.6;
    twist_.linear.y = l_scale_*translation_values[2];
 // }else{
 //   twist_.angular.z =0;
 //   twist_.linear.x = 0;
 //   twist_.linear.y = 0;
// }
  
}

void joyPub()
{
  vel_pub_.publish(twist_);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "aruco");
    ros::NodeHandle n;
    ros::NodeHandle nh;
    ros::Subscriber sub = nh.subscribe("/fiducial_transforms", 10, callback);

    vel_pub_ = nh.advertise<geometry_msgs::Twist>("rover_twist", 1);

    ros::Rate r(10);
    while(n.ok()){
      ros::spinOnce();
      sendCallback();
      joyPub();
      r.sleep();
    }
}
*/
