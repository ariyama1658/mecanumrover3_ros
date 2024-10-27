#include <ros/ros.h>
#include <geometry_msgs/TransformStamped.h>
#include <fiducial_msgs/FiducialTransformArray.h>
#include <vector>

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

int main(int argc, char** argv) {
    ros::init(argc, argv, "fiducial_transform_listener");
    ros::NodeHandle nh;

    ros::Subscriber sub = nh.subscribe("/fiducial_transforms", 10, callback);

    ros::spin();

    return 0;
}