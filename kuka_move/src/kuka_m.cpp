#include "ros/ros.h"
#include <iostream>
#include "sensor_msgs/JointState.h"
#include "std_msgs/Float64.h"
#include "eigen3/Eigen/Dense"
#include "eigen3/Eigen/SVD"

using namespace std;

typedef Eigen::Matrix<double, 7, 1> Vector7d;

std::vector<double> q_std(7,1);

void joint_position_cb(const sensor_msgs::JointState &q_msg){

    q_std = q_msg.position;
}

int main(int argc, char** argv){

    ros::init(argc, argv, "kuka_move");
    ros::NodeHandle nh;
    ros::Rate r(100);

    ros::Subscriber joint_state_sub = nh.subscribe("/iiwa/joint_states", 1000, joint_position_cb);

    Vector7d q = Vector7d::Zero();

    ros::Publisher joint_pos_cmd[7];

    joint_pos_cmd[0] = nh.advertise<std_msgs::Float64>("/iiwa/joint1_position_controller/command", 1000);
    joint_pos_cmd[1] = nh.advertise<std_msgs::Float64>("/iiwa/joint2_position_controller/command", 1000);
    joint_pos_cmd[2] = nh.advertise<std_msgs::Float64>("/iiwa/joint3_position_controller/command", 1000);
    joint_pos_cmd[3] = nh.advertise<std_msgs::Float64>("/iiwa/joint4_position_controller/command", 1000);
    joint_pos_cmd[4] = nh.advertise<std_msgs::Float64>("/iiwa/joint5_position_controller/command", 1000);
    joint_pos_cmd[5] = nh.advertise<std_msgs::Float64>("/iiwa/joint6_position_controller/command", 1000);
    joint_pos_cmd[6] = nh.advertise<std_msgs::Float64>("/iiwa/joint7_position_controller/command", 1000);

    Vector7d q_data = Vector7d::Zero();
    std_msgs::Float64 q_cmd[7];

    while(ros::ok()){

        q << q_std[0], q_std[1], q_std[2], q_std[3], q_std[4], q_std[5], q_std[6];

        cout << "q" << endl;
        cout << q << endl;

        q_data[0] = 0.0;
        q_data[1] = 0.0;
        q_data[2] = 0.0;
        q_data[3] = 0.0;
        q_data[4] = 0.0;
        q_data[5] = 0.0;
        q_data[6] = 0.0;

        /*q_data[0] = -0.46;
      	q_data[1] = 0.33;
      	q_data[2] = -0.51;
      	q_data[3] = -1.5;
      	q_data[4] = 0.16;
      	q_data[5] = 1.33;
      	q_data[6] = -0.99;*/

        for(int i=0; i<7; i++){
            q_cmd[i].data = q_data[i];
            joint_pos_cmd[i].publish(q_cmd[i]);
        }

        ros::spinOnce();
        r.sleep();
    }

}
