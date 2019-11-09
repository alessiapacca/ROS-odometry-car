/**
  * @file odometry.cpp
  * @author Alessia Paccagnella
  * @author Antonino Elia Mandri
  *
  * Project of the robotics course of Politecnico di Milano
  * This software reads left wheel speed, right wheel speed and 
  * steering angle from three topics and calculates the odometry 
  * of the car.
  *
  * Computes two different odometry: 
  *     using Differential Drive Kinematics
  *     using Ackerman model
  *
  * Uses dynamic reconfigure to switch between different published
  * odometry and to reset the odometry to (0,0) or to set to a
  * specific starting point (x,y).
  *
  * Publishes odometry as tf, odom topic with standard ros message 
  * "nav_msg/Odometry", simple_odom topic with a custom message
  * that contains position and type of algorithm used.
**/

#include "ros/ros.h"
#include "projects_robotics/floatStamped.h"
#include "projects_robotics/customOdometry.h"
#include "message_filters/subscriber.h"
#include "message_filters/time_synchronizer.h"
#include "message_filters/sync_policies/approximate_time.h"
#include "tf/transform_broadcaster.h"
#include "projects_robotics/dynamic_ricConfig.h"
#include <dynamic_reconfigure/server.h>
#include <nav_msgs/Odometry.h>
#include <math.h>


#define _USE_MATH_DEFINES
#define STEERING_FACTOR 18.0
#define BASELINE 1.3
#define INTERAXIS 1.765


typedef message_filters::sync_policies::ApproximateTime<projects_robotics::floatStamped, projects_robotics::floatStamped, projects_robotics::floatStamped> MySyncPolicy;

typedef struct input_data
{
    double speed_L;
    double speed_R;
    double steer_angle;

    double time_SL;
    double time_SR;
    double time_SA;
}Data;

typedef struct pose
{
    double x;
    double y;
    double theta;
}Pose;

typedef struct velocity
{
    double speed_x;
    double speed_y;
    double omega;
}Velocity;

class odometry_car
{
private:

    tf::TransformBroadcaster odom_broadcaster;
    Pose current_pose;
    Velocity velocity;
    double steering_factor;
    double baseline;
    double interaxis;

    double prev_theta;

    double prev_time_vr;
    double prev_time_vl;
    double prev_time_steerangle;

    ros::NodeHandle pub_node;
    ros::Publisher odometry_pub;
    ros::Publisher custom_odometry_pub;

    int chosen_odometry; // 0 differential, 1 ackermannnnnnnnn
    int reset_signal; // set position when 
    int change_coordinates_signal;


public:

    odometry_car(double x, double y)
    {
        current_pose.x = x;
        current_pose.y = y;
        current_pose.theta = 0.0;
        prev_theta = 0.0;
        steering_factor = STEERING_FACTOR;
        baseline = BASELINE;
        interaxis = INTERAXIS;
        chosen_odometry = 0;
        prev_time_vr = 0.0;
        prev_time_vl = 0.0;
        prev_time_steerangle = 0.0;

        chosen_odometry = 0;
        reset_signal = 0;
        change_coordinates_signal = 0;

        odometry_pub = pub_node.advertise<nav_msgs::Odometry>("/odom", 50);
        custom_odometry_pub = pub_node.advertise<projects_robotics::customOdometry>("/simple_odom", 50);

    }

    void compute_odometry(Data *input_topics)
    {
        if(chosen_odometry == 0)
        {
            differential_drive_compute(input_topics);
        }
        if(chosen_odometry == 1)
        {
            ackermann_model_compute(input_topics);
        }
    }

    void differential_drive_compute(Data *input_topics)
    {
        double dt_vr = (input_topics->time_SR - prev_time_vr);
        double dt_vl = (input_topics->time_SL - prev_time_vl);
        double dt_v = (dt_vr + dt_vl)/2.0;
        double dt_steer = (input_topics->time_SA - prev_time_steerangle);

        prev_time_vr = input_topics->time_SR;
        prev_time_vl = input_topics->time_SL;
        prev_time_steerangle = input_topics->time_SA;

        velocity.omega = (input_topics->speed_R - input_topics->speed_L)/BASELINE;
        double speed = (input_topics->speed_R + input_topics->speed_L)/(2.0); 
        prev_theta = current_pose.theta;
        current_pose.theta +=  velocity.omega*( (dt_vr + dt_vl)/2.0);
        velocity.speed_x = speed * (cos( (current_pose.theta + prev_theta)/2.0));
        velocity.speed_y = speed * (sin( (current_pose.theta + prev_theta)/2.0));
        current_pose.x += velocity.speed_x * dt_v;
        current_pose.y += velocity.speed_y * dt_v; 
    }

    void ackermann_model_compute(Data *input_topics)
    {
        double dt_vr = (input_topics->time_SR - prev_time_vr);
        double dt_vl = (input_topics->time_SL - prev_time_vl);
        double dt_steer = (input_topics->time_SA - prev_time_steerangle);
        double dt_v = (dt_vr + dt_vl)/2.0;

        prev_time_vr = input_topics->time_SR;
        prev_time_vl = input_topics->time_SL;
        prev_time_steerangle = input_topics->time_SA;

        double speed = (input_topics->speed_R + input_topics->speed_L)/(2.0); 
        prev_theta = current_pose.theta;
        velocity.omega = (speed * (tan(input_topics->steer_angle))/interaxis);
        current_pose.theta +=  velocity.omega*( (dt_vr + dt_vl + dt_steer)/3.0);
        velocity.speed_x = speed * (cos( (current_pose.theta + prev_theta)/2.0));
        velocity.speed_y = speed * (sin( (current_pose.theta + prev_theta)/2.0));
        current_pose.x += velocity.speed_x * dt_v;
        current_pose.y += velocity.speed_y * dt_v;
    }

    void publish_odom(nav_msgs::Odometry *odom, projects_robotics::customOdometry *custom_odom)
    {   
        ros::Time current_time = ros::Time::now();
        geometry_msgs::TransformStamped odom_trans;
        geometry_msgs::Quaternion odom_quat = tf::createQuaternionMsgFromYaw(current_pose.theta);

        custom_odom->header.stamp = current_time;
        custom_odom->header.frame_id = "odom";
        if(chosen_odometry == 0)
            custom_odom->algorithm_type = "Differetial drive model";
        else if(chosen_odometry == 1)
            custom_odom->algorithm_type= "Ackermann model";

        odom->header.stamp = current_time;
        odom->header.frame_id = "odom";

        //set position
        odom->pose.pose.position.x = current_pose.x;
        odom->pose.pose.position.y = current_pose.y;
        odom->pose.pose.position.z = 0.0;
        odom->pose.pose.orientation = odom_quat;

        custom_odom->x = current_pose.x;
        custom_odom->y = current_pose.y;
        custom_odom->theta = current_pose.theta;

        //set velocity
        odom->child_frame_id = "base_link";
        odom->twist.twist.linear.x = velocity.speed_x;
        odom->twist.twist.linear.y = velocity.speed_y;
        odom->twist.twist.angular.z = velocity.omega;

        odom_trans.header.stamp = current_time;
        odom_trans.header.frame_id = "odom";
        odom_trans.child_frame_id = "base_link";

        odom_trans.transform.translation.x = current_pose.x;
        odom_trans.transform.translation.y = current_pose.y;
        odom_trans.transform.translation.z = 0.0;
        odom_trans.transform.rotation = odom_quat;

        odom_broadcaster.sendTransform(odom_trans);
        odometry_pub.publish(*odom);
        custom_odometry_pub.publish(*custom_odom);

    }

    void set_position(double x, double y)
    {
        current_pose.x = x;
        current_pose.y = y;
        current_pose.theta = 0.0;
        prev_theta = current_pose.theta;
    }


    void set_chosen_odometry(int chosen)
    {
        if(chosen == 0 || chosen == 1)
            chosen_odometry = chosen;
    }

    int get_reset_signal()
    {
        return reset_signal;
    }

    void set_reset_signal()
    {
        reset_signal = (reset_signal+1)%2;
    }

    int get_change_coordinates_signal()
    {
        return change_coordinates_signal;
    }

    void set_change_coordinates_signal()
    {
        change_coordinates_signal = (change_coordinates_signal+1)%2;
    }

};


void Callback(const projects_robotics::floatStamped::ConstPtr& msg1, const projects_robotics::floatStamped::ConstPtr& msg2, 
                    const projects_robotics::floatStamped::ConstPtr& msg3, odometry_car *odom_car, Data *input_topics)
{
    nav_msgs::Odometry odom_msg;
    projects_robotics::customOdometry custom_odom;
    double temp_steer_angle = (msg3->data/STEERING_FACTOR) * (M_PI/180.0);
    input_topics->speed_L = msg1->data;
    input_topics->time_SL = msg1->header.stamp.toSec();
    input_topics->speed_R = msg2->data;
    input_topics->time_SR = msg2->header.stamp.toSec();
    input_topics->steer_angle = temp_steer_angle;
    input_topics->time_SA = msg3->header.stamp.toSec();
    odom_car->compute_odometry(input_topics);
    odom_car->publish_odom(&odom_msg, &custom_odom);
}

void param_callback(projects_robotics::dynamic_ricConfig &config, uint32_t level, odometry_car *odom_car) //
{
    odom_car->set_chosen_odometry(config.computation_type);

    if(config.change_coordinates_signal != odom_car->get_change_coordinates_signal())
    {
        odom_car->set_position(config.x_coordinate, config.y_coordinate);
        odom_car->set_change_coordinates_signal();
    }
    
    if(config.reset_signal != odom_car->get_reset_signal())
    {
        odom_car->set_position(0.0, 0.0);
        odom_car->set_reset_signal();
    }
}



int main(int argc, char *argv[])
{
    ros::init(argc, argv, "odometry");
    Data input;
    
    odometry_car *odometry = NULL;
    odometry = new odometry_car(0.0, 0.0);

    ros::NodeHandle sub_node;

    dynamic_reconfigure::Server<projects_robotics::dynamic_ricConfig> server;
    dynamic_reconfigure::Server<projects_robotics::dynamic_ricConfig>::CallbackType f; 
    f = boost::bind(&param_callback, _1, _2, odometry);
    server.setCallback(f);

    message_filters::Subscriber<projects_robotics::floatStamped> sub1(sub_node, "/speedL_stamped", 1);
    message_filters::Subscriber<projects_robotics::floatStamped> sub2(sub_node, "/speedR_stamped", 1);
    message_filters::Subscriber<projects_robotics::floatStamped> sub3(sub_node, "/steer_stamped", 1);
    
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), sub1, sub2, sub3);
    sync.registerCallback(boost::bind(&Callback, _1, _2, _3, odometry, &input));
    
    ros::spin();
    return 0;
}