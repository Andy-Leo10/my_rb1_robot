//the service of the robot is = /rotate_robot
//the name of this node is = node_rb1_degrees_service

#include <cstdint>
#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include "nav_msgs/Odometry.h"
#include <my_rb1_ros/Rotate.h>
#include <string>
#include <cmath>
#include <iostream>
#include <geometry_msgs/Quaternion.h>

class RB1
{
  
    private:
        // communication with nodes
        ros::NodeHandle nh_;
        // rotation service
        ros::ServiceServer my_service;
        std::string rotation_service;
        // velocity and position
        ros::Publisher vel_pub;
        geometry_msgs::Twist vel_msg;
        std::string vel_topic;
        // odometry
        ros::Subscriber odom_sub;
        std::string odom_topic;
        // robot atributes
        float linear_velocity;
        float angular_velocity;
        int rb1_orientation=0;
        //readings
        float x_pos;
        float y_pos;
        float z_pos;
        float x_orientation;
        float y_orientation;
        float z_orientation;
        float w_orientation;
    
    public:
        RB1()
        {
            rotation_service="/rotate_robot";
            vel_topic = "/cmd_vel";
            odom_topic = "/odom";
            linear_velocity=0.5;
            angular_velocity=0.5;
            my_service = nh_.advertiseService(rotation_service, &RB1::service_callback, this);
            ROS_INFO("The Service                 /rotate_robot                is READY");
            vel_pub = nh_.advertise<geometry_msgs::Twist>(vel_topic, 1);
            odom_sub = nh_.subscribe(odom_topic, 10, &RB1::odom_callback, this);
            ROS_INFO("Initializing node .................................");
            //wait 2 seconds
            usleep(2000000);
        }
        

            
        void odom_callback(const nav_msgs::Odometry::ConstPtr &odom_msg) {
            x_pos = odom_msg->pose.pose.position.x;
            y_pos = odom_msg->pose.pose.position.y;
            z_pos = odom_msg->pose.pose.position.z;
            x_orientation=odom_msg->pose.pose.orientation.x;
            y_orientation=odom_msg->pose.pose.orientation.y;
            z_orientation=odom_msg->pose.pose.orientation.z;
            w_orientation=odom_msg->pose.pose.orientation.w;
            rb1_orientation = calculateYawAngle(w_orientation, x_orientation, y_orientation, z_orientation);
            ROS_DEBUG("------------------    Updated orientation angle (yaw): %d degrees", rb1_orientation);
            ros::spinOnce();
        }

        bool service_callback(my_rb1_ros::Rotate::Request &req,
                      my_rb1_ros::Rotate::Response &res)
        {
            ROS_INFO(">>>>>>>>>>>>>>>>>> Initiating service /rotate_robot");

            int initial_angle = rb1_orientation;
            int goal_angle = initial_angle + req.degrees;

            // Ensure goal_angle is in the range [0, 359]
            goal_angle = (goal_angle % 360 + 360) % 360;

            // Calculate the positive and negative angle differences
            int diff_positive = (goal_angle - initial_angle + 360) % 360;
            int diff_negative = (initial_angle - goal_angle + 360) % 360;

            if (req.degrees > 0)
            {
                while (diff_positive > 0)
                {
                    rotate_positive();
                    delay(0.05);
                    ros::spinOnce();
                    //initial_angle = rb1_orientation;
                    diff_positive = (goal_angle - rb1_orientation + 360) % 360;
                    ROS_INFO("difference = %d", diff_positive);
                }
            }
            else
            {
                while (diff_negative > 0)
                {
                    rotate_negative();
                    delay(0.05);
                    ros::spinOnce();
                    //initial_angle = rb1_orientation;
                    diff_negative = (rb1_orientation - goal_angle + 360) % 360;
                    ROS_INFO("difference = %d", diff_negative);
                }
            }

            // Stop the rotation
            stop();

            // Print the final angle
            ROS_INFO("orientation angle (yaw): %d degrees", rb1_orientation);

            res.result = true;
            ROS_INFO(">>>>>>>>>>>>>>>>>> Finished service /rotate_robot");
            return true;
        }


        void delay(float n_seconds)
        {
            usleep(1000000*n_seconds);
        }

        void move(float forward_vel, float turning_vel)
        {
            vel_msg.linear.x = forward_vel;
            vel_msg.angular.z = turning_vel;
            vel_pub.publish(vel_msg);
        }
        void rotate_positive()
        {
            vel_msg.linear.x = 0;
            vel_msg.angular.z = angular_velocity;
            vel_pub.publish(vel_msg);
        }
        void rotate_negative()
        {
            vel_msg.linear.x = 0;
            vel_msg.angular.z = -angular_velocity;
            vel_pub.publish(vel_msg);
        }
        void stop()
        {
            vel_msg.linear.x = 0;
            vel_msg.angular.z = 0;
            vel_pub.publish(vel_msg);
        }

        int calculateYawAngle(double w, double x, double y, double z)
        {
            // Calcular ángulo de orientación (yaw) en radianes
            double yaw_radians = std::atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));

            // Convertir de radianes a grados
            double yaw_degrees = yaw_radians * 180.0 / M_PI;

            // Asegurarse de que el ángulo esté en el rango [0, 360)
            yaw_degrees = fmod(yaw_degrees + 360.0, 360.0);

            // Devolver el ángulo de orientación calculado
            return static_cast<int>(yaw_degrees);
        }


};

int main(int argc, char** argv)
{
    if( ros::console::set_logger_level(ROSCONSOLE_DEFAULT_NAME, ros::console::levels::Debug) ) 
    {
        ros::console::notifyLoggerLevelsChanged();
    }
  ros::init(argc, argv, "node_rb1_degrees_service");
  
  RB1 myRobot;

  ros::spin();
  
  return 0;
}


