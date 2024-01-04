#pragma once

#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <mage_msgs/msg/part.hpp>
#include <rclcpp/timer.hpp>
#include <tf2_ros/buffer.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

namespace final_project
{ /**
   * @brief This class is used to broadcast the static transform between parts and camera frames and map and odom frames
   *
   */
    class StaticBroadcaster : public rclcpp::Node
    {
    public:
        StaticBroadcaster(std::string node_name) : Node(node_name)
        {
            // Create a QoS profile
            rclcpp::QoS qos_profile = rclcpp::QoS(rclcpp::KeepLast(10));
            // Set the reliability policy to best effort
            qos_profile.best_effort();
            // Set the durability policy to volatile
            qos_profile.durability_volatile();
            // Set the lifespan to 9223372036854775807 nanoseconds
            qos_profile.lifespan(std::chrono::nanoseconds(9223372036854775807));
            // Set the deadline to 9223372036854775807 nanoseconds
            qos_profile.deadline(std::chrono::nanoseconds(9223372036854775807));
            // Set the liveliness policy to automatic
            qos_profile.liveliness(RMW_QOS_POLICY_LIVELINESS_AUTOMATIC);
            // Set the liveliness lease duration to 9223372036854775807 nanoseconds
            qos_profile.liveliness_lease_duration(std::chrono::nanoseconds(9223372036854775807));

            // initialize static transform broadcaster for map and odom frame
            tf_static_broadcaster_map_odom_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // initialize static transform broadcaster for part1 and camera1 frame
            tf_static_broadcaster_part1_camera1_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // initialize static transform broadcaster for part2 and camera2 frame
            tf_static_broadcaster_part2_camera2_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // initialize static transform broadcaster for part3 and camera3 frame
            tf_static_broadcaster_part3_camera3_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // initialize static transform broadcaster for part4 and camera4 frame
            tf_static_broadcaster_part4_camera4_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // initialize static transform broadcaster for part5 and camera5 frame
            tf_static_broadcaster_part5_camera5_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

            // initialize the subscriber for the part 1
            parts_subscriber1_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera1/image", qos_profile, std::bind(&StaticBroadcaster::parts1_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 2
            parts_subscriber2_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera2/image", qos_profile, std::bind(&StaticBroadcaster::parts2_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 3
            parts_subscriber3_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera3/image", qos_profile, std::bind(&StaticBroadcaster::parts3_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 4
            parts_subscriber4_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera4/image", qos_profile, std::bind(&StaticBroadcaster::parts4_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 5
            parts_subscriber5_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera5/image", qos_profile, std::bind(&StaticBroadcaster::parts5_cb, this, std::placeholders::_1));

            // Create a wall timer object for the static broadcaster for map and odom frame
            // static_broadcast_timer_odom_ = this->create_wall_timer(
            //     100ms,
            //     std::bind(&StaticBroadcaster::static_broadcast_timer_cb_odom_, this));

            // initialize a wall timer object for the static broadcaster for parts and camera frame
            static_broadcast_timer_parts_camera_ = this->create_wall_timer(
                100ms,
                std::bind(&StaticBroadcaster::static_broadcast_timer_cb_parts_camera_, this));
            // initialize the pose of the map and odom frame
            map_odom_pose_.position.x = 0.0;
            map_odom_pose_.position.y = 0.0;
            map_odom_pose_.position.z = 0.0;
            map_odom_pose_.orientation.x = 0.0;
            map_odom_pose_.orientation.y = 0.0;
            map_odom_pose_.orientation.z = 0.0;
            map_odom_pose_.orientation.w = 1.0;

            // initialize the pose of the part1 and camera1 frame
            part1_camera1_pose_.position.x = 0.0;
            part1_camera1_pose_.position.y = 0.0;
            part1_camera1_pose_.position.z = 0.0;
            part1_camera1_pose_.orientation.x = 0.0;
            part1_camera1_pose_.orientation.y = 0.0;
            part1_camera1_pose_.orientation.z = 0.0;
            part1_camera1_pose_.orientation.w = 1.0;

            // initialize the pose of the part2 and camera2 frame
            part2_camera2_pose_.position.x = 0.0;
            part2_camera2_pose_.position.y = 0.0;
            part2_camera2_pose_.position.z = 0.0;
            part2_camera2_pose_.orientation.x = 0.0;
            part2_camera2_pose_.orientation.y = 0.0;
            part2_camera2_pose_.orientation.z = 0.0;
            part2_camera2_pose_.orientation.w = 1.0;

            // initialize the pose of the part3 and camera3 frame
            part3_camera3_pose_.position.x = 0.0;
            part3_camera3_pose_.position.y = 0.0;
            part3_camera3_pose_.position.z = 0.0;
            part3_camera3_pose_.orientation.x = 0.0;
            part3_camera3_pose_.orientation.y = 0.0;
            part3_camera3_pose_.orientation.z = 0.0;
            part3_camera3_pose_.orientation.w = 1.0;

            // initialize the pose of the part4 and camera4 frame
            part4_camera4_pose_.position.x = 0.0;
            part4_camera4_pose_.position.y = 0.0;
            part4_camera4_pose_.position.z = 0.0;
            part4_camera4_pose_.orientation.x = 0.0;
            part4_camera4_pose_.orientation.y = 0.0;
            part4_camera4_pose_.orientation.z = 0.0;
            part4_camera4_pose_.orientation.w = 1.0;

            // initialize the pose of the part5 and camera5 frame
            part5_camera5_pose_.position.x = 0.0;
            part5_camera5_pose_.position.y = 0.0;
            part5_camera5_pose_.position.z = 0.0;
            part5_camera5_pose_.orientation.x = 0.0;
            part5_camera5_pose_.orientation.y = 0.0;
            part5_camera5_pose_.orientation.z = 0.0;
            part5_camera5_pose_.orientation.w = 1.0;

            // Print that the node is started
            RCLCPP_INFO(this->get_logger(), "Static Broadcaster node started");
        }

    private:
        // Static broadcaster object for the static transform between the map and odom
        /**
         * @brief Static broadcaster object for the static transform between the map and odom
         *
         */
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_map_odom_;
        // Static broadcaster object for the static transform between the part1_frame and camera1_frame
        /**
         * @brief Static broadcaster object for the static transform between the part1_frame and camera1_frame
         *
         */
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_part1_camera1_;
        // Static broadcaster object for the static transform between the part2_frame and camera2_frame
        /**
         * @brief Static broadcaster object for the static transform between the part2_frame and camera2_frame
         *
         */
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_part2_camera2_;
        // Static broadcaster object for the static transform between the part3_frame and camera3_frame
        /**
         * @brief Static broadcaster object for the static transform between the part3_frame and camera3_frame
         *
         */
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_part3_camera3_;
        // Static broadcaster object for the static transform between the part4_frame and camera4_frame
        /**
         * @brief Static broadcaster object for the static transform between the part4_frame and camera4_frame
         *
         */
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_part4_camera4_;
        // Static broadcaster object for the static transform between the part5_frame and camera5_frame
        /**
         * @brief Static broadcaster object for the static transform between the part5_frame and camera5_frame
         *
         */
        std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_part5_camera5_;

        // Timer for the static broadcaster for map and odom frame
        /**
         * @brief Timer for the static broadcaster for map and odom frame
         *
         */
        rclcpp::TimerBase::SharedPtr static_broadcast_timer_odom_;

        // Timer for the static broadcaster for parts and camera frame
        /**
         * @brief Timer for the static broadcaster for parts and camera frame
         *
         */
        rclcpp::TimerBase::SharedPtr static_broadcast_timer_parts_camera_;

        // Create a subscriber 1 for the logical camera image
        /**
         * @brief Create a subscriber 1 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber1_;
        // Create a subscriber 2 for the logical camera image
        /**
         * @brief Create a subscriber 2 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber2_;
        // Create a subscriber 3 for the logical camera image
        /**
         * @brief Create a subscriber 3 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber3_;
        // Create a subscriber 4 for the logical camera image
        /**
         * @brief Create a subscriber 4 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber4_;
        // Create a subscriber 5 for the logical camera image
        /**
         * @brief Create a subscriber 5 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber5_;

        // Create callback function for subscriber of part1
        /**
         * @brief Create callback function for subscriber of part1
         *
         * @param msg The message from the subscriber of part1 camera
         */
        void parts1_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part2
        /**
         * @brief Create callback function for subscriber of part2
         *
         * @param msg The message from the subscriber of part2 camera
         */
        void parts2_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part3
        /**
         * @brief Create callback function for subscriber of part3
         *
         * @param msg Message from the subscriber of part3 camera
         */
        void parts3_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part4
        /**
         * @brief Create callback function for subscriber of part4
         *
         * @param msg Message from the subscriber of part4 camera
         */
        void parts4_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part5
        /**
         * @brief Create callback function for subscriber of part5
         *
         * @param msg Message from the subscriber of part5 camera
         */
        void parts5_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create a pose object for the transform between the map and odom
        /**
         * @brief Create a pose object for the transform between the map and odom
         *
         */
        geometry_msgs::msg::Pose map_odom_pose_;

        // Create a pose object for the transform between the part1_frame and camera1_frame
        /**
         * @brief Create a pose object for the transform between the part1_frame and camera1_frame
         *
         */
        geometry_msgs::msg::Pose part1_camera1_pose_;

        // Create a pose object for the transform between the part2_frame and camera2_frame
        /**
         * @brief Create a pose object for the transform between the part2_frame and camera2_frame
         *
         */
        geometry_msgs::msg::Pose part2_camera2_pose_;

        // Create a pose object for the transform between the part3_frame and camera3_frame
        /**
         * @brief Create a pose object for the transform between the part3_frame and camera3_frame
         *
         */
        geometry_msgs::msg::Pose part3_camera3_pose_;

        // Create a pose object for the transform between the part4_frame and camera4_frame
        /**
         * @brief Create a pose object for the transform between the part4_frame and camera4_frame
         *
         */
        geometry_msgs::msg::Pose part4_camera4_pose_;

        // Create a pose object for the transform between the part5_frame and camera5_frame
        /**
         * @brief Create a pose object for the transform between the part5_frame and camera5_frame
         *
         */
        geometry_msgs::msg::Pose part5_camera5_pose_;

        // Create a call back function for the timer for the static broadcaster for map and odom frame
        /**
         * @brief Create a call back function for the timer for the static broadcaster for map and odom frame
         *
         */
        void static_broadcast_timer_cb_odom_();

        // Create a call back function for the timer for the static broadcaster for parts and camera frame
        /**
         * @brief Create a call back function for the timer for the static broadcaster for parts and camera frame
         *
         */
        void static_broadcast_timer_cb_parts_camera_();

    }; // StaticBroadcaster class
} // namespace final_project
