#include "nav2_msgs/action/follow_waypoints.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2_ros/transform_broadcaster.h"
#include <chrono>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <mage_msgs/msg/advanced_logical_camera_image.hpp>
#include <mage_msgs/msg/part.hpp>
#include <mage_msgs/msg/part_pose.hpp>
#include <memory>
#include <nav_msgs/msg/odometry.hpp>
#include <rclcpp/rclcpp.hpp>
#include <ros2_aruco_interfaces/msg/aruco_markers.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

using namespace std::chrono_literals;

// Create a struct to store the waypoint data
struct Waypoint
{
    int type_id;                   /// 0 for battery, 1 for pump, 2 for sensor, 3 for regulator
    int color_id;                  // 0 for red, 1 for green, 2 for blue, 3 for orange, 4 for purple
    geometry_msgs::msg::Pose pose; // Pose of the waypoint
};

namespace final_project
{
    /**
     * @brief  Class for the tb3_aruco_detector node for waypoint navigation
     *
     */
    class Tb3ArucoDetector : public rclcpp::Node
    {
    public:
        // Create a type for the action server
        using NavigateToPose = nav2_msgs::action::FollowWaypoints;
        // Create a type for the goal handle
        using GoalHandleNavigation = rclcpp_action::ClientGoalHandle<NavigateToPose>;
        /**
         * @brief Construct a new Tb3 Aruco Detector object
         *
         * @param node_name Name of the node
         */
        Tb3ArucoDetector(std::string node_name) : Node(node_name)
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
            subscription_ = this->create_subscription<ros2_aruco_interfaces::msg::ArucoMarkers>(
                "/aruco_markers", 10, std::bind(&Tb3ArucoDetector::aruco_markers_cb, this, std::placeholders::_1));

            // Initialize the marker id to 0
            marker_id = 1;

            // Get the parameters from the parameter server
            this->declare_parameter("aruco_0.wp1.type", "battery");
            this->declare_parameter("aruco_0.wp1.color", "green");
            this->declare_parameter("aruco_0.wp2.type", "battery");
            this->declare_parameter("aruco_0.wp2.color", "red");
            this->declare_parameter("aruco_0.wp3.type", "battery");
            this->declare_parameter("aruco_0.wp3.color", "orange");
            this->declare_parameter("aruco_0.wp4.type", "battery");
            this->declare_parameter("aruco_0.wp4.color", "purple");
            this->declare_parameter("aruco_0.wp5.type", "battery");
            this->declare_parameter("aruco_0.wp5.color", "blue");
            this->declare_parameter("aruco_1.wp1.type", "battery");
            this->declare_parameter("aruco_1.wp1.color", "blue");
            this->declare_parameter("aruco_1.wp2.type", "battery");
            this->declare_parameter("aruco_1.wp2.color", "green");
            this->declare_parameter("aruco_1.wp3.type", "battery");
            this->declare_parameter("aruco_1.wp3.color", "orange");
            this->declare_parameter("aruco_1.wp4.type", "battery");
            this->declare_parameter("aruco_1.wp4.color", "red");
            this->declare_parameter("aruco_1.wp5.type", "battery");
            this->declare_parameter("aruco_1.wp5.color", "purple");

            // Initialize variables to store waypoint data
            wp1_type_id = 0;
            wp1_color_id = 0;
            wp2_type_id = 0;
            wp2_color_id = 0;
            wp3_type_id = 0;
            wp3_color_id = 0;
            wp4_type_id = 0;
            wp4_color_id = 0;
            wp5_type_id = 0;
            wp5_color_id = 0;

            // initialize the waypoints vector
            waypoints = std::vector<Waypoint>(5);

            // // Initialize the color_id_to_camera_frame mapper with no values
            // color_id_to_camera_frame = std::map<int, std::string>();

            // Create a transform listener to listen to the transform between the source and the target frame
            tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
            tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

            // initialize the client
            client_ =
                rclcpp_action::create_client<NavigateToPose>(this, "follow_waypoints");
            // initialize the publisher
            initial_pose_pub_ =
                this->create_publisher<geometry_msgs::msg::PoseWithCovarianceStamped>(
                    "initialpose", 10);

            // initialize the subscriber to the topic /amcl_pose
            amcl_pose_sub_ = this->create_subscription<geometry_msgs::msg::PoseWithCovarianceStamped>(
                "/amcl_pose", 10, std::bind(&Tb3ArucoDetector::amcl_pose_cb, this, std::placeholders::_1));

            // initialize the subscriber to the topic /odom
            odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
                "/odom", 10, std::bind(&Tb3ArucoDetector::odom_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 1
            parts_subscriber1_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera1/image", qos_profile, std::bind(&Tb3ArucoDetector::parts1_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 2
            parts_subscriber2_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera2/image", qos_profile, std::bind(&Tb3ArucoDetector::parts2_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 3
            parts_subscriber3_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera3/image", qos_profile, std::bind(&Tb3ArucoDetector::parts3_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 4
            parts_subscriber4_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera4/image", qos_profile, std::bind(&Tb3ArucoDetector::parts4_cb, this, std::placeholders::_1));

            // initialize the subscriber for the part 5
            parts_subscriber5_ = this->create_subscription<mage_msgs::msg::AdvancedLogicalCameraImage>(
                "/mage/camera5/image", qos_profile, std::bind(&Tb3ArucoDetector::parts5_cb, this, std::placeholders::_1));

            // While the set_initial_pose() function returns false, keep calling it
            // while (!set_initial_pose())
            // {
            //     RCLCPP_INFO(this->get_logger(), "Waiting for initial pose to be set");
            // }
            // Print "Setting initial pose" to the terminal
            RCLCPP_INFO(this->get_logger(), "Setting initial pose");
            RCLCPP_INFO(this->get_logger(), "waiting for 20 seconds");
            set_initial_pose();
            // sleep for 5 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
            set_initial_pose();
            // sleep for 5 seconds
            std::this_thread::sleep_for(std::chrono::seconds(10));
            // Initialize the goal message

            goal_msg.poses.resize(5);
        }

    private:
        /**
         * @brief Callback function for /aruco_markers topic
         *
         * @param msg Message published on /aruco_markers topic
         */
        void aruco_markers_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg);

        /**
         * @brief Subscription for /aruco_markers topic
         */
        rclcpp::Subscription<ros2_aruco_interfaces::msg::ArucoMarkers>::SharedPtr subscription_;

        // Create a variable to store the marker id of type int
        /**
         * @brief Variable to store the marker id of type int
         *
         */
        int marker_id;

        // Create a susbcriber to the topic /amcl_pose
        /**
         * @brief Subscriber to the topic /amcl_pose
         *
         */
        rclcpp::Subscription<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            amcl_pose_sub_;

        // Create a callback function for the topic /amcl_pose
        /**
         * @brief Callback function for the topic /amcl_pose
         *
         * @param msg Message published on the topic /amcl_pose
         */
        void amcl_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg);

        // Create a subscriber to the topic /odom
        /**
         * @brief Subscriber to the topic /odom
         *
         */
        rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

        // Create a callback function for the topic /odom
        /**
         * @brief Callback function for the topic /odom
         *
         * @param msg Message published on the topic /odom
         */
        void odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg);

        // Create a variable to store the current pose of the robot
        /**
         * @brief Variable to store the current pose of the robot
         *
         */
        geometry_msgs::msg::Pose current_pose_odom;

        // Create a variable to store the current pose of the robot
        /**
         * @brief Variable to store the current pose of the robot
         *
         */
        geometry_msgs::msg::Pose current_pose;

        // Create a vector of type Waypoint to store the waypoints
        /**
         * @brief Vector of type Waypoint to store the waypoints
         *
         */
        std::vector<Waypoint> waypoints;

        // Create a function to create the waypoints vector based on the marker id
        /**
         * @brief Function to create the waypoints vector based on the marker id
         *
         * @param marker_id Marker id of the aruco marker
         */
        void create_waypoints(int marker_id);

        // Create a part object to store the part data
        /**
         * @brief Part object to store the part data
         *
         */
        mage_msgs::msg::Part part;

        // Create a mapper from the part type to the part type id
        /**
         * @brief Mapper from the part type to the part type id
         *
         */
        std::map<std::string, int> part_type_to_id = {{"battery", part.BATTERY}, {"pump", part.PUMP}, {"sensor", part.SENSOR}, {"regulator", part.REGULATOR}};
        // Create a mapper from the part color to the part color id
        /**
         * @brief Mapper from the part color to the part color id
         *
         */
        std::map<std::string, int> part_color_to_id = {{"red", part.RED}, {"green", part.GREEN}, {"blue", part.BLUE}, {"orange", part.ORANGE}, {"purple", part.PURPLE}};
        // Create a mapper from the marker to marker id
        /**
         * @brief Mapper from the marker to marker id
         *
         */
        std::map<std::string, int> marker_to_id = {{"aruco_0", 0}, {"aruco_1", 1}};

        // Create a mapper between color id and camera_frame
        // std::map<int, std::string> color_id_to_camera_frame = {{part.BLUE, "part1_frame"},
        //                                                        {part.GREEN, "part4_frame"},
        //                                                        {part.ORANGE, "part2_frame"},
        //                                                        {part.PURPLE, "part3_frame"},
        //                                                        {part.RED, "part5_frame"}};

        // Create a mapper between color id and camera_frame
        /**
         * @brief Mapper between color id and camera_frame
         *
         */
        std::map<int, std::string> color_id_to_camera_frame;

        // Decalre a variables to store the waypoint type and color ids
        /**
         * @brief Variables to store the waypoint type and color ids
         *
         */
        int wp1_type_id,
            wp1_color_id, wp2_type_id, wp2_color_id, wp3_type_id, wp3_color_id, wp4_type_id, wp4_color_id, wp5_type_id, wp5_color_id;

        // Declare a variable to store the waypoint pose
        /**
         * @brief Variable to store the waypoint pose
         *
         */
        geometry_msgs::msg::Pose wp1_pose, wp2_pose, wp3_pose, wp4_pose, wp5_pose;

        //  Create a listener to listen to the transform between the source and the target frame
        /**
         * @brief Listener to listen to the transform between the source and the target frame
         *
         */
        std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
        /**
         * @brief Transform listener to listen to the transform between the source and the target frame
         *
         */
        std::shared_ptr<tf2_ros::TransformListener> tf_listener_;

        /**
         * @brief Listen to a transform
         *
         * @param source_frame Source frame (child frame) of the transform
         * @param target_frame Target frame (parent frame) of the transform
         */
        geometry_msgs::msg::Pose listen_transform(const std::string &source_frame, const std::string &target_frame);

        /**
         * @brief Publisher to the topic /initialpose
         *
         */
        rclcpp::Publisher<geometry_msgs::msg::PoseWithCovarianceStamped>::SharedPtr
            initial_pose_pub_;
        /**
         * @brief Action client for the action server navigate_to_pose
         *
         */
        rclcpp_action::Client<NavigateToPose>::SharedPtr client_;
        //   rclcpp::TimerBase::SharedPtr timer_;
        /**
         * @brief Response from the server after sending the goal
         */
        void goal_response_callback(
            std::shared_future<GoalHandleNavigation::SharedPtr> future);
        /**
         * @brief Feedback received while the robot is driving towards the goal
         *
         * @param feedback
         */
        void feedback_callback(
            GoalHandleNavigation::SharedPtr,
            const std::shared_ptr<const NavigateToPose::Feedback> feedback);
        /**
         * @brief Result after the action has completed
         *
         * @param result
         */
        void result_callback(const GoalHandleNavigation::WrappedResult &result);
        /**
         * @brief Method to build and send a goal using the client
         *
         */
        void send_goal();
        /**
         * @brief Set the initial pose object
         *
         * @return true
         * @return false
         */
        bool set_initial_pose();

        // Variable to store the goal msg to be sent to the server
        /**
         * @brief Variable to store the goal msg to be sent to the server
         *
         */
        NavigateToPose::Goal goal_msg;

        // Create a subscriber 1 for the logical camera image
        /**
         * @brief Subscriber 1 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber1_;
        /**
         * @brief Callback function for subscriber of part1
         *
         */
        // Create a subscriber 2 for the logical camera image
        /**
         * @brief Subscriber 2 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber2_;
        /**
         * @brief Callback function for subscriber of part2
         *
         */
        // Create a subscriber 3 for the logical camera image
        /**
         * @brief Subscriber 3 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber3_;
        // Create a subscriber 4 for the logical camera image
        /**
         * @brief Subscriber 4 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber4_;
        // Create a subscriber 5 for the logical camera image
        /**
         * @brief Subscriber 5 for the logical camera image
         *
         */
        rclcpp::Subscription<mage_msgs::msg::AdvancedLogicalCameraImage>::SharedPtr parts_subscriber5_;

        // Create callback function for subscriber of part1
        /**
         * @brief Callback function for subscriber of part1
         *
         * @param msg Message published on the topic /mage/camera1/image
         */
        void parts1_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part2
        /**
         * @brief Callback function for subscriber of part2
         *
         * @param msg Message published on the topic /mage/camera2/image
         */
        void parts2_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part3
        /**
         * @brief Callback function for subscriber of part3
         *
         * @param msg Message published on the topic /mage/camera3/image
         */
        void parts3_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part4
        /**
         * @brief Callback function for subscriber of part4
         *
         * @param msg Message published on the topic /mage/camera4/image
         */
        void parts4_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

        // Create callback function for subscriber of part5
        /**
         * @brief Callback function for subscriber of part5
         *
         * @param msg Message published on the topic /mage/camera5/image
         */
        void parts5_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg);

    }; // class Tb3ArucoDetector
} // namespace final_project
