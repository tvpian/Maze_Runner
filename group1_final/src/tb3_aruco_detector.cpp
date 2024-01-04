#include <group1_final/tb3_aruco_detector.hpp>

// Definition of callback function of the subscriber of part1
void final_project::Tb3ArucoDetector::parts1_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // // if the msg is empty, return
    // if (msg->part_poses.empty())
    // {
    //     return;
    // }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the map between color and camera frame
        color_id_to_camera_frame[part.part.color] = "part1_frame";
        // Print the color_id_to_camera_frame
        RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame: %s", color_id_to_camera_frame[part.part.color].c_str());
    }
}

// Definition of callback function of the subscriber of part2
void final_project::Tb3ArucoDetector::parts2_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // // if the msg is empty, return
    // if (msg->part_poses.empty())
    // {
    //     return;
    // }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the map between color and camera frame
        color_id_to_camera_frame[part.part.color] = "part2_frame";
        RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame: %s", color_id_to_camera_frame[part.part.color].c_str());
    }
}

// Definition of callback function of the subscriber of part3
void final_project::Tb3ArucoDetector::parts3_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // // if the msg is empty, return
    // if (msg->part_poses.empty())
    // {
    //     return;
    // }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the map between color and camera frame
        color_id_to_camera_frame[part.part.color] = "part3_frame";
        RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame: %s", color_id_to_camera_frame[part.part.color].c_str());
    }
}

// Definition of callback function of the subscriber of part4
void final_project::Tb3ArucoDetector::parts4_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

    // // if the msg is empty, return
    // if (msg->part_poses.empty())
    // {
    //     return;
    // }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the map between color and camera frame
        color_id_to_camera_frame[part.part.color] = "part4_frame";
        RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame: %s", color_id_to_camera_frame[part.part.color].c_str());
    }
}

// Definition of callback function of the subscriber of part5
void final_project::Tb3ArucoDetector::parts5_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

    // // if the msg is empty, return
    // if (msg->part_poses.empty())
    // {
    //     return;
    // }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the map between color and camera frame
        color_id_to_camera_frame[part.part.color] = "part5_frame";
        RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame: %s", color_id_to_camera_frame[part.part.color].c_str());
    }
}

// Define the callback function for the /amcl_pose topic
void final_project::Tb3ArucoDetector::amcl_pose_cb(const geometry_msgs::msg::PoseWithCovarianceStamped::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received amcl_pose message");
    // Store the pose in the class variable

    // If the message is empty, return to prevent the rest of the function from executing
    if (!msg)
    {
        // Assign the current pose to the amcl_pose variable
        current_pose.position.x = current_pose_odom.position.x;
        current_pose.position.y = current_pose_odom.position.y;
        current_pose.position.z = current_pose_odom.position.z;
        current_pose.orientation.x = current_pose_odom.orientation.x;
        current_pose.orientation.y = current_pose_odom.orientation.y;
        current_pose.orientation.z = current_pose_odom.orientation.z;
        current_pose.orientation.w = current_pose_odom.orientation.w;

        // // Print taking odom pose to the terminal
        // RCLCPP_INFO(this->get_logger(), "Taking odom pose");
        // return;
    }
    current_pose.position.x = msg->pose.pose.position.x;
    current_pose.position.y = msg->pose.pose.position.y;
    current_pose.position.z = msg->pose.pose.position.z;
    current_pose.orientation.x = msg->pose.pose.orientation.x;
    current_pose.orientation.y = msg->pose.pose.orientation.y;
    current_pose.orientation.z = msg->pose.pose.orientation.z;
    current_pose.orientation.w = msg->pose.pose.orientation.w;
    // Print the current pose to the terminal

    // RCLCPP_INFO(this->get_logger(), "amcl_pose: %f, %f, %f", amcl_pose.position.x, amcl_pose.position.y, amcl_pose.position.z);
}

// Define the callback function for the /odom topic
void final_project::Tb3ArucoDetector::odom_cb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    // RCLCPP_INFO(this->get_logger(), "Received odom message");
    // Store the pose in the class variable

    // If the message is empty, return to prevent the rest of the function from executing
    if (!msg)
    {
        return;
    }
    current_pose_odom.position.x = msg->pose.pose.position.x;
    current_pose_odom.position.y = msg->pose.pose.position.y;
    current_pose_odom.position.z = msg->pose.pose.position.z;
    current_pose_odom.orientation.x = msg->pose.pose.orientation.x;
    current_pose_odom.orientation.y = msg->pose.pose.orientation.y;
    current_pose_odom.orientation.z = msg->pose.pose.orientation.z;
    current_pose_odom.orientation.w = msg->pose.pose.orientation.w;
}

void final_project::Tb3ArucoDetector::aruco_markers_cb(const ros2_aruco_interfaces::msg::ArucoMarkers::SharedPtr msg)
{

    // Check if the message is empty
    if (!msg)
    {
        // Print "No markers detected" to the terminal
        RCLCPP_INFO(this->get_logger(), "No markers detected");
        // Return to prevent the rest of the function from executing
        return;
    }
    else
    {
        // // Print "Markers detected" to the terminal
        RCLCPP_INFO(this->get_logger(), "Markers detected");
        // Store the marker id in the class variable
        marker_id = msg->marker_ids[0];
        // // Print the marker id to the terminal
        RCLCPP_INFO(this->get_logger(), "Marker ID: %d", marker_id);
        // Wait for the rest of the parts to be detected
        // Print "Waiting for parts" to the terminal
        if (color_id_to_camera_frame.size() < 5)
        {
            RCLCPP_INFO(this->get_logger(), "Waiting for parts");
            return;
        }
        else
        {
            // Print "Got all parts" to the terminal
            RCLCPP_INFO(this->get_logger(), "Got all parts");
            // RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame size: %d", color_id_to_camera_frame.size());
            create_waypoints(marker_id);
            subscription_.reset();
            ////////////////////////////////
            // After receiving the pose of the part1 in the camera1 frame once, stop the subscriber
            parts_subscriber1_.reset();
            // After receiving the pose of the part2 in the camera2 frame once, stop the subscriber
            parts_subscriber2_.reset();
            // After receiving the pose of the part3 in the camera3 frame once, stop the subscriber
            parts_subscriber3_.reset();
            // After receiving the pose of the part4 in the camera4 frame once, stop the subscriber
            parts_subscriber4_.reset();
            // After receiving the pose of the part5 in the camera5 frame once, stop the subscriber
            parts_subscriber5_.reset();
        }

        // RCLCPP_INFO(this->get_logger(), "Got all parts");
        // RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame size: %d", color_id_to_camera_frame.size());
        // create_waypoints(marker_id);
        // subscription_.reset();
        ////////////////////////////////
    }
}

geometry_msgs::msg::Pose final_project::Tb3ArucoDetector::listen_transform(const std::string &source_frame, const std::string &target_frame)
{
    // Create a TransformStamped variable to store the transform between the source and target frames
    geometry_msgs::msg::TransformStamped t_stamped;
    // Create a Pose variable to store the pose of the target frame in the source frame
    geometry_msgs::msg::Pose pose_out;
    try
    {
        t_stamped = tf_buffer_->lookupTransform(source_frame, target_frame, tf2::TimePointZero, 50ms);
    }
    catch (const tf2::TransformException &ex)
    {
    }

    pose_out.position.x = t_stamped.transform.translation.x;
    pose_out.position.y = t_stamped.transform.translation.y;
    pose_out.position.z = t_stamped.transform.translation.z;
    pose_out.orientation = t_stamped.transform.rotation;

    return pose_out;
}

void final_project::Tb3ArucoDetector::create_waypoints(int marker_id)
{
    // Check if the color_id_to_camera_frame has 5 elements if not wait for the rest of the parts in a while loop
    // Print enter the  create_waypoints function to the terminal
    RCLCPP_INFO(this->get_logger(), "Enter the create_waypoints function");
    // while (color_id_to_camera_frame.size() < 5)
    // {
    //     // // Print the current size of the color_id_to_camera_frame to the terminal
    //     // RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame size: %d", color_id_to_camera_frame.size());
    //     // // Print "Waiting for parts" to the terminal
    //     // RCLCPP_INFO(this->get_logger(), "Waiting for parts");
    //     RCLCPP_INFO(this->get_logger(), "color_id_to_camera_frame: %s", color_id_to_camera_frame[0].c_str());
    // }

    // Check if the marker id is 'aruco_0'
    if (marker_id == 0)
    {
        // Print the marker id
        RCLCPP_INFO(this->get_logger(), "Marker ID: %d", marker_id);
        // Assign the type and color ids using the parameters and mapper for aruco_0 wp1
        wp1_type_id = part_type_to_id[get_parameter("aruco_0.wp1.type").as_string()];
        wp1_color_id = part_color_to_id[get_parameter("aruco_0.wp1.color").as_string()];
        wp1_pose = listen_transform("odom", color_id_to_camera_frame[wp1_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_0 wp2
        wp2_type_id = part_type_to_id[get_parameter("aruco_0.wp2.type").as_string()];
        wp2_color_id = part_color_to_id[get_parameter("aruco_0.wp2.color").as_string()];
        wp2_pose = listen_transform("odom", color_id_to_camera_frame[wp2_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_0 wp3
        wp3_type_id = part_type_to_id[get_parameter("aruco_0.wp3.type").as_string()];
        wp3_color_id = part_color_to_id[get_parameter("aruco_0.wp3.color").as_string()];
        wp3_pose = listen_transform("odom", color_id_to_camera_frame[wp3_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_0 wp4
        wp4_type_id = part_type_to_id[get_parameter("aruco_0.wp4.type").as_string()];
        wp4_color_id = part_color_to_id[get_parameter("aruco_0.wp4.color").as_string()];
        wp4_pose = listen_transform("odom", color_id_to_camera_frame[wp4_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_0 wp5
        wp5_type_id = part_type_to_id[get_parameter("aruco_0.wp5.type").as_string()];
        wp5_color_id = part_color_to_id[get_parameter("aruco_0.wp5.color").as_string()];
        wp5_pose = listen_transform("odom", color_id_to_camera_frame[wp5_color_id]);

        // Add the waypoints to the vector
        waypoints[0] = {wp1_type_id, wp1_color_id, wp1_pose};
        waypoints[1] = {wp2_type_id, wp2_color_id, wp2_pose};
        waypoints[2] = {wp3_type_id, wp3_color_id, wp3_pose};
        waypoints[3] = {wp4_type_id, wp4_color_id, wp4_pose};
        waypoints[4] = {wp5_type_id, wp5_color_id, wp5_pose};

        // Print the waypoints to the terminal
        RCLCPP_INFO(this->get_logger(), "waypoints: %f, %f, %f", waypoints[0].pose.position.x, waypoints[0].pose.position.y, waypoints[0].pose.position.z);
        RCLCPP_INFO(this->get_logger(), "waypoints: %f, %f, %f", waypoints[1].pose.position.x, waypoints[1].pose.position.y, waypoints[1].pose.position.z);
        RCLCPP_INFO(this->get_logger(), "waypoints: %f, %f, %f", waypoints[2].pose.position.x, waypoints[2].pose.position.y, waypoints[2].pose.position.z);
        RCLCPP_INFO(this->get_logger(), "waypoints: %f, %f, %f", waypoints[3].pose.position.x, waypoints[3].pose.position.y, waypoints[3].pose.position.z);
        RCLCPP_INFO(this->get_logger(), "waypoints: %f, %f, %f", waypoints[4].pose.position.x, waypoints[4].pose.position.y, waypoints[4].pose.position.z);

        // Add the waypoint poses in order to the goal msg of the waypoint_navigator_ object
        for (int i = 0; i < 5; i++)
        {
            goal_msg.poses[i].header.frame_id = "map";
            goal_msg.poses[i].header.stamp = this->now();
            goal_msg.poses[i].pose.position.x = waypoints[i].pose.position.x;
            goal_msg.poses[i].pose.position.y = waypoints[i].pose.position.y;
            goal_msg.poses[i].pose.position.z = waypoints[i].pose.position.z;
            goal_msg.poses[i].pose.orientation.x = waypoints[i].pose.orientation.x;
            goal_msg.poses[i].pose.orientation.y = waypoints[i].pose.orientation.y;
            goal_msg.poses[i].pose.orientation.z = waypoints[i].pose.orientation.z;
            goal_msg.poses[i].pose.orientation.w = waypoints[i].pose.orientation.w;
        }

        // Print "Navigation Started" to the terminal
        RCLCPP_INFO(this->get_logger(), "Navigation Started");
        // Publish the goal msgq
        send_goal();
    }
    else if (marker_id == 1)
    {
        // Print the marker id
        RCLCPP_INFO(this->get_logger(), "Marker ID: %d", marker_id);
        // Assign the type and color ids using the parameters and mapper for aruco_1 wp1
        wp1_type_id = part_type_to_id[get_parameter("aruco_1.wp1.type").as_string()];
        wp1_color_id = part_color_to_id[get_parameter("aruco_1.wp1.color").as_string()];
        wp1_pose = listen_transform("odom", color_id_to_camera_frame[wp1_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_1 wp2
        wp2_type_id = part_type_to_id[get_parameter("aruco_1.wp2.type").as_string()];
        wp2_color_id = part_color_to_id[get_parameter("aruco_1.wp2.color").as_string()];
        wp2_pose = listen_transform("odom", color_id_to_camera_frame[wp2_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_1 wp3
        wp3_type_id = part_type_to_id[get_parameter("aruco_1.wp3.type").as_string()];
        wp3_color_id = part_color_to_id[get_parameter("aruco_1.wp3.color").as_string()];
        wp3_pose = listen_transform("odom", color_id_to_camera_frame[wp3_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_1 wp4
        wp4_type_id = part_type_to_id[get_parameter("aruco_1.wp4.type").as_string()];
        wp4_color_id = part_color_to_id[get_parameter("aruco_1.wp4.color").as_string()];
        wp4_pose = listen_transform("odom", color_id_to_camera_frame[wp4_color_id]);
        // Assign the type and color ids using the parameters and mapper for aruco_1 wp5
        wp5_type_id = part_type_to_id[get_parameter("aruco_1.wp5.type").as_string()];
        wp5_color_id = part_color_to_id[get_parameter("aruco_1.wp5.color").as_string()];
        wp5_pose = listen_transform("odom", color_id_to_camera_frame[wp5_color_id]);

        // Add the waypoints to the vector
        waypoints[0] = {wp1_type_id, wp1_color_id, wp1_pose};
        waypoints[1] = {wp2_type_id, wp2_color_id, wp2_pose};
        waypoints[2] = {wp3_type_id, wp3_color_id, wp3_pose};
        waypoints[3] = {wp4_type_id, wp4_color_id, wp4_pose};
        waypoints[4] = {wp5_type_id, wp5_color_id, wp5_pose};

        // Add the waypoint poses in order to the goal msg of the waypoint_navigator_ object
        for (int i = 0; i < 5; i++)
        {
            goal_msg.poses[i].header.frame_id = "map";
            goal_msg.poses[i].header.stamp = this->now();
            goal_msg.poses[i].pose.position.x = waypoints[i].pose.position.x;
            goal_msg.poses[i].pose.position.y = waypoints[i].pose.position.y;
            goal_msg.poses[i].pose.position.z = waypoints[i].pose.position.z;
            goal_msg.poses[i].pose.orientation.x = waypoints[i].pose.orientation.x;
            goal_msg.poses[i].pose.orientation.y = waypoints[i].pose.orientation.y;
            goal_msg.poses[i].pose.orientation.z = waypoints[i].pose.orientation.z;
            goal_msg.poses[i].pose.orientation.w = waypoints[i].pose.orientation.w;
        }

        // Print "Navigation Started" to the terminal
        RCLCPP_INFO(this->get_logger(), "Navigation Started");
        // Publish the goal msgq
        send_goal();
    }
    else
    {
        RCLCPP_ERROR(this->get_logger(), "Invalid marker id");
    }
}

//===============================================
bool final_project::Tb3ArucoDetector::set_initial_pose()
{
    // Create a PoseWithCovarianceStamped variable to store the initial pose
    geometry_msgs::msg::PoseWithCovarianceStamped initial_pose;
    // Set the header of the initial pose
    initial_pose.header.frame_id = "map";
    initial_pose.header.stamp = this->now();
    // Set the pose of the initial pose
    initial_pose.pose.pose.position.x = 1.0000003151192325;
    initial_pose.pose.pose.position.y = -1.5914327499934535;
    initial_pose.pose.pose.position.z = 0.007829696781041461;
    initial_pose.pose.pose.orientation.x = 0.002345522247684733;
    initial_pose.pose.pose.orientation.y = 0.0023292917752085023;
    initial_pose.pose.pose.orientation.z = -0.7067853636345984;
    initial_pose.pose.pose.orientation.w = 0.7074203295616548;
    // Publish the initial pose
    initial_pose_pub_->publish(initial_pose);
    // Sleep for 5 seconds
    // set the valid_initial_pose to true
}
//===============================================
void final_project::Tb3ArucoDetector::send_goal()
{
    using namespace std::placeholders;

    if (!this->client_->wait_for_action_server())
    {
        RCLCPP_ERROR(this->get_logger(),
                     "Action server not available after waiting");
        rclcpp::shutdown();
    }

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options =
        rclcpp_action::Client<NavigateToPose>::SendGoalOptions();
    send_goal_options.goal_response_callback =
        std::bind(&Tb3ArucoDetector::goal_response_callback, this, _1);
    // send_goal_options.feedback_callback =
    //     std::bind(&Tb3ArucoDetector::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
        std::bind(&Tb3ArucoDetector::result_callback, this, _1);

    client_->async_send_goal(goal_msg, send_goal_options);
}

//===============================================
void final_project::Tb3ArucoDetector::goal_response_callback(
    std::shared_future<GoalHandleNavigation::SharedPtr> future)
{
    auto goal_handle = future.get();
    if (!goal_handle)
    {
        RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
        rclcpp::shutdown();
    }
    else
    {
        RCLCPP_INFO(this->get_logger(),
                    "Goal accepted by server, waiting for result");
    }
}

//===============================================
void final_project::Tb3ArucoDetector::feedback_callback(
    GoalHandleNavigation::SharedPtr,
    const std::shared_ptr<const NavigateToPose::Feedback> feedback)
{
    RCLCPP_INFO(this->get_logger(), "Robot is driving towards the goal");
}

//===============================================
void final_project::Tb3ArucoDetector::result_callback(
    const GoalHandleNavigation::WrappedResult &result)
{
    switch (result.code)
    {
    case rclcpp_action::ResultCode::SUCCEEDED:
        break;
    case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
    case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
    default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    rclcpp::shutdown();
}
int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<final_project::Tb3ArucoDetector>("tb3_aruco_detector");
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
