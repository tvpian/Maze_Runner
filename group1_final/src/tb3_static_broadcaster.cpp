#include <group1_final/tb3_static_broadcaster.hpp>

// Definition of callback function of the subscriber of part1
void final_project::StaticBroadcaster::parts1_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // if the msg is empty, return
    if (msg->part_poses.empty())
    {
        return;
    }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the pose of the part1_frame
        part1_camera1_pose_ = part.pose;
    }

    // After receiving the pose of the part1 in the camera1 frame once, stop the subscriber
    parts_subscriber1_.reset();
}

// Definition of callback function of the subscriber of part2
void final_project::StaticBroadcaster::parts2_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // if the msg is empty, return
    if (msg->part_poses.empty())
    {
        return;
    }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the pose of the part2_frame
        part2_camera2_pose_ = part.pose;
    }

    // After receiving the pose of the part2 in the camera2 frame once, stop the subscriber
    parts_subscriber2_.reset();
}

// Definition of callback function of the subscriber of part3
void final_project::StaticBroadcaster::parts3_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{
    // if the msg is empty, return
    if (msg->part_poses.empty())
    {
        return;
    }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the pose of the part3_frame
        part3_camera3_pose_ = part.pose;
    }

    // After receiving the pose of the part3 in the camera3 frame once, stop the subscriber
    parts_subscriber3_.reset();
}

// Definition of callback function of the subscriber of part4
void final_project::StaticBroadcaster::parts4_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

    // if the msg is empty, return
    if (msg->part_poses.empty())
    {
        return;
    }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the pose of the part4_frame
        part4_camera4_pose_ = part.pose;
    }

    // After receiving the pose of the part4 in the camera4 frame once, stop the subscriber
    parts_subscriber4_.reset();
}

// Definition of callback function of the subscriber of part5
void final_project::StaticBroadcaster::parts5_cb(const mage_msgs::msg::AdvancedLogicalCameraImage::SharedPtr msg)
{

    // if the msg is empty, return
    if (msg->part_poses.empty())
    {
        return;
    }
    // Access the parts in the message and save the color and type of the first part
    for (auto part : msg->part_poses)
    {
        // save the pose of the part5_frame
        part5_camera5_pose_ = part.pose;
    }

    // After receiving the pose of the part5 in the camera5 frame once, stop the subscriber
    parts_subscriber5_.reset();
}

// Definition of callback function of the parts and camera subscriber
// void final_project::StaticBroadcaster::static_broadcast_timer_cb_odom_()
// {
//     // Create a geometry_msgs::msg::TransformStamped object for the transform between the map and odom
//     geometry_msgs::msg::TransformStamped map_odom_transform_stamped;
//     // The odom frame and the map frame are the same
//     map_odom_transform_stamped.header.frame_id = "world";
//     map_odom_transform_stamped.child_frame_id = "odom";
//     // Set the translation of the transform
//     map_odom_transform_stamped.transform.translation.x = map_odom_pose_.position.x;
//     map_odom_transform_stamped.transform.translation.y = map_odom_pose_.position.y;
//     map_odom_transform_stamped.transform.translation.z = map_odom_pose_.position.z;
//     // Set the rotation of the transform
//     map_odom_transform_stamped.transform.rotation.x = map_odom_pose_.orientation.x;
//     map_odom_transform_stamped.transform.rotation.y = map_odom_pose_.orientation.y;
//     map_odom_transform_stamped.transform.rotation.z = map_odom_pose_.orientation.z;
//     map_odom_transform_stamped.transform.rotation.w = map_odom_pose_.orientation.w;
//     // Send the transform
//     // tf_static_broadcaster_map_odom_->sendTransform(map_odom_transform_stamped);

//     // Stop the timer
//     static_broadcast_timer_parts_camera_->cancel();
// }

// Definition of callback function of the timer
void final_project::StaticBroadcaster::static_broadcast_timer_cb_parts_camera_()
{
    // Create a geometry_msgs::msg::TransformStamped object for the transform between the camera1 and part1
    geometry_msgs::msg::TransformStamped camera1_part1_transform_stamped;
    // The camera1 frame and the part1 frame are the same
    camera1_part1_transform_stamped.header.frame_id = "camera1_frame";
    camera1_part1_transform_stamped.child_frame_id = "part1_frame";
    // Set the translation of the transform
    camera1_part1_transform_stamped.transform.translation.x = part1_camera1_pose_.position.x;
    camera1_part1_transform_stamped.transform.translation.y = part1_camera1_pose_.position.y;
    camera1_part1_transform_stamped.transform.translation.z = part1_camera1_pose_.position.z;
    // Set the rotation of the transform
    camera1_part1_transform_stamped.transform.rotation.x = part1_camera1_pose_.orientation.x;
    camera1_part1_transform_stamped.transform.rotation.y = part1_camera1_pose_.orientation.y;
    camera1_part1_transform_stamped.transform.rotation.z = part1_camera1_pose_.orientation.z;
    camera1_part1_transform_stamped.transform.rotation.w = part1_camera1_pose_.orientation.w;
    // Send the transform
    tf_static_broadcaster_part1_camera1_->sendTransform(camera1_part1_transform_stamped);

    // Create a geometry_msgs::msg::TransformStamped object for the transform between the camera2 and part2
    geometry_msgs::msg::TransformStamped camera2_part2_transform_stamped;
    // The camera2 frame and the part2 frame are the same
    camera2_part2_transform_stamped.header.frame_id = "camera2_frame";
    camera2_part2_transform_stamped.child_frame_id = "part2_frame";
    // Set the translation of the transform
    camera2_part2_transform_stamped.transform.translation.x = part2_camera2_pose_.position.x;
    camera2_part2_transform_stamped.transform.translation.y = part2_camera2_pose_.position.y;
    camera2_part2_transform_stamped.transform.translation.z = part2_camera2_pose_.position.z;
    // Set the rotation of the transform
    camera2_part2_transform_stamped.transform.rotation.x = part2_camera2_pose_.orientation.x;
    camera2_part2_transform_stamped.transform.rotation.y = part2_camera2_pose_.orientation.y;
    camera2_part2_transform_stamped.transform.rotation.z = part2_camera2_pose_.orientation.z;
    camera2_part2_transform_stamped.transform.rotation.w = part2_camera2_pose_.orientation.w;
    // Send the transform
    tf_static_broadcaster_part2_camera2_->sendTransform(camera2_part2_transform_stamped);

    // Create a geometry_msgs::msg::TransformStamped object for the transform between the camera3 and part3
    geometry_msgs::msg::TransformStamped camera3_part3_transform_stamped;
    // The camera3 frame and the part3 frame are the same
    camera3_part3_transform_stamped.header.frame_id = "camera3_frame";
    camera3_part3_transform_stamped.child_frame_id = "part3_frame";
    // Set the translation of the transform
    camera3_part3_transform_stamped.transform.translation.x = part3_camera3_pose_.position.x;
    camera3_part3_transform_stamped.transform.translation.y = part3_camera3_pose_.position.y;
    camera3_part3_transform_stamped.transform.translation.z = part3_camera3_pose_.position.z;
    // Set the rotation of the transform
    camera3_part3_transform_stamped.transform.rotation.x = part3_camera3_pose_.orientation.x;
    camera3_part3_transform_stamped.transform.rotation.y = part3_camera3_pose_.orientation.y;
    camera3_part3_transform_stamped.transform.rotation.z = part3_camera3_pose_.orientation.z;
    camera3_part3_transform_stamped.transform.rotation.w = part3_camera3_pose_.orientation.w;
    // Send the transform
    tf_static_broadcaster_part3_camera3_->sendTransform(camera3_part3_transform_stamped);

    // Create a geometry_msgs::msg::TransformStamped object for the transform between the camera4 and part4
    geometry_msgs::msg::TransformStamped camera4_part4_transform_stamped;
    // The camera4 frame and the part4 frame are the same
    camera4_part4_transform_stamped.header.frame_id = "camera4_frame";
    camera4_part4_transform_stamped.child_frame_id = "part4_frame";
    // Set the translation of the transform
    camera4_part4_transform_stamped.transform.translation.x = part4_camera4_pose_.position.x;
    camera4_part4_transform_stamped.transform.translation.y = part4_camera4_pose_.position.y;
    camera4_part4_transform_stamped.transform.translation.z = part4_camera4_pose_.position.z;
    // Set the rotation of the transform
    camera4_part4_transform_stamped.transform.rotation.x = part4_camera4_pose_.orientation.x;
    camera4_part4_transform_stamped.transform.rotation.y = part4_camera4_pose_.orientation.y;
    camera4_part4_transform_stamped.transform.rotation.z = part4_camera4_pose_.orientation.z;
    camera4_part4_transform_stamped.transform.rotation.w = part4_camera4_pose_.orientation.w;
    // Send the transform
    tf_static_broadcaster_part4_camera4_->sendTransform(camera4_part4_transform_stamped);

    // Create a geometry_msgs::msg::TransformStamped object for the transform between the camera5 and part5
    geometry_msgs::msg::TransformStamped camera5_part5_transform_stamped;
    // The camera5 frame and the part5 frame are the same
    camera5_part5_transform_stamped.header.frame_id = "camera5_frame";
    camera5_part5_transform_stamped.child_frame_id = "part5_frame";
    // Set the translation of the transform
    camera5_part5_transform_stamped.transform.translation.x = part5_camera5_pose_.position.x;
    camera5_part5_transform_stamped.transform.translation.y = part5_camera5_pose_.position.y;
    camera5_part5_transform_stamped.transform.translation.z = part5_camera5_pose_.position.z;
    // Set the rotation of the transform
    camera5_part5_transform_stamped.transform.rotation.x = part5_camera5_pose_.orientation.x;
    camera5_part5_transform_stamped.transform.rotation.y = part5_camera5_pose_.orientation.y;
    camera5_part5_transform_stamped.transform.rotation.z = part5_camera5_pose_.orientation.z;
    camera5_part5_transform_stamped.transform.rotation.w = part5_camera5_pose_.orientation.w;
    // Send the transform
    tf_static_broadcaster_part5_camera5_->sendTransform(camera5_part5_transform_stamped);

    // tf_static_broadcaster_map_odom_ = nullptr;
}

int main(int argc, char **argv)
{
    // Initialize the node
    rclcpp::init(argc, argv);
    // Create a node
    auto node = std::make_shared<final_project::StaticBroadcaster>("tb3_static_broadcaster");
    // Spin the node
    rclcpp::spin(node);
    // Shut down the node
    rclcpp::shutdown();
}
