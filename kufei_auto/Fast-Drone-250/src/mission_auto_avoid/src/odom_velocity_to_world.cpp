#include <ros/ros.h>

#include <nav_msgs/Odometry.h>

#include <Eigen/Geometry>

#include <string>

namespace mission_auto_avoid
{

/**
 * @brief Adapt MAVROS odometry for planners that expect twist in world axes.
 *
 * nav_msgs/Odometry defines twist in child_frame_id (normally base_link),
 * while EGO-Planner uses twist.linear directly as a world-frame start velocity.
 * This node keeps pose and timestamps unchanged, rotates the complete twist and
 * its covariance into the odometry header frame, and republishes an EGO-only
 * compatibility topic.
 */
class OdomVelocityToWorld
{
public:
  OdomVelocityToWorld()
    : nh_(), private_nh_("~")
  {
    private_nh_.param<std::string>(
        "input_odom_topic", input_odom_topic_, "/mavros/local_position/odom");
    private_nh_.param<std::string>(
        "output_odom_topic", output_odom_topic_, "/ego/odom_world_velocity");
    private_nh_.param("input_twist_in_body_frame", input_twist_in_body_frame_, true);
    private_nh_.param("queue_size", queue_size_, 10);

    if (queue_size_ < 1)
    {
      ROS_WARN("odom_velocity_to_world: queue_size must be positive; using 10.");
      queue_size_ = 10;
    }

    odom_pub_ = nh_.advertise<nav_msgs::Odometry>(output_odom_topic_, queue_size_);
    odom_sub_ = nh_.subscribe(
        input_odom_topic_, queue_size_, &OdomVelocityToWorld::odomCallback, this,
        ros::TransportHints().tcpNoDelay());

    ROS_INFO("odom_velocity_to_world started:");
    ROS_INFO("  input=%s", input_odom_topic_.c_str());
    ROS_INFO("  output=%s", output_odom_topic_.c_str());
    ROS_INFO("  input_twist_in_body_frame=%s",
             input_twist_in_body_frame_ ? "true" : "false");
  }

private:
  ros::NodeHandle nh_;
  ros::NodeHandle private_nh_;
  ros::Subscriber odom_sub_;
  ros::Publisher odom_pub_;

  std::string input_odom_topic_;
  std::string output_odom_topic_;
  bool input_twist_in_body_frame_ = true;
  int queue_size_ = 10;

  void odomCallback(const nav_msgs::OdometryConstPtr &input_msg)
  {
    nav_msgs::Odometry output_msg = *input_msg;

    // Some odometry sources already publish twist in world axes. In that case
    // this node acts as a pass-through so launch wiring can stay unchanged.
    if (!input_twist_in_body_frame_)
    {
      odom_pub_.publish(output_msg);
      return;
    }

    Eigen::Quaterniond world_from_body(
        input_msg->pose.pose.orientation.w,
        input_msg->pose.pose.orientation.x,
        input_msg->pose.pose.orientation.y,
        input_msg->pose.pose.orientation.z);

    if (!world_from_body.coeffs().allFinite() || world_from_body.norm() < 1e-9)
    {
      ROS_WARN_THROTTLE(
          1.0,
          "odom_velocity_to_world: invalid odometry orientation; dropping message.");
      return;
    }
    world_from_body.normalize();

    const Eigen::Matrix3d rotation = world_from_body.toRotationMatrix();
    const Eigen::Vector3d linear_body(
        input_msg->twist.twist.linear.x,
        input_msg->twist.twist.linear.y,
        input_msg->twist.twist.linear.z);
    const Eigen::Vector3d angular_body(
        input_msg->twist.twist.angular.x,
        input_msg->twist.twist.angular.y,
        input_msg->twist.twist.angular.z);

    const Eigen::Vector3d linear_world = rotation * linear_body;
    const Eigen::Vector3d angular_world = rotation * angular_body;

    output_msg.twist.twist.linear.x = linear_world.x();
    output_msg.twist.twist.linear.y = linear_world.y();
    output_msg.twist.twist.linear.z = linear_world.z();
    output_msg.twist.twist.angular.x = angular_world.x();
    output_msg.twist.twist.angular.y = angular_world.y();
    output_msg.twist.twist.angular.z = angular_world.z();

    // Rotate the 6x6 twist covariance with diag(R, R), including linear/angular
    // cross-covariance terms. EGO currently ignores it, but the output remains
    // internally consistent for diagnostics.
    Eigen::Matrix<double, 6, 6> covariance_body;
    for (int row = 0; row < 6; ++row)
    {
      for (int col = 0; col < 6; ++col)
      {
        covariance_body(row, col) =
            input_msg->twist.covariance[static_cast<std::size_t>(row * 6 + col)];
      }
    }

    Eigen::Matrix<double, 6, 6> rotation_6d =
        Eigen::Matrix<double, 6, 6>::Zero();
    rotation_6d.block<3, 3>(0, 0) = rotation;
    rotation_6d.block<3, 3>(3, 3) = rotation;
    const Eigen::Matrix<double, 6, 6> covariance_world =
        rotation_6d * covariance_body * rotation_6d.transpose();

    for (int row = 0; row < 6; ++row)
    {
      for (int col = 0; col < 6; ++col)
      {
        output_msg.twist.covariance[static_cast<std::size_t>(row * 6 + col)] =
            covariance_world(row, col);
      }
    }

    odom_pub_.publish(output_msg);

    ROS_INFO_THROTTLE(
        5.0,
        "odom_velocity_to_world: body velocity=(%.3f, %.3f, %.3f), "
        "world velocity=(%.3f, %.3f, %.3f)",
        linear_body.x(), linear_body.y(), linear_body.z(),
        linear_world.x(), linear_world.y(), linear_world.z());
  }
};

}  // namespace mission_auto_avoid

int main(int argc, char **argv)
{
  ros::init(argc, argv, "odom_velocity_to_world");
  mission_auto_avoid::OdomVelocityToWorld node;
  ros::spin();
  return 0;
}
