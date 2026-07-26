#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>
#include <livox_ros_driver2/CustomMsg.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/synchronizer.h>
#include <Eigen/Geometry>

#include <cmath>
#include <cstddef>
#include <string>

class LivoxToWorldTransformer
{
private:
    ros::NodeHandle nh_;
    ros::Publisher world_cloud_pub_;

    typedef message_filters::sync_policies::ApproximateTime<
        livox_ros_driver2::CustomMsg,
        nav_msgs::Odometry> SyncPolicy;

    message_filters::Subscriber<livox_ros_driver2::CustomMsg> *livox_sub_;
    message_filters::Subscriber<nav_msgs::Odometry> *odom_sub_;
    message_filters::Synchronizer<SyncPolicy> *sync_;

    std::string livox_topic_;
    std::string odom_topic_;
    std::string output_frame_;
    double time_sync_window_;

    bool self_filter_enabled_;
    double self_filter_x_;
    double self_filter_y_;
    double self_filter_z_;

    // Sensor frame -> base_link. Angles are degrees, translation is metres.
    Eigen::Matrix3d sensor_to_body_;
    Eigen::Vector3d sensor_translation_;

    static constexpr std::size_t MAX_BUFFER_SIZE = 100000;

    // This buffer is always stored in output/world coordinates.
    pcl::PointCloud<pcl::PointXYZ>::Ptr accumulated_world_cloud_;

public:
    LivoxToWorldTransformer()
        : nh_("~"),
          livox_sub_(nullptr),
          odom_sub_(nullptr),
          sync_(nullptr),
          sensor_to_body_(Eigen::Matrix3d::Identity()),
          sensor_translation_(Eigen::Vector3d::Zero())
    {
        nh_.param<std::string>("livox_topic", livox_topic_, "/livox/lidar");
        nh_.param<std::string>("odom_topic", odom_topic_,
                                "/mavros/local_position/odom");
        nh_.param<std::string>("output_frame", output_frame_, "world");
        nh_.param<double>("time_sync_window", time_sync_window_, 0.1);

        double lidar_x, lidar_y, lidar_z;
        double roll_deg, pitch_deg, yaw_deg;
        nh_.param<double>("lidar_x", lidar_x, 0.0);
        nh_.param<double>("lidar_y", lidar_y, 0.0);
        nh_.param<double>("lidar_z", lidar_z, 0.0);
        nh_.param<double>("lidar_roll_deg", roll_deg, 0.0);
        nh_.param<double>("lidar_pitch_deg", pitch_deg, 0.0);
        nh_.param<double>("lidar_yaw_deg", yaw_deg, 0.0);

        nh_.param<bool>("self_filter_enabled", self_filter_enabled_, true);
        nh_.param<double>("self_filter_x", self_filter_x_, 0.7);
        nh_.param<double>("self_filter_y", self_filter_y_, 0.7);
        nh_.param<double>("self_filter_z", self_filter_z_, 0.5);

        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        const Eigen::AngleAxisd roll(roll_deg * kDegToRad,
                                     Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch(pitch_deg * kDegToRad,
                                      Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw(yaw_deg * kDegToRad,
                                    Eigen::Vector3d::UnitZ());

        sensor_to_body_ = (yaw * pitch * roll).toRotationMatrix();
        sensor_translation_ = Eigen::Vector3d(lidar_x, lidar_y, lidar_z);

        accumulated_world_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        accumulated_world_cloud_->reserve(MAX_BUFFER_SIZE);

        world_cloud_pub_ =
            nh_.advertise<sensor_msgs::PointCloud2>("/world_cloud", 10);

        livox_sub_ =
            new message_filters::Subscriber<livox_ros_driver2::CustomMsg>(
                nh_, livox_topic_, 10);
        odom_sub_ =
            new message_filters::Subscriber<nav_msgs::Odometry>(
                nh_, odom_topic_, 10);

        // ApproximateTime queue size, not seconds.
        sync_ = new message_filters::Synchronizer<SyncPolicy>(
            SyncPolicy(100), *livox_sub_, *odom_sub_);
        sync_->registerCallback(
            boost::bind(&LivoxToWorldTransformer::callback, this, _1, _2));

        ROS_INFO("LivoxToWorldTransformer initialized");
        ROS_INFO("  Livox topic: %s", livox_topic_.c_str());
        ROS_INFO("  Odometry topic: %s", odom_topic_.c_str());
        ROS_INFO("  Output frame: %s", output_frame_.c_str());
        ROS_INFO("  Sensor translation: (%.3f, %.3f, %.3f)",
                 lidar_x, lidar_y, lidar_z);
        ROS_INFO("  Sensor RPY(deg): (%.2f, %.2f, %.2f)",
                 roll_deg, pitch_deg, yaw_deg);
        ROS_INFO("  Self filter: %s box=(%.2f, %.2f, %.2f)",
                 self_filter_enabled_ ? "enabled" : "disabled",
                 self_filter_x_, self_filter_y_, self_filter_z_);
    }

    ~LivoxToWorldTransformer()
    {
        delete livox_sub_;
        delete odom_sub_;
        delete sync_;
    }

    void addToWorldBuffer(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &world_cloud)
    {
        *accumulated_world_cloud_ += *world_cloud;

        if (accumulated_world_cloud_->size() > MAX_BUFFER_SIZE)
        {
            const std::size_t remove_count =
                accumulated_world_cloud_->size() - MAX_BUFFER_SIZE;
            accumulated_world_cloud_->points.erase(
                accumulated_world_cloud_->points.begin(),
                accumulated_world_cloud_->points.begin() + remove_count);
        }

        accumulated_world_cloud_->width =
            static_cast<std::uint32_t>(accumulated_world_cloud_->size());
        accumulated_world_cloud_->height = 1;
        accumulated_world_cloud_->is_dense = true;
    }

    void callback(const livox_ros_driver2::CustomMsg::ConstPtr &livox_msg,
                  const nav_msgs::Odometry::ConstPtr &odom_msg)
    {
        Eigen::Quaterniond q(
            odom_msg->pose.pose.orientation.w,
            odom_msg->pose.pose.orientation.x,
            odom_msg->pose.pose.orientation.y,
            odom_msg->pose.pose.orientation.z);
        q.normalize();

        const Eigen::Matrix3d world_from_body = q.toRotationMatrix();
        const Eigen::Vector3d body_in_world(
            odom_msg->pose.pose.position.x,
            odom_msg->pose.pose.position.y,
            odom_msg->pose.pose.position.z);

        pcl::PointCloud<pcl::PointXYZ>::Ptr current_world(
            new pcl::PointCloud<pcl::PointXYZ>());
        current_world->reserve(livox_msg->point_num);

        for (const auto &point : livox_msg->points)
        {
            if (std::isnan(point.x) || std::isnan(point.y) ||
                std::isnan(point.z))
            {
                continue;
            }

            const Eigen::Vector3d sensor_point(point.x, point.y, point.z);
            const Eigen::Vector3d body_point =
                sensor_to_body_ * sensor_point + sensor_translation_;

            // Remove points belonging to the aircraft body before mapping.
            // This is a configurable body box, not a guessed sensor rotation.
            if (self_filter_enabled_ &&
                std::abs(body_point.x()) <= self_filter_x_ &&
                std::abs(body_point.y()) <= self_filter_y_ &&
                std::abs(body_point.z()) <= self_filter_z_)
            {
                continue;
            }

            const Eigen::Vector3d world_point =
                world_from_body * body_point + body_in_world;

            pcl::PointXYZ pcl_point;
            pcl_point.x = static_cast<float>(world_point.x());
            pcl_point.y = static_cast<float>(world_point.y());
            pcl_point.z = static_cast<float>(world_point.z());
            current_world->push_back(pcl_point);
        }

        current_world->width =
            static_cast<std::uint32_t>(current_world->size());
        current_world->height = 1;
        current_world->is_dense = true;

        // Important: transform this scan with its own synchronized pose
        // before adding it to the world buffer.
        addToWorldBuffer(current_world);

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*accumulated_world_cloud_, output_msg);
        output_msg.header.stamp = livox_msg->header.stamp;
        output_msg.header.frame_id = output_frame_;
        world_cloud_pub_.publish(output_msg);

        ROS_INFO_THROTTLE(
            1.0,
            "Published world cloud: current=%zu buffered=%zu frame=%s",
            current_world->size(), accumulated_world_cloud_->size(),
            output_frame_.c_str());
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "livox_to_world");
    LivoxToWorldTransformer transformer;
    ros::spin();
    return 0;
}
