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

#include <algorithm>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <string>
#include <unordered_map>

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

    struct VoxelKey
    {
        int x;
        int y;
        int z;

        bool operator==(const VoxelKey &other) const
        {
            return x == other.x && y == other.y && z == other.z;
        }
    };

    struct VoxelKeyHash
    {
        std::size_t operator()(const VoxelKey &key) const
        {
            std::size_t seed = std::hash<int>()(key.x);
            seed ^= std::hash<int>()(key.y) + 0x9e3779b9 +
                    (seed << 6) + (seed >> 2);
            seed ^= std::hash<int>()(key.z) + 0x9e3779b9 +
                    (seed << 6) + (seed >> 2);
            return seed;
        }
    };

    struct FrameVoxelStats
    {
        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        int point_count = 0;
    };

    struct PersistentVoxel
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
        int consecutive_frames = 0;
        std::uint64_t last_frame_id = 0;
        ros::Time last_seen;
        bool confirmed = false;
    };

    typedef std::unordered_map<VoxelKey, FrameVoxelStats, VoxelKeyHash>
        FrameVoxelMap;
    typedef std::unordered_map<VoxelKey, PersistentVoxel, VoxelKeyHash>
        PersistentVoxelMap;

    double cloud_cache_duration_;
    double voxel_size_;
    int min_points_per_voxel_;
    int min_consecutive_frames_;
    std::uint64_t frame_id_;
    ros::Time last_cloud_stamp_;
    PersistentVoxelMap persistent_voxels_;
    pcl::PointCloud<pcl::PointXYZ>::Ptr filtered_world_cloud_;

public:
    LivoxToWorldTransformer()
        : nh_("~"),
          livox_sub_(nullptr),
          odom_sub_(nullptr),
          sync_(nullptr),
          sensor_to_body_(Eigen::Matrix3d::Identity()),
          sensor_translation_(Eigen::Vector3d::Zero()),
          frame_id_(0)
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

        nh_.param<double>("cloud_cache_duration", cloud_cache_duration_, 0.8);
        nh_.param<double>("voxel_size", voxel_size_, 0.25);
        nh_.param<int>("min_points_per_voxel", min_points_per_voxel_, 2);
        nh_.param<int>("min_consecutive_frames", min_consecutive_frames_, 3);

        cloud_cache_duration_ = std::max(0.1, cloud_cache_duration_);
        voxel_size_ = std::max(0.05, voxel_size_);
        min_points_per_voxel_ = std::max(1, min_points_per_voxel_);
        min_consecutive_frames_ = std::max(1, min_consecutive_frames_);

        constexpr double kDegToRad = 3.14159265358979323846 / 180.0;
        const Eigen::AngleAxisd roll(roll_deg * kDegToRad,
                                     Eigen::Vector3d::UnitX());
        const Eigen::AngleAxisd pitch(pitch_deg * kDegToRad,
                                      Eigen::Vector3d::UnitY());
        const Eigen::AngleAxisd yaw(yaw_deg * kDegToRad,
                                    Eigen::Vector3d::UnitZ());

        sensor_to_body_ = (yaw * pitch * roll).toRotationMatrix();
        sensor_translation_ = Eigen::Vector3d(lidar_x, lidar_y, lidar_z);

        filtered_world_cloud_.reset(new pcl::PointCloud<pcl::PointXYZ>());
        filtered_world_cloud_->reserve(50000);

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
        ROS_INFO("  Cloud filter: cache=%.2fs voxel=%.2fm min_points=%d min_frames=%d",
                 cloud_cache_duration_, voxel_size_, min_points_per_voxel_,
                 min_consecutive_frames_);
    }

    ~LivoxToWorldTransformer()
    {
        delete livox_sub_;
        delete odom_sub_;
        delete sync_;
    }

    VoxelKey pointToVoxel(const pcl::PointXYZ &point) const
    {
        VoxelKey key;
        key.x = static_cast<int>(std::floor(point.x / voxel_size_));
        key.y = static_cast<int>(std::floor(point.y / voxel_size_));
        key.z = static_cast<int>(std::floor(point.z / voxel_size_));
        return key;
    }

    void updateFilteredWorldCloud(
        const pcl::PointCloud<pcl::PointXYZ>::Ptr &current_world,
        const ros::Time &cloud_stamp)
    {
        ros::Time stamp = cloud_stamp;
        if (stamp.isZero())
        {
            stamp = ros::Time::now();
        }

        // bag 回放或时钟回退时清空状态，避免把上一段时间的障碍带入新地图。
        if (!last_cloud_stamp_.isZero() && stamp < last_cloud_stamp_)
        {
            persistent_voxels_.clear();
            frame_id_ = 0;
        }
        last_cloud_stamp_ = stamp;
        ++frame_id_;

        // 每帧先做体素统计，同一体素内的多个激光点只保留一个质心。
        FrameVoxelMap frame_voxels;
        frame_voxels.reserve(current_world->size());
        for (const pcl::PointXYZ &point : current_world->points)
        {
            FrameVoxelStats &stats = frame_voxels[pointToVoxel(point)];
            stats.sum_x += point.x;
            stats.sum_y += point.y;
            stats.sum_z += point.z;
            ++stats.point_count;
        }

        std::size_t qualified_voxels = 0;
        for (const auto &entry : frame_voxels)
        {
            const FrameVoxelStats &stats = entry.second;
            if (stats.point_count < min_points_per_voxel_)
            {
                continue;
            }

            ++qualified_voxels;
            PersistentVoxel &state = persistent_voxels_[entry.first];
            if (state.last_frame_id + 1 == frame_id_)
            {
                ++state.consecutive_frames;
            }
            else
            {
                state.consecutive_frames = 1;
            }

            const double count = static_cast<double>(stats.point_count);
            state.x = stats.sum_x / count;
            state.y = stats.sum_y / count;
            state.z = stats.sum_z / count;
            state.last_frame_id = frame_id_;
            state.last_seen = stamp;
            if (state.consecutive_frames >= min_consecutive_frames_)
            {
                state.confirmed = true;
            }
        }

        // 只发布已连续确认的体素；确认后最多保留短时间，不再无条件累计 100000 个历史点。
        filtered_world_cloud_->clear();
        std::size_t confirmed_voxels = 0;
        for (PersistentVoxelMap::iterator it = persistent_voxels_.begin();
             it != persistent_voxels_.end();)
        {
            const double age = (stamp - it->second.last_seen).toSec();
            if (age > cloud_cache_duration_)
            {
                it = persistent_voxels_.erase(it);
                continue;
            }

            if (it->second.confirmed)
            {
                pcl::PointXYZ point;
                point.x = static_cast<float>(it->second.x);
                point.y = static_cast<float>(it->second.y);
                point.z = static_cast<float>(it->second.z);
                filtered_world_cloud_->push_back(point);
                ++confirmed_voxels;
            }
            ++it;
        }

        filtered_world_cloud_->width =
            static_cast<std::uint32_t>(filtered_world_cloud_->size());
        filtered_world_cloud_->height = 1;
        filtered_world_cloud_->is_dense = true;

        ROS_INFO_THROTTLE(
            1.0,
            "World cloud filter: current=%zu frame_voxels=%zu qualified=%zu confirmed=%zu",
            current_world->size(), frame_voxels.size(), qualified_voxels,
            confirmed_voxels);
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

        updateFilteredWorldCloud(current_world, livox_msg->header.stamp);

        sensor_msgs::PointCloud2 output_msg;
        pcl::toROSMsg(*filtered_world_cloud_, output_msg);
        output_msg.header.stamp = livox_msg->header.stamp;
        output_msg.header.frame_id = output_frame_;
        world_cloud_pub_.publish(output_msg);

        ROS_INFO_THROTTLE(
            1.0,
            "Published world cloud: current=%zu filtered=%zu frame=%s",
            current_world->size(), filtered_world_cloud_->size(),
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
