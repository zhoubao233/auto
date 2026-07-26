#include <iostream>
#include <vector>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <nav_msgs/Odometry.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <livox_ros_driver2/CustomMsg.h>

#include <message_filters/subscriber.h>
#include <message_filters/sync_policies/approximate_time.h>
#include <message_filters/sync_policies/exact_time.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>

#define BUFFERCAPACITY 40000

pcl::PointCloud<pcl::PointXYZL>::Ptr PCbuffer; 
ros::Publisher cloud_pub;
ros::Publisher test_pub;
ros::Publisher test2_pub;
// 定义同步策略，设置时间窗口为0.1秒
using namespace message_filters;
typedef sync_policies::ApproximateTime<sensor_msgs::PointCloud2, nav_msgs::Odometry> MySyncPolicy;
// typedef sync_policies::ExactTime<livox_ros_driver2::CustomMsg, nav_msgs::Odometry> MySyncPolicy;

// 将livox自定义点映射到 PCL 点
// 将原来的函数改为：
void convertToPCL(const sensor_msgs::PointCloud2::ConstPtr &msg, 
                  pcl::PointCloud<pcl::PointXYZL>::Ptr &pcl_cloud)
{
    pcl::PointCloud<pcl::PointXYZL> temp_cloud;
    pcl::fromROSMsg(*msg, temp_cloud);
    *pcl_cloud += temp_cloud;
    
    pcl_cloud->width = pcl_cloud->points.size();
    pcl_cloud->height = 1;
    pcl_cloud->is_dense = true;
}

// void AddBuffer(const pcl::PointCloud<pcl::PointXYZL>::Ptr& pc){
//     *PCbuffer+=*pc;
//     if(PCbuffer->size()>BUFFERCAPACITY){
//         PCbuffer->points.erase(PCbuffer->points.begin(),PCbuffer->points.begin()+pc->size());
//     }

// }

void AddBuffer(const pcl::PointCloud<pcl::PointXYZL>::Ptr& pc) {
    // 新增点加入缓冲区
    *PCbuffer += *pc;
    // 计算需要删除的老点数量
    if (PCbuffer->size() > BUFFERCAPACITY) {
        size_t n_to_remove = PCbuffer->size() - BUFFERCAPACITY;
        PCbuffer->points.erase(
            PCbuffer->points.begin(),
            PCbuffer->points.begin() + n_to_remove
        );
        PCbuffer->width = PCbuffer->points.size();
        PCbuffer->height = 1;
        PCbuffer->is_dense = true;
    }

    // sensor_msgs::PointCloud2 test_msg;
    // pcl::toROSMsg(*PCbuffer, test_msg);
    // // test_msg.header.stamp = pc->header.stamp;
    // test_msg.header.frame_id = "world"; // 或者"odom"
    // test2_pub.publish(test_msg);
}

// void callback(const livox_ros_driver2::CustomMsg::ConstPtr &pc,const nav_msgs::OdometryConstPtr& odom){
    
//     // 创建一个 PCL 点云指针
//     pcl::PointCloud<pcl::PointXYZL>::Ptr latestCloud(new pcl::PointCloud<pcl::PointXYZL>());
//     pcl::PointCloud<pcl::PointXYZL>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZL>());

//     // 转换到 PCL 点云类型
//     convertToPCL(pc, latestCloud);

//     // 将ROS点云消息转换为PCL点云
//     // pcl::fromROSMsg(*msg, *latestCloud);

//     std::cout << "latest cloud size: " << latestCloud->points.size() << std::endl;

//     AddBuffer(latestCloud);

//     std::cout << "buffer size: " << PCbuffer->points.size() << std::endl;

//     // 创建体素网格滤波器
//     pcl::VoxelGrid<pcl::PointXYZL> sor;
//     sor.setInputCloud(PCbuffer);
    
//     // 设置体素的大小
//     float voxel_size = 0.1f;  // 体素的大小
//     sor.setLeafSize(voxel_size, voxel_size, voxel_size);

//     // 执行降采样
//     sor.filter(*cloud_filtered);

//     Eigen::Quaterniond pose_q_ = Eigen::Quaterniond(
//         odom->pose.pose.orientation.w,
//         odom->pose.pose.orientation.x,
//         odom->pose.pose.orientation.y,
//         odom->pose.pose.orientation.z
//     );
//     Eigen::Vector3d pose_t_ = Eigen::Vector3d(
//         odom->pose.pose.position.x,
//         odom->pose.pose.position.y,
//         odom->pose.pose.position.z
//     );

//     // 构造 Eigen 4x4 变换矩阵（机体->世界）
//     Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();

//     transform.block<3,3>(0,0) = pose_q_.toRotationMatrix();
//     transform.block<3,1>(0,3) = pose_t_;
//     Eigen::Matrix4f transform_f = transform.cast<float>();

//     // 点云变换
//     pcl::PointCloud<pcl::PointXYZL> cloud_out;
//     pcl::transformPointCloud(*cloud_filtered, cloud_out, transform_f);

//     // 发布
//     sensor_msgs::PointCloud2 cloud_msg;
//     pcl::toROSMsg(cloud_out, cloud_msg);
//     cloud_msg.header.stamp = pc->header.stamp;
//     cloud_msg.header.frame_id = "world"; // 或者"odom"，按你坐标系定义
//     cloud_pub.publish(cloud_msg);
    
// }

void pcfilter(pcl::PointCloud<pcl::PointXYZL>::Ptr& latestCloud , pcl::PointCloud<pcl::PointXYZL>::Ptr& filtered_cloud){
    // 创建条件
    pcl::ConditionOr<pcl::PointXYZL> condition;
    condition.addComparison(pcl::FieldComparison<pcl::PointXYZL>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZL>("x", pcl::ComparisonOps::GT, 1.7))); // X大于1
    condition.addComparison(pcl::FieldComparison<pcl::PointXYZL>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZL>("x", pcl::ComparisonOps::LT, -1.7))); // X小于-1
    condition.addComparison(pcl::FieldComparison<pcl::PointXYZL>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZL>("y", pcl::ComparisonOps::GT, 1.7))); // Y大于1
    condition.addComparison(pcl::FieldComparison<pcl::PointXYZL>::ConstPtr(
        new pcl::FieldComparison<pcl::PointXYZL>("y", pcl::ComparisonOps::LT, -1.7))); // Y小于-1

    // 应用条件过滤器
    pcl::ConditionalRemoval<pcl::PointXYZL> filter;
    filter.setCondition(boost::shared_ptr<pcl::ConditionOr<pcl::PointXYZL>>(new pcl::ConditionOr<pcl::PointXYZL>(condition)));
    filter.setInputCloud(latestCloud);
    filter.setKeepOrganized(true);  // 保持输出点云的结构
    filter.filter(*filtered_cloud);  // 过滤后的点云

    // pcl::PassThrough<pcl::PointXYZL> pass;

    // pass.setInputCloud(latestCloud);
    // pass.setFilterFieldName("x");
    // pass.setFilterLimits(-0.1, 0.1);
    // pass.filter(*latestCloud);

    // pass.setInputCloud(latestCloud);
    // pass.setFilterFieldName("y");
    // pass.setFilterLimits(-this->localLidarRange_y, this->localLidarRange_y);
    // pass.filter(*latestCloud);

    // pass.setInputCloud(latestCloud);
    // pass.setFilterFieldName("z");
    // pass.setFilterLimits(this->localLidarRange_zdown, this->localLidarRange_zup);
    // pass.filter(*filtered_cloud);
}

void callback(const sensor_msgs::PointCloud2::ConstPtr &pc, const nav_msgs::OdometryConstPtr& odom) {
    pcl::PointCloud<pcl::PointXYZL>::Ptr latestCloud(new pcl::PointCloud<pcl::PointXYZL>());
    pcl::PointCloud<pcl::PointXYZL>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZL>);
    convertToPCL(pc, latestCloud);

    // sensor_msgs::PointCloud2 test_msg;
    // pcl::toROSMsg(*latestCloud, test_msg);
    // test_msg.header.stamp = pc->header.stamp;
    // test_msg.header.frame_id = "world"; // 或者"odom"
    // test_pub.publish(test_msg);

    pcfilter(latestCloud,filtered_cloud);

    ROS_INFO("latest cloud size: %zu", filtered_cloud->points.size());

    AddBuffer(filtered_cloud);
    ROS_INFO("buffer size: %zu", PCbuffer->points.size());

    // pcl::VoxelGrid<pcl::PointXYZL> sor;
    // sor.setInputCloud(PCbuffer);
    // float voxel_size = 0.05f;  // 体素的大小
    // sor.setLeafSize(voxel_size, voxel_size, voxel_size);
    // pcl::PointCloud<pcl::PointXYZL> cloud_filtered;
    // sor.filter(cloud_filtered);

    // ROS_INFO("downsample cloud size: %zu", cloud_filtered.points.size());

    Eigen::Quaterniond pose_q_(odom->pose.pose.orientation.w,
                                odom->pose.pose.orientation.x,
                                odom->pose.pose.orientation.y,
                                odom->pose.pose.orientation.z);
    Eigen::Vector3d pose_t_(odom->pose.pose.position.x,
                             odom->pose.pose.position.y,
                             odom->pose.pose.position.z);
    Eigen::Matrix4d transform = Eigen::Matrix4d::Identity();
    transform.block<3,3>(0,0) = pose_q_.toRotationMatrix();
    transform.block<3,1>(0,3) = pose_t_;
    Eigen::Matrix4f transform_f = transform.cast<float>();

    pcl::PointCloud<pcl::PointXYZL> cloud_out;
    pcl::transformPointCloud(*PCbuffer, cloud_out, transform_f);

    sensor_msgs::PointCloud2 cloud_msg;
    pcl::toROSMsg(cloud_out, cloud_msg);
    cloud_msg.header.stamp = pc->header.stamp;
    cloud_msg.header.frame_id = "world"; // 或者"odom"
    cloud_pub.publish(cloud_msg);

    // std::cout<<"end"<<std::endl;
}

// void testcallback(const livox_ros_driver2::CustomMsg::ConstPtr &pc){
//     std::cout<<pc->header.stamp<<std::endl;
// }

// 主程序示例
int main(int argc, char** argv)
{
    ros::init(argc, argv, "livox_pointcloud_processor");
    ros::NodeHandle nh;

    std::string odometry_topic_name , pointcloud_topic_name;
    nh.param<std::string>("odometry_topic", odometry_topic_name, "/mavros/local_position/odom" );
    nh.param<std::string>("pointcloud_topic", pointcloud_topic_name, "/livox/lidar" );

    PCbuffer.reset(new pcl::PointCloud<pcl::PointXYZL>());

    // cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox2pcl_cloud", 1);
    cloud_pub = nh.advertise<sensor_msgs::PointCloud2>("/world_cloud", 1);
    // test_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox2pcl_cloud_test", 1);
    // test2_pub = nh.advertise<sensor_msgs::PointCloud2>("/livox2pcl_cloud_test2", 1);

    // ros::Subscriber sub=nh.subscribe("/livox/lidar",1,testcallback);

    // 创建消息订阅者
    message_filters::Subscriber<sensor_msgs::PointCloud2> pc_sub(nh, pointcloud_topic_name , 50);
    message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, odometry_topic_name , 50);
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/mavros/local_position/odom" , 5);
    // message_filters::Subscriber<nav_msgs::Odometry> odom_sub(nh, "/Odometry" , 5);
    // 创建同步器
    // Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
    // sync.registerCallback(boost::bind(&callback, _1, _2));
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), pc_sub, odom_sub);
    sync.setInterMessageLowerBound(ros::Duration(0, 200000000));  // 设置时间窗口, 0.2秒
    sync.registerCallback(boost::bind(&callback, _1, _2));
    std::cout << "wait for callback " << std::endl;

    ros::spin();  // 循环等待回调
    std::cout << "end" << std::endl;

    return 0;

}