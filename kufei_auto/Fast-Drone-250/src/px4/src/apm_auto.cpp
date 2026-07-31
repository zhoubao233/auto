/*
无人机控制
0. 飞行器解锁
1. 定点飞行
在外部控制模式下进行定点飞行
2. local_position 指点飞行
以ENU（东北天）坐标系为坐标进行定点飞行
3. 速度控制飞行(多模式)
    3.1 控制Vx、Vy、偏航角Yaw  Z不变定高飞行    (推荐！！！)
    3.2 控制Vx、Vy、Vz、偏航角Yaw            （比较危险不推荐！！！）
    3.3
*/
/* Includes-------------------------------------------------------*/
#include "ros/ros.h"
#include "mavros_msgs/PositionTarget.h"
#include "geometry_msgs/PoseStamped.h"
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/State.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/RCIn.h>
#include <mavros_msgs/OverrideRCIn.h>
#include <sensor_msgs/BatteryState.h>
#include <sensor_msgs/MagneticField.h>
#include <sensor_msgs/NavSatFix.h>
#include <sensor_msgs/Imu.h>
#include <tf/transform_datatypes.h>
#include <array>
#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <thread>
#include "cmath"
#include "algorithm"
#include <quadrotor_msgs/PositionCommand.h>
#include <std_msgs/Bool.h>
#include <std_msgs/UInt8.h>
#include <tf/tf.h>

#define PI 3.14159265358979323846

#define STRAIGHT_CTRL      1
#define EGO_CTRL           2
#define GATE_CTRL          3

uint8_t ctrl_mode_flag = 2;    // 代表控制模式的标志

/* Private typedef -----------------------------------------------*/
//飞行围栏（安全区域）
float safe_area_xmax;
float safe_area_ymax;
float safe_area_hightmax;
//飞行速度限制
float vx_max;
float vy_max;
float vz_max;

static ros::Publisher local_pose_vel_pub;   //位置和速度消息发布
static ros::Publisher mode_select_flag_pub; //当前模式标志发布
static ros::ServiceClient set_mode_client;  //飞行模式设置
static ros::Subscriber state_sub;           //状态订阅
static ros::Subscriber local_position_pose_sub; //位置数据订阅 local position
static ros::Subscriber control_data_sub;    //地面站期望指令订阅

static ros::Subscriber straight_control_data_sub; //直接控制模式订阅消息
static ros::Subscriber ego_control_data_sub;      //ego控制模式订阅消息
static ros::Subscriber flag_sub;                  //控制模式订阅消息
static ros::Subscriber avoidance_enable_sub;      //避障接管使能
volatile bool received_local_pose = false;        //表明是否接收到无人机位姿初始数据

static ros::Subscriber rcin_sub;           //遥控器数据输入订阅
static ros::Subscriber rc_override_sub;    //遥控器覆盖输入订阅
static ros::ServiceClient arming_client;   //解锁订阅

static mavros_msgs::SetMode offb_set_mode; //要发布的AUTO/LOITER模式
static mavros_msgs::CommandBool arm_cmd;   //要发布的解锁模式
mavros_msgs::PositionTarget control_data;  //从地面站接收到的控制指令
mavros_msgs::PositionTarget command_position_pose;          //位置信息
mavros_msgs::PositionTarget command_position_pose_and_vel;  //x y 轴速度和高度信息

static uint8_t fightarea_check_flag;       //飞行区域安全检查 0：没有异常 1：有异常
static uint8_t fightarea_flag_change_allow; //无人机进行飞行区域检查标志位改变的权利 1：允许 0：不允许
static uint8_t mode_check_flag;            //模式安全检查 0：没有异常 1：有异常
static uint8_t mode_select_flag;           //模式切换选择 0:LOITER 1:AUTO/GUIDED
static uint8_t arm_select_flag;            //解锁选择 0：不解锁 1：解锁

ros::Timer timer_safe_check;        //飞行器安全检查定时器
ros::Timer timer_mode_change;       //飞行器模式切换检查定时器

static mavros_msgs::State current_state; //状态数据
static mavros_msgs::RCIn raw_rc_data;    //飞控回传的原始遥控器数据
static mavros_msgs::RCIn rc_data;        //叠加 override 后的有效遥控器数据
static std::array<uint16_t, 18> rc_override_channels;
static geometry_msgs::PoseStamped local_pose_data;       //无人机位置数据
static mavros_msgs::PositionTarget local_pose_data_last; //无人机模式切换前的位置数据
static mavros_msgs::PositionTarget local_pose_data_boforeOutarea; //无人机飞出安全区域前的位置数据

// 互斥锁
std::mutex control_data_mutex;

// LOITER 下彻底停发，GUIDED 下按避障逻辑接管，AUTO 下不发点
volatile bool send_setpoint_enable = false;

static std::atomic<bool> avoidance_enabled(false);
static bool waiting_for_avoidance_traj = false;
static bool have_control_planner_traj_id = false;
static bool avoidance_activation_has_control_traj_id = false;
static uint32_t control_planner_traj_id = 0;
static uint32_t avoidance_activation_control_traj_id = 0;
static double avoidance_takeover_max_setpoint_jump = 1.0;
static double control_cmd_timeout = 0.2;
static double acceleration_ff_scale = 1.0;
static bool received_ego_control_cmd = false;
static ros::Time last_ego_control_cmd_time;
static std::string guided_mode_name = "GUIDED";
static std::string auto_mode_name = "AUTO";
static std::string loiter_mode_name = "LOITER";

/* function define -----------------------------------------------*/
// **************** 功能函数 ****************
void publish_mode_select_flag()
{
    std_msgs::UInt8 msg;
    msg.data = mode_select_flag;
    mode_select_flag_pub.publish(msg);
}

double getYawFromPose(const geometry_msgs::PoseStamped& pose_msg)
{
    geometry_msgs::Quaternion quat = pose_msg.pose.orientation;

    tf::Quaternion tf_quat;
    tf::quaternionMsgToTF(quat, tf_quat);

    double roll, pitch, yaw;
    tf::Matrix3x3(tf_quat).getRPY(roll, pitch, yaw);
    return yaw;
}

void apply_rc_override_to_effective_input()
{
    rc_data = raw_rc_data;
    if (rc_data.channels.size() < rc_override_channels.size())
        rc_data.channels.resize(rc_override_channels.size(), 0);

    for (size_t i = 0; i < rc_override_channels.size(); ++i)
    {
        const uint16_t override_value = rc_override_channels[i];
        if (override_value != mavros_msgs::OverrideRCIn::CHAN_RELEASE &&
            override_value != mavros_msgs::OverrideRCIn::CHAN_NOCHANGE)
        {
            rc_data.channels[i] = override_value;
        }
    }
}

void holdCurrentPoseLocked()/* 让飞机飞向最近一次缓存的当前位置，并在那个点悬停 */
{
    if (received_local_pose)
    {
        control_data.position.x = local_pose_data.pose.position.x;
        control_data.position.y = local_pose_data.pose.position.y;
        control_data.position.z = local_pose_data.pose.position.z;
        control_data.yaw = getYawFromPose(local_pose_data);
    }

    control_data.velocity.x = 0.0;
    control_data.velocity.y = 0.0;
    control_data.velocity.z = 0.0;
    control_data.acceleration_or_force.x = 0.0;
    control_data.acceleration_or_force.y = 0.0;
    control_data.acceleration_or_force.z = 0.0;
    control_data.type_mask = command_position_pose.type_mask;
}

void resetAvoidanceTakeoverStateLocked()
{
    waiting_for_avoidance_traj = false;
    avoidance_activation_has_control_traj_id = false;
    received_ego_control_cmd = false;
    last_ego_control_cmd_time = ros::Time(0);
}

void armIfRequested()
{
    if (arm_select_flag == 1 && !current_state.armed)
    {
        if (arming_client.call(arm_cmd) && arm_cmd.response.success)
            ROS_INFO("Vehicle armed");
    }
}

bool plannerControlEnabled()
{
    return mode_select_flag == 1 && avoidance_enabled.load();
}

//******************ROS sub 回调函数**************
//状态回调函数
void state_callback(const mavros_msgs::State::ConstPtr& msg)
{
    current_state = *msg;
}

//遥控器输入回调函数
void rc_input_callback(const mavros_msgs::RCIn::ConstPtr& msg)
{
    raw_rc_data = *msg;
    apply_rc_override_to_effective_input();
}

void rc_override_callback(const mavros_msgs::OverrideRCIn::ConstPtr& msg)
{
    const size_t count = std::min(rc_override_channels.size(), msg->channels.size());
    for (size_t i = 0; i < count; ++i)
    {
        if (msg->channels[i] == mavros_msgs::OverrideRCIn::CHAN_NOCHANGE)
            continue;

        rc_override_channels[i] = msg->channels[i];
    }

    apply_rc_override_to_effective_input();
}

//位置信息回调函数
void localpositionposeCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    /*
        ENU坐标系 三轴朝向：前左上：东北天
    */
    received_local_pose = true;
    local_pose_data = *msg;
}

void avoidanceEnableCallback(const std_msgs::Bool::ConstPtr& msg)
{
    const bool was_enabled = avoidance_enabled.exchange(msg->data);

    std::lock_guard<std::mutex> lock(control_data_mutex);

    if (msg->data && !was_enabled)
    {
        waiting_for_avoidance_traj = true;
        avoidance_activation_has_control_traj_id = have_control_planner_traj_id;
        avoidance_activation_control_traj_id = control_planner_traj_id;
        received_ego_control_cmd = false;
        last_ego_control_cmd_time = ros::Time(0);
        holdCurrentPoseLocked();// 当前实现：锁定最近一次缓存位置
        ROS_WARN("Avoidance enabled. Holding current pose until a fresh planner trajectory is received.");
    }
    else if (!msg->data && was_enabled)
    {
        resetAvoidanceTakeoverStateLocked();// 清除“等待新避障轨迹”的状态
        holdCurrentPoseLocked();// 当前实现：锁定最近一次缓存位置
        ROS_INFO("Avoidance disabled. Holding current pose until FCU leaves GUIDED.");
    }
}

//****************** px4 控制函数 ******************
/*
guided模式下的定点飞行
输入：需要发布的位置信息，发布者
输出：0（发布失败）1（发布成功）
*/
int command_position_control(double x, double y, double hight, float yaw)
{
    //首先判断设置的飞行参数有没有超出地理围栏
    if (hight > safe_area_hightmax || fabs(x) > safe_area_xmax || fabs(y) > safe_area_ymax)
    {
        ROS_INFO("give position out of the safe_area");
        return 0;
    }
    if (mode_check_flag != 0)
    {
        ROS_INFO("mode is not AUTO/GUIDED control mode");
        return 0;
    }
    if (fightarea_check_flag != 0)
    {
        ROS_INFO("uav is out of safe_area");
        local_pose_vel_pub.publish(local_pose_data_boforeOutarea);
        return 0;
    }

    /*实机飞行用*/
    command_position_pose.position.x = x;
    command_position_pose.position.y = y;
    command_position_pose.position.z = hight;
    command_position_pose.yaw = yaw;

    local_pose_vel_pub.publish(command_position_pose);
    return 1;
}

/*
EGO 位置 + 速度 + 加速度前馈控制。
同一条 PositionTarget 同时启用 XYZ 位置、XYZ 速度、XYZ 加速度和 yaw；
只忽略 yaw_rate。
*/
int command_position_velocity_acceleration_control(
    double x, double y, double height,
    double vx, double vy, double vz,
    double ax, double ay, double az,
    float yaw)
{
    if (height > safe_area_hightmax || fabs(x) > safe_area_xmax || fabs(y) > safe_area_ymax)
    {
        ROS_INFO("give position out of the safe_area");
        return 0;
    }
    if (mode_check_flag != 0)
    {
        ROS_INFO("mode is not AUTO/GUIDED control mode");
        return 0;
    }
    if (fightarea_check_flag != 0)
    {
        ROS_INFO("uav is out of safe_area");
        local_pose_vel_pub.publish(local_pose_data_boforeOutarea);
        return 0;
    }

    mavros_msgs::PositionTarget position_velocity_cmd;
    position_velocity_cmd.header.stamp = ros::Time::now();
    position_velocity_cmd.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    position_velocity_cmd.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;

    position_velocity_cmd.position.x = x;
    position_velocity_cmd.position.y = y;
    position_velocity_cmd.position.z = height;
    position_velocity_cmd.velocity.x = vx;
    position_velocity_cmd.velocity.y = vy;
    position_velocity_cmd.velocity.z = vz;
    position_velocity_cmd.acceleration_or_force.x = ax;
    position_velocity_cmd.acceleration_or_force.y = ay;
    position_velocity_cmd.acceleration_or_force.z = az;
    position_velocity_cmd.yaw = yaw;

    local_pose_vel_pub.publish(position_velocity_cmd);
    return 1;
}

/*
无人机x、y轴速度控制， Z轴高度控制 偏航角控制
输入：需要发布的位置信息，发布者
输出：0（发布失败）1（发布成功）
*/
int command_VelHightYaw_control(double vx, double vy, double hight, float yaw)
{
    ROS_INFO("vx:%f vy:%f hight:%f yaw:%f", vx, vy, hight, yaw);
    //首先判断设置的飞行参数有没有超出地理围栏
    if (hight > safe_area_hightmax)
    {
        ROS_INFO("give hight is too high");
        return 0;
    }
    if (mode_check_flag != 0)
    {
        ROS_INFO("mode is not AUTO/GUIDED control mode");
        return 0;
    }
    if (fightarea_check_flag != 0)
    {
        ROS_INFO("uav is out of safe_area");
        local_pose_vel_pub.publish(local_pose_data_boforeOutarea);
        return 0;
    }
    command_position_pose_and_vel.velocity.x = vx;
    command_position_pose_and_vel.velocity.y = vy;
    command_position_pose_and_vel.position.z = hight;
    command_position_pose_and_vel.yaw = yaw;

    local_pose_vel_pub.publish(command_position_pose_and_vel);
    return 1;
}

//**********************定时器回调函数******************
/*
无人机飞行安全检查函数
LOITER 下不发点；
AUTO/GUIDED 档下，AUTO 由飞控自身 mission 飞行，GUIDED 时才接管 planner setpoint。
*/
void safe_check_Callback(const ros::TimerEvent& event)
{
    switch (mode_select_flag)
    {
        case 0:
            send_setpoint_enable = false;
            /* 避免锁定模式  只有在遥控器通道的时候变化一次*/
            // if (current_state.mode != loiter_mode_name)
            // {
            //     mode_check_flag = 1;
            //     offb_set_mode.request.custom_mode = loiter_mode_name;
            //     if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
            //         ROS_INFO_THROTTLE(2, "Attempting to switch FCU to %s.", loiter_mode_name.c_str());
            // }
            // else
            // {
                mode_check_flag = 0;
            // }
            break;

        case 1:
            if (current_state.mode == guided_mode_name)
            {
                mode_check_flag = 0;
                send_setpoint_enable = true;
                if (!avoidance_enabled.load())
                {
                    std::lock_guard<std::mutex> lock(control_data_mutex);
                    holdCurrentPoseLocked();
                }
                armIfRequested();
            }
            else if (current_state.mode == auto_mode_name)
            {
                mode_check_flag = 0;
                send_setpoint_enable = false;
                if (!avoidance_enabled.load())
                {
                    std::lock_guard<std::mutex> lock(control_data_mutex);
                    resetAvoidanceTakeoverStateLocked();
                }
                armIfRequested();
            }
            else
            {
                mode_check_flag = 1;
                send_setpoint_enable = false;
                ROS_INFO_THROTTLE(
                    2,
                    "FCU mode %s is outside %s/%s while mode_select_flag=1; keep current mode.",
                    current_state.mode.c_str(),
                    auto_mode_name.c_str(),
                    guided_mode_name.c_str());
            }
            break;
    }

    publish_mode_select_flag();

    //飞行区域安全检查
    if (fabs(local_pose_data.pose.position.x) > safe_area_xmax ||
        fabs(local_pose_data.pose.position.y) > safe_area_ymax ||
        fabs(local_pose_data.pose.position.z) > safe_area_hightmax)
    {
        if (fightarea_flag_change_allow)
        {
            fightarea_check_flag = 1;
            fightarea_flag_change_allow = 0;
            send_setpoint_enable = false;
            ROS_ERROR("fightarea_check_flag change to 0");
        }
        ROS_ERROR("The UAV is out of the safe area");
    }
    else
    {
        if (fightarea_flag_change_allow)
        {
            local_pose_data_boforeOutarea.position.x = local_pose_data.pose.position.x;
            local_pose_data_boforeOutarea.position.y = local_pose_data.pose.position.y;
            local_pose_data_boforeOutarea.position.z = local_pose_data.pose.position.z;
            local_pose_data_boforeOutarea.yaw = getYawFromPose(local_pose_data);
            fightarea_check_flag = 0;
            ROS_ERROR("The UAV is in the safe area");
        }
    }
}

void mode_check_Callback(const ros::TimerEvent& event)
{
    static uint16_t rc_data_channel6_last = 0;

    // 检测 ch6 是否有变化
    if (!((rc_data.channels[5] > rc_data_channel6_last - 20) &&
          (rc_data.channels[5] < rc_data_channel6_last + 20)))
    {
        ROS_INFO("mode change detected (ch6 changed from %d to %d)",
                 rc_data_channel6_last, rc_data.channels[5]);

        local_pose_data_last.position.x = local_pose_data.pose.position.x;
        local_pose_data_last.position.y = local_pose_data.pose.position.y;
        local_pose_data_last.position.z = local_pose_data.pose.position.z;
        local_pose_data_last.yaw = getYawFromPose(local_pose_data);

        {
            std::lock_guard<std::mutex> lock(control_data_mutex);
            holdCurrentPoseLocked();
        }

        if (rc_data.channels[5] >= 1850)
        {
            mode_select_flag = 1;
            offb_set_mode.request.custom_mode = auto_mode_name;
            if (current_state.mode != auto_mode_name && current_state.mode != guided_mode_name)
            {
                ROS_INFO_THROTTLE(
                    2,
                    "FCU mode %s is outside %s/%s while switching to AUTO/GUIDED; keep current mode.",
                    current_state.mode.c_str(),
                    auto_mode_name.c_str(),
                    guided_mode_name.c_str());
            }
            send_setpoint_enable = (current_state.mode == guided_mode_name);
            ROS_INFO("Mode select: AUTO/GUIDED");
        }
        else
        {
            mode_select_flag = 0;
            offb_set_mode.request.custom_mode = loiter_mode_name;
            send_setpoint_enable = false;

            {
                std::lock_guard<std::mutex> lock(control_data_mutex);
                resetAvoidanceTakeoverStateLocked();
                holdCurrentPoseLocked();
            }

            if (set_mode_client.call(offb_set_mode) && offb_set_mode.response.mode_sent)
                ROS_INFO("Switch to LOITER success!");
            else
                ROS_WARN("Switch to LOITER failed");

            ROS_INFO("Mode select: LOITER");
        }

        fightarea_flag_change_allow = 1;
        fightarea_check_flag = 0;
    }

    rc_data_channel6_last = rc_data.channels[5];

    // 通道10解锁/上锁
    if (rc_data.channels[9] >= 1850)
        arm_select_flag = 0;
    else
        arm_select_flag = 1;

    ROS_INFO_THROTTLE(1, "arm:%d mode:%d send:%d avoid:%d ch6=%d ch10=%d",
                      arm_select_flag, mode_select_flag, send_setpoint_enable,
                      avoidance_enabled.load() ? 1 : 0,
                      rc_data.channels[5], rc_data.channels[9]);
    publish_mode_select_flag();
}

/*直接控制模式的回调函数*/
void straightControldataCallback(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
    if (ctrl_mode_flag == STRAIGHT_CTRL)
    {
        std::lock_guard<std::mutex> lock(control_data_mutex);
        control_data.position.x = msg->pose.position.x;
        control_data.position.y = msg->pose.position.y;
        control_data.position.z = msg->pose.position.z;
        control_data.yaw = getYawFromPose(*msg);
        control_data.type_mask = command_position_pose.type_mask;
        ROS_INFO_THROTTLE(1, "Straight ctrl data accept");
    }
}

/*ego控制模式回调函数*/
void EGOControldataCallback(const quadrotor_msgs::PositionCommand::ConstPtr& msg)
{
    if (ctrl_mode_flag != EGO_CTRL || !plannerControlEnabled())
    {
        return;
    }

    std::lock_guard<std::mutex> lock(control_data_mutex);

    if (waiting_for_avoidance_traj)
    {
        const bool traj_ready =
            msg->trajectory_flag == quadrotor_msgs::PositionCommand::TRAJECTORY_STATUS_READY;
        const bool new_for_this_takeover =
            !avoidance_activation_has_control_traj_id ||
            msg->trajectory_id != avoidance_activation_control_traj_id;

        if (!traj_ready || !new_for_this_takeover)
        {
            ROS_WARN_THROTTLE(1,
                              "Holding current pose while waiting for a fresh avoidance trajectory. latest_traj=%u",
                              msg->trajectory_id);
            return;
        }

        if (received_local_pose && avoidance_takeover_max_setpoint_jump > 0.0)
        {
            const double dx = msg->position.x - local_pose_data.pose.position.x;
            const double dy = msg->position.y - local_pose_data.pose.position.y;
            const double dz = msg->position.z - local_pose_data.pose.position.z;
            const double setpoint_jump = std::sqrt(dx * dx + dy * dy + dz * dz);
            if (setpoint_jump > avoidance_takeover_max_setpoint_jump)
            {
                ROS_WARN_THROTTLE(1,
                                  "Holding current pose because first avoidance setpoint jumps %.2fm from current pose (limit %.2fm, traj=%u).",
                                  setpoint_jump, avoidance_takeover_max_setpoint_jump, msg->trajectory_id);
                return;
            }
        }

        waiting_for_avoidance_traj = false;
        ROS_WARN("Fresh avoidance trajectory accepted: trajectory_id=%u", msg->trajectory_id);
    }

    control_data.position.x = msg->position.x;
    control_data.position.y = msg->position.y;
    control_data.position.z = msg->position.z;
    control_data.yaw = msg->yaw;
    control_data.velocity.x = msg->velocity.x;
    control_data.velocity.y = msg->velocity.y;
    control_data.velocity.z = msg->velocity.z;
    control_data.acceleration_or_force.x = acceleration_ff_scale * msg->acceleration.x;
    control_data.acceleration_or_force.y = acceleration_ff_scale * msg->acceleration.y;
    control_data.acceleration_or_force.z = acceleration_ff_scale * msg->acceleration.z;
    control_data.type_mask = mavros_msgs::PositionTarget::IGNORE_YAW_RATE;
    received_ego_control_cmd = true;
    last_ego_control_cmd_time = ros::Time::now();

    control_planner_traj_id = msg->trajectory_id;
    have_control_planner_traj_id = true;

    ROS_INFO_THROTTLE(1, "EGO ctrl data accept: traj=%u pos=[%.2f, %.2f, %.2f]",
                      msg->trajectory_id,
                      control_data.position.x, control_data.position.y, control_data.position.z);
}

/*控制模式回调*/
void FlagCallback(const std_msgs::UInt8::ConstPtr& msg)
{
    ctrl_mode_flag = msg->data;
}

void velocity_and_position_limit(mavros_msgs::PositionTarget& msg)
{
    if (msg.velocity.x > 0 && msg.velocity.x > vx_max)
        msg.velocity.x = vx_max;
    else if (msg.velocity.x < 0 && msg.velocity.x < -vx_max)
        msg.velocity.x = -vx_max;
    if (msg.velocity.y > 0 && msg.velocity.y > vy_max)
        msg.velocity.y = vy_max;
    else if (msg.velocity.y < 0 && msg.velocity.y < -vy_max)
        msg.velocity.y = -vy_max;
    if (msg.velocity.z > 0 && msg.velocity.z > vz_max)
        msg.velocity.z = vz_max;
    else if (msg.velocity.z < 0 && msg.velocity.z < -vz_max)
        msg.velocity.z = -vz_max;
}

/*
px4 control初始化函数
*/
int px4control_init(ros::NodeHandle nh)
{
    /*****发布的消息初始化***********************************************************/
    //解锁消息
    arm_cmd.request.value = true;
    //发布位置消息的初始化
    command_position_pose.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    command_position_pose.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                      mavros_msgs::PositionTarget::IGNORE_AFY |
                                      mavros_msgs::PositionTarget::IGNORE_AFZ |
                                      mavros_msgs::PositionTarget::IGNORE_VX |
                                      mavros_msgs::PositionTarget::IGNORE_VY |
                                      mavros_msgs::PositionTarget::IGNORE_VZ;

    command_position_pose.position.x = 0;
    command_position_pose.position.y = 0;
    command_position_pose.position.z = 0;

    local_pose_data_last.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_pose_data_last.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                     mavros_msgs::PositionTarget::IGNORE_AFY |
                                     mavros_msgs::PositionTarget::IGNORE_AFZ |
                                     mavros_msgs::PositionTarget::IGNORE_VX |
                                     mavros_msgs::PositionTarget::IGNORE_VY |
                                     mavros_msgs::PositionTarget::IGNORE_VZ;
    local_pose_data_last.position.x = 0;
    local_pose_data_last.position.y = 0;
    local_pose_data_last.position.z = 0;

    local_pose_data_boforeOutarea.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    local_pose_data_boforeOutarea.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                              mavros_msgs::PositionTarget::IGNORE_AFY |
                                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                                              mavros_msgs::PositionTarget::IGNORE_VX |
                                              mavros_msgs::PositionTarget::IGNORE_VY |
                                              mavros_msgs::PositionTarget::IGNORE_VZ;
    local_pose_data_boforeOutarea.position.x = 0;
    local_pose_data_boforeOutarea.position.y = 0;
    local_pose_data_boforeOutarea.position.z = 0;

    //发布高度和速度消息初始化
    command_position_pose_and_vel.coordinate_frame = mavros_msgs::PositionTarget::FRAME_LOCAL_NED;
    command_position_pose_and_vel.type_mask = mavros_msgs::PositionTarget::IGNORE_AFX |
                                              mavros_msgs::PositionTarget::IGNORE_AFY |
                                              mavros_msgs::PositionTarget::IGNORE_AFZ |
                                              mavros_msgs::PositionTarget::IGNORE_PX |
                                              mavros_msgs::PositionTarget::IGNORE_PY |
                                              mavros_msgs::PositionTarget::IGNORE_VZ;

    command_position_pose_and_vel.position.z = 0;
    command_position_pose_and_vel.velocity.x = 0;
    command_position_pose_and_vel.velocity.y = 0;

    //控制指令数据初始化
    control_data.position.x = 0;
    control_data.position.y = 0;
    control_data.position.z = 0;
    control_data.velocity.x = 0;
    control_data.velocity.y = 0;
    control_data.velocity.z = 0;
    control_data.acceleration_or_force.x = 0;
    control_data.acceleration_or_force.y = 0;
    control_data.acceleration_or_force.z = 0;

    /*****相关定时器回调函数定义******************************************************/
    timer_safe_check = nh.createTimer(ros::Duration(0.2), safe_check_Callback);
    timer_safe_check.stop();
    timer_mode_change = nh.createTimer(ros::Duration(0.1), mode_check_Callback);
    timer_mode_change.stop();

    /*****初始化的一些数据读**********************************************************/
    ros::param::get("flight_area/flight_area_xmax", safe_area_xmax);
    ROS_INFO("safe_area_xmax:%f", safe_area_xmax);
    ros::param::get("flight_area/flight_area_ymax", safe_area_ymax);
    ROS_INFO("safe_area_ymax:%f", safe_area_ymax);
    ros::param::get("flight_area/flight_area_hightmax", safe_area_hightmax);
    ROS_INFO("safe_area_hightmax:%f", safe_area_hightmax);
    ros::param::get("flight_velocity/vx", vx_max);
    ROS_INFO("vx_max:%f", vx_max);
    ros::param::get("flight_velocity/vy", vy_max);
    ROS_INFO("vy_max:%f", vy_max);
    ros::param::get("flight_velocity/vz", vz_max);
    ROS_INFO("vz_max:%f", vz_max);

    std::string avoidance_enable_topic;
    nh.param("guided_mode", guided_mode_name, std::string("GUIDED"));
    nh.param("auto_mode", auto_mode_name, std::string("AUTO"));
    nh.param("loiter_mode", loiter_mode_name, std::string("LOITER"));
    nh.param("avoidance_enable_topic", avoidance_enable_topic, std::string("/mission_auto_avoid/avoidance_active"));
    nh.param("avoidance_takeover_max_setpoint_jump", avoidance_takeover_max_setpoint_jump, 1.0);
    nh.param("velocity_tracking/cmd_timeout", control_cmd_timeout, 0.2);
    nh.param("acceleration_ff_scale", acceleration_ff_scale, 1.0);
    if (acceleration_ff_scale < 0.0)
    {
        ROS_WARN("acceleration_ff_scale must be non-negative; clamping %.3f to 0.0",
                 acceleration_ff_scale);
        acceleration_ff_scale = 0.0;
    }
    ROS_INFO("guided_mode:%s auto_mode:%s loiter_mode:%s avoidance_enable_topic:%s takeover_jump:%.2f cmd_timeout:%.2f acceleration_ff_scale:%.2f",
             guided_mode_name.c_str(), auto_mode_name.c_str(), loiter_mode_name.c_str(),
             avoidance_enable_topic.c_str(), avoidance_takeover_max_setpoint_jump,
             control_cmd_timeout, acceleration_ff_scale);

    /*****初始化一些状态数据**********************************************************/
    fightarea_check_flag = 1;
    fightarea_flag_change_allow = 1;
    mode_check_flag = 1;
    mode_select_flag = 0;
    arm_select_flag = 0;
    raw_rc_data.channels.assign(18, 0);
    rc_data.channels.assign(18, 0);
    rc_override_channels.fill(mavros_msgs::OverrideRCIn::CHAN_RELEASE);
    send_setpoint_enable = false;
    avoidance_enabled.store(false);
    waiting_for_avoidance_traj = false;
    have_control_planner_traj_id = false;
    avoidance_activation_has_control_traj_id = false;
    control_planner_traj_id = 0;
    avoidance_activation_control_traj_id = 0;

    /*****ROS句柄、订阅、发布、服务器初始化*********************************************/
    state_sub = nh.subscribe<mavros_msgs::State>("mavros/state", 10, state_callback);
    rcin_sub = nh.subscribe<mavros_msgs::RCIn>("mavros/rc/in", 10, rc_input_callback);
    rc_override_sub = nh.subscribe<mavros_msgs::OverrideRCIn>("mavros/rc/override", 10, rc_override_callback);
    local_position_pose_sub = nh.subscribe<geometry_msgs::PoseStamped>("mavros/local_position/pose", 10, localpositionposeCallback);

    straight_control_data_sub = nh.subscribe<geometry_msgs::PoseStamped>("straight_output_target", 2, straightControldataCallback);
    ego_control_data_sub = nh.subscribe<quadrotor_msgs::PositionCommand>("/position_cmd", 2, EGOControldataCallback);
    flag_sub = nh.subscribe<std_msgs::UInt8>("/control_data/task_status", 1, FlagCallback);
    avoidance_enable_sub = nh.subscribe<std_msgs::Bool>(avoidance_enable_topic, 1, avoidanceEnableCallback);

    // 等待收到mavros/local_position/pose初始位姿
    ROS_INFO("等待接收初始位姿...");
    ros::Rate rate(5);
    while (ros::ok() && !received_local_pose)
    {
        ros::spinOnce();
        rate.sleep();
    }
    std::cout << "初始位姿：" << local_pose_data.pose.position.x << " "
              << local_pose_data.pose.position.y << " "
              << local_pose_data.pose.position.z << std::endl;

    control_data.position.x = command_position_pose.position.x = local_pose_data.pose.position.x;
    control_data.position.y = command_position_pose.position.y = local_pose_data.pose.position.y;
    control_data.position.z = command_position_pose.position.z = local_pose_data.pose.position.z;
    tf::Quaternion quat;
    double roll, pitch, yaw;
    tf::quaternionMsgToTF(local_pose_data.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    control_data.yaw = command_position_pose.yaw = yaw;
    control_data.type_mask = command_position_pose.type_mask;
    ROS_INFO("已接收初始位姿，继续后续初始化!");

    local_pose_vel_pub = nh.advertise<mavros_msgs::PositionTarget>("mavros/setpoint_raw/local", 10);
    mode_select_flag_pub = nh.advertise<std_msgs::UInt8>("mode_select_flag", 10);

    arming_client = nh.serviceClient<mavros_msgs::CommandBool>("mavros/cmd/arming");
    set_mode_client = nh.serviceClient<mavros_msgs::SetMode>("mavros/set_mode");

    return 1;
}

/*
px4_control start函数
任务：MAVROS连接 飞行安全检查的初始化
*/
int px4control_start()
{
    ros::Rate rate(20);
    while (ros::ok() && !current_state.connected)
    {
        ros::spinOnce();
        rate.sleep();
    }
    ROS_INFO("connected with px4");
    timer_mode_change.start();
    ros::Duration du(0.5);
    du.sleep();
    ROS_INFO("mode change check start");

    timer_safe_check.start();
    du.sleep();
    return 1;
}

void uavcontrol()
{
    ros::Rate rate(30);
    while (ros::ok())
    {
        if (!send_setpoint_enable || mode_select_flag == 0 || current_state.mode != guided_mode_name)
        {
            rate.sleep();
            continue;
        }

        const bool use_ego_position_velocity_acceleration =
            ctrl_mode_flag == EGO_CTRL && plannerControlEnabled();

        mavros_msgs::PositionTarget tracked_cmd;
        bool command_stale = false;
        {
            std::lock_guard<std::mutex> lock(control_data_mutex);
            tracked_cmd = control_data;
            command_stale =
                use_ego_position_velocity_acceleration &&
                (!received_ego_control_cmd ||
                 (ros::Time::now() - last_ego_control_cmd_time).toSec() > control_cmd_timeout);
        }

        velocity_and_position_limit(tracked_cmd);

        if (use_ego_position_velocity_acceleration)
        {
            if (command_stale)
            {
                tracked_cmd.velocity.x = 0.0;
                tracked_cmd.velocity.y = 0.0;
                tracked_cmd.velocity.z = 0.0;
                tracked_cmd.acceleration_or_force.x = 0.0;
                tracked_cmd.acceleration_or_force.y = 0.0;
                tracked_cmd.acceleration_or_force.z = 0.0;
                ROS_WARN_THROTTLE(
                    1.0,
                    "EGO command timeout: keep the last position target and clear velocity/acceleration feed-forward");
            }

            command_position_velocity_acceleration_control(
                tracked_cmd.position.x, tracked_cmd.position.y, tracked_cmd.position.z,
                tracked_cmd.velocity.x, tracked_cmd.velocity.y, tracked_cmd.velocity.z,
                tracked_cmd.acceleration_or_force.x,
                tracked_cmd.acceleration_or_force.y,
                tracked_cmd.acceleration_or_force.z,
                tracked_cmd.yaw);
            ROS_INFO_THROTTLE(
                1,
                "give EGO position+velocity+acceleration feed-forward command to uav");
        }
        else
        {
            command_position_control(tracked_cmd.position.x, tracked_cmd.position.y,
                                     tracked_cmd.position.z, tracked_cmd.yaw);
            ROS_INFO_THROTTLE(1, "give position command to uav");
        }
        rate.sleep();
    }
}

int main(int argc, char* argv[])
{
    ros::init(argc, argv, "apm_auto");
    ros::NodeHandle nh;

    ROS_INFO("OK");
    px4control_init(nh);
    ROS_INFO("contrl init sucess");
    while (!px4control_start())
    {
    }
    ROS_INFO("control start sucess");

    std::thread control_thread(uavcontrol);

    ros::spin();
}
