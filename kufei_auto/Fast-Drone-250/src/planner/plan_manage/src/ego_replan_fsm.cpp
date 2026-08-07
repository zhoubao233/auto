
#include <plan_manage/ego_replan_fsm.h>
#include <cmath>

namespace ego_planner
{

  void EGOReplanFSM::init(ros::NodeHandle &nh)
  {
    node_ = nh;
    current_wp_ = 0;
    exec_state_ = FSM_EXEC_STATE::INIT;
    have_target_ = false;
    have_odom_ = false;
    have_recv_pre_agent_ = false;

    /*  fsm param  */
    nh.param("fsm/flight_type", target_type_, -1);// (1: RViz手动点, 2: launch文件预设航点, 3: 外部其他节点输入的动态目标点)
    nh.param("fsm/thresh_replan_time", replan_thresh_, -1.0);// 2. 读取定期“重规划时间周期阈值”（单位：秒）
    nh.param("fsm/thresh_no_replan_meter", no_replan_thresh_, -1.0);// 3. 读取“接近目标点停止重规划距离阈值”
    nh.param("fsm/planning_horizon", planning_horizen_, -1.0);// 4. 读取局部规划的“空间前瞻距离/局部地图大小”
    nh.param("fsm/planning_horizen_time", planning_horizen_time_, -1.0);// 5. 读取局部规划的“时间前瞻跨度”生成的局部 B 样条轨迹在时间维度上覆盖未来多久
    nh.param("fsm/emergency_time", emergency_time_, 1.0);// 6. 读取“紧急刹车时间阈值/碰撞时间（TTC）”// (安全检查时，若预测到与障碍物发生碰撞的时间小于该值，放弃重规划，直接触发紧急刹车/悬停)
    nh.param("fsm/realworld_experiment", flag_realworld_experiment_, false);// (若为 true，系统必须等待遥控器起飞开关触发才开始规划，防止真机上电后意外误飞)
    nh.param("fsm/fail_safe", enable_fail_safe_, true);// 8. 读取“故障保护机制（Fail-Safe）开关” (若为 true，当丢失里程计定位信号或规划严重失败时，自动触发安全保护，防止坠机)
    nh.param("fsm/planning_enabled_on_start", planning_enabled_, true);// 9. 读取“启动时默认开启规划”

    nh.param("fsm/goal_arrival_xy_tolerance", goal_arrival_xy_tolerance_, 0.4);
    nh.param("fsm/goal_arrival_z_tolerance", goal_arrival_z_tolerance_, 0.3);
    nh.param("fsm/goal_arrival_max_xy_speed", goal_arrival_max_xy_speed_, 0.3);
    nh.param("fsm/goal_arrival_max_z_speed", goal_arrival_max_z_speed_, 0.15);
    have_trigger_ = !flag_realworld_experiment_;

    //预设航点读取
    nh.param("fsm/waypoint_num", waypoint_num_, -1);
    for (int i = 0; i < waypoint_num_; i++)
    {
      nh.param("fsm/waypoint" + to_string(i) + "_x", waypoints_[i][0], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_y", waypoints_[i][1], -1.0);
      nh.param("fsm/waypoint" + to_string(i) + "_z", waypoints_[i][2], -1.0);
    }

    /* initialize main modules */
    visualization_.reset(new PlanningVisualization(nh)); // 1. 可视化模块（在 RViz 里画轨迹和障碍物）
    planner_manager_.reset(new EGOPlannerManager);// 2. 核心规划管理器（包含 A* 算法和 B 样条轨迹优化器）
    planner_manager_->initPlanModules(nh, visualization_);// 初始化地图和优化器
    planner_manager_->deliverTrajToOptimizer();  // 初始化轨迹存储
    planner_manager_->setDroneIdtoOpt(); // 设置多机协作时的无人机 ID

    /*
      1. 状态机主定时器：每 0.01 秒（100 Hz）执行一次exec_timer_ (100Hz)： 
      状态机的心脏。 它不断循环检查当前状态
      （INIT、WAIT_TARGET、REPLAN_TRAJ、EXEC_TRAJ、EMERGENCY_STOP），
      并决定什么时候重新规划轨迹，什么时候把轨迹发给飞控。
    */
    exec_timer_ = nh.createTimer(ros::Duration(0.01), &EGOReplanFSM::execFSMCallback, this);
    /*
      2. 安全检查定时器：每 0.05 秒（20 Hz）执行一次
      安全守护进程。 它独立于状态机，
      不断检查**“当前正在飞行的轨迹前方是否突发出现了新障碍物”**。
      如果突然出现障硬物，它会强制切换状态机进入 EMERGENCY_STOP（紧急刹车）或触发立即重规划。
    */ 
    safety_timer_ = nh.createTimer(ros::Duration(0.05), &EGOReplanFSM::checkCollisionCallback, this);

    string odom_topic;// 1. 声明一个字符串变量 odom_topic，用来存放里程计话题的名称
    /*
      2. 从 ROS 参数服务器读取 "odometry_topic" 参数，存入 odom_topic。
      若 launch 文件里没配置该参数，则默认使用话题名 "odom_world"
    */
    nh.param("odometry_topic", odom_topic, string("odom_world")); 
    /*  
      3. 在终端控制台打印一行绿色/白色 INFO 日志，告知开发者系统正在订阅哪个里程计话题
      (.c_str() 是 C++ 语法，把 std::string 转换为 C 语言风格字符串以供 %s 打印)
    */
    ROS_INFO("Subscribing to odometry topic: %s", odom_topic.c_str());
    // 4.  订阅无人机自身的里程计（定位信息）
    //    - 话题名：odom_topic
    //    - 消息队列大小：1（只保留最新的一帧定位，旧数据直接丢弃，保证实时性）
    //    - 回调函数：&EGOReplanFSM::odometryCallback（收到新数据时立即触发该函数）
    //    - 类实例指针：this（指向当前的 FSM 状态机对象）
    odom_sub_ = nh.subscribe(odom_topic, 1, &EGOReplanFSM::odometryCallback, this);

    /*集群*/
    if (planner_manager_->pp_.drone_id >= 1)
    {/*订阅前面无人机的轨迹*/
      string sub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id - 1) + string("_planning/swarm_trajs");
      swarm_trajs_sub_ = nh.subscribe(sub_topic_name.c_str(), 10, &EGOReplanFSM::swarmTrajsCallback, this, ros::TransportHints().tcpNoDelay());
    }
    string pub_topic_name = string("/drone_") + std::to_string(planner_manager_->pp_.drone_id) + string("_planning/swarm_trajs");
    // 广播自己的轨迹给其他无人机
    swarm_trajs_pub_ = nh.advertise<traj_utils::MultiBsplines>(pub_topic_name.c_str(), 10);
    /*创建广播发布者：把本机生成的 B 样条轨迹发给无线电/Mesh组网模块，广播给其他无人机*/
    broadcast_bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/broadcast_bspline_from_planner", 10);
    /*创建广播订阅者：监听无线电收到的其他无人机的 B 样条轨迹*/
    broadcast_bspline_sub_ = nh.subscribe("planning/broadcast_bspline_to_planner", 100, &EGOReplanFSM::BroadcastBsplineCallback, this, ros::TransportHints().tcpNoDelay());
    // 3. 发布最终 轨迹 给底层飞控（如 n3ctrl 或 so3_control） 后面traj_server 轨迹节点转化成 期望位置 速度 加速度 角度这些
    bspline_pub_ = nh.advertise<traj_utils::Bspline>("planning/bspline", 10);
    /* 发布算法运行诊断数据（如计算耗时、规划距离、当前速度、加速等），用于 RQT 仪表盘或地面站显示*/
    data_disp_pub_ = nh.advertise<traj_utils::DataDisp>("planning/data_display", 100);
    /* 注册 ROS 服务（Service）：提供一个远程控制接口，允许通过命令行或地面站随时“开启”或“暂停”规划器*/
    planning_enable_srv_ = nh.advertiseService("set_planning_enabled", &EGOReplanFSM::setPlanningEnabledCallback, this);
    // 在终端打印一行日志，告知开发者当前规划器启动时是“使能(enabled)”还是“禁能(disabled)”状态
    ROS_INFO("Planner startup state: %s", planning_enabled_ ? "enabled" : "disabled");

    if (target_type_ == TARGET_TYPE::MANUAL_TARGET)/*手动模式给点*/
    {
      waypoint_sub_ = nh.subscribe("/move_base_simple/goal", 1, &EGOReplanFSM::waypointCallback, this);
    }
    else if (target_type_ == TARGET_TYPE::PRESET_TARGET)/*预设航点模式*/
    {
      trigger_sub_ = nh.subscribe("/traj_start_trigger", 1, &EGOReplanFSM::triggerCallback, this);

      ROS_INFO("Wait for 1 second.");
      int count = 0;
      while (ros::ok() && count++ < 1000)
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      ROS_WARN("Waiting for trigger from [n3ctrl] from RC");

      while (ros::ok() && (!have_odom_ || !have_trigger_))
      {
        ros::spinOnce();
        ros::Duration(0.001).sleep();
      }

      readGivenWps();
    }
    else if(target_type_ == TARGET_TYPE::OUTPUT_TARGET){/*外部算法给点*/
      ROS_ERROR("external target mode");
      waypoint_sub_ = nh.subscribe("/ego_input_target", 1, &EGOReplanFSM::outputWaypointCallback, this);
    }
    else
      cout << "Wrong target_type_ value! target_type_=" << target_type_ << endl;
  }
  /* 加载预设航点 先不看*/
  void EGOReplanFSM::readGivenWps()
  {
    if (waypoint_num_ <= 0)
    {
      ROS_ERROR("Wrong waypoint_num_ = %d", waypoint_num_);
      return;
    }

    wps_.resize(waypoint_num_);
    for (int i = 0; i < waypoint_num_; i++)
    {
      wps_[i](0) = waypoints_[i][0];
      wps_[i](1) = waypoints_[i][1];
      wps_[i](2) = waypoints_[i][2];

      // end_pt_ = wps_.back();
    }

    // bool success = planner_manager_->planGlobalTrajWaypoints(
    //   odom_pos_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero(),
    //   wps_, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero());

    for (size_t i = 0; i < (size_t)waypoint_num_; i++)
    {
      visualization_->displayGoalPoint(wps_[i], Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, i);
      ros::Duration(0.001).sleep();
    }

    // plan first global waypoint
    wp_id_ = 0;
    planNextWaypoint(wps_[wp_id_]);

    // if (success)
    // {

    //   /*** display ***/
    //   constexpr double step_size_t = 0.1;
    //   int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
    //   std::vector<Eigen::Vector3d> gloabl_traj(i_end);
    //   for (int i = 0; i < i_end; i++)
    //   {
    //     gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
    //   }

    //   end_vel_.setZero();
    //   have_target_ = true;
    //   have_new_target_ = true;

    //   /*** FSM ***/
    //   // if (exec_state_ == WAIT_TARGET)
    //   //changeFSMExecState(GEN_NEW_TRAJ, "TRIG");
    //   // trigger_ = true;
    //   // else if (exec_state_ == EXEC_TRAJ)
    //   //   changeFSMExecState(REPLAN_TRAJ, "TRIG");

    //   // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
    //   ros::Duration(0.001).sleep();
    //   visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    //   ros::Duration(0.001).sleep();
    // }
    // else
    // {
    //   ROS_ERROR("Unable to generate global trajectory!");
    // }
  }
  /*当接收到一个新的目标点 next_wp 时，先生成一条全局参考轨迹*/
  void EGOReplanFSM::planNextWaypoint(const Eigen::Vector3d next_wp)
  {
    if (!planning_enabled_)/*使能检查*/
    {
      ROS_INFO("Planner disabled, skip planNextWaypoint request.");
      return;
    }

    Eigen::Vector3d global_start_pos = odom_pos_;
    Eigen::Vector3d global_start_vel = odom_vel_;
    Eigen::Vector3d global_target = next_wp;

    if (planner_manager_->planarModeEnabled())
    {
      const double requested_z = next_wp(2);
      if (std::fabs(odom_pos_(2) - requested_z) > planner_manager_->planarEntryTolerance() ||
          std::fabs(odom_vel_(2)) > planner_manager_->planarEntryMaxVz())
      {
        pending_planar_target_ = next_wp;
        have_pending_planar_target_ = true;
        ROS_WARN_THROTTLE(1.0,
                          "Wait waypoint height: odom_z=%.3f vz=%.3f requested_z=%.3f",
                          odom_pos_(2), odom_vel_(2), requested_z);
        return;
      }

      planner_manager_->setPlanarLockZ(requested_z);
      global_start_pos(2) = requested_z;
      global_start_vel(2) = 0.0;
      global_target(2) = requested_z;
      have_pending_planar_target_ = false;
    }

    /*
    求解全局参考轨迹
      存到了  planner_manager_->global_data_ 
      1. 轨迹的总时长（秒）：
        planner_manager_->global_data_.global_duration_ 
      2. 真正的全局 B 样条轨迹对象：
        planner_manager_->global_data_.global_traj_

        这里他根据当前速度位置 和目标速度位置 生成了一个直线
        这个实现里面是 生成全局参考多项式
      */
    bool success = planner_manager_->planGlobalTraj(
      global_start_pos,      // 1. 起点位置：二维模式下投影到目标航点高度
      global_start_vel,      // 2. 起点速度：二维模式下垂直速度固定为 0
      Eigen::Vector3d::Zero(),// 3. 起点加速度：默认 0
      global_target,         // 4. 终点位置：当前目标航点
      Eigen::Vector3d::Zero(),// 5. 终点速度：直接写死了 Vector3d::Zero()！！(即 0, 0, 0)
      Eigen::Vector3d::Zero() // 6. 终点加速度：直接写死了 Vector3d::Zero()！！(即 0, 0, 0)
  );
    // visualization_->displayGoalPoint(next_wp, Eigen::Vector4d(0, 0.5, 0.5, 1), 0.3, 0);

    if (success)
    {
      const bool target_changed =
          !have_target_ || (global_target - end_pt_).norm() > 1e-3;

      end_pt_ = global_target;// 记录当前航点；二维模式下 Z 是本航段锁定高度
      ROS_WARN("Global trajectory accepted: start=[%.2f, %.2f, %.2f] target=[%.2f, %.2f, %.2f] state=%d.",
               global_start_pos(0), global_start_pos(1), global_start_pos(2),
               global_target(0), global_target(1), global_target(2),
               static_cast<int>(exec_state_));

      // A failure while planning the previous (already reached) target must not
      // make the first attempt for a different target use a random polynomial.
      // Keep random initialization available only after a deterministic attempt
      // for this target has genuinely failed.
      if (target_changed &&
          (exec_state_ == GEN_NEW_TRAJ || exec_state_ == SEQUENTIAL_START))
      {
        continously_called_times_ = 1;
        ROS_WARN("New global target accepted; reset trajectory-generation retry state "
                 "so its first local B-spline uses deterministic initialization.");
      }

      /*** display ***/
      constexpr double step_size_t = 0.1;
      int i_end = floor(planner_manager_->global_data_.global_duration_ / step_size_t);
      vector<Eigen::Vector3d> gloabl_traj(i_end);
      for (int i = 0; i < i_end; i++)
      {
        // 每隔 0.1 秒对全局 B 样条曲线求值，采样出一系列 3D 点
        gloabl_traj[i] = planner_manager_->global_data_.global_traj_.evaluate(i * step_size_t);
      }

      end_vel_.setZero(); // 记录终点速度为 0
      have_target_ = true;// 标志位：当前有目标点
      have_new_target_ = true;// 标志位：这是一个刚来的新目标点

      /*** FSM 状态切换 ***/
      if (exec_state_ == WAIT_TARGET || exec_state_ == INIT) /*等待目标中 */
      {
        changeFSMExecState(GEN_NEW_TRAJ, "TRIG"); // 直接把状态切换为 GEN_NEW_TRAJ（生成新轨迹）
      }
      else if (exec_state_ == EXEC_TRAJ)
      {
        changeFSMExecState(REPLAN_TRAJ, "TRIG");/*重新规划轨迹*/
      }
      else
      {
        // Do not spin recursively waiting for EXEC_TRAJ here. The manager may
        // retry an identical goal while the first trajectory is still being
        // generated; recursively calling ros::spinOnce() from this callback can
        // nest more waypoint callbacks and starve the FSM timer. The global
        // target above is already refreshed, so let the current FSM state finish.
        ROS_INFO_THROTTLE(1.0,
                          "Planner target refreshed while trajectory generation is already in progress (state=%d).",
                          static_cast<int>(exec_state_));
      }

      // visualization_->displayGoalPoint(end_pt_, Eigen::Vector4d(1, 0, 0, 1), 0.3, 0);
      // 在 RViz 里把刚才采样的全局路径画成红/黄线展示出来
      visualization_->displayGlobalPathList(gloabl_traj, 0.1, 0);
    }
    else
    {
      ROS_ERROR("Unable to generate global trajectory!");//求解失败 
    }
  }
  /* 收到/move_base_simple/goal 后执行  */
  void EGOReplanFSM::triggerCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (!planning_enabled_)
    {
      ROS_INFO_THROTTLE(1.0, "Planner disabled, ignore trigger input.");
      return;
    }

    have_trigger_ = true;
    cout << "Triggered!" << endl;
    init_pt_ = odom_pos_;
  }
  /*/move_base_simple/goal 给点模式后触发的*/
  void EGOReplanFSM::waypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (!planning_enabled_)
    {
      ROS_INFO_THROTTLE(1.0, "Planner disabled, ignore manual waypoint input.");
      return;
    }

    ROS_WARN("[TEST] Waypoint callback triggered!");

    if (msg->pose.position.z < -0.1)
      return ;

    cout << "Triggered!" << endl;
    // trigger_ = true;
    init_pt_ = odom_pos_;

    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y, msg->pose.position.z);

    planNextWaypoint(end_wp);
  }
  /* ego_input_target" */
  void EGOReplanFSM::outputWaypointCallback(const geometry_msgs::PoseStampedPtr &msg)
  {
    if (!planning_enabled_)
    {
      ROS_INFO_THROTTLE(1.0, "Planner disabled, ignore external target input.");
      return;
    }

    if (msg->pose.position.z < -0.1){
      ROS_ERROR("Illegal target!!");
      return;
    }

    cout << "Triggered!" << endl;
    // trigger_ = true;
    init_pt_ = odom_pos_;

    // 默认高度是1.0m 将msg的三维坐标系xyz 提取出来变成Eigen 库的三维向量 end_wp。
    Eigen::Vector3d end_wp(msg->pose.position.x, msg->pose.position.y,msg->pose.position.z);
    /*生成全局轨迹*/
    planNextWaypoint(end_wp);
  }
  /*远程规划“开启关闭”服务回调函数*/
  bool EGOReplanFSM::setPlanningEnabledCallback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res)
  {
    if (planning_enabled_ == req.data)
    {
      res.success = true;
      res.message = req.data ? "planner already enabled" : "planner already disabled";
      return true;
    }

    planning_enabled_ = req.data;

    if (!planning_enabled_)
    {
      have_target_ = false;
      have_new_target_ = false;
      have_pending_planar_target_ = false;

      if (exec_state_ != INIT && exec_state_ != WAIT_TARGET)
      {
        changeFSMExecState(WAIT_TARGET, "PLANNING_SRV");
      }

      ROS_WARN("Planner disabled by service request.");
      res.success = true;
      res.message = "planner disabled";
      return true;
    }

    ROS_WARN("Planner enabled by service request.");
    res.success = true;
    res.message = "planner enabled";
    return true;
  }
  /*里程计订阅  更新位置速度 方向*/
  void EGOReplanFSM::odometryCallback(const nav_msgs::OdometryConstPtr &msg)
  {

    static bool first_received = false;
    if (!first_received) {
        ROS_WARN("[TEST] First odom received!");
        first_received = true;
    }

    odom_pos_(0) = msg->pose.pose.position.x;
    odom_pos_(1) = msg->pose.pose.position.y;
    odom_pos_(2) = msg->pose.pose.position.z;

    odom_vel_(0) = msg->twist.twist.linear.x;
    odom_vel_(1) = msg->twist.twist.linear.y;
    odom_vel_(2) = msg->twist.twist.linear.z;

    //odom_acc_ = estimateAcc( msg );

    odom_orient_.w() = msg->pose.pose.orientation.w;
    odom_orient_.x() = msg->pose.pose.orientation.x;
    odom_orient_.y() = msg->pose.pose.orientation.y;
    odom_orient_.z() = msg->pose.pose.orientation.z;

    have_odom_ = true;
  }
  /*其他无人机的线条订阅*/
  void EGOReplanFSM::BroadcastBsplineCallback(const traj_utils::BsplinePtr &msg)
  {
    size_t id = msg->drone_id;
    if ((int)id == planner_manager_->pp_.drone_id)
      return;

    if (abs((ros::Time::now() - msg->start_time).toSec()) > 0.25)
    {
      ROS_ERROR("Time difference is too large! Local - Remote Agent %d = %fs",
                msg->drone_id, (ros::Time::now() - msg->start_time).toSec());
      return;
    }

    /* Fill up the buffer */
    if (planner_manager_->swarm_trajs_buf_.size() <= id)
    {
      for (size_t i = planner_manager_->swarm_trajs_buf_.size(); i <= id; i++)
      {
        OneTrajDataOfSwarm blank;
        blank.drone_id = -1;
        planner_manager_->swarm_trajs_buf_.push_back(blank);
      }
    }

    /* Test distance to the agent */
    Eigen::Vector3d cp0(msg->pos_pts[0].x, msg->pos_pts[0].y, msg->pos_pts[0].z);
    Eigen::Vector3d cp1(msg->pos_pts[1].x, msg->pos_pts[1].y, msg->pos_pts[1].z);
    Eigen::Vector3d cp2(msg->pos_pts[2].x, msg->pos_pts[2].y, msg->pos_pts[2].z);
    Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
    if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
    {
      planner_manager_->swarm_trajs_buf_[id].drone_id = -1;
      return; // if the current drone is too far to the received agent.
    }

    /* Store data */
    Eigen::MatrixXd pos_pts(3, msg->pos_pts.size());
    Eigen::VectorXd knots(msg->knots.size());
    for (size_t j = 0; j < msg->knots.size(); ++j)
    {
      knots(j) = msg->knots[j];
    }
    for (size_t j = 0; j < msg->pos_pts.size(); ++j)
    {
      pos_pts(0, j) = msg->pos_pts[j].x;
      pos_pts(1, j) = msg->pos_pts[j].y;
      pos_pts(2, j) = msg->pos_pts[j].z;
    }

    planner_manager_->swarm_trajs_buf_[id].drone_id = id;

    if (msg->order % 2)
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = msg->knots[msg->knots.size() - ceil(cutback)];
    }
    else
    {
      double cutback = (double)msg->order / 2 + 1.5;
      planner_manager_->swarm_trajs_buf_[id].duration_ = (msg->knots[msg->knots.size() - floor(cutback)] + msg->knots[msg->knots.size() - ceil(cutback)]) / 2;
    }

    UniformBspline pos_traj(pos_pts, msg->order, msg->knots[1] - msg->knots[0]);
    pos_traj.setKnot(knots);
    planner_manager_->swarm_trajs_buf_[id].position_traj_ = pos_traj;

    planner_manager_->swarm_trajs_buf_[id].start_pos_ = planner_manager_->swarm_trajs_buf_[id].position_traj_.evaluateDeBoorT(0);

    planner_manager_->swarm_trajs_buf_[id].start_time_ = msg->start_time;
    // planner_manager_->swarm_trajs_buf_[id].start_time_ = ros::Time::now(); // Un-reliable time sync

    /* Check Collision */
    if (planner_manager_->checkCollision(id))
    {
      changeFSMExecState(REPLAN_TRAJ, "TRAJ_CHECK");
    }
  }
  /*也是群集 前面无人机的轨迹*/
  void EGOReplanFSM::swarmTrajsCallback(const traj_utils::MultiBsplinesPtr &msg)
  {

    ROS_WARN("[TEST] Swarm trajs callback triggered!");
    
    multi_bspline_msgs_buf_.traj.clear();
    multi_bspline_msgs_buf_ = *msg;

    // cout << "\033[45;33mmulti_bspline_msgs_buf.drone_id_from=" << multi_bspline_msgs_buf_.drone_id_from << " multi_bspline_msgs_buf_.traj.size()=" << multi_bspline_msgs_buf_.traj.size() << "\033[0m" << endl;

    if (!have_odom_)
    {
      ROS_ERROR("swarmTrajsCallback(): no odom!, return.");
      return;
    }

    if ((int)msg->traj.size() != msg->drone_id_from + 1) // drone_id must start from 0
    {
      ROS_ERROR("Wrong trajectory size! msg->traj.size()=%d, msg->drone_id_from+1=%d", (int)msg->traj.size(), msg->drone_id_from + 1);
      return;
    }

    if (msg->traj[0].order != 3) // only support B-spline order equals 3.
    {
      ROS_ERROR("Only support B-spline order equals 3.");
      return;
    }

    // Step 1. receive the trajectories
    planner_manager_->swarm_trajs_buf_.clear();
    planner_manager_->swarm_trajs_buf_.resize(msg->traj.size());

    for (size_t i = 0; i < msg->traj.size(); i++)
    {

      Eigen::Vector3d cp0(msg->traj[i].pos_pts[0].x, msg->traj[i].pos_pts[0].y, msg->traj[i].pos_pts[0].z);
      Eigen::Vector3d cp1(msg->traj[i].pos_pts[1].x, msg->traj[i].pos_pts[1].y, msg->traj[i].pos_pts[1].z);
      Eigen::Vector3d cp2(msg->traj[i].pos_pts[2].x, msg->traj[i].pos_pts[2].y, msg->traj[i].pos_pts[2].z);
      Eigen::Vector3d swarm_start_pt = (cp0 + 4 * cp1 + cp2) / 6;
      if ((swarm_start_pt - odom_pos_).norm() > planning_horizen_ * 4.0f / 3.0f)
      {
        planner_manager_->swarm_trajs_buf_[i].drone_id = -1;
        continue;
      }

      Eigen::MatrixXd pos_pts(3, msg->traj[i].pos_pts.size());
      Eigen::VectorXd knots(msg->traj[i].knots.size());
      for (size_t j = 0; j < msg->traj[i].knots.size(); ++j)
      {
        knots(j) = msg->traj[i].knots[j];
      }
      for (size_t j = 0; j < msg->traj[i].pos_pts.size(); ++j)
      {
        pos_pts(0, j) = msg->traj[i].pos_pts[j].x;
        pos_pts(1, j) = msg->traj[i].pos_pts[j].y;
        pos_pts(2, j) = msg->traj[i].pos_pts[j].z;
      }

      planner_manager_->swarm_trajs_buf_[i].drone_id = i;

      if (msg->traj[i].order % 2)
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)];
      }
      else
      {
        double cutback = (double)msg->traj[i].order / 2 + 1.5;
        planner_manager_->swarm_trajs_buf_[i].duration_ = (msg->traj[i].knots[msg->traj[i].knots.size() - floor(cutback)] + msg->traj[i].knots[msg->traj[i].knots.size() - ceil(cutback)]) / 2;
      }

      // planner_manager_->swarm_trajs_buf_[i].position_traj_ =
      UniformBspline pos_traj(pos_pts, msg->traj[i].order, msg->traj[i].knots[1] - msg->traj[i].knots[0]);
      pos_traj.setKnot(knots);
      planner_manager_->swarm_trajs_buf_[i].position_traj_ = pos_traj;

      planner_manager_->swarm_trajs_buf_[i].start_pos_ = planner_manager_->swarm_trajs_buf_[i].position_traj_.evaluateDeBoorT(0);

      planner_manager_->swarm_trajs_buf_[i].start_time_ = msg->traj[i].start_time;
    }

    have_recv_pre_agent_ = true;
  }
  /*状态机切换*/
  void EGOReplanFSM::changeFSMExecState(FSM_EXEC_STATE new_state, string pos_call)
  {

    if (new_state == exec_state_)
      continously_called_times_++;
    else
      continously_called_times_ = 1;

    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};
    int pre_s = int(exec_state_);
    exec_state_ = new_state;
    cout << "[" + pos_call + "]: from " + state_str[pre_s] + " to " + state_str[int(new_state)] << endl;
  }
  /*返回当前状态second (exec_state_) ，以及状态持续时间first：(continously_called_times_)*/
  std::pair<int, EGOReplanFSM::FSM_EXEC_STATE> EGOReplanFSM::timesOfConsecutiveStateCalls()
  {
    return std::pair<int, FSM_EXEC_STATE>(continously_called_times_, exec_state_);
  }

  void EGOReplanFSM::printFSMExecState()
  {
    static string state_str[8] = {"INIT", "WAIT_TARGET", "GEN_NEW_TRAJ", "REPLAN_TRAJ", "EXEC_TRAJ", "EMERGENCY_STOP", "SEQUENTIAL_START"};

    cout << "[FSM]: state: " + state_str[int(exec_state_)] << endl;
  }
  /*状态机函数 不同状态做不同事情*/
  void EGOReplanFSM::execFSMCallback(const ros::TimerEvent &e)
  {
    exec_timer_.stop(); // To avoid blockage

    if (planning_enabled_ && have_odom_ && have_pending_planar_target_ &&
        (exec_state_ == WAIT_TARGET || exec_state_ == EXEC_TRAJ))
    {
      const double requested_z = pending_planar_target_(2);
      if (std::fabs(odom_pos_(2) - requested_z) <= planner_manager_->planarEntryTolerance() &&
          std::fabs(odom_vel_(2)) <= planner_manager_->planarEntryMaxVz())
      {
        const Eigen::Vector3d pending_target = pending_planar_target_;
        have_pending_planar_target_ = false;
        planNextWaypoint(pending_target);
      }
    }

    static int fsm_num = 0;
    fsm_num++;
    if (fsm_num == 100)
    {
      printFSMExecState();
      if (!have_odom_)
        cout << "no odom." << endl;
      if (!have_target_)
        cout << "wait for goal or trigger." << endl;
      fsm_num = 0;
    }

    switch (exec_state_)
    {
    case INIT:
    {
      if (!have_odom_)
      {
        goto force_return;
        // return;
      }
      changeFSMExecState(WAIT_TARGET, "FSM");
      break;
    }

    case WAIT_TARGET:
    {
      if (!have_target_ || !have_trigger_)
        goto force_return;
      // return;
      else
      {
        // if ( planner_manager_->pp_.drone_id <= 0 )
        // {
        //   changeFSMExecState(GEN_NEW_TRAJ, "FSM");
        // }
        // else
        // {
        changeFSMExecState(SEQUENTIAL_START, "FSM");
        // }
      }
      break;
    }

    case SEQUENTIAL_START: // for swarm
    {
      // cout << "id=" << planner_manager_->pp_.drone_id << " have_recv_pre_agent_=" << have_recv_pre_agent_ << endl;
      if (planner_manager_->pp_.drone_id <= 0 || (planner_manager_->pp_.drone_id >= 1 && have_recv_pre_agent_))
      {
        if (have_odom_ && have_target_ && have_trigger_)
        {
          bool success = planFromGlobalTraj(10); // zx-todo
          if (success)
          {
            changeFSMExecState(EXEC_TRAJ, "FSM");

            publishSwarmTrajs(true);
          }
          else
          {
            ROS_ERROR("Failed to generate the first trajectory!!!");
            changeFSMExecState(SEQUENTIAL_START, "FSM");
          }
        }
        else
        {
          ROS_ERROR("No odom or no target! have_odom_=%d, have_target_=%d", have_odom_, have_target_);
        }
      }

      break;
    }

    case GEN_NEW_TRAJ:
    {

      // Eigen::Vector3d rot_x = odom_orient_.toRotationMatrix().block(0, 0, 3, 1);
      // start_yaw_(0)         = atan2(rot_x(1), rot_x(0));
      // start_yaw_(1) = start_yaw_(2) = 0.0;

      bool success = planFromGlobalTraj(10); // zx-todo
      if (success)
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        flag_escape_emergency_ = true;
        publishSwarmTrajs(false);
      }
      else
      {
        ROS_WARN_THROTTLE(1.0, "Failed to generate the first local B-spline; retrying GEN_NEW_TRAJ.");
        changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }
      break;
    }

    case REPLAN_TRAJ:
    {

      if (planFromCurrentTraj(1))
      {
        changeFSMExecState(EXEC_TRAJ, "FSM");
        publishSwarmTrajs(false);
      }
      else
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EXEC_TRAJ:
    {
      /* determine if need to replan */
      LocalTrajData *info = &planner_manager_->local_data_;
      ros::Time time_now = ros::Time::now();
      double t_cur = (time_now - info->start_time_).toSec();
      t_cur = min(info->duration_, t_cur);

      Eigen::Vector3d pos = info->position_traj_.evaluateDeBoorT(t_cur);

      /* && (end_pt_ - pos).norm() < 0.5 */
      if ((target_type_ == TARGET_TYPE::PRESET_TARGET) &&
          (wp_id_ < waypoint_num_ - 1) &&
          (end_pt_ - pos).norm() < no_replan_thresh_)
      {
        wp_id_++;
        planNextWaypoint(wps_[wp_id_]);
      }
      else if ((local_target_pt_ - end_pt_).norm() < 1e-3) // close to the global target
      {
        if (t_cur > info->duration_ - 1e-2)
        {
          const double xy_error = (odom_pos_.head<2>() - end_pt_.head<2>()).norm();
          const double z_error = std::fabs(odom_pos_(2) - end_pt_(2));
          const double xy_speed = odom_vel_.head<2>().norm();
          const double z_speed = std::fabs(odom_vel_(2));
          const bool actual_goal_reached =
              xy_error <= goal_arrival_xy_tolerance_ &&
              z_error <= goal_arrival_z_tolerance_ &&
              xy_speed <= goal_arrival_max_xy_speed_ &&
              z_speed <= goal_arrival_max_z_speed_;

          if (actual_goal_reached)
          {
            have_target_ = false;
            have_trigger_ = false;

            if (target_type_ == TARGET_TYPE::PRESET_TARGET)
            {
              wp_id_ = 0;
              planNextWaypoint(wps_[wp_id_]);
            }

            ROS_WARN("Trajectory finished and actual goal reached: xy=%.3f z=%.3f vxy=%.3f vz=%.3f.",
                     xy_error, z_error, xy_speed, z_speed);
            changeFSMExecState(WAIT_TARGET, "FSM");
            goto force_return;
          }

          ROS_WARN_THROTTLE(
              1.0,
              "Trajectory duration finished; keep publishing terminal hold until the aircraft settles "
              "(xy=%.3f/%.3f z=%.3f/%.3f vxy=%.3f/%.3f vz=%.3f/%.3f).",
              xy_error, goal_arrival_xy_tolerance_,
              z_error, goal_arrival_z_tolerance_,
              xy_speed, goal_arrival_max_xy_speed_,
              z_speed, goal_arrival_max_z_speed_);
        }
        else if ((end_pt_ - pos).norm() > no_replan_thresh_ && t_cur > replan_thresh_)
        {
          changeFSMExecState(REPLAN_TRAJ, "FSM");
        }
      }
      else if (t_cur > replan_thresh_)
      {
        changeFSMExecState(REPLAN_TRAJ, "FSM");
      }

      break;
    }

    case EMERGENCY_STOP:
    {

      if (flag_escape_emergency_) // Avoiding repeated calls
      {
        callEmergencyStop(odom_pos_);
      }
      else
      {
        if (enable_fail_safe_ && odom_vel_.norm() < 0.1)
          changeFSMExecState(GEN_NEW_TRAJ, "FSM");
      }

      flag_escape_emergency_ = false;
      break;
    }
    }

    data_disp_.header.stamp = ros::Time::now();
    data_disp_pub_.publish(data_disp_);

  force_return:;
    exec_timer_.start();
  }

  /*收到全局轨迹后开始进行的 第一步局部规划触发器  trial_times 重试次数*/
  bool EGOReplanFSM::planFromGlobalTraj(const int trial_times /*=1*/) //zx-todo
  {
    start_pt_ = odom_pos_; // 局部轨迹起点位置 = 当前飞机位置
    start_vel_ = odom_vel_;// 局部轨迹起点速度 = 当前飞机速度
    start_acc_.setZero();// 局部轨迹起点加速度 = 0

    const bool retrying_previous_fsm_cycle =
        timesOfConsecutiveStateCalls().first > 1;

    for (int i = 0; i < trial_times; i++)
    {
      // Keep a fresh target deterministic on its first attempt. If that
      // candidate genuinely fails safety validation, make later attempts
      // distinct instead of repeating the same numerical failure ten times.
      const bool flag_random_poly_init =
          retrying_previous_fsm_cycle || i > 0;
      if (flag_random_poly_init && i == 1 && !retrying_previous_fsm_cycle)
      {
        ROS_WARN("Deterministic B-spline candidate failed safety validation; "
                 "enable a randomized obstacle-escape initialization.");
      }

        // 调用 EGO-Planner 核心的 Rebound（反弹）重规划算法
        // 参数 1 (true)：代表这是基于全局轨迹进行的初始化规划
        // 参数 2 (flag_random_poly_init)：是否注入随机扰动
      if (callReboundReplan(true, flag_random_poly_init))
      {
        return true;
      }
    }
    return false;
  }

  bool EGOReplanFSM::planFromCurrentTraj(const int trial_times /*=1*/)
  {

    LocalTrajData *info = &planner_manager_->local_data_;
    ros::Time time_now = ros::Time::now();
    double t_cur = (time_now - info->start_time_).toSec();

    //cout << "info->velocity_traj_=" << info->velocity_traj_.get_control_points() << endl;

    start_pt_ = info->position_traj_.evaluateDeBoorT(t_cur);
    start_vel_ = info->velocity_traj_.evaluateDeBoorT(t_cur);
    start_acc_ = info->acceleration_traj_.evaluateDeBoorT(t_cur);

    bool success = callReboundReplan(false, false);

    if (!success)
    {
      success = callReboundReplan(true, false);
      //changeFSMExecState(EXEC_TRAJ, "FSM");
      if (!success)
      {
        for (int i = 0; i < trial_times; i++)
        {
          success = callReboundReplan(true, true);
          if (success)
            break;
        }
        if (!success)
        {
          return false;
        }
      }
    }

    return true;
  }
  /*突然出现障碍物的时候触发 */
  void EGOReplanFSM::checkCollisionCallback(const ros::TimerEvent &e)
  {
    if (!planning_enabled_)
      return;

    LocalTrajData *info = &planner_manager_->local_data_;
    auto map = planner_manager_->grid_map_;

    if (exec_state_ == WAIT_TARGET || info->start_time_.toSec() < 1e-5)
      return;

    /* ---------- check lost of depth ---------- */
    if (map->getOdomDepthTimeout())
    {
      ROS_ERROR("Depth Lost! EMERGENCY_STOP");
      enable_fail_safe_ = false;
      changeFSMExecState(EMERGENCY_STOP, "SAFETY");
    }

    /* ---------- check trajectory ---------- */
    constexpr double time_step = 0.01;
    double t_cur = (ros::Time::now() - info->start_time_).toSec();
    Eigen::Vector3d p_cur = info->position_traj_.evaluateDeBoorT(t_cur);
    const double CLEARANCE = 1.0 * planner_manager_->getSwarmClearance();
    double t_cur_global = ros::Time::now().toSec();
    double t_2_3 = info->duration_ * 2 / 3;
    for (double t = t_cur; t < info->duration_; t += time_step)
    {
      if (t_cur < t_2_3 && t >= t_2_3) // If t_cur < t_2_3, only the first 2/3 partition of the trajectory is considered valid and will get checked.
        break;

      bool occ = false;
      occ |= map->getInflateOccupancy(info->position_traj_.evaluateDeBoorT(t));

      for (size_t id = 0; id < planner_manager_->swarm_trajs_buf_.size(); id++)
      {
        if ((planner_manager_->swarm_trajs_buf_.at(id).drone_id != (int)id) || (planner_manager_->swarm_trajs_buf_.at(id).drone_id == planner_manager_->pp_.drone_id))
        {
          continue;
        }

        double t_X = t_cur_global - planner_manager_->swarm_trajs_buf_.at(id).start_time_.toSec();
        Eigen::Vector3d swarm_pridicted = planner_manager_->swarm_trajs_buf_.at(id).position_traj_.evaluateDeBoorT(t_X);
        double dist = (p_cur - swarm_pridicted).norm();

        if (dist < CLEARANCE)
        {
          occ = true;
          break;
        }
      }

      if (occ)
      {

        if (planFromCurrentTraj()) // Make a chance
        {
          changeFSMExecState(EXEC_TRAJ, "SAFETY");
          publishSwarmTrajs(false);
          return;
        }
        else
        {
          if (t - t_cur < emergency_time_) // 0.8s of emergency time
          {
            ROS_WARN("Suddenly discovered obstacles. emergency stop! time=%f", t - t_cur);
            changeFSMExecState(EMERGENCY_STOP, "SAFETY");
          }
          else
          {
            //ROS_WARN("current traj in collision, replan.");
            changeFSMExecState(REPLAN_TRAJ, "SAFETY");
          }
          return;
        }
        break;
      }
    }
  }
  /*
    flag_use_poly_init  true 全新初始化 生成新路线
    flag_randomPolyTraj true 强行叠加高斯随机噪声 梯度下降算法在对称的障碍物面前会陷入“不知道该往左推还是往右推”的死锁。
  */
  bool EGOReplanFSM::callReboundReplan(bool flag_use_poly_init, bool flag_randomPolyTraj)
  {

    getLocalTarget();//计算局部目标点

    bool plan_and_refine_success =
    planner_manager_->reboundReplan(
        start_pt_,//本次局部规划起点
        start_vel_,//起点速度
        start_acc_,//起点加速度
        local_target_pt_,//局部目标T
        local_target_vel_,//到达T时的期望速度
        (have_new_target_ || flag_use_poly_init),//是否重新生成初始多项式
        flag_randomPolyTraj);//是否使用随机初始轨迹

    have_new_target_ = false;

    cout << "refine_success=" << plan_and_refine_success << endl;

    if (plan_and_refine_success)
    {

      auto info = &planner_manager_->local_data_;

      traj_utils::Bspline bspline;
      bspline.order = 3;
      bspline.start_time = info->start_time_;
      bspline.traj_id = info->traj_id_;

      Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
      bspline.pos_pts.reserve(pos_pts.cols());
      for (int i = 0; i < pos_pts.cols(); ++i)
      {
        geometry_msgs::Point pt;
        pt.x = pos_pts(0, i);
        pt.y = pos_pts(1, i);
        pt.z = pos_pts(2, i);
        bspline.pos_pts.push_back(pt);
      }

      Eigen::VectorXd knots = info->position_traj_.getKnot();
      // cout << knots.transpose() << endl;
      bspline.knots.reserve(knots.rows());
      for (int i = 0; i < knots.rows(); ++i)
      {
        bspline.knots.push_back(knots(i));
      }

      /* 1. publish traj to traj_server */
      bspline_pub_.publish(bspline);

      /* 2. publish traj to the next drone of swarm */

      /* 3. publish traj for visualization */
      visualization_->displayOptimalList(info->position_traj_.get_control_points(), 0);
    }

    return plan_and_refine_success;
  }

  void EGOReplanFSM::publishSwarmTrajs(bool startup_pub)
  {
    auto info = &planner_manager_->local_data_;

    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.drone_id = planner_manager_->pp_.drone_id;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    // cout << knots.transpose() << endl;
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    if (startup_pub)
    {
      multi_bspline_msgs_buf_.drone_id_from = planner_manager_->pp_.drone_id; // zx-todo
      if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id + 1)
      {
        multi_bspline_msgs_buf_.traj.back() = bspline;
      }
      else if ((int)multi_bspline_msgs_buf_.traj.size() == planner_manager_->pp_.drone_id)
      {
        multi_bspline_msgs_buf_.traj.push_back(bspline);
      }
      else
      {
        ROS_ERROR("Wrong traj nums and drone_id pair!!! traj.size()=%d, drone_id=%d", (int)multi_bspline_msgs_buf_.traj.size(), planner_manager_->pp_.drone_id);
        // return plan_and_refine_success;
      }
      swarm_trajs_pub_.publish(multi_bspline_msgs_buf_);
    }

    broadcast_bspline_pub_.publish(bspline);
  }

  bool EGOReplanFSM::callEmergencyStop(Eigen::Vector3d stop_pos)
  {

    planner_manager_->EmergencyStop(stop_pos);

    auto info = &planner_manager_->local_data_;

    /* publish traj */
    traj_utils::Bspline bspline;
    bspline.order = 3;
    bspline.start_time = info->start_time_;
    bspline.traj_id = info->traj_id_;

    Eigen::MatrixXd pos_pts = info->position_traj_.getControlPoint();
    bspline.pos_pts.reserve(pos_pts.cols());
    for (int i = 0; i < pos_pts.cols(); ++i)
    {
      geometry_msgs::Point pt;
      pt.x = pos_pts(0, i);
      pt.y = pos_pts(1, i);
      pt.z = pos_pts(2, i);
      bspline.pos_pts.push_back(pt);
    }

    Eigen::VectorXd knots = info->position_traj_.getKnot();
    bspline.knots.reserve(knots.rows());
    for (int i = 0; i < knots.rows(); ++i)
    {
      bspline.knots.push_back(knots(i));
    }

    bspline_pub_.publish(bspline);

    return true;
  }


  /*计算局部目标点  这里 空间前瞻（planning_horizen_米）与时间前瞻（3秒）里面取最小值
 
    从飞机当前位置出发，沿着全局轨迹往前走，
    只要【距离超过5米】或【时间超过3秒】或【撞到大终点】，就立刻停下，把这个点的
    (X，Y,Z)和期望速度(Va,Vy,Vz)切出来，交给局部优化器去避障! 
  */
  void EGOReplanFSM::getLocalTarget()
  {
    double t;

    /*
      planning_horizen_ 是局部规划的距离视野（比如 5 米）。
      max_vel_ 是无人机的最大速度。
      planning_horizen_ / max_vel_ 代表以最大速度飞完这个视野所需的时间。
      再除以 20，意味着将这个视野对应的时间切分成 20 等份。
      这样可以在兼顾计算效率的同时，以足够精细的密度去采样轨迹上的点
    */
    double t_step = planning_horizen_ / 20 / planner_manager_->pp_.max_vel_;
    

    /*
      dist_min 用于找到的离飞机最近的距离 ，先设一个大值然后做比较
      dist_min_t 这个距离最近的点，发生在全局轨迹第几秒
    */
    double dist_min = 9999, dist_min_t = 0.0;
    /*搜索起点：从 last_progress_time_（上一次记录的全局轨迹进度时间）开始，
      而不是从 0 开始。这保证了无人机只会向前看，不会倒退。
      时间不能超过全局轨迹总时长 global_duration_
      每次往后走 t_step秒
    */
    for (t = planner_manager_->global_data_.last_progress_time_; t < planner_manager_->global_data_.global_duration_; t += t_step)
    {
      //  将时间 t 映射为三维空间坐标 (时空转换)
      Eigen::Vector3d pos_t = planner_manager_->global_data_.getPosition(t);
      //计算该坐标到无人机的直线几何距离 (空间几何域) 
      double dist = (pos_t - start_pt_).norm();

      /*
      条件
         t < planner_manager_->global_data_.last_progress_time_ + 1e-5
         判断是否是第一次循环（起点）
         dist > planning_horizen_
         是否规划距离大于视野

      起点距离就已经超过视野了！不能直接按正常逻辑跳出循环，必须赶紧执行里面的修复代码
      正常情况不会运行 坐标突然被偏移则会运行，比如说遇到风，或者说坐标漂移
      */
      if (t < planner_manager_->global_data_.last_progress_time_ + 1e-5 && dist > planning_horizen_)
      {
        // Important conor case!
        for (; t < planner_manager_->global_data_.global_duration_; t += t_step)/*小于总时长就快进*/
        {
          Eigen::Vector3d pos_t_temp = planner_manager_->global_data_.getPosition(t);
          double dist_temp = (pos_t_temp - start_pt_).norm();/*计算起始点的距离*/
          if (dist_temp < planning_horizen_)/*小于视野 则修复成功*/
          {
            pos_t = pos_t_temp;
            dist = (pos_t - start_pt_).norm();
            cout << "Escape conor case \"getLocalTarget\"" << endl;
            break;
          }
        }
      }
 
      if (dist < dist_min)/*记录最近的全局进度  一般都是 较远 → 越来越近 → 最近点 → 越来越远*/
      {
        dist_min = dist;
        dist_min_t = t;
      }
      /*距离大于等于视野 这次视野的规划跑完了local_target_pt_ 目标点就是 pos_t*/
      if (dist >= planning_horizen_) 
      {
        local_target_pt_ = pos_t;
        planner_manager_->global_data_.last_progress_time_ = dist_min_t;
        break;
      }
    }

    if (t > planner_manager_->global_data_.global_duration_) // Last global point
    {
      local_target_pt_ = end_pt_;
      planner_manager_->global_data_.last_progress_time_ = planner_manager_->global_data_.global_duration_;
    }
    /*局部目标T 到 最终目标G 的直线距离 < 刹车距离  这里在局部目标的速度就应该是0*/
    if ((end_pt_ - local_target_pt_).norm() < (planner_manager_->pp_.max_vel_ * planner_manager_->pp_.max_vel_) / (2 * planner_manager_->pp_.max_acc_))
    {
      local_target_vel_ = Eigen::Vector3d::Zero();
    }
    else
    {
      /*离得远就保持全局轨迹的速度前进了*/
      local_target_vel_ = planner_manager_->global_data_.getVelocity(t);
    }
  }

} // namespace ego_planner
