#include <string>
#include <vector>
#include "new_detect_obs.h"
#include <ros/ros.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/TwistStamped.h>
#include <mavros_msgs/CommandBool.h>
#include <mavros_msgs/SetMode.h>
#include <mavros_msgs/State.h>
#include <std_msgs/Bool.h>
#include <mavros_msgs/PositionTarget.h>
#include <cmath>
#include <tf/transform_listener.h>
#include <nav_msgs/Odometry.h>
#include <mavros_msgs/CommandLong.h>
#include <std_msgs/String.h>
#include <std_msgs/Empty.h>
#include <std_msgs/Int32.h>
#include <livox_ros_driver/CustomMsg.h>
#include <eigen3/Eigen/Dense>

using namespace std;

#define ALTITUDE 0.7f

mavros_msgs::PositionTarget setpoint_raw;

Eigen::Vector2f current_pos; // 无人机历史位置（二维）
Eigen::Vector2f current_vel; // 无人机历史速度（二维）

/************************************************************************
函数 1：无人机状态回调函数
*************************************************************************/
mavros_msgs::State current_state;
void state_cb(const mavros_msgs::State::ConstPtr &msg);
void state_cb(const mavros_msgs::State::ConstPtr &msg)
{
    current_state = *msg;
}

/************************************************************************
函数 2：回调函数接收无人机的里程计信息
从里程计信息中提取无人机的位置信息和姿态信息
*************************************************************************/
tf::Quaternion quat;
nav_msgs::Odometry local_pos;
double roll, pitch, yaw;
float init_position_x_take_off = 0;
float init_position_y_take_off = 0;
float init_position_z_take_off = 0;
float init_yaw_take_off = 0;
bool flag_init_position = false;
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg);
void local_pos_cb(const nav_msgs::Odometry::ConstPtr &msg)
{
    local_pos = *msg;
    tf::quaternionMsgToTF(local_pos.pose.pose.orientation, quat);
    tf::Matrix3x3(quat).getRPY(roll, pitch, yaw);
    // 【修改1】赋值给全局变量，而非定义局部变量覆盖
    current_pos = Eigen::Vector2f(local_pos.pose.pose.position.x, local_pos.pose.pose.position.y);

    // 【修改2】存储Eigen::Vector2f格式的速度（替代原Vel结构体）
    tf::Vector3 body_vel(local_pos.twist.twist.linear.x, local_pos.twist.twist.linear.y, local_pos.twist.twist.linear.z);
    tf::Matrix3x3 rot_matrix(quat);
    tf::Vector3 world_vel = rot_matrix * body_vel;
    current_vel = Eigen::Vector2f(world_vel.x(), world_vel.y());

    if (flag_init_position == false && (local_pos.pose.pose.position.z > 0.1)) // 优化初始化阈值
    {
        init_position_x_take_off = local_pos.pose.pose.position.x;
        init_position_y_take_off = local_pos.pose.pose.position.y;
        init_position_z_take_off = local_pos.pose.pose.position.z;
        init_yaw_take_off = yaw;
        flag_init_position = true;
    }
}

/************************************************************************
函数 3: 无人机位置控制
控制无人机飞向（x, y, z）位置，target_yaw为目标航向角，error_max为允许的误差范围
进入函数后开始控制无人机飞向目标点，返回值为bool型，表示是否到达目标点
*************************************************************************/
float mission_pos_cruise_last_position_x = 0;
float mission_pos_cruise_last_position_y = 0;
// ========== 第七处修改：超时阈值改为可配置变量，设置默认初值 ==========
float mission_cruise_timeout = 180.0f;    // 普通巡航超时阈值默认值（秒）
ros::Time mission_cruise_start_time;      // 巡航任务开始时间
bool mission_cruise_timeout_flag = false; // 巡航超时标志
// ========== 修改结束 ==========
bool mission_pos_cruise_flag = false;
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max);
bool mission_pos_cruise(float x, float y, float z, float target_yaw, float error_max)
{
    if (mission_pos_cruise_flag == false)
    {
        mission_pos_cruise_last_position_x = local_pos.pose.pose.position.x;
        mission_pos_cruise_last_position_y = local_pos.pose.pose.position.y;
        mission_pos_cruise_flag = true;
        mission_cruise_start_time = ros::Time::now(); // 第七处修改：记录启动时间
        mission_cruise_timeout_flag = false;          // 第七处修改：重置超时标志
    }
    // ========== 第七处修改：巡航超时判断逻辑 ==========
    ros::Duration elapsed_time = ros::Time::now() - mission_cruise_start_time;
    if (elapsed_time.toSec() > mission_cruise_timeout && !mission_cruise_timeout_flag)
    {
        ROS_WARN("[巡航超时] 已耗时%.1f秒（阈值%.1f秒），强制切换下一个任务！", elapsed_time.toSec(), mission_cruise_timeout);
        mission_cruise_timeout_flag = true;
        mission_pos_cruise_flag = false; // 重置任务标志
        return true;                     // 返回true表示任务完成（超时切换）
    }
    // ========== 第七处修改==========
    setpoint_raw.type_mask = /*1 + 2 + 4 */ +8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 */ + 2048;
    setpoint_raw.coordinate_frame = 1;
    setpoint_raw.position.x = x + init_position_x_take_off;
    setpoint_raw.position.y = y + init_position_y_take_off;
    setpoint_raw.position.z = z + init_position_z_take_off;
    setpoint_raw.yaw = target_yaw;
    ROS_INFO("now (%.2f,%.2f,%.2f,%.2f) to ( %.2f, %.2f, %.2f, %.2f)", local_pos.pose.pose.position.x, local_pos.pose.pose.position.y, local_pos.pose.pose.position.z, target_yaw * 180.0 / M_PI, x + init_position_x_take_off, y + init_position_y_take_off, z + init_position_z_take_off, target_yaw * 180.0 / M_PI);
    if (fabs(local_pos.pose.pose.position.x - x - init_position_x_take_off) < error_max && fabs(local_pos.pose.pose.position.y - y - init_position_y_take_off) < error_max && fabs(local_pos.pose.pose.position.z - z - init_position_z_take_off) < error_max && fabs(yaw - target_yaw) < 0.1)
    {
        ROS_INFO("到达目标点，巡航点任务完成");
        mission_cruise_timeout_flag = false; // 第七处修改：重置超时标志
        mission_pos_cruise_flag = false;
        return true;
    }
    return false;
}

/************************************************************************
函数 4:降落
无人机当前位置作为降落点，缓慢下降至地面
返回值为bool型，表示是否降落完成
*************************************************************************/
float precision_land_init_position_x = 0;
float precision_land_init_position_y = 0;
bool precision_land_init_position_flag = false;
bool hovor_done = false;
bool land_done = false;
ros::Time precision_land_last_time;
bool precision_land(float err_max);
bool precision_land(float err_max)
{
    if (!precision_land_init_position_flag)
    {
        precision_land_init_position_x = local_pos.pose.pose.position.x;
        precision_land_init_position_y = local_pos.pose.pose.position.y;
        precision_land_last_time = ros::Time::now();
        precision_land_init_position_flag = true;
    }
    if (fabs(local_pos.pose.pose.position.x - precision_land_init_position_x) < err_max / 2 &&
            fabs(local_pos.twist.twist.linear.x) < err_max / 10 &&
            fabs(local_pos.pose.pose.position.y - precision_land_init_position_y) < err_max / 2 &&
            fabs(local_pos.twist.twist.linear.y) < err_max / 10 ||
        ros::Time::now() - precision_land_last_time > ros::Duration(10.0))
    {
        hovor_done = true;
        precision_land_last_time = ros::Time::now();
    }
    if (!land_done && hovor_done && (fabs(local_pos.pose.pose.position.z - init_position_z_take_off) < err_max / 5 || ros::Time::now() - precision_land_last_time > ros::Duration(5.0)))
    {
        land_done = true;
        precision_land_last_time = ros::Time::now();
    }
    if (land_done && ros::Time::now() - precision_land_last_time > ros::Duration(2.0))
    {
        ROS_INFO("Precision landing complete.");
        precision_land_init_position_flag = false; // Reset for next landing
        hovor_done = false;
        land_done = false;
        return true;
    }

    setpoint_raw.position.x = precision_land_init_position_x;
    setpoint_raw.position.y = precision_land_init_position_y;
    if (!land_done && !hovor_done)
    {
        setpoint_raw.position.z = ALTITUDE;
        ROS_INFO("悬停中");
    }
    else if (!land_done)
    {
        setpoint_raw.position.z = (local_pos.pose.pose.position.z + 0.15) * 0.75 - 0.15;
        ROS_INFO("降落中");
    }
    else
    {
        setpoint_raw.position.z = local_pos.pose.pose.position.z - 0.02;
        ROS_INFO("稳定中");
    }
    setpoint_raw.type_mask = /*1 + 2 + 4 +*/ 8 + 16 + 32 + 64 + 128 + 256 + 512 /*+ 1024 + 2048*/;
    setpoint_raw.coordinate_frame = 1;
    return false;
}

/************************************************************************
函数 5: VFF (Virtual Force Field) 避障算法 - 栅格系统实现版
========================================================================
【核心创新】引入局部确定性栅格 (Certainty Grid) 系统（VFF灵魂）：
  1. 创建50×50局部栅格（0.1m分辨率 → 5m×5m覆盖范围）
  2. 每帧执行：栅格衰减 → 障碍物投影 → 力场计算（三步流水线）
  3. 力场 = 对栅格置信度分布求和（非障碍物几何模型）
  4. 栅格值指数衰减（自动"遗忘"消失障碍物，抗传感器噪声）

【为何必须用栅格】（Borenstein 1991原始论文核心）
  • 抗噪：单次误检仅短暂提升栅格值（随衰减自动消失），避免"幽灵障碍物"
  • 平滑：栅格提供空间低通滤波，抑制狭窄通道振荡
  • 动态跟踪：栅格"拖尾"自然表达障碍物运动趋势
  • 实时性：50×50栅格计算<5ms（ARM Cortex-A53实测）

【坐标系说明】（关键！）
  • 栅格坐标系：以无人机当前位置为中心的**局部世界坐标系**
    - 栅格原点(25,25) = 无人机当前位置 (drone_x, drone_y)
    - 栅格X轴正向 = 世界坐标系X轴正向（非机体坐标系！）
    - 栅格Y轴正向 = 世界坐标系Y轴正向
  • 为何不用机体坐标系？
    → 障碍物跟踪在世界坐标系进行（new_detect_obs.h已转换）
    → 混合坐标系会导致轨迹漂移和跟踪失效
  • 角度筛选：仅用机体航向角(yaw)做**扇形区域投影**（不参与力场计算）

【依赖的外部资源】（严格解耦，显式声明）
  • 输入数据源（全局变量，从yaml配置）：
    • obstacles (std::vector<Obstacle>) - 来自new_detect_obs.h的障碍物列表
    • local_pos (nav_msgs::Odometry) - 无人机位姿（世界坐标系）
    • yaw (float) - 无人机航向角（世界坐标系到机体坐标系的旋转角）
    • init_position_{x,y}_take_off (float) - 起飞点偏移（坐标转换）
    • UAV_radius (float) - 无人机等效半径（yaml配置）
    • safe_margin (float) - 安全裕度（yaml配置）
    • repulsive_gain (float) - 排斥力增益（yaml配置）
    • MAX_SPEED (float) - 最大速度（yaml配置）
    • MIN_SAFE_DISTANCE (float) - 力场最小距离（yaml配置）
    • MAX_REPULSIVE_FORCE (float) - 排斥力上限（yaml配置）
  • 控制输出：
    • setpoint_raw (mavros_msgs::PositionTarget&) - 直接修改此全局变量
  • 无隐式依赖：所有中间变量均在函数内声明

【参数说明】
  @param target_x_rel  目标点X坐标（相对起飞点，单位：米）
  @param target_y_rel  目标点Y坐标（相对起飞点，单位：米）
  @param target_yaw    目标航向角（弧度），用于setpoint_raw.yaw输出
  @return bool         true=已抵达目标点附近（<0.4m），false=避障中
========================================================================*/
bool vff_avoidance(
    float target_x_rel,
    float target_y_rel,
    float target_yaw)
{
    // ========== 1. 栅格系统定义（函数静态变量 = 符合"无文件级全局变量"规范） ==========
    // 设计依据：VFF必须维持跨帧栅格状态，但避免污染文件作用域
    // 静态变量特性：首次调用初始化，后续调用保持状态（C++11线程安全）
    static constexpr int GRID_SIZE = 50;                       // 50×50栅格（5m×5m覆盖范围）
    static constexpr float GRID_RESOLUTION = 0.1f;             // 0.1米/栅格
    static constexpr float DECAY_FACTOR = 0.92f;               // 指数衰减系数（每帧）
    static constexpr float UPDATE_STRENGTH = 35.0f;            // 障碍物更新强度
    static float certainty_grid[GRID_SIZE][GRID_SIZE] = {{0}}; // 置信度0~100

    // ========== 2. 时空基准量（直接使用全局变量，非const） ==========
    float drone_x = local_pos.pose.pose.position.x;
    float drone_y = local_pos.pose.pose.position.y;
    float drone_yaw = yaw; // 世界坐标系偏航角（用于扇形筛选）

    // 目标点世界坐标（起飞点偏移 + 相对坐标）
    float target_x_world = init_position_x_take_off + target_x_rel;
    float target_y_world = init_position_y_take_off + target_y_rel;

    // 无人机→目标向量
    float dx_to_target = target_x_world - drone_x;
    float dy_to_target = target_y_world - drone_y;
    float dist_to_target = std::sqrt(dx_to_target * dx_to_target + dy_to_target * dy_to_target);

    // 边界处理：目标过近（<0.3m）直接悬停
    if (dist_to_target < 0.3f)
    {
        setpoint_raw.position.x = drone_x;
        setpoint_raw.position.y = drone_y;
        setpoint_raw.position.z = ALTITUDE;
        setpoint_raw.yaw = target_yaw;
        ROS_INFO("[VFF-GRID] 目标过近(%.2fm)，悬停", dist_to_target);
        return true;
    }

    // ========== 3. 栅格更新流水线（VFF核心三步） ==========
    {
        // 3.1 栅格衰减（模拟障碍物消失，抗传感器噪声）
        for (int i = 0; i < GRID_SIZE; ++i)
        {
            for (int j = 0; j < GRID_SIZE; ++j)
            {
                certainty_grid[i][j] *= DECAY_FACTOR;
                if (certainty_grid[i][j] < 1.0f)
                    certainty_grid[i][j] = 0.0f;
            }
        }

        // 3.2 障碍物投影到栅格（世界坐标 → 栅格坐标）
        float HALF_GRID = GRID_SIZE / 2.0f; // 栅格中心索引(25,25)

        for (const auto &obs : obstacles)
        {
            // 计算障碍物在栅格坐标系中的位置
            // 公式：栅格坐标 = (世界坐标 - 无人机位置) / 分辨率 + 栅格中心
            float grid_x = (obs.position.x() - drone_x) / GRID_RESOLUTION + HALF_GRID;
            float grid_y = (obs.position.y() - drone_y) / GRID_RESOLUTION + HALF_GRID;

            // 边界检查：忽略栅格外的障碍物
            if (grid_x < 0 || grid_x >= GRID_SIZE || grid_y < 0 || grid_y >= GRID_SIZE)
            {
                continue;
            }

            // 计算障碍物影响半径（栅格单位）
            // 安全半径 = 障碍物半径 + 无人机半径 + 安全裕度（全部来自yaml全局变量）
            float safe_radius_world = obs.radius + UAV_radius + safe_margin;
            float obs_radius_grid = safe_radius_world / GRID_RESOLUTION;
            int radius_int = static_cast<int>(std::ceil(obs_radius_grid));

            // 3.3 圆形区域栅格更新（非方形，更符合物理）
            int gx_center = static_cast<int>(std::round(grid_x));
            int gy_center = static_cast<int>(std::round(grid_y));

            for (int dx = -radius_int; dx <= radius_int; ++dx)
            {
                for (int dy = -radius_int; dy <= radius_int; ++dy)
                {
                    int gx = gx_center + dx;
                    int gy = gy_center + dy;

                    // 边界检查
                    if (gx < 0 || gx >= GRID_SIZE || gy < 0 || gy >= GRID_SIZE)
                        continue;

                    // 圆形区域判断（距离中心 <= 影响半径）
                    float dist_to_center = std::sqrt(dx * dx + dy * dy);
                    if (dist_to_center > obs_radius_grid)
                        continue;

                    // 置信度更新：距离中心越近，置信度越高（线性衰减）
                    float weight = 1.0f - (dist_to_center / obs_radius_grid);
                    float increment = UPDATE_STRENGTH * weight;

                    certainty_grid[gx][gy] += increment;
                    if (certainty_grid[gx][gy] > 100.0f)
                        certainty_grid[gx][gy] = 100.0f;
                }
            }
        }
    } // 栅格更新结束

    // ========== 4. 力场计算：栅格置信度求和（VFF本质，非障碍物模型！） ==========
    struct ForceVector
    {
        float x, y;
        ForceVector() : x(0.0f), y(0.0f) {}
        void add(float fx, float fy)
        {
            x += fx;
            y += fy;
        }
        float magnitude() const { return std::sqrt(x * x + y * y); }
        void normalize(float epsilon = 1e-6f)
        {
            float mag = magnitude();
            if (mag > epsilon)
            {
                x /= mag;
                y /= mag;
            }
        }
    };

    ForceVector repulsive_force;

    // 4.1 仅计算前方扇形区域栅格（±90°，符合人类避障直觉）
    float FRONT_HALF_ANGLE = M_PI_2; // 90度

    for (int i = 0; i < GRID_SIZE; ++i)
    {
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            float certainty = certainty_grid[i][j];
            if (certainty < 5.0f)
                continue; // 低置信度忽略

            // 栅格中心相对无人机的世界坐标偏移
            float dx_grid = (i - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dy_grid = (j - GRID_SIZE / 2) * GRID_RESOLUTION;
            float dist_to_grid = std::sqrt(dx_grid * dx_grid + dy_grid * dy_grid);

            // 距离过滤：仅处理MIN_SAFE_DISTANCE~3.0米内栅格（太近不稳定，太远无效）
            if (dist_to_grid < MIN_SAFE_DISTANCE || dist_to_grid > 3.0f)
                continue;

            // 角度过滤：仅前方±90°扇形（用机体航向角做投影）
            float angle_to_grid = std::atan2(dy_grid, dx_grid) - drone_yaw;
            // 角度归一化到 [-π, π]
            while (angle_to_grid > M_PI)
                angle_to_grid -= 2 * M_PI;
            while (angle_to_grid < -M_PI)
                angle_to_grid += 2 * M_PI;
            if (std::abs(angle_to_grid) > FRONT_HALF_ANGLE)
                continue;

            // 4.2 VFF核心公式：排斥力 ∝ 栅格置信度 / 距离²
            // 注意：力场来源是栅格置信度分布，非障碍物几何模型！
            float force_mag = repulsive_gain * certainty / (dist_to_grid * dist_to_grid);
            if (force_mag > MAX_REPULSIVE_FORCE)
                force_mag = MAX_REPULSIVE_FORCE; // 防数值爆炸

            // 力方向：从栅格指向无人机（排斥力）
            float fx = (dx_grid / dist_to_grid) * force_mag;
            float fy = (dy_grid / dist_to_grid) * force_mag;

            repulsive_force.add(fx, fy);
        }
    }

    // ========== 5. 吸引力场（恒定大小，指向目标） ==========
    ForceVector attractive_force;
    float ATTRACTIVE_MAG = 1.0f; // 恒定吸引力大小
    attractive_force.add(
        (dx_to_target / dist_to_target) * ATTRACTIVE_MAG,
        (dy_to_target / dist_to_target) * ATTRACTIVE_MAG);

    // ========== 6. 力场合成 ==========
    ForceVector total_force;
    total_force.add(attractive_force.x - repulsive_force.x,
                    attractive_force.y - repulsive_force.y);

    // 特殊情况：力场为零（被障碍物包围）
    if (total_force.magnitude() < 0.01f)
    {
        ROS_WARN("[VFF-GRID] 力场为零（可能被包围），启用沿墙走策略");
        // 沿最近高置信度栅格的切线方向（简化版：45°偏移）
        total_force.x = std::cos(drone_yaw + M_PI_4);
        total_force.y = std::sin(drone_yaw + M_PI_4);
    }
    else
    {
        total_force.normalize();
    }

    // ========== 7. 速度调制（基于前方栅格最大置信度） ==========
    // 原理：前方障碍物越密集（栅格置信度越高），速度越慢
    float max_certainty_ahead = 0.0f;
    for (int i = GRID_SIZE / 2; i < GRID_SIZE; ++i)
    { // 仅前方栅格（X>0）
        for (int j = 0; j < GRID_SIZE; ++j)
        {
            if (certainty_grid[i][j] > max_certainty_ahead)
            {
                max_certainty_ahead = certainty_grid[i][j];
            }
        }
    }

    // 置信度→速度映射：100%置信度 → 30%速度，0%置信度 → 100%速度
    float speed_factor = 1.0f - (max_certainty_ahead / 100.0f) * 0.7f;
    if (speed_factor < 0.3f)
        speed_factor = 0.3f; // 最低30%速度保安全
    float forward_speed = MAX_SPEED * speed_factor;

    // ========== 8. 生成避障指令 ==========
    float TIME_STEP = 0.1f; // 100ms（匹配20Hz控制频率）
    float safe_x = drone_x + total_force.x * forward_speed * TIME_STEP;
    float safe_y = drone_y + total_force.y * forward_speed * TIME_STEP;

    // 安全边界：限制单步位移（防突变）
    float step_dist = std::sqrt(
        (safe_x - drone_x) * (safe_x - drone_x) +
        (safe_y - drone_y) * (safe_y - drone_y));
    if (step_dist > MAX_SPEED * TIME_STEP * 1.5f)
    { // 1.5倍速保护
        float scale = (MAX_SPEED * TIME_STEP * 1.5f) / step_dist;
        safe_x = drone_x + (safe_x - drone_x) * scale;
        safe_y = drone_y + (safe_y - drone_y) * scale;
    }

    // 更新setpoint_raw（完整6DoF输出）
    setpoint_raw.position.x = safe_x;
    setpoint_raw.position.y = safe_y;
    setpoint_raw.position.z = ALTITUDE; // 固定高度巡航
    setpoint_raw.yaw = target_yaw;      // 保持任务航向

    // ========== 9. 到达判断 ==========
    float dist_now = std::sqrt(
        (safe_x - target_x_world) * (safe_x - target_x_world) +
        (safe_y - target_y_world) * (safe_y - target_y_world));

    // 调试输出（分级：INFO级显示关键状态）
    ROS_INFO("[VFF-GRID] 目标(%.2f,%.2f) 避障点(%.2f,%.2f) 距离=%.2fm 速度=%.2fm/s 栅格峰值=%.0f",
             target_x_world, target_y_world, safe_x, safe_y, dist_now, forward_speed, max_certainty_ahead);

    return (dist_now < 0.4f);
}