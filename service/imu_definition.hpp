/**
 * @file imu_definition.hpp
 * @author ChoseB
 * @brief IMU 通用数据定义
 * @version 0.1
 * @date 2026-10-27
 *
 * @note 本文件只保存所有 IMU 都具有的可复制数据，不包含具体通信协议、寄存器、硬件所有权或在线检查。
 *       协议原始值和器件配置由具体 IMU 类型自行定义（如 BMI088 的 Bmi088RawData）。
 * @note 不同器件能填的字段不同：自带姿态解算的器件全部字段都填；只有原始传感器的器件由驱动自己解算
 *       （BMI088 在自动刷新且 Config::attitude 时用 VQF 解算，否则只填 gyro / acc，欧拉角与四元数保持 0、不代表实际姿态）。
 *       温度等并非所有器件都有的数据，放在具体器件自己的数据里。
 */
#ifndef __IMU_DEFINITION_HPP__
#define __IMU_DEFINITION_HPP__

#ifndef PI
#define PI 3.1415926f
#endif  // PI

/**
 * @brief 单个 IMU 的通用数据
 *
 * @note gyro / acc 的轴与器件坐标的对应关系由具体器件说明（器件驱动给芯片本体坐标，安装方向的旋转由上层处理）。
 */
struct ImuData
{
  struct Gyro{
    double yaw;
    double pitch;
    double roll;
  }gyro;  // 角速度，单位：rad/s

  struct Acceleration{
    double x,y,z;
  }acc; // 加速度，单位：m/s²

  /**
   * @brief 欧拉角，单位：度
   */
  struct EulerAngle
  {
    double yaw;   ///< 偏航角
    double pitch; ///< 俯仰角
    double roll;  ///< 横滚角
  } euler_angle;

  /**
   * @brief 欧拉角，单位：rad
   */
  struct EulerRadian
  {
    double yaw;   ///< 偏航角
    double pitch; ///< 俯仰角
    double roll;  ///< 横滚角
  } euler_radian;

  double quaternion[4]; ///< 姿态四元数，顺序 w / x / y / z
};

#endif // __IMU_DEFINITION_HPP__
