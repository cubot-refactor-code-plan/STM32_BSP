/**
 * @file quaternion.hpp
 * @author ChoseB
 * @brief 姿态四元数运算：乘法 / 共轭、旋转向量、姿态误差、角速度积分、插值、指向角与欧拉角、角度折回
 * @version 0.1
 * @date 2026-10-30
 *
 * @copyright Copyright (c) 2026
 *
 * @details 用四元数做姿态计算与控制，欧拉角只在最后取需要的量或做显示，避开 pitch = ±90° 的万向节锁。
 *          纯计算，不依赖硬件与 RTOS，任意上下文可调；双精度（与 ImuData、VQF 一致，H7 有双精度 FPU）。
 *
 * @note 约定（与 DeviceBmi088 / VQF 输出一致）：
 *       - 分量顺序 w / x / y / z，Hamilton 乘法，右手系；
 *       - 姿态四元数 q 表示“机体系 → 世界系”的旋转：v_世界 = q ⊗ v_机体 ⊗ q*（即 q.rotate(v_机体)）；
 *       - 角度一律弧度，角速度 rad/s。
 *
 * @note 用法：
 *
 *   // 1) 取姿态：ImuData::quaternion → Quaternion
 *   ImuData d;
 *   imu_bmi088.snapshot(d);
 *   Quaternion q = Quaternion::from_array(d.quaternion);
 *
 *   // 2) 安装旋转：q_mount = 机构系 → 芯片系的固定旋转（例如芯片绕机构 y 轴转了 90°）
 *   const Quaternion q_mount = Quaternion::from_axis_angle({0.0, 1.0, 0.0}, PI_D / 2.0);
 *   Quaternion q_body = q * q_mount;      // 机构系 → 世界系
 *
 *   // 3) 云台：取枪管（机构 x 轴）在世界系的水平角与仰角，不算 roll，只有竖直朝天 / 朝地时奇异
 *   double yaw = 0.0, elevation = 0.0;
 *   q_body.pointing({1.0, 0.0, 0.0}, yaw, elevation);
 *   double yaw_err = angle_diff(yaw_ref, yaw); // 折回 [-π, π]，跨 ±180° 不跳
 *
 *   // 4) 三轴姿态控制：误差 = 从当前姿态转到目标姿态的旋转向量（机体系，走短路径），外环 P → 角速度目标，内环直接比陀螺仪
 *   Vector3 e = q_body.error_to(q_ref);
 *
 *   // 5) 目标姿态按角速度推进（遥控 / 自瞄给角速度时）
 *   q_ref = q_ref.integrate({0.0, 0.0, yaw_rate}, 0.001);
 *
 * @note 输入的四元数应为单位四元数；连续相乘 / 积分后用 normalized() 收回（integrate() 已自带）。
 */

#ifndef __QUATERNION_HPP__
#define __QUATERNION_HPP__

constexpr double PI_D = 3.14159265358979323846; ///< 双精度 π（imu_definition.hpp 的 PI 是 float）

/**
 * @brief 三维向量（聚合类型，可 {x, y, z} 初始化）
 */
struct Vector3
{
  double x; ///< x 分量
  double y; ///< y 分量
  double z; ///< z 分量

  /** @brief 模长 */
  double norm() const;
};

/**
 * @brief 四元数（聚合类型，可 {w, x, y, z} 初始化）
 */
struct Quaternion
{
  double w; ///< 实部
  double x; ///< 虚部 i
  double y; ///< 虚部 j
  double z; ///< 虚部 k

  // ---------------- 构造 ----------------

  /** @brief 单位四元数（不旋转） */
  static Quaternion identity();

  /** @brief 从数组 {w, x, y, z} 构造（ImuData::quaternion、VQF 的输出） */
  static Quaternion from_array(const double q[4]);

  /**
   * @brief 绕轴旋转
   * @param axis  转轴（不必归一化；零向量时返回单位四元数）
   * @param angle 转角，rad，按右手定则
   */
  static Quaternion from_axis_angle(const Vector3 &axis, double angle);

  /**
   * @brief 旋转向量（方向 = 转轴，模长 = 转角 rad）→ 四元数，即 exp(rv / 2)
   * @note rotation_vector() 的逆运算；模长很小时按一阶近似，不会除零。
   */
  static Quaternion from_rotation_vector(const Vector3 &rv);

  /** @brief 写回数组 {w, x, y, z} */
  void to_array(double out[4]) const;

  // ----------------
  // ---------------- 基本运算 ----------------

  /** @brief Hamilton 乘法：(a * b).rotate(v) = a.rotate(b.rotate(v))，先转 b 再转 a */
  Quaternion operator*(const Quaternion &rhs) const;

  /** @brief 共轭（单位四元数的逆） */
  Quaternion conj() const;

  /** @brief 模长 */
  double norm() const;

  /** @brief 归一化（模长为 0 时返回单位四元数） */
  Quaternion normalized() const;

  /** @brief 点积（两姿态的接近程度：|dot| = cos(夹角 / 2)） */
  double dot(const Quaternion &rhs) const;

  // ----------------
  // ---------------- 旋转 ----------------

  /** @brief 旋转向量：机体系 → 世界系（q ⊗ v ⊗ q*） */
  Vector3 rotate(const Vector3 &v) const;

  /** @brief 反向旋转：世界系 → 机体系（q* ⊗ v ⊗ q） */
  Vector3 inverse_rotate(const Vector3 &v) const;

  /**
   * @brief 旋转矩阵（机体系 → 世界系），v_世界 = R · v_机体
   * @param r 行优先 3×3；第 i 列 = 机体第 i 轴在世界系的方向
   * @note 要旋转很多向量时先转成矩阵更省（每个向量 9 次乘法）。
   */
  void to_matrix(double r[3][3]) const;

  /**
   * @brief 旋转向量（方向 = 转轴，模长 = 转角 rad，取 [0, π] 的短路径）：对数映射 2·log(q)
   * @note q 与 -q 是同一姿态，这里先把 w 转成非负再算，结果连续、不会给出绕远路的 2π - θ。
   */
  Vector3 rotation_vector() const;

  // ----------------
  // ---------------- 控制 ----------------

  /**
   * @brief 姿态误差：从本姿态转到 target 的旋转向量，表在本机体系（rad）
   * @return (conj() * target).rotation_vector()；三轴分别交给外环 P，得到机体系角速度目标，内环直接与陀螺仪比较
   * @note 任意姿态下连续，没有欧拉角的奇异点与 ±180° 跳变。
   */
  Vector3 error_to(const Quaternion &target) const;

  /**
   * @brief 按机体系角速度积分 dt 秒：this ⊗ exp(ω·dt / 2)，并归一化
   * @param omega_body 机体系角速度（rad/s，与陀螺仪同系）
   * @param dt         时间步长（s）
   */
  Quaternion integrate(const Vector3 &omega_body, double dt) const;

  /**
   * @brief 球面线性插值：t = 0 得 a，t = 1 得 b，中间按等角速度过渡，走短路径
   * @param t 插值系数，取 [0, 1]（超出时外推）
   */
  static Quaternion slerp(const Quaternion &a, const Quaternion &b, double t);

  // ----------------
  // ---------------- 取角 ----------------

  /**
   * @brief 机体上某轴在世界系的指向：水平角与仰角（云台用，不涉及 roll）
   *
   * @param axis_body 机体系里的轴（如枪管方向 {1, 0, 0}，不必归一化）
   * @param yaw       输出：水平角，rad，[-π, π]，世界系 x → y 为正（绕 z 轴右手）
   * @param elevation 输出：仰角，rad，[-π/2, π/2]，朝世界 z（上）为正
   *
   * @note 唯一奇异点是该轴竖直（朝天 / 朝地），此时 yaw 无意义；云台机械限位一般到不了。
   * @note 符号与 to_euler_zyx() 的 pitch 相反：Z-Y-X 欧拉角里 x 轴朝下转（低头）为正 pitch。
   */
  void pointing(const Vector3 &axis_body, double &yaw, double &elevation) const;

  /**
   * @brief Z-Y-X 欧拉角（先 yaw 绕 z、再 pitch 绕 y、最后 roll 绕 x），rad
   * @param yaw   输出：[-π, π]
   * @param pitch 输出：[-π/2, π/2]
   * @param roll  输出：[-π, π]
   * @warning pitch 接近 ±π/2 时 yaw 与 roll 无法区分（万向节锁）：两者大幅反向摆动，越过后各跳 180°；
   *          只用于显示或确定到不了 ±π/2 的场合，控制用 error_to() / pointing()。
   */
  void to_euler_zyx(double &yaw, double &pitch, double &roll) const;

  // ----------------
};

/**
 * @brief 角度折回 [-π, π]
 * @param rad 任意大小的角度（rad）
 */
double angle_wrap_pi(double rad);

/**
 * @brief 角度差 target - current，折回 [-π, π]（走短路径；跨 ±180° 时不会给出接近 2π 的误差）
 */
double angle_diff(double target, double current);

#endif // __QUATERNION_HPP__
