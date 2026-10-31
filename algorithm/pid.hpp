/**
 * @file pid.hpp
 * @author ChoseB (ChoseB@cumt.edu.cn)
 * @brief PID算法的封装
 * @version 0.2
 * @date 2026-03-07
 *
 * @copyright Copyright (c) 2026
 *
 * @note 使用示例
 *
 * @note 实例化（链式配置，参数含义一目了然）
 *
 *      // 只给系数，其余全默认
 *      PID motor1;
 *      motor1.kp(0.8).ki(0).kd(0);
 *
 *      // 全参数链式配置
 *      PID motor2;
 *      motor2.kp(0.8).ki(1).kd(1)
 *            .limit_output(10000).limit_p(3000)
 *            .limit_d(8000).integral_sep(50);
 *
 *      // 用配置结构体一次性配置（匿名按序传入，参数顺序 = 字段顺序）
 *      // kp, ki, kd, out_max, p_max, i_max, d_max, f_max, i_sep, diff_mode
 *      PID motor3({0.8f, 1.0f, 1.0f, 10000.0f, 3000.0f, 0.0f, 8000.0f, 0.0f, 50.0f});
 *
 *      // 复制（数组批量创建）
 *      PID motors[4] = {motor2, motor2, motor2, motor2};
 *
 *      // 微分模式默认 DIFF_FEEDBACK（微分先行），不适合时切换
 *      motor3.diff_mode(DiffMode::DIFF_ERROR);
 *
 * @note 使用(假设已有PID对象pid)
 *
 *      pid.feed_forward(FrictionFF());           // optional
 *      pid.feed_forward(FollowFF(dTarget));      // optional
 *      pid.calc(tar, motor.angle, motor.speed); // 最后一个参数是可去掉的
 *
 * @note 姿态 PID（QuaternionPID）：输入目标 / 反馈四元数，输出机体系三轴控制量
 *
 *      // 三轴各一份 PID::Config（机体 x / y / z），统一的微分模式、控制周期与输出模长限幅
 *      // {x 轴, y 轴, z 轴, 微分模式, 控制周期 s, 输出模长限幅}；每轴 {kp, ki, kd, out_max, ...} 同 PID::Config
 *      QuaternionPID att({{8.0f, 0.0f, 0.5f, 20.0f}, {8.0f, 0.0f, 0.5f, 20.0f}, {5.0f, 0.0f, 0.3f, 20.0f},
 *                         DiffMode::DIFF_FEEDBACK, 0.001f, 25.0f});
 *
 *      // 每个控制周期：q 来自 IMU（机体 → 世界），gyro 为机体系角速度 rad/s
 *      Vector3 omega_ref = att.calc(q_ref, q, gyro); // 外环：输出机体系角速度目标，交给三轴标量角速度 PID
 */

#ifndef __PID_HPP__
#define __PID_HPP__

#include <stdint.h>

#include "quaternion.hpp" // 姿态 PID 的输入 / 输出类型

/**
 * @brief 微分项计算模式
 */
enum class DiffMode
{
  DISABLE_DIFF  = 0, ///< 不计算微分项
  DIFF_FEEDBACK = 1, ///< 微分先行：只对反馈值微分，D = -kd·Δfeedback，目标阶跃不产生微分冲击
  DIFF_ERROR    = 2, ///< 常规微分：对误差微分，D = kd·Δerror
};

/**
 * @brief PID 输入缓存结构体
 */
struct PidInput
{
  float target       = 0.0f; ///< 当前目标值
  float feedback     = 0.0f; ///< 当前反馈值
  float error        = 0.0f; ///< 当前误差，error = target - feedback
  float last_feedback  = 0.0f; ///< 上一反馈值
  float last_error   = 0.0f; ///< 上一误差值
  float delta_feedback = 0.0f; ///< 反馈值微分，delta_feedback = feedback - last_feedback
  float delta_error  = 0.0f; ///< 误差值微分，delta_error = error - last_error
};

/**
 * @brief PID 各项计算值结构体
 */
struct PidTerm
{
  float p_term = 0.0f; ///< 比例项计算值
  float i_term = 0.0f; ///< 积分项计算值
  float d_term = 0.0f; ///< 微分项计算值
  float f_term = 0.0f; ///< 前馈项计算值
};

/**
 * @brief PID 控制器类
 *
 * @note 位置式 PID，支持积分分离、微分先行、前馈、各项独立限幅
 */
class PID
{
public:
  /* ==================== 构造与链式配置 ==================== */

  /**
   * @brief PID 配置结构体（可匿名按序传入）
   *
   * @note 限幅写 0 表示不限制；i_max 写 0 表示跟随 out_max
   */
  struct Config
  {
    /**
     * @brief 按序构造配置（参数顺序 = 字段顺序）
     */
    Config(float kp = 0.0f, float ki = 0.0f, float kd = 0.0f, float out_max = 0.0f, float p_max = 0.0f,
           float i_max = 0.0f, float d_max = 0.0f, float f_max = 0.0f, float i_sep = 0.0f,
           DiffMode diff_mode = DiffMode::DIFF_FEEDBACK)
      : kp(kp),
        ki(ki),
        kd(kd),
        out_max(out_max),
        p_max(p_max),
        i_max(i_max),
        d_max(d_max),
        f_max(f_max),
        i_sep(i_sep),
        diff_mode(diff_mode)
    {
    }

    float kp;       ///< 比例项系数
    float ki;       ///< 积分项系数
    float kd;       ///< 微分项系数
    float out_max;  ///< 总输出限幅，0=不限
    float p_max;    ///< 比例项限幅，0=不限
    float i_max;    ///< 积分项限幅，0=跟随out_max
    float d_max;    ///< 微分项限幅，0=不限
    float f_max;    ///< 前馈项限幅，0=不限
    float i_sep;    ///< 积分分离阈值，0=不分离
    DiffMode diff_mode; ///< 微分计算模式
  };

  PID() = default;

  /**
   * @brief 用配置结构体构造
   * @param cfg PID 配置（可匿名按序传入）
   */
  PID(const Config& cfg);

  /**
   * @brief 设置比例系数
   */
  PID& kp(float value)
  {
    _config.kp = value;
    return *this;
  }

  /**
   * @brief 设置积分系数
   */
  PID& ki(float value)
  {
    _config.ki = value;
    return *this;
  }

  /**
   * @brief 设置微分系数
   */
  PID& kd(float value)
  {
    _config.kd = value;
    return *this;
  }

  /**
   * @brief 设置总输出限幅
   */
  PID& limit_output(float value)
  {
    _config.out_max = value;
    return *this;
  }

  /**
   * @brief 设置比例项限幅
   */
  PID& limit_p(float value)
  {
    _config.p_max = value;
    return *this;
  }

  /**
   * @brief 设置积分项限幅
   */
  PID& limit_i(float value)
  {
    _config.i_max = value;
    return *this;
  }

  /**
   * @brief 设置微分项限幅
   */
  PID& limit_d(float value)
  {
    _config.d_max = value;
    return *this;
  }

  /**
   * @brief 设置前馈项限幅
   */
  PID& limit_f(float value)
  {
    _config.f_max = value;
    return *this;
  }

  /**
   * @brief 设置积分分离阈值
   */
  PID& integral_sep(float value)
  {
    _config.i_sep = value;
    return *this;
  }

  /**
   * @brief 切换微分项计算模式
   */
  PID& diff_mode(DiffMode mode)
  {
    _diff_mode = mode;
    return *this;
  }

  /* ==================== 计算接口 ==================== */

  /**
   * @brief 将某一前馈函数预测的值引入计算
   *
   * @param feedforward 某一前馈函数返回的数值
   * @return float 前馈项总和
   */
  float feed_forward(float feedforward);

  /**
   * @brief pid计算函数
   *
   * @param target 目标值
   * @param feedback 反馈值
   * @return float 输出
   */
  float calc(float target, float feedback);

  /**
   * @brief pid计算函数，反馈值一阶导数自行导入
   *
   * @param target 目标值
   * @param feedback 反馈值
   * @param df_dt 反馈值一阶导数（代替相邻两次反馈的差分；单位与 kd 整定时一致）。
   *              DIFF_FEEDBACK：D = -kd·df_dt；DIFF_ERROR：按目标不动处理，误差变化率 = -df_dt，D = -kd·df_dt
   * @note 示例:
   * 当动态系统被控量为位移x时，df_dt项可填入速度v，以提高微分项计算精度
   * @return float 输出
   */
  float calc(float target, float feedback, float df_dt);

  /**
   * @brief 清空运行状态：积分项、前馈项、输入缓存与输出；配置（含链式设置）不变
   * @note 控制器停用一段时间后重新启用时调用，避免旧积分与旧反馈造成冲击；之后第一次 calc() 的差分微分为 0。
   */
  void reset();

  /**
   * @brief 快速打印target和feedback到vofa [暂未实现]
   */
  void print();

  /* ==================== 数据接口 ==================== */

  PidInput _input;         ///< 输入缓存
  float    _output = 0.0f; ///< 输出

protected:
  /**
   * @brief 根据传入计算出其他输入项
   * @note virtual，允许继承后自定义输入处理
   */
  virtual void calc_input(float target, float feedback);

private:
  /* ==================== 私有成员函数 ==================== */

  /**
   * @brief 计算微分项（依据当前微分模式）
   */
  void calc_d_term();

  /**
   * @brief 计算 P/I 项、各项限幅与总输出
   * @return float 输出
   */
  float apply_limits_and_output();

  /* ==================== 私有成员变量 ==================== */

  Config _config;                                 ///< PID 配置
  PidTerm   _term;                              ///< 各项计算值
  DiffMode  _diff_mode = DiffMode::DIFF_FEEDBACK; ///< 微分计算模式
  bool      _has_last  = false;                   ///< 是否有上一帧输入（首次调用 / reset() 后为 false，该帧微分为 0）
};


/**
 * @brief 姿态 PID：输入目标 / 反馈姿态四元数，输出机体系三轴控制量
 *
 * @details 误差用旋转向量代替减法：e = feedback.error_to(target) = Log(feedback* ⊗ target)，
 *          即从反馈姿态转到目标姿态的旋转（机体系，rad，取 [0, π] 的短路径；小误差时 e 的三个分量 ≈ 绕机体 x / y / z 的角度误差）。
 *          三轴各交给一个 PID（复用比例、积分分离、各项限幅、前馈），输出向量 u 也在机体系；
 *          作外环时 u 是角速度目标（交给三轴标量角速度 PID，反馈直接用陀螺仪），作单环时 u 是力矩。
 *
 * @note 微分项 = 误差变化率 = ω_target（转到当前机体系）− ω_feedback，按每周期转角计（与标量 PID 的每周期差分一致）：
 *       - DIFF_FEEDBACK（默认，微分先行）：只对反馈微分，D = −kd · ω_fb · dt，目标突变不产生微分冲击；
 *       - DIFF_ERROR：D = kd · (ω_tar − ω_fb) · dt；
 *       - DISABLE_DIFF：D = 0。
 *       不传角速度时，ω_fb · dt 由相邻两次反馈四元数估算（Log(q_fb,上次* ⊗ q_fb)），ω_tar · dt 同理由相邻两次目标估算；
 *       传入陀螺仪角速度时按 Config::dt 换算，kd 的含义不变。不传角速度时，首次调用（或 reset() 后）没有上一帧，D = 0。
 *
 * @note 限幅：先按各轴 PID::Config 的限幅（p / i / d / f / out_max，分量限幅），再按 out_norm_max 把输出向量等比例缩小（保持方向）。
 *       只用分量限幅时，饱和会改变输出向量的方向（机构沿歪掉的轴转），三轴耦合的场合建议设 out_norm_max。
 *
 * @note 适用：转轴就是机体 x / y / z 的全姿态控制（飞行器、平衡、三轴云台）。两轴云台的 yaw 电机转轴是底盘竖直方向而不是机体 z 轴，
 *       用 Quaternion::pointing() 取水平角 / 仰角、再配标量 PID（误差用 angle_diff()）更直接。
 * @note 误差正好越过 180° 时短路径方向反转，输出随之反向（任何姿态误差定义都有这一点）；正常控制下误差不会到这么大。
 * @note 三轴的微分模式由本类统一决定：构造时把三轴 PID 设为 DIFF_ERROR，再把算好的误差变化率 de 以 PID::calc(e, 0, -de) 传入（df_dt 是反馈变化率，等于 -de），
 *       之后不要再对 axis(i) 调 diff_mode()。
 */
class QuaternionPID
{
public:
  /* ==================== 构造 ==================== */

  /**
   * @brief 姿态 PID 配置（可匿名按序传入：{{x 轴}, {y 轴}, {z 轴}, diff_mode, dt, out_norm_max}）
   * @note 各轴 PID::Config::diff_mode 不起作用，微分模式统一用这里的 diff_mode。
   */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config(const PID::Config &x = PID::Config(), const PID::Config &y = PID::Config(), const PID::Config &z = PID::Config(),
           DiffMode diff_mode = DiffMode::DIFF_FEEDBACK, float dt = 0.001f, float out_norm_max = 0.0f)
      : axis{x, y, z},
        diff_mode(diff_mode),
        dt(dt),
        out_norm_max(out_norm_max)
    {
    }

    PID::Config axis[3];      ///< 机体 x / y / z 三轴的 PID 配置（误差单位 rad）
    DiffMode    diff_mode;    ///< 微分模式（三轴统一）
    float       dt;           ///< 控制周期 (s)：只用于把传入的角速度 (rad/s) 换算成每周期转角
    float       out_norm_max; ///< 输出向量模长限幅，0=不限；等比例缩放、保持方向
  };

  QuaternionPID() = default;

  /**
   * @brief 用配置结构体构造
   * @param cfg 姿态 PID 配置（可匿名按序传入）
   */
  QuaternionPID(const Config &cfg);

  /* ==================== 计算接口 ==================== */

  /**
   * @brief 姿态 PID 计算，微分输入由相邻两次的目标 / 反馈四元数估算
   *
   * @param target   目标姿态（机体 → 世界，单位四元数）
   * @param feedback 反馈姿态（同上，通常来自 IMU）
   * @return Vector3 机体系三轴输出
   */
  Vector3 calc(const Quaternion &target, const Quaternion &feedback);

  /**
   * @brief 姿态 PID 计算，反馈角速度由陀螺仪导入（推荐：比四元数差分噪声小、没有一帧延迟）
   *
   * @param target         目标姿态
   * @param feedback       反馈姿态
   * @param omega_feedback 反馈角速度，机体系 rad/s（陀螺仪读数）
   * @note 目标角速度按 0 处理：DIFF_ERROR 下 D = −kd · ω_fb · dt，与 DIFF_FEEDBACK 相同；目标在动时用下面带 omega_target 的重载。
   * @return Vector3 机体系三轴输出
   */
  Vector3 calc(const Quaternion &target, const Quaternion &feedback, const Vector3 &omega_feedback);

  /**
   * @brief 姿态 PID 计算，目标与反馈角速度都由外部导入
   *
   * @param target         目标姿态
   * @param feedback       反馈姿态
   * @param omega_target   目标角速度，目标机体系 rad/s（例如生成目标轨迹时用的角速度）；内部按姿态误差转到当前机体系
   * @param omega_feedback 反馈角速度，机体系 rad/s
   * @note 只有 DIFF_ERROR 用到 omega_target；DIFF_FEEDBACK 按定义只对反馈微分。
   * @return Vector3 机体系三轴输出
   */
  Vector3 calc(const Quaternion &target, const Quaternion &feedback, const Vector3 &omega_target, const Vector3 &omega_feedback);

  /**
   * @brief 三轴前馈（机体系），每周期需要重新喂，语义同 PID::feed_forward()
   * @param feedforward 三轴前馈值（例如重力矩补偿）
   */
  void feed_forward(const Vector3 &feedforward);

  /**
   * @brief 清空运行状态：三轴 PID 的积分 / 历史输入、上一帧目标 / 反馈；配置不变
   */
  void reset();

  /* ==================== 数据接口 ==================== */

  /**
   * @brief 取某一轴的 PID：调参（链式设置 kp / ki / kd / 限幅）或查看该轴各项
   * @param i 0 / 1 / 2 = 机体 x / y / z；越界时取 z 轴
   */
  PID &axis(uint32_t i)
  {
    return _axis[(i < 3U) ? i : 2U];
  }

  /** @brief 最近一次的姿态误差（机体系旋转向量，rad） */
  const Vector3 &error() const
  {
    return _error;
  }

  /** @brief 最近一次的输出（机体系） */
  const Vector3 &output() const
  {
    return _output;
  }

private:
  /**
   * @brief 三个 calc() 的公共部分：求姿态误差、按微分模式求误差变化率、记下本帧
   * @param rot_target   本周期目标转角（目标机体系，rad）
   * @param rot_feedback 本周期反馈转角（当前机体系，rad）
   */
  Vector3 step(const Quaternion &target, const Quaternion &feedback, const Vector3 &rot_target, const Vector3 &rot_feedback);

  /**
   * @brief 三轴 PID + 输出模长限幅
   * @param error 姿态误差（rad）
   * @param d_in  三轴误差变化率（每周期，已按微分模式算好）
   */
  Vector3 run_axes(const Vector3 &error, const Vector3 &d_in);

  Config     _config;                               ///< 配置
  PID        _axis[3];                              ///< 机体 x / y / z 三轴 PID
  Vector3    _error         = {0.0, 0.0, 0.0};      ///< 最近一次姿态误差
  Vector3    _output        = {0.0, 0.0, 0.0};      ///< 最近一次输出
  Quaternion _last_target   = {1.0, 0.0, 0.0, 0.0}; ///< 上一次目标（差分估算 ω_tar 用）
  Quaternion _last_feedback = {1.0, 0.0, 0.0, 0.0}; ///< 上一次反馈（差分估算 ω_fb 用）
  bool       _has_last      = false;                ///< 是否有上一帧（首次调用 / reset() 后为 false，D = 0）
};

#endif // __PID_HPP__
