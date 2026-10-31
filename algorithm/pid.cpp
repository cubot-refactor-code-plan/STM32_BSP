#include "pid.hpp"
#include <math.h>

PID::PID(const Config& cfg) : _config(cfg)
{
  // 隐藏逻辑显式化：i_max 未设置（为0）时跟随 out_max
  if (_config.i_max == 0.0f)
  {
    _config.i_max = _config.out_max;
  }

  // 微分模式沿用配置值
  _diff_mode = _config.diff_mode;
}

void PID::calc_input(float target, float feedback)
{
  // 首次调用（或 reset() 后）没有上一帧：把上一帧当作本帧，差分为 0，避免从 0 跳到当前值的微分冲击
  if (!_has_last)
  {
    _input.feedback = feedback;
    _input.error    = target - feedback;
    _has_last       = true;
  }

  _input.last_feedback  = _input.feedback;
  _input.last_error   = _input.error;
  _input.target       = target;
  _input.feedback     = feedback;
  _input.error        = _input.target - _input.feedback;
  _input.delta_feedback = _input.feedback - _input.last_feedback;
  _input.delta_error  = _input.error - _input.last_error;
}

void PID::calc_d_term()
{
  switch (_diff_mode)
  {
    case DiffMode::DIFF_FEEDBACK:
      // 微分先行：error = target - feedback，目标不动时误差变化率 = -反馈变化率，所以取负号（阻尼方向）
      _term.d_term = -_config.kd * _input.delta_feedback;
      break;
    case DiffMode::DIFF_ERROR:
      _term.d_term = _config.kd * _input.delta_error; // 常规微分
      break;
    case DiffMode::DISABLE_DIFF:
    default:
      _term.d_term = 0.0f;
      break;
  }
}

float PID::apply_limits_and_output()
{
  // 对称限幅辅助 lambda：limit 为 0 表示不限制
  auto clamp_symmetric = [](float value, float limit)
  {
    if (limit == 0.0f)
    {
      return value;
    }
    if (value > limit)
    {
      return limit;
    }
    if (value < -limit)
    {
      return -limit;
    }
    return value;
  };

  _term.p_term = _config.kp * _input.error;

  // 积分分离：误差小于阈值（或阈值未设置）时累加，否则清零（注意：是清零而非冻结）
  _term.i_term = (fabsf(_input.error) < _config.i_sep || _config.i_sep == 0.0f)
                   ? _term.i_term + _config.ki * _input.error
                   : 0.0f;

  // 积分项累加后立即钳位，防止积分无限累积（即使后续总输出有限幅）
  _term.i_term = clamp_symmetric(_term.i_term, _config.i_max);

  calc_d_term();

  // 各项独立限幅
  _term.p_term = clamp_symmetric(_term.p_term, _config.p_max);
  _term.d_term = clamp_symmetric(_term.d_term, _config.d_max);
  _term.f_term = clamp_symmetric(_term.f_term, _config.f_max);

  // 总输出限幅
  _output = _term.p_term + _term.i_term + _term.d_term + _term.f_term;
  _output = clamp_symmetric(_output, _config.out_max);

  // 前馈项每周期清零，需要每周期重新喂
  _term.f_term = 0.0f;

  return _output;
}

float PID::feed_forward(float feedforward)
{
  _term.f_term += feedforward;
  return _term.f_term;
}

float PID::calc(float target, float feedback)
{
  calc_input(target, feedback);
  return apply_limits_and_output();
}

float PID::calc(float target, float feedback, float df_dt)
{
  calc_input(target, feedback);

  // 外部导入的反馈值一阶导数代替差分；DIFF_ERROR 按目标不动处理，误差变化率 = -反馈变化率
  if (_diff_mode == DiffMode::DIFF_FEEDBACK)
  {
    _input.delta_feedback = df_dt;
  }
  else if (_diff_mode == DiffMode::DIFF_ERROR)
  {
    _input.delta_error = -df_dt;
  }

  return apply_limits_and_output();
}

void PID::reset()
{
  _input    = PidInput();
  _term     = PidTerm();
  _output   = 0.0f;
  _has_last = false;
}

void PID::print()
{
  // 暂未实现
  return;
}


/* ==================== QuaternionPID ==================== */

QuaternionPID::QuaternionPID(const Config &cfg) : _config(cfg), _axis {PID(cfg.axis[0]), PID(cfg.axis[1]), PID(cfg.axis[2])}
{
  // 隐藏逻辑显式化：三轴一律用 DIFF_ERROR，本类自己按 _config.diff_mode 算好误差变化率再传进去
  for (uint32_t i = 0U; i < 3U; ++i)
  {
    _axis[i].diff_mode(DiffMode::DIFF_ERROR);
  }
}

Vector3 QuaternionPID::calc(const Quaternion &target, const Quaternion &feedback)
{
  // 每周期转角由相邻两次估算：反馈表在上一帧机体系、目标表在上一帧目标机体系（相邻两帧相差很小，按当前系用）
  Vector3 rot_target   = {0.0, 0.0, 0.0};
  Vector3 rot_feedback = {0.0, 0.0, 0.0};
  if (_has_last)
  {
    rot_target   = _last_target.error_to(target);
    rot_feedback = _last_feedback.error_to(feedback);
  }
  return step(target, feedback, rot_target, rot_feedback);
}

Vector3 QuaternionPID::calc(const Quaternion &target, const Quaternion &feedback, const Vector3 &omega_feedback)
{
  const double dt = static_cast<double>(_config.dt);
  return step(target, feedback, {0.0, 0.0, 0.0}, {omega_feedback.x * dt, omega_feedback.y * dt, omega_feedback.z * dt});
}

Vector3 QuaternionPID::calc(const Quaternion &target, const Quaternion &feedback, const Vector3 &omega_target, const Vector3 &omega_feedback)
{
  const double dt = static_cast<double>(_config.dt);
  return step(target, feedback, {omega_target.x * dt, omega_target.y * dt, omega_target.z * dt}, {omega_feedback.x * dt, omega_feedback.y * dt, omega_feedback.z * dt});
}

void QuaternionPID::feed_forward(const Vector3 &feedforward)
{
  _axis[0].feed_forward(static_cast<float>(feedforward.x));
  _axis[1].feed_forward(static_cast<float>(feedforward.y));
  _axis[2].feed_forward(static_cast<float>(feedforward.z));
}

void QuaternionPID::reset()
{
  for (uint32_t i = 0U; i < 3U; ++i)
  {
    _axis[i].reset();
  }
  _error    = {0.0, 0.0, 0.0};
  _output   = {0.0, 0.0, 0.0};
  _has_last = false;
}

Vector3 QuaternionPID::step(const Quaternion &target, const Quaternion &feedback, const Vector3 &rot_target, const Vector3 &rot_feedback)
{
  // 误差：从反馈姿态转到目标姿态的旋转向量（当前机体系），代替 target - feedback
  const Quaternion q_err = feedback.conj() * target;
  _error                 = q_err.rotation_vector();

  // 误差变化率（每周期）= 目标转角（转到当前机体系）- 反馈转角
  Vector3 de = {0.0, 0.0, 0.0};
  switch (_config.diff_mode)
  {
    case DiffMode::DIFF_FEEDBACK:
      de = {-rot_feedback.x, -rot_feedback.y, -rot_feedback.z}; // 只对反馈微分
      break;
    case DiffMode::DIFF_ERROR:
    {
      const Vector3 rt = q_err.rotate(rot_target); // 目标机体系 → 当前机体系
      de               = {rt.x - rot_feedback.x, rt.y - rot_feedback.y, rt.z - rot_feedback.z};
      break;
    }
    case DiffMode::DISABLE_DIFF:
    default:
      break;
  }

  _last_target   = target;
  _last_feedback = feedback;
  _has_last      = true;

  return run_axes(_error, de);
}

Vector3 QuaternionPID::run_axes(const Vector3 &error, const Vector3 &d_in)
{
  // 三轴 PID 都是 DIFF_ERROR：df_dt 为反馈变化率，等于 -误差变化率
  Vector3 out = {static_cast<double>(_axis[0].calc(static_cast<float>(error.x), 0.0f, static_cast<float>(-d_in.x))),
                 static_cast<double>(_axis[1].calc(static_cast<float>(error.y), 0.0f, static_cast<float>(-d_in.y))),
                 static_cast<double>(_axis[2].calc(static_cast<float>(error.z), 0.0f, static_cast<float>(-d_in.z)))};

  // 模长限幅：等比例缩小，保持输出方向（各轴分量限幅已在 PID 里做过）
  if (_config.out_norm_max > 0.0f)
  {
    const double n   = out.norm();
    const double max = static_cast<double>(_config.out_norm_max);
    if (n > max)
    {
      const double k = max / n;
      out            = {out.x * k, out.y * k, out.z * k};
    }
  }

  _output = out;
  return _output;
}
