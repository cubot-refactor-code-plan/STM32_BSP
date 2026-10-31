/**
 * @file rls.hpp
 * @author ChoseB
 * @brief 带遗忘因子的递归最小二乘（RLS）在线参数辨识，多输出共用回归向量
 * @version 0.1
 * @date 2026-10-31
 *
 * @copyright Copyright (c) 2026
 *
 * @details 线性模型 y = Ωᵀ · x + e：x 为 X_DIM×1 回归向量（输入），y 为 Y_DIM×1 观测（输出），Ω 为 X_DIM×Y_DIM 待辨识参数。
 *          各路输出共用同一个 x，所以共用一个协方差 P（X_DIM×X_DIM）。每次 update()：
 *
 *            k = P·x / (λ + xᵀ·P·x)        增益（X_DIM×1）
 *            e = y − Ωᵀ·x                   先验误差（Y_DIM×1）
 *            Ω ← Ω + k·eᵀ
 *            P ← (P − k·xᵀ·P) / λ
 *
 *          λ 为遗忘因子：1 = 不遗忘（参数恒定时用，P 逐渐收敛到 0、最终不再更新）；< 1 时记忆长度约 1/(1−λ) 个样本，可跟踪慢变参数。
 *
 * @note 边界情况的处理：
 *       - 参数非法（λ ∉ (0, 1]、p0 ≤ 0、tr(P) 上限小于初值、含 NaN）：init() 返回 BAD_ARG；init() 之前 update() 返回 NOT_INIT；
 *       - 协方差爆炸：λ < 1 而输入激励不足（x ≈ 0 或长期同一方向）时，P 每步乘 1/λ 指数增长直至溢出；
 *         这里限制 tr(P) ≤ p_trace_max（默认 X_DIM·p0，即不比初始更“不确定”），到上限后把放大倍数降到刚好不超；
 *       - 非有限输入：x、y 含 NaN / inf，或本次结果不是有限值时，丢弃本样本、状态不变，返回 BAD_ARG 并计数（一个 NaN 会永久污染 Ω 与 P）；
 *       - 单精度舍入：每步把 P 对称化；P 失去正定（对角元 ≤ 0、xᵀPx < 0、非有限）时重置为 p0·I 并计数，Ω 保留。
 *
 * @note 用法：
 *
 *   // 辨识 y = a·u + b（x = {u, 1}，Ω = {a; b}），遗忘因子 0.99、P0 = 1000·I
 *   Rls<2, 1> rls({0.99f, 1000.0f});
 *   configASSERT(rls.init() == Status::OK);   // 初值 Ω = 0；也可传入先验 rls.init(omega0)
 *
 *   // 每个样本
 *   const float x_data[2] = {u, 1.0f};
 *   const float y_data[1] = {y};
 *   if (rls.update(MatrixF32<2, 1>::from_array(x_data), MatrixF32<1, 1>::from_array(y_data)) == Status::OK)
 *   {
 *     float a = rls.omega()[0][0];
 *     float b = rls.omega()[1][0];
 *   }
 *
 * @note 单精度下长时间运行（λ 很接近 1、数十万步以上）若仍频繁重置 P，可改用平方根 / UD 分解形式（Bierman），这里未实现。
 * @note 纯计算，不依赖硬件；P 与临时矩阵在栈上，适合 X_DIM ≤ 6 左右的小模型。
 */

#ifndef __RLS_HPP__
#define __RLS_HPP__

#include <math.h>
#include <stdint.h>

#include "matrix.hpp"
#include "status.hpp" // 统一状态码

/**
 * @brief 带遗忘因子的递归最小二乘
 * @tparam X_DIM 回归向量维度（参数个数 / 每路输出）
 * @tparam Y_DIM 输出路数
 */
template <uint32_t X_DIM, uint32_t Y_DIM>
class Rls
{
public:
  /* ==================== 构造与初始化 ==================== */

  /** @brief 配置（可匿名按序传入：{lambda, p0, p_trace_max}） */
  struct Config
  {
    /** @brief 按序构造（参数顺序 = 字段顺序） */
    Config(float lambda = 1.0f, float p0 = 1000.0f, float p_trace_max = 0.0f) : lambda(lambda), p0(p0), p_trace_max(p_trace_max)
    {
    }

    float lambda;      ///< 遗忘因子，(0, 1]；1 = 不遗忘，常用 0.95 ~ 0.999
    float p0;          ///< 初始协方差 P0 = p0·I，> 0；越大初期越信任新数据、收敛越快（常用 100 ~ 1e4）
    float p_trace_max; ///< tr(P) 上限，防止激励不足时协方差爆炸；0 = 取 X_DIM·p0；非 0 时不得小于 X_DIM·p0
  };

  /**
   * @brief 构造函数（只做赋值）
   * @param cfg 配置（可匿名按序传入）
   * @note 只传一个配置项时匿名花括号有二义性（{0.99f} 也能匹配拷贝构造），写成 Rls<2, 1> r(Rls<2, 1>::Config(0.99f))；两项及以上正常。
   */
  Rls(const Config &cfg = Config()) : _config(cfg)
  {
  }

  /**
   * @brief 检查配置，置 P = p0·I、Ω = omega0，清空计数
   * @param omega0 参数初值（先验），默认全 0
   * @return OK=成功；BAD_ARG=配置非法（λ ∉ (0, 1]、p0 ≤ 0、p_trace_max 非 0 且小于 X_DIM·p0、含 NaN / inf）
   * @note 可重复调用，重新开始辨识。
   */
  Status init(const MatrixF32<X_DIM, Y_DIM> &omega0 = MatrixF32<X_DIM, Y_DIM>());

  /* ==================== 计算接口 ==================== */

  /**
   * @brief 用一个样本更新参数估计
   * @param x 回归向量（X_DIM×1）
   * @param y 观测（Y_DIM×1）
   * @return OK=已更新；NOT_INIT=init() 没成功过；BAD_ARG=x / y 含 NaN / inf 或本次结果非有限，样本被丢弃、状态不变
   * @note P 失去正定时自动重置为 p0·I 并继续本次更新（仍返回 OK），次数见 covariance_reset_count()。
   */
  Status update(const MatrixF32<X_DIM, 1> &x, const MatrixF32<Y_DIM, 1> &y);

  /**
   * @brief 用当前参数预测输出：ŷ = Ωᵀ·x
   * @param x 回归向量
   */
  MatrixF32<Y_DIM, 1> predict(const MatrixF32<X_DIM, 1> &x) const;

  /** @brief 协方差重置为 p0·I（参数保留）；参数突变后想加快重新收敛时调用 */
  void reset_covariance();

  /* ==================== 数据接口 ==================== */

  /** @brief 参数估计 Ω（X_DIM×Y_DIM） */
  const MatrixF32<X_DIM, Y_DIM> &omega() const
  {
    return _omega;
  }

  /** @brief 协方差 P（X_DIM×X_DIM）；对角元反映各参数的不确定度 */
  const MatrixF32<X_DIM, X_DIM> &covariance() const
  {
    return _p;
  }

  /** @brief 最近一次成功 update() 的先验误差 e = y − Ωᵀ·x（更新前的 Ω） */
  const MatrixF32<Y_DIM, 1> &error() const
  {
    return _error;
  }

  /** @brief 成功更新的样本数 */
  uint32_t update_count() const
  {
    return _update_cnt;
  }

  /** @brief 因非有限值被丢弃的样本数 */
  uint32_t reject_count() const
  {
    return _reject_cnt;
  }

  /** @brief P 失去正定而重置的次数；持续增加说明单精度不够或模型 / 数据有问题 */
  uint32_t covariance_reset_count() const
  {
    return _reset_cnt;
  }

  /** @brief tr(P) 触到上限而限制放大的次数；持续增加说明输入激励不足 */
  uint32_t trace_limit_count() const
  {
    return _trace_limit_cnt;
  }

private:
  /** @brief 矩阵所有元素都是有限值 */
  template <uint32_t R, uint32_t C>
  static bool all_finite(const MatrixF32<R, C> &m);

  /** @brief P 对角元全部为正且有限（正定的必要条件，代价低） */
  bool covariance_ok(const MatrixF32<X_DIM, X_DIM> &p) const;

  Config                  _config;               ///< 配置
  float                   _p_trace_max = 0.0f;   ///< 生效的 tr(P) 上限（init() 时由配置确定）
  bool                    _init        = false;  ///< init() 是否成功过
  MatrixF32<X_DIM, Y_DIM> _omega;                ///< 参数估计
  MatrixF32<X_DIM, X_DIM> _p;                    ///< 协方差
  MatrixF32<Y_DIM, 1>     _error;                ///< 最近一次先验误差
  uint32_t                _update_cnt      = 0U; ///< 成功更新次数
  uint32_t                _reject_cnt      = 0U; ///< 丢弃样本数
  uint32_t                _reset_cnt       = 0U; ///< P 重置次数
  uint32_t                _trace_limit_cnt = 0U; ///< tr(P) 触限次数
};

/* ==================== 模板实现（只有头文件：任意尺寸都能直接用） ==================== */

template <uint32_t X_DIM, uint32_t Y_DIM>
Status Rls<X_DIM, Y_DIM>::init(const MatrixF32<X_DIM, Y_DIM> &omega0)
{
  _init = false;

  const float lambda = _config.lambda;
  const float p0     = _config.p0;
  // 写成 !(…)：NaN 也判为非法
  if (!(lambda > 0.0f && lambda <= 1.0f) || !(p0 > 0.0f) || isinf(p0) || !all_finite(omega0))
  {
    return Status::BAD_ARG;
  }

  // 隐藏逻辑显式化：p_trace_max 为 0 时取初始迹 X_DIM·p0；非 0 时不得小于初始迹，否则一开始就超限
  const float trace0 = static_cast<float>(X_DIM) * p0;
  if (_config.p_trace_max == 0.0f)
  {
    _p_trace_max = trace0;
  }
  else if (_config.p_trace_max >= trace0 && !isinf(_config.p_trace_max))
  {
    _p_trace_max = _config.p_trace_max;
  }
  else
  {
    return Status::BAD_ARG;
  }

  _omega           = omega0;
  _p               = MatrixF32<X_DIM, X_DIM>::identity() * p0;
  _error           = MatrixF32<Y_DIM, 1>();
  _update_cnt      = 0U;
  _reject_cnt      = 0U;
  _reset_cnt       = 0U;
  _trace_limit_cnt = 0U;
  _init            = true;
  return Status::OK;
}

template <uint32_t X_DIM, uint32_t Y_DIM>
Status Rls<X_DIM, Y_DIM>::update(const MatrixF32<X_DIM, 1> &x, const MatrixF32<Y_DIM, 1> &y)
{
  if (!_init)
  {
    return Status::NOT_INIT;
  }
  if (!all_finite(x) || !all_finite(y))
  {
    ++_reject_cnt;
    return Status::BAD_ARG;
  }

  const float lambda = _config.lambda;

  // P 失去正定（单精度舍入累积）：重置后用新的 P 继续本次更新
  MatrixF32<X_DIM, 1> px  = _p * x;
  float               xpx = (x.transpose() * px).scalar();
  if (!covariance_ok(_p) || !(xpx >= 0.0f))
  {
    reset_covariance();
    ++_reset_cnt;
    px  = _p * x;
    xpx = (x.transpose() * px).scalar();
  }

  // 增益与先验误差（分母 ≥ λ > 0）
  const MatrixF32<X_DIM, 1> k = px / (lambda + xpx);
  const MatrixF32<Y_DIM, 1> e = y - _omega.transpose() * x;

  // 新参数与新协方差先算到局部变量，全部有限才写回
  const MatrixF32<X_DIM, Y_DIM> omega = _omega + k * e.transpose();

  // P − k·xᵀ·P：P 对称，xᵀ·P = (P·x)ᵀ；再对称化抵消舍入
  MatrixF32<X_DIM, X_DIM> p = _p - k * px.transpose();
  p                         = (p + p.transpose()) * 0.5f;

  // 遗忘：P / λ，但 tr(P) 不超过上限（激励不足时 P 不再指数增长）
  const float trace = p.trace();
  float       scale = 1.0f / lambda;
  if (trace * scale > _p_trace_max && trace > 0.0f)
  {
    scale = _p_trace_max / trace;
    ++_trace_limit_cnt;
  }
  p *= scale;

  if (!all_finite(omega) || !all_finite(p) || !all_finite(e))
  {
    ++_reject_cnt;
    return Status::BAD_ARG;
  }

  _omega = omega;
  _p     = p;
  _error = e;
  ++_update_cnt;
  return Status::OK;
}

template <uint32_t X_DIM, uint32_t Y_DIM>
MatrixF32<Y_DIM, 1> Rls<X_DIM, Y_DIM>::predict(const MatrixF32<X_DIM, 1> &x) const
{
  return _omega.transpose() * x;
}

template <uint32_t X_DIM, uint32_t Y_DIM>
void Rls<X_DIM, Y_DIM>::reset_covariance()
{
  _p = MatrixF32<X_DIM, X_DIM>::identity() * _config.p0;
}

template <uint32_t X_DIM, uint32_t Y_DIM>
template <uint32_t R, uint32_t C>
bool Rls<X_DIM, Y_DIM>::all_finite(const MatrixF32<R, C> &m)
{
  for (uint32_t i = 0U; i < R * C; ++i)
  {
    if (!isfinite(m.data()[i]))
    {
      return false;
    }
  }
  return true;
}

template <uint32_t X_DIM, uint32_t Y_DIM>
bool Rls<X_DIM, Y_DIM>::covariance_ok(const MatrixF32<X_DIM, X_DIM> &p) const
{
  for (uint32_t i = 0U; i < X_DIM; ++i)
  {
    if (!(p[i][i] > 0.0f) || isinf(p[i][i]))
    {
      return false;
    }
  }
  return true;
}

#endif // __RLS_HPP__
