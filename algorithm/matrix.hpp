/**
 * @file matrix.hpp
 * @author yssickjgd (1345578933@qq.com)（原作 alg_matrix.h / alg_matrix.cpp）；ChoseB（移植到 stm32_bsp 并修改）
 * @brief 定长单精度矩阵：四则运算、转置、求逆（报告奇异）、向量模长 / 归一化 / 点乘 / 叉乘、常用旋转矩阵
 * @version 0.2
 * @date 2026-10-31
 *
 * @copyright 原作 Copyright (c) 2025 USTC-RoboWalker，经作者授权移植；移植与修改部分 Copyright (c) 2026。
 *            本模块不适用本库的 MIT 许可证。麻老师原话:"可以的，只要是RM里的就随便用"。若使用用途超出RoboMaster赛事范畴内，建议自行再去征求原作者意见
 *
 * @details 来源：github.com/yssickjgd/damiao_mc02_bsp，dm02_test/User_File/1_Middleware/Algorithm/Matrix（0.1，2025-08-21）。
 *          移植时的改动：
 *          - 命名改为本库规范（类 MatrixF32、方法小写蛇形、私有成员 _ 前缀、模板参数全大写），按 C++11 编译（原作需要 C++14 / 17），
 *            不再依赖 arm_math.h（三角函数改用 math.h）；
 *          - 求逆 inverse() 返回是否成功（原作奇异时静默返回单位矩阵），奇异判定改为相对阈值（原作是绝对阈值 1e-6，
 *            量级小但良态的矩阵会被误判）；
 *          - 除以标量不再在除数接近 0 时静默返回原矩阵，按 IEEE 浮点语义得到 inf / NaN；
 *          - 修正 operator/= 不能传字面量、operator!= 无法调用且返回类型错误、矩阵乘法不能用于 const 矩阵；
 *          - 近似比较的容差由全局可写变量改为常量与参数，含 NaN 的矩阵判为不相等；
 *          - 拷贝 / 移动由编译器生成（原作手写 memcpy，语义相同）；*= 推广到右乘任意 COLS×COLS 方阵；
 *            matrix_from_axis_angle() 参数顺序改为 (axis, angle)（与 Quaternion::from_axis_angle() 一致），并在内部归一化转轴。
 *
 * @note 数据按行优先存放；尺寸是模板参数，编译期确定，尺寸不匹配的运算直接编译报错。
 *       矩阵在栈上按值传递 / 返回，适合 6×6 以内的小矩阵（inverse() 另需 2·N² 个 float 的栈空间）。
 *       纯计算，不依赖硬件与 RTOS，任意上下文可调。
 *
 * @note 用法：
 *
 *   // 构造：默认零矩阵；从数组（行优先）；单位 / 常量矩阵
 *   const float r_data[9] = {0.4f, 0.0f, 0.0f, 0.0f, 0.4f, 0.0f, 0.0f, 0.0f, 0.5f};
 *   MatrixF32<3, 3> r = MatrixF32<3, 3>::from_array(r_data);
 *   MatrixF32<4, 4> p = MatrixF32<4, 4>::identity();
 *
 *   // 元素访问：m[行][列]
 *   p[0][3] = 1.0f;
 *
 *   // 运算：尺寸由模板推导，(3×4)·(4×4)·(4×3) = 3×3
 *   MatrixF32<3, 4> h;
 *   MatrixF32<3, 3> s = h * p * h.transpose() + r;
 *
 *   // 求逆：失败（奇异）时返回 false，s_inv 保持不变，由调用方决定怎么处理
 *   MatrixF32<3, 3> s_inv;
 *   if (!s.inverse(s_inv))
 *   {
 *     // 跳过本次更新 / 计数 / 重置协方差 ...
 *   }
 *
 *   // 向量（列向量 N×1）
 *   MatrixF32<3, 1> g = matrix_axis_z_3d() * 9.80665f;
 *   float           n = g.norm();
 *   MatrixF32<3, 1> c = vector_cross_3d(g, matrix_axis_x_3d());
 */

#ifndef __MATRIX_HPP__
#define __MATRIX_HPP__

#include <math.h>
#include <stdint.h>

constexpr float MATRIX_COMPARE_EPS      = 1e-6f; ///< 近似比较（==、!=）的默认绝对容差
constexpr float MATRIX_SINGULAR_REL_EPS = 1e-6f; ///< 求逆时主元绝对值 ≤ 它 × 矩阵最大元素绝对值，判为奇异（相对阈值，与数值量级无关）

/**
 * @brief 定长单精度矩阵
 * @tparam ROWS 行数（≥ 1）
 * @tparam COLS 列数（≥ 1）
 */
template <uint32_t ROWS, uint32_t COLS>
class MatrixF32
{
  static_assert(ROWS > 0U && COLS > 0U, "MatrixF32: 行数、列数至少为 1");

  template <uint32_t, uint32_t>
  friend class MatrixF32; // 不同尺寸之间相互访问 _data（乘法、转置、取行列）

public:
  static constexpr uint32_t SIZE = ROWS * COLS; ///< 元素个数

  /* ==================== 构造 ==================== */

  /** @brief 零矩阵 */
  MatrixF32()
  {
    for (uint32_t i = 0U; i < SIZE; ++i)
    {
      _data[i] = 0.0f;
    }
  }

  /** @brief 从数组构造（行优先，ROWS × COLS 个元素） */
  static MatrixF32 from_array(const float data[ROWS * COLS])
  {
    MatrixF32 result;
    for (uint32_t i = 0U; i < SIZE; ++i)
    {
      result._data[i] = data[i];
    }
    return result;
  }

  /** @brief 零矩阵 */
  static MatrixF32 zero()
  {
    return MatrixF32();
  }

  /** @brief 单位矩阵；非方阵时主对角线为 1、其余为 0 */
  static MatrixF32 identity()
  {
    MatrixF32 result;
    for (uint32_t i = 0U; i < ((ROWS < COLS) ? ROWS : COLS); ++i)
    {
      result._data[i * COLS + i] = 1.0f;
    }
    return result;
  }

  /** @brief 全部元素为 value 的矩阵 */
  static MatrixF32 constant(float value)
  {
    MatrixF32 result;
    for (uint32_t i = 0U; i < SIZE; ++i)
    {
      result._data[i] = value;
    }
    return result;
  }

  /* ==================== 访问 ==================== */

  /**
   * @brief 取一行的首地址，m[row][col] 访问元素
   * @warning 不检查越界（与原生数组一致）。
   */
  float *operator[](uint32_t row)
  {
    return &_data[row * COLS];
  }

  /** @brief 同上（只读） */
  const float *operator[](uint32_t row) const
  {
    return &_data[row * COLS];
  }

  /** @brief 行优先的数据首地址（共 SIZE 个元素） */
  float *data()
  {
    return _data;
  }

  /** @brief 同上（只读） */
  const float *data() const
  {
    return _data;
  }

  /** @brief 取第 index 行（1 × COLS） */
  MatrixF32<1, COLS> row(uint32_t index) const;

  /** @brief 取第 index 列（ROWS × 1） */
  MatrixF32<ROWS, 1> column(uint32_t index) const;

  /* ==================== 运算 ==================== */

  /** @brief 正号（返回副本） */
  MatrixF32 operator+() const
  {
    return *this;
  }

  /** @brief 取负 */
  MatrixF32 operator-() const;

  /** @brief 每个元素加 value */
  MatrixF32 operator+(float value) const;

  /** @brief 逐元素相加 */
  MatrixF32 operator+(const MatrixF32 &rhs) const;

  /** @brief 每个元素减 value */
  MatrixF32 operator-(float value) const;

  /** @brief 逐元素相减 */
  MatrixF32 operator-(const MatrixF32 &rhs) const;

  /** @brief 数乘 */
  MatrixF32 operator*(float value) const;

  /** @brief 数乘（标量在左） */
  friend MatrixF32 operator*(float value, const MatrixF32 &m)
  {
    return m * value;
  }

  /**
   * @brief 矩阵乘法：(ROWS × COLS) · (COLS × K) = ROWS × K
   */
  template <uint32_t K>
  MatrixF32<ROWS, K> operator*(const MatrixF32<COLS, K> &rhs) const;

  /**
   * @brief 除以标量
   * @note value 为 0 时按 IEEE 浮点语义得到 inf / NaN（不做特殊处理，错误会在结果里显现）。
   */
  MatrixF32 operator/(float value) const;

  /** @brief 每个元素加 value */
  MatrixF32 &operator+=(float value);

  /** @brief 逐元素相加 */
  MatrixF32 &operator+=(const MatrixF32 &rhs);

  /** @brief 每个元素减 value */
  MatrixF32 &operator-=(float value);

  /** @brief 逐元素相减 */
  MatrixF32 &operator-=(const MatrixF32 &rhs);

  /** @brief 数乘 */
  MatrixF32 &operator*=(float value);

  /** @brief 右乘方阵：*this = *this · rhs（rhs 为 COLS × COLS，结果尺寸不变） */
  MatrixF32 &operator*=(const MatrixF32<COLS, COLS> &rhs);

  /** @brief 除以标量（语义同 operator/） */
  MatrixF32 &operator/=(float value);

  /**
   * @brief 近似相等：每个元素之差的绝对值都 ≤ tol
   * @param tol 绝对容差
   * @note 任一元素为 NaN 时判为不相等。
   */
  bool approx_equal(const MatrixF32 &rhs, float tol) const;

  /** @brief 近似相等，容差 MATRIX_COMPARE_EPS（不同尺寸不能比较，编译报错） */
  bool operator==(const MatrixF32 &rhs) const
  {
    return approx_equal(rhs, MATRIX_COMPARE_EPS);
  }

  /** @brief 近似不相等，容差 MATRIX_COMPARE_EPS */
  bool operator!=(const MatrixF32 &rhs) const
  {
    return !approx_equal(rhs, MATRIX_COMPARE_EPS);
  }

  /** @brief 转置 */
  MatrixF32<COLS, ROWS> transpose() const;

  /** @brief 迹（主对角线元素之和；非方阵取 min(ROWS, COLS) 个） */
  float trace() const;

  /* ==================== 方阵 ==================== */

  /**
   * @brief 求逆（Gauss-Jordan 消元，部分主元）
   *
   * @param out 输出：逆矩阵；失败时保持不变。可以就是 *this（m.inverse(m)）
   * @return true=成功；false=奇异或接近奇异（某一步的主元 ≤ MATRIX_SINGULAR_REL_EPS × 矩阵最大元素绝对值），
   *         全零矩阵、含 NaN / inf 也返回 false
   * @note 只有方阵可调用（否则编译报错）。
   */
  bool inverse(MatrixF32 &out) const;

  /* ==================== 列向量（COLS = 1） ==================== */

  /** @brief 模长（只有列向量可调用） */
  float norm() const;

  /**
   * @brief 归一化（只有列向量可调用）
   * @note 零向量按 IEEE 浮点语义得到 NaN（不做特殊处理）。
   */
  MatrixF32 normalized() const;

  /** @brief 叉乘矩阵 [v]×，满足 [v]× · u = v × u（只有 3×1 可调用） */
  MatrixF32<3, 3> cross_matrix() const;

  /* ==================== 1×1 ==================== */

  /** @brief 取唯一的元素（只有 1×1 可调用，例如 vᵀ·A·v 的结果） */
  float scalar() const;

private:
  float _data[ROWS * COLS]; ///< 数据，行优先
};

/* ==================== 常用向量与旋转矩阵（matrix.cpp） ==================== */

/** @brief 二维 x 轴单位向量 */
MatrixF32<2, 1> matrix_axis_x_2d();

/** @brief 二维 y 轴单位向量 */
MatrixF32<2, 1> matrix_axis_y_2d();

/** @brief 三维 x 轴单位向量 */
MatrixF32<3, 1> matrix_axis_x_3d();

/** @brief 三维 y 轴单位向量 */
MatrixF32<3, 1> matrix_axis_y_3d();

/** @brief 三维 z 轴单位向量 */
MatrixF32<3, 1> matrix_axis_z_3d();

/**
 * @brief 二维旋转矩阵（逆时针为正）
 * @param angle 转角，rad
 */
MatrixF32<2, 2> matrix_rotation_2d(float angle);

/**
 * @brief 三维绕轴旋转矩阵（Rodrigues 公式，右手定则）
 * @param axis  转轴（内部归一化；零向量时返回单位矩阵）
 * @param angle 转角，rad
 */
MatrixF32<3, 3> matrix_from_axis_angle(const MatrixF32<3, 1> &axis, float angle);

/**
 * @brief Z-Y-X 欧拉角 → 旋转矩阵：R = Rz(yaw) · Ry(pitch) · Rx(roll)（机体 → 世界，与 Quaternion::to_euler_zyx() 同约定）
 * @param yaw   偏航，rad
 * @param pitch 俯仰，rad
 * @param roll  横滚，rad
 */
MatrixF32<3, 3> matrix_from_euler_zyx(float yaw, float pitch, float roll);

/** @brief 三维向量叉乘 a × b */
MatrixF32<3, 1> vector_cross_3d(const MatrixF32<3, 1> &a, const MatrixF32<3, 1> &b);

/** @brief 二维向量叉乘（标量）：a.x · b.y - a.y · b.x */
float vector_cross_2d(const MatrixF32<2, 1> &a, const MatrixF32<2, 1> &b);

/** @brief 向量点乘 */
template <uint32_t N>
float vector_dot(const MatrixF32<N, 1> &a, const MatrixF32<N, 1> &b)
{
  float result = 0.0f;
  for (uint32_t i = 0U; i < N; ++i)
  {
    result += a[i][0] * b[i][0];
  }
  return result;
}

/* ==================== 模板实现 ==================== */

template <uint32_t ROWS, uint32_t COLS>
constexpr uint32_t MatrixF32<ROWS, COLS>::SIZE;

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<1, COLS> MatrixF32<ROWS, COLS>::row(uint32_t index) const
{
  MatrixF32<1, COLS> result;
  for (uint32_t j = 0U; j < COLS; ++j)
  {
    result._data[j] = _data[index * COLS + j];
  }
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, 1> MatrixF32<ROWS, COLS>::column(uint32_t index) const
{
  MatrixF32<ROWS, 1> result;
  for (uint32_t i = 0U; i < ROWS; ++i)
  {
    result._data[i] = _data[i * COLS + index];
  }
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::operator-() const
{
  MatrixF32 result;
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    result._data[i] = -_data[i];
  }
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::operator+(float value) const
{
  MatrixF32 result = *this;
  result += value;
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::operator+(const MatrixF32 &rhs) const
{
  MatrixF32 result = *this;
  result += rhs;
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::operator-(float value) const
{
  MatrixF32 result = *this;
  result -= value;
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::operator-(const MatrixF32 &rhs) const
{
  MatrixF32 result = *this;
  result -= rhs;
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::operator*(float value) const
{
  MatrixF32 result = *this;
  result *= value;
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
template <uint32_t K>
MatrixF32<ROWS, K> MatrixF32<ROWS, COLS>::operator*(const MatrixF32<COLS, K> &rhs) const
{
  // i-k-j 顺序：内层沿 rhs 与 result 的行连续访问
  MatrixF32<ROWS, K> result;
  for (uint32_t i = 0U; i < ROWS; ++i)
  {
    for (uint32_t k = 0U; k < COLS; ++k)
    {
      const float a_ik = _data[i * COLS + k];
      for (uint32_t j = 0U; j < K; ++j)
      {
        result._data[i * K + j] += a_ik * rhs._data[k * K + j];
      }
    }
  }
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::operator/(float value) const
{
  MatrixF32 result = *this;
  result /= value;
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> &MatrixF32<ROWS, COLS>::operator+=(float value)
{
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    _data[i] += value;
  }
  return *this;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> &MatrixF32<ROWS, COLS>::operator+=(const MatrixF32 &rhs)
{
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    _data[i] += rhs._data[i];
  }
  return *this;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> &MatrixF32<ROWS, COLS>::operator-=(float value)
{
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    _data[i] -= value;
  }
  return *this;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> &MatrixF32<ROWS, COLS>::operator-=(const MatrixF32 &rhs)
{
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    _data[i] -= rhs._data[i];
  }
  return *this;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> &MatrixF32<ROWS, COLS>::operator*=(float value)
{
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    _data[i] *= value;
  }
  return *this;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> &MatrixF32<ROWS, COLS>::operator*=(const MatrixF32<COLS, COLS> &rhs)
{
  *this = (*this) * rhs; // 先算到临时量再赋值，rhs 就是 *this 时也正确
  return *this;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> &MatrixF32<ROWS, COLS>::operator/=(float value)
{
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    _data[i] /= value;
  }
  return *this;
}

template <uint32_t ROWS, uint32_t COLS>
bool MatrixF32<ROWS, COLS>::approx_equal(const MatrixF32 &rhs, float tol) const
{
  for (uint32_t i = 0U; i < SIZE; ++i)
  {
    // 写成 !(… ≤ tol)：差值为 NaN 时比较为假，判为不相等
    if (!(fabsf(_data[i] - rhs._data[i]) <= tol))
    {
      return false;
    }
  }
  return true;
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<COLS, ROWS> MatrixF32<ROWS, COLS>::transpose() const
{
  MatrixF32<COLS, ROWS> result;
  for (uint32_t i = 0U; i < ROWS; ++i)
  {
    for (uint32_t j = 0U; j < COLS; ++j)
    {
      result._data[j * ROWS + i] = _data[i * COLS + j];
    }
  }
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
float MatrixF32<ROWS, COLS>::trace() const
{
  float result = 0.0f;
  for (uint32_t i = 0U; i < ((ROWS < COLS) ? ROWS : COLS); ++i)
  {
    result += _data[i * COLS + i];
  }
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
bool MatrixF32<ROWS, COLS>::inverse(MatrixF32 &out) const
{
  static_assert(ROWS == COLS, "MatrixF32::inverse(): 只有方阵可求逆");

  // 增广矩阵 [A | I]；先拷出来，out 可以就是 *this
  float aug[ROWS][2U * ROWS];
  float scale = 0.0f;
  for (uint32_t i = 0U; i < ROWS; ++i)
  {
    for (uint32_t j = 0U; j < ROWS; ++j)
    {
      aug[i][j]        = _data[i * ROWS + j];
      aug[i][ROWS + j] = (i == j) ? 1.0f : 0.0f;
      const float a    = fabsf(_data[i * ROWS + j]);
      scale            = (a > scale) ? a : scale;
    }
  }
  // 写成 !(> 0)：全零与含 NaN 都在这里返回
  if (!(scale > 0.0f) || isinf(scale))
  {
    return false;
  }
  const float threshold = scale * MATRIX_SINGULAR_REL_EPS;

  for (uint32_t col = 0U; col < ROWS; ++col)
  {
    // 部分主元：本列（col 行及以下）绝对值最大的行
    uint32_t pivot = col;
    float    best  = fabsf(aug[col][col]);
    for (uint32_t r = col + 1U; r < ROWS; ++r)
    {
      const float a = fabsf(aug[r][col]);
      if (a > best)
      {
        best  = a;
        pivot = r;
      }
    }
    if (!(best > threshold)) // 主元相对太小：奇异或接近奇异（NaN 也走这里）
    {
      return false;
    }

    if (pivot != col)
    {
      for (uint32_t j = 0U; j < 2U * ROWS; ++j)
      {
        const float t = aug[col][j];
        aug[col][j]   = aug[pivot][j];
        aug[pivot][j] = t;
      }
    }

    // 主元行归一化，再从其余各行消去本列
    const float inv_pivot = 1.0f / aug[col][col];
    for (uint32_t j = 0U; j < 2U * ROWS; ++j)
    {
      aug[col][j] *= inv_pivot;
    }
    for (uint32_t r = 0U; r < ROWS; ++r)
    {
      if (r == col)
      {
        continue;
      }
      const float factor = aug[r][col];
      for (uint32_t j = 0U; j < 2U * ROWS; ++j)
      {
        aug[r][j] -= factor * aug[col][j];
      }
    }
  }

  for (uint32_t i = 0U; i < ROWS; ++i)
  {
    for (uint32_t j = 0U; j < ROWS; ++j)
    {
      out._data[i * ROWS + j] = aug[i][ROWS + j];
    }
  }
  return true;
}

template <uint32_t ROWS, uint32_t COLS>
float MatrixF32<ROWS, COLS>::norm() const
{
  static_assert(COLS == 1U, "MatrixF32::norm(): 只有列向量可调用");
  float sum = 0.0f;
  for (uint32_t i = 0U; i < ROWS; ++i)
  {
    sum += _data[i] * _data[i];
  }
  return sqrtf(sum);
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<ROWS, COLS> MatrixF32<ROWS, COLS>::normalized() const
{
  static_assert(COLS == 1U, "MatrixF32::normalized(): 只有列向量可调用");
  return (*this) * (1.0f / norm());
}

template <uint32_t ROWS, uint32_t COLS>
MatrixF32<3, 3> MatrixF32<ROWS, COLS>::cross_matrix() const
{
  static_assert(ROWS == 3U && COLS == 1U, "MatrixF32::cross_matrix(): 只有 3×1 向量可调用");
  MatrixF32<3, 3> result;
  result._data[1] = -_data[2];
  result._data[2] = _data[1];
  result._data[3] = _data[2];
  result._data[5] = -_data[0];
  result._data[6] = -_data[1];
  result._data[7] = _data[0];
  return result;
}

template <uint32_t ROWS, uint32_t COLS>
float MatrixF32<ROWS, COLS>::scalar() const
{
  static_assert(ROWS == 1U && COLS == 1U, "MatrixF32::scalar(): 只有 1×1 可调用");
  return _data[0];
}

#endif // __MATRIX_HPP__
