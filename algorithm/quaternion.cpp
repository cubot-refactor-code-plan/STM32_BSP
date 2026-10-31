#include "quaternion.hpp"

#include <math.h>


namespace
{
constexpr double SMALL_ANGLE = 1e-9;   ///< 转角 / 虚部模长低于它时按一阶近似，避免 0 / 0
constexpr double SLERP_LERP  = 0.9995; ///< 两姿态 |dot| 高于它（夹角约 < 3.6°）时 slerp 退化为线性插值再归一化
} // namespace


// ---------------- Vector3 ----------------

double Vector3::norm() const
{
  return sqrt(x * x + y * y + z * z);
}

// ----------------


// ---------------- 构造 ----------------

Quaternion Quaternion::identity()
{
  return {1.0, 0.0, 0.0, 0.0};
}

Quaternion Quaternion::from_array(const double q[4])
{
  return {q[0], q[1], q[2], q[3]};
}

Quaternion Quaternion::from_axis_angle(const Vector3 &axis, double angle)
{
  const double n = axis.norm();
  if (n < SMALL_ANGLE)
  {
    return identity();
  }
  const double s = sin(angle / 2.0) / n;
  return {cos(angle / 2.0), axis.x * s, axis.y * s, axis.z * s};
}

Quaternion Quaternion::from_rotation_vector(const Vector3 &rv)
{
  const double angle = rv.norm();
  if (angle < SMALL_ANGLE)
  {
    // sin(θ/2) / θ → 1/2
    return Quaternion {1.0, rv.x / 2.0, rv.y / 2.0, rv.z / 2.0}.normalized();
  }
  const double s = sin(angle / 2.0) / angle;
  return {cos(angle / 2.0), rv.x * s, rv.y * s, rv.z * s};
}

void Quaternion::to_array(double out[4]) const
{
  out[0] = w;
  out[1] = x;
  out[2] = y;
  out[3] = z;
}

// ----------------


// ---------------- 基本运算 ----------------

Quaternion Quaternion::operator*(const Quaternion &rhs) const
{
  return {w * rhs.w - x * rhs.x - y * rhs.y - z * rhs.z,
          w * rhs.x + x * rhs.w + y * rhs.z - z * rhs.y,
          w * rhs.y - x * rhs.z + y * rhs.w + z * rhs.x,
          w * rhs.z + x * rhs.y - y * rhs.x + z * rhs.w};
}

Quaternion Quaternion::conj() const
{
  return {w, -x, -y, -z};
}

double Quaternion::norm() const
{
  return sqrt(w * w + x * x + y * y + z * z);
}

Quaternion Quaternion::normalized() const
{
  const double n = norm();
  if (n <= 0.0)
  {
    return identity();
  }
  return {w / n, x / n, y / n, z / n};
}

double Quaternion::dot(const Quaternion &rhs) const
{
  return w * rhs.w + x * rhs.x + y * rhs.y + z * rhs.z;
}

// ----------------


// ---------------- 旋转 ----------------

Vector3 Quaternion::rotate(const Vector3 &v) const
{
  // v' = v + w·t + u × t，其中 u = (x, y, z)，t = 2·(u × v)；比两次四元数乘法省一半运算
  const double tx = 2.0 * (y * v.z - z * v.y);
  const double ty = 2.0 * (z * v.x - x * v.z);
  const double tz = 2.0 * (x * v.y - y * v.x);
  return {v.x + w * tx + (y * tz - z * ty),
          v.y + w * ty + (z * tx - x * tz),
          v.z + w * tz + (x * ty - y * tx)};
}

Vector3 Quaternion::inverse_rotate(const Vector3 &v) const
{
  return conj().rotate(v);
}

void Quaternion::to_matrix(double r[3][3]) const
{
  r[0][0] = 1.0 - 2.0 * (y * y + z * z);
  r[0][1] = 2.0 * (x * y - w * z);
  r[0][2] = 2.0 * (x * z + w * y);
  r[1][0] = 2.0 * (x * y + w * z);
  r[1][1] = 1.0 - 2.0 * (x * x + z * z);
  r[1][2] = 2.0 * (y * z - w * x);
  r[2][0] = 2.0 * (x * z - w * y);
  r[2][1] = 2.0 * (y * z + w * x);
  r[2][2] = 1.0 - 2.0 * (x * x + y * y);
}

Vector3 Quaternion::rotation_vector() const
{
  // q 与 -q 同一姿态：取 w ≥ 0 的那个，转角落在 [0, π]
  const double sign = (w < 0.0) ? -1.0 : 1.0;
  const double qw   = sign * w;
  const double qx   = sign * x;
  const double qy   = sign * y;
  const double qz   = sign * z;

  const double s = sqrt(qx * qx + qy * qy + qz * qz);
  // 转角 θ = 2·atan2(s, w)（比 2·acos(w) 在小角度时精度高）；小角度时 θ / s → 2 / w
  const double k = (s < SMALL_ANGLE) ? (2.0 / qw) : (2.0 * atan2(s, qw) / s);
  return {qx * k, qy * k, qz * k};
}

// ----------------


// ---------------- 控制 ----------------

Vector3 Quaternion::error_to(const Quaternion &target) const
{
  // target = this ⊗ q_err  →  q_err = this* ⊗ target，误差表在本机体系
  return (conj() * target).rotation_vector();
}

Quaternion Quaternion::integrate(const Vector3 &omega_body, double dt) const
{
  // 机体系角速度：右乘
  return ((*this) * from_rotation_vector({omega_body.x * dt, omega_body.y * dt, omega_body.z * dt})).normalized();
}

Quaternion Quaternion::slerp(const Quaternion &a, const Quaternion &b, double t)
{
  // 走短路径：点积为负时用 -b（同一姿态）
  double     d  = a.dot(b);
  Quaternion bb = b;
  if (d < 0.0)
  {
    d  = -d;
    bb = {-b.w, -b.x, -b.y, -b.z};
  }

  double ka = 1.0 - t;
  double kb = t;
  if (d < SLERP_LERP) // 夹角太小时 sin(θ) 趋 0，按线性插值
  {
    const double theta = acos(d);
    const double s     = sin(theta);
    ka                 = sin((1.0 - t) * theta) / s;
    kb                 = sin(t * theta) / s;
  }
  return Quaternion {ka * a.w + kb * bb.w, ka * a.x + kb * bb.x, ka * a.y + kb * bb.y, ka * a.z + kb * bb.z}.normalized();
}

// ----------------


// ---------------- 取角 ----------------

void Quaternion::pointing(const Vector3 &axis_body, double &yaw, double &elevation) const
{
  const Vector3 f = rotate(axis_body);
  yaw             = atan2(f.y, f.x);
  elevation       = atan2(f.z, sqrt(f.x * f.x + f.y * f.y)); // 比 asin(f.z / |f|) 在 ±90° 附近精度高，也不怕舍入越界
}

void Quaternion::to_euler_zyx(double &yaw, double &pitch, double &roll) const
{
  // pitch 在 ±90° 附近时 asin 的参数可能因舍入略超出 ±1，先限幅
  double sinp = 2.0 * (w * y - z * x);
  if (sinp > 1.0)
  {
    sinp = 1.0;
  }
  else if (sinp < -1.0)
  {
    sinp = -1.0;
  }
  yaw   = atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z));
  pitch = asin(sinp);
  roll  = atan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y));
}

// ----------------


// ---------------- 角度 ----------------

double angle_wrap_pi(double rad)
{
  return remainder(rad, 2.0 * PI_D); // IEEE 余数：结果在 [-π, π]，任意大小的输入一步到位
}

double angle_diff(double target, double current)
{
  return angle_wrap_pi(target - current);
}

// ----------------
