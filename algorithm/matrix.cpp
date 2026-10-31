#include "matrix.hpp"


/* ==================== 常用向量 ==================== */

MatrixF32<2, 1> matrix_axis_x_2d()
{
  MatrixF32<2, 1> result;
  result[0][0] = 1.0f;
  return result;
}

MatrixF32<2, 1> matrix_axis_y_2d()
{
  MatrixF32<2, 1> result;
  result[1][0] = 1.0f;
  return result;
}

MatrixF32<3, 1> matrix_axis_x_3d()
{
  MatrixF32<3, 1> result;
  result[0][0] = 1.0f;
  return result;
}

MatrixF32<3, 1> matrix_axis_y_3d()
{
  MatrixF32<3, 1> result;
  result[1][0] = 1.0f;
  return result;
}

MatrixF32<3, 1> matrix_axis_z_3d()
{
  MatrixF32<3, 1> result;
  result[2][0] = 1.0f;
  return result;
}


/* ==================== 旋转矩阵 ==================== */

MatrixF32<2, 2> matrix_rotation_2d(float angle)
{
  const float c = cosf(angle);
  const float s = sinf(angle);

  MatrixF32<2, 2> result;
  result[0][0] = c;
  result[0][1] = -s;
  result[1][0] = s;
  result[1][1] = c;
  return result;
}

MatrixF32<3, 3> matrix_from_axis_angle(const MatrixF32<3, 1> &axis, float angle)
{
  const float n = axis.norm();
  if (!(n > 0.0f))
  {
    return MatrixF32<3, 3>::identity();
  }
  const float x = axis[0][0] / n;
  const float y = axis[1][0] / n;
  const float z = axis[2][0] / n;
  const float c = cosf(angle);
  const float s = sinf(angle);
  const float t = 1.0f - c;

  // Rodrigues：R = c·I + s·[n]× + (1 - c)·n·nᵀ
  MatrixF32<3, 3> result;
  result[0][0] = c + x * x * t;
  result[0][1] = x * y * t - z * s;
  result[0][2] = x * z * t + y * s;
  result[1][0] = y * x * t + z * s;
  result[1][1] = c + y * y * t;
  result[1][2] = y * z * t - x * s;
  result[2][0] = z * x * t - y * s;
  result[2][1] = z * y * t + x * s;
  result[2][2] = c + z * z * t;
  return result;
}

MatrixF32<3, 3> matrix_from_euler_zyx(float yaw, float pitch, float roll)
{
  const float cy = cosf(yaw);
  const float sy = sinf(yaw);
  const float cp = cosf(pitch);
  const float sp = sinf(pitch);
  const float cr = cosf(roll);
  const float sr = sinf(roll);

  // R = Rz(yaw) · Ry(pitch) · Rx(roll)
  MatrixF32<3, 3> result;
  result[0][0] = cy * cp;
  result[0][1] = cy * sp * sr - sy * cr;
  result[0][2] = cy * sp * cr + sy * sr;
  result[1][0] = sy * cp;
  result[1][1] = sy * sp * sr + cy * cr;
  result[1][2] = sy * sp * cr - cy * sr;
  result[2][0] = -sp;
  result[2][1] = cp * sr;
  result[2][2] = cp * cr;
  return result;
}


/* ==================== 叉乘 ==================== */

MatrixF32<3, 1> vector_cross_3d(const MatrixF32<3, 1> &a, const MatrixF32<3, 1> &b)
{
  MatrixF32<3, 1> result;
  result[0][0] = a[1][0] * b[2][0] - a[2][0] * b[1][0];
  result[1][0] = a[2][0] * b[0][0] - a[0][0] * b[2][0];
  result[2][0] = a[0][0] * b[1][0] - a[1][0] * b[0][0];
  return result;
}

float vector_cross_2d(const MatrixF32<2, 1> &a, const MatrixF32<2, 1> &b)
{
  return a[0][0] * b[1][0] - a[1][0] * b[0][0];
}
