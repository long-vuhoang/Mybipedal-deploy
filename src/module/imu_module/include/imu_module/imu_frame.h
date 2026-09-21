#pragma once

// Chuyển dữ liệu IMU từ khung của cảm biến sang khung mà policy (và sim MuJoCo) giả định:
//   thân  : FLU  (x phía trước, y sang trái, z hướng lên)
//   thế giới: ENU (z hướng lên)  -> trọng lực chiếu vào thân = R^T * (0,0,-1)
//
// Cấu hình (imu.yaml), mặc định TẮT (không có khối imu_frame = không đổi gì):
//   imu_frame:
//     body_signs: [1, -1, -1]   # base = diag(s) * imu ; phải là phép quay thật (det = +1)
//     world_z_down: true        # khung thế giới của IMU có z hướng xuống (NED)
//     quat_inverse: false       # quaternion của IMU là thế giới->thân (ngược)
//
// Công thức:  q_out = qW ⊗ q_in ⊗ qB ,  vector_out = diag(s) * vector_in
//   qB : quay thật tương ứng diag(s);  qW : NED->ENU (quay 180° quanh (1,1,0)/√2) nếu world_z_down.

#include <array>
#include <cmath>
#include <stdexcept>

namespace mybipedal_deploy::imu_module {

struct ImuFrameFix {
  using Quat = std::array<double, 4>;  // (w, x, y, z)

  bool enabled = false;
  std::array<double, 3> s{1, 1, 1};
  bool world_z_down = false;
  bool quat_inverse = false;
  Quat qB{1, 0, 0, 0};
  Quat qW{1, 0, 0, 0};

  static Quat Mul(const Quat& a, const Quat& b) {
    return {a[0] * b[0] - a[1] * b[1] - a[2] * b[2] - a[3] * b[3],
            a[0] * b[1] + a[1] * b[0] + a[2] * b[3] - a[3] * b[2],
            a[0] * b[2] - a[1] * b[3] + a[2] * b[0] + a[3] * b[1],
            a[0] * b[3] + a[1] * b[2] - a[2] * b[1] + a[3] * b[0]};
  }
  static Quat Conj(const Quat& q) { return {q[0], -q[1], -q[2], -q[3]}; }

  void Init(const std::array<double, 3>& body_signs, bool z_down, bool inverse) {
    for (double v : body_signs) {
      if (std::fabs(std::fabs(v) - 1.0) > 1e-9) throw std::invalid_argument("imu_frame.body_signs must be +1 or -1");
    }
    s = body_signs;
    const double det = s[0] * s[1] * s[2];
    if (det < 0) throw std::invalid_argument("imu_frame.body_signs must be a proper rotation (product = +1)");
    if (s[0] > 0 && s[1] > 0) qB = {1, 0, 0, 0};        // (+,+,+)
    else if (s[0] > 0 && s[1] < 0) qB = {0, 1, 0, 0};   // (+,-,-)  quay 180° quanh x
    else if (s[0] < 0 && s[1] > 0) qB = {0, 0, 1, 0};   // (-,+,-)  quay 180° quanh y
    else qB = {0, 0, 0, 1};                              // (-,-,+)  quay 180° quanh z
    world_z_down = z_down;
    quat_inverse = inverse;
    const double r = std::sqrt(0.5);
    qW = z_down ? Quat{0, r, r, 0} : Quat{1, 0, 0, 0};
    enabled = true;
  }

  void ApplyVec(double& x, double& y, double& z) const {
    if (!enabled) return;
    x *= s[0];
    y *= s[1];
    z *= s[2];
  }

  void ApplyQuat(double& w, double& x, double& y, double& z) const {
    if (!enabled) return;
    Quat q{w, x, y, z};
    if (quat_inverse) q = Conj(q);
    Quat o = Mul(Mul(qW, q), qB);
    const double n = std::sqrt(o[0] * o[0] + o[1] * o[1] + o[2] * o[2] + o[3] * o[3]);
    if (n > 1e-9) for (auto& v : o) v /= n;
    w = o[0]; x = o[1]; y = o[2]; z = o[3];
  }
};

}  // namespace mybipedal_deploy::imu_module
