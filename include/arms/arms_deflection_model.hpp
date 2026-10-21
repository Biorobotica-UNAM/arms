#pragma once

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstddef>
#include <utility>
#include <vector>

namespace arms
{

struct Vec3
{
  double x{0.0}, y{0.0}, z{0.0};
};

inline Vec3 operator+(const Vec3 & a, const Vec3 & b) { return {a.x + b.x, a.y + b.y, a.z + b.z}; }
inline Vec3 operator-(const Vec3 & a, const Vec3 & b) { return {a.x - b.x, a.y - b.y, a.z - b.z}; }
inline double dot(const Vec3 & a, const Vec3 & b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3 & a, const Vec3 & b)
{
  return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}

struct Mat3
{
  double m[3][3]{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};

  Vec3 operator*(const Vec3 & v) const
  {
    return {
      m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
      m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
      m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
  }

  Mat3 operator*(const Mat3 & o) const
  {
    Mat3 r;
    for (int i = 0; i < 3; ++i) {
      for (int j = 0; j < 3; ++j) {
        r.m[i][j] = m[i][0] * o.m[0][j] + m[i][1] * o.m[1][j] + m[i][2] * o.m[2][j];
      }
    }
    return r;
  }
};

inline Mat3 axisAngle(const Vec3 & a, double th)
{
  const double c = std::cos(th), s = std::sin(th), t = 1.0 - c;
  Mat3 R;
  R.m[0][0] = c + t * a.x * a.x;
  R.m[0][1] = t * a.x * a.y - s * a.z;
  R.m[0][2] = t * a.x * a.z + s * a.y;
  R.m[1][0] = t * a.x * a.y + s * a.z;
  R.m[1][1] = c + t * a.y * a.y;
  R.m[1][2] = t * a.y * a.z - s * a.x;
  R.m[2][0] = t * a.x * a.z - s * a.y;
  R.m[2][1] = t * a.y * a.z + s * a.x;
  R.m[2][2] = c + t * a.z * a.z;
  return R;
}

class ArmDeflectionModel
{
public:
  static constexpr double kGravity = 9.81;

  struct Joint
  {
    Vec3 xyz; 
    Vec3 axis{0.0, 0.0, 1.0};
    double mass{0.0};
    Vec3 com;    
    double stiffness{0.0};  
    double hidden{1.0};    
    double max_defl{0.5};
  };

  void configure(std::vector<Joint> joints, const Vec3 & payload_point)
  {
    joints_ = std::move(joints);
    for (auto & j : joints_) {
      const double n = std::sqrt(dot(j.axis, j.axis));
      if (n > 1e-9) {
        j.axis = {j.axis.x / n, j.axis.y / n, j.axis.z / n};
      }
      j.hidden = std::clamp(j.hidden, 0.0, 1.0);
      j.max_defl = std::max(0.0, j.max_defl);
    }
    payload_point_ = payload_point;
  }

  size_t size() const { return joints_.size(); }

  void setPayload(double kg) { payload_mass_.store(std::max(0.0, kg)); }
  double payload() const { return payload_mass_.load(); }

  std::vector<double> gravityTorque(const std::vector<double> & q) const
  {
    const size_t n = joints_.size();
    std::vector<double> tau(n, 0.0);
    if (q.size() != n) {
      return tau;
    }

    std::vector<Vec3> p(n), a(n), c(n);
    Mat3 R;
    Vec3 pos;
    for (size_t i = 0; i < n; ++i) {
      pos = pos + R * joints_[i].xyz;
      a[i] = R * joints_[i].axis;
      p[i] = pos;
      R = R * axisAngle(joints_[i].axis, q[i]);
      c[i] = pos + R * joints_[i].com;
    }

    const double m_payload = payload_mass_.load();
    const Vec3 payload_pos = pos + R * payload_point_;
    const Vec3 g{0.0, 0.0, -kGravity};

    for (size_t i = 0; i < n; ++i) {
      for (size_t j = i; j < n; ++j) {
        tau[i] += joints_[j].mass * dot(g, cross(a[i], c[j] - p[i]));
      }
      if (m_payload > 0.0) {
        tau[i] += m_payload * dot(g, cross(a[i], payload_pos - p[i]));
      }
    }
    return tau;
  }

  std::vector<double> deflection(const std::vector<double> & q) const
  {
    std::vector<double> d = gravityTorque(q);
    for (size_t i = 0; i < d.size(); ++i) {
      if (joints_[i].stiffness <= 0.0) {
        d[i] = 0.0;
      } else {
        d[i] = std::clamp(d[i] / joints_[i].stiffness, -joints_[i].max_defl, joints_[i].max_defl);
      }
    }
    return d;
  }

  std::vector<double> commandFromDesired(const std::vector<double> & q_des) const
  {
    std::vector<double> cmd = q_des;
    const auto d = deflection(q_des);
    for (size_t i = 0; i < cmd.size() && i < d.size(); ++i) {
      cmd[i] -= d[i];
    }
    return cmd;
  }

  std::vector<double> desiredFromEncoder(const std::vector<double> & enc) const
  {
    std::vector<double> q = enc;
    if (enc.size() != joints_.size()) {
      return q;
    }
    for (int it = 0; it < 8; ++it) {
      const auto d = deflection(q);
      double err = 0.0;
      for (size_t i = 0; i < q.size(); ++i) {
        const double qn = enc[i] + joints_[i].hidden * d[i];
        err = std::max(err, std::abs(qn - q[i]));
        q[i] = qn;
      }
      if (err < 1e-6) {
        break;
      }
    }
    return q;
  }

private:
  std::vector<Joint> joints_;
  Vec3 payload_point_;
  std::atomic<double> payload_mass_{0.0};
};

}
