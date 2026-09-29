#include "service/TableBallisticSolver.hpp"

#include "dto/BallisticTableDTO.hpp"

#include <algorithm>
#include <cstddef>
#include <vector>

namespace {

struct Interp {
  int lo;
  float frac;
};

Interp find_interp(float value, const std::vector<float>& axis)
{
  if (axis.size() < 2 || value <= axis.front()) {
    return {0, 0.0F};
  }
  if (value >= axis.back()) {
    return {static_cast<int>(axis.size()) - 2, 1.0F};
  }

  auto it = std::lower_bound(axis.begin(), axis.end(), value);
  int i = static_cast<int>(it - axis.begin()) - 1;
  if (i < 0) {
    i = 0;
  }

  const float frac = (value - axis[i]) / (axis[i + 1] - axis[i]);  // real gap -> non-uniform spacing
  return {i, frac};
}

BallisticTableDTO::Result lerp(const BallisticTableDTO::Result& a, const BallisticTableDTO::Result& b, float t)
{
  return {
    a.ammo_time_to_fall + (b.ammo_time_to_fall - a.ammo_time_to_fall) * t,
    a.ammo_distance_to_fall + (b.ammo_distance_to_fall - a.ammo_distance_to_fall) * t,
  };
}

std::size_t flat_index(const BallisticTableDTO& t, int iz, int iv, int im, int id, int il)
{
  return (((static_cast<std::size_t>(iz) * t.v.size() + iv) * t.m.size() + im) * t.d.size() + id) * t.l.size() + il;
}

}  // namespace

FallResult TableBallisticSolver::fall(const Ammo& ammo, float altitude, float speed) const
{
  const BallisticTableDTO& t = table_;

  const Interp iz = find_interp(altitude, t.z);
  const Interp iv = find_interp(speed, t.v);
  const Interp im = find_interp(ammo.get_mass(), t.m);
  const Interp id = find_interp(ammo.get_drag(), t.d);
  const Interp il = find_interp(ammo.get_lift(), t.l);

  const auto at = [&t](int a, int b, int c, int e, int f) -> const BallisticTableDTO::Result& { return t.data[flat_index(t, a, b, c, e, f)]; };

  BallisticTableDTO::Result v[16];
  for (int a = 0; a < 2; ++a) {
    for (int b = 0; b < 2; ++b) {
      for (int c = 0; c < 2; ++c) {
        for (int e = 0; e < 2; ++e) {
          const auto& lo = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo);
          const auto& hi = at(iz.lo + a, iv.lo + b, im.lo + c, id.lo + e, il.lo + 1);
          v[a * 8 + b * 4 + c * 2 + e] = lerp(lo, hi, il.frac);
        }
      }
    }
  }

  BallisticTableDTO::Result w[8];
  for (int a = 0; a < 2; ++a) {
    for (int b = 0; b < 2; ++b) {
      for (int c = 0; c < 2; ++c) {
        w[a * 4 + b * 2 + c] = lerp(v[a * 8 + b * 4 + c * 2], v[a * 8 + b * 4 + c * 2 + 1], id.frac);
      }
    }
  }

  BallisticTableDTO::Result u[4];
  for (int a = 0; a < 2; ++a) {
    for (int b = 0; b < 2; ++b) {
      u[a * 2 + b] = lerp(w[a * 4 + b * 2], w[a * 4 + b * 2 + 1], im.frac);
    }
  }

  BallisticTableDTO::Result s[2];
  for (int a = 0; a < 2; ++a) {
    s[a] = lerp(u[a * 2], u[a * 2 + 1], iv.frac);
  }

  const BallisticTableDTO::Result out = lerp(s[0], s[1], iz.frac);

  return {out.ammo_time_to_fall, out.ammo_distance_to_fall};
}
