#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

enum class ViewMode { Speed = 0, Heat, Pressure, Acceleration };

inline const char* viewName(ViewMode m) {
  switch (m) {
    case ViewMode::Speed: return "speed";
    case ViewMode::Heat: return "heat";
    case ViewMode::Pressure: return "pressure";
    default: return "acceleration";
  }
}

// t in [0,1]:  0 = purple, then blue, green, yellow, orange, 1 = red.
inline void spectrum(float t, float rgb[3]) {
  static const float stops[6][3] = {
      {0.55f, 0.00f, 0.90f},  // purple
      {0.00f, 0.25f, 1.00f},  // blue
      {0.00f, 0.85f, 0.20f},  // green
      {1.00f, 0.95f, 0.00f},  // yellow
      {1.00f, 0.50f, 0.00f},  // orange
      {1.00f, 0.00f, 0.00f},  // red
  };
  t = std::min(1.0f, std::max(0.0f, t)) * 5.0f;
  int i = std::min(4, (int)t);
  float f = t - (float)i;
  for (int c = 0; c < 3; ++c) rgb[c] = stops[i][c] + (stops[i + 1][c] - stops[i][c]) * f;
}

// Maps a metric to [0,1]. The range adapts smoothly to the data (1st..99th
// percentile) so a few outliers do not wash the picture out. Speed and
// acceleration are anchored at 0 so "zero" is always purple.
struct Range {
  float lo = 0, hi = 1;
  bool init = false;
  std::vector<float> scratch;

  void update(const std::vector<float>& v, bool zeroBased, float minSpan) {
    if (v.empty()) return;
    scratch = v;
    size_t n = scratch.size();
    size_t iHi = std::min(n - 1, (size_t)(0.99 * (double)n));
    size_t iLo = (size_t)(0.01 * (double)n);
    std::nth_element(scratch.begin(), scratch.begin() + iHi, scratch.end());
    float tHi = scratch[iHi];
    std::nth_element(scratch.begin(), scratch.begin() + iLo, scratch.begin() + iHi + 1);
    float tLo = zeroBased ? 0.0f : scratch[iLo];
    if (tHi - tLo < minSpan) tHi = tLo + minSpan;
    if (!init) { lo = tLo; hi = tHi; init = true; }
    else { lo += (tLo - lo) * 0.08f; hi += (tHi - hi) * 0.08f; }
  }
  float map(float x) const { return (x - lo) / std::max(1e-9f, hi - lo); }
};
