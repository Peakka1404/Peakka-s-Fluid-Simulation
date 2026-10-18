#pragma once
// 2D Smoothed Particle Hydrodynamics (Mueller et al. 2003 style, weakly
// compressible).  Poly6 density, Spiky pressure gradient, viscosity/heat
// Laplacian.  Neighbour search with a uniform hash grid.
#include <algorithm>
#include <cmath>
#include <random>
#include "config.h"
#include "solver.h"

class SphSolver : public Solver {
 public:
  SphSolver() { calibrateMass(); reset(); }
  const char* name() const override { return "SPH"; }
  float particleRadius() const override { return 0.55f * cfg::kSphSpacing; }
  const ParticleSet& particles() const override { return out_; }

  void reset() override {
    using namespace cfg;
    std::mt19937 rng(1234);
    std::uniform_real_distribution<float> jit(-0.05f * kSphSpacing, 0.05f * kSphSpacing);
    px_.clear(); py_.clear();
    float x0 = 0.5f * kSphSpacing, y0 = 0.5f * kSphSpacing;
    int nx = (int)(kSphBlockW / kSphSpacing), ny = (int)(kSphBlockH / kSphSpacing);
    for (int j = 0; j < ny; ++j)
      for (int i = 0; i < nx; ++i) {
        px_.push_back(x0 + i * kSphSpacing + jit(rng));
        py_.push_back(y0 + j * kSphSpacing + jit(rng));
      }
    n_ = px_.size();
    vx_.assign(n_, 0); vy_.assign(n_, 0);
    ax_.assign(n_, 0); ay_.assign(n_, 0);
    rho_.assign(n_, kSphRestDens); prs_.assign(n_, 0);
    T_.assign(n_, kTCold); dT_.assign(n_, 0);
    accS_.assign(n_, 0); prsS_.assign(n_, 0);
    order_.resize(n_);
    cellsX_ = (int)(kWorldW / kSphH) + 2;
    cellsY_ = (int)(kWorldH / kSphH) + 2;
    cellStart_.assign(cellsX_ * cellsY_ + 1, 0);
    out_.resize(n_);
    publish();
  }

  void interact(float x, float y, float vx, float vy) override {
    const float R = 0.12f;
    for (size_t i = 0; i < n_; ++i) {
      float dx = px_[i] - x, dy = py_[i] - y, d = std::sqrt(dx * dx + dy * dy);
      if (d < R) {
        float w = 1.0f - d / R;
        vx_[i] += 0.35f * w * vx;
        vy_[i] += 0.35f * w * vy;
      }
    }
  }

  void step() override {
    for (int s = 0; s < cfg::kSphSubsteps; ++s) substep(cfg::kSphDt);
    publish();
  }

 private:
  // --- kernels (2D) ---
  static constexpr float kPi = 3.14159265358979f;
  const float H = cfg::kSphH, H2 = H * H;
  const float poly6_ = 4.0f / (kPi * std::pow(cfg::kSphH, 8.0f));
  const float spiky_ = -30.0f / (kPi * std::pow(cfg::kSphH, 5.0f));  // gradient
  const float lap_   = 40.0f / (kPi * std::pow(cfg::kSphH, 5.0f));   // viscous laplacian

  // Choose the particle mass so a perfect lattice has exactly rest density.
  void calibrateMass() {
    float s = cfg::kSphSpacing, sum = 0;
    int r = (int)std::ceil(H / s) + 1;
    for (int j = -r; j <= r; ++j)
      for (int i = -r; i <= r; ++i) {
        float r2 = (i * s) * (i * s) + (j * s) * (j * s);
        if (r2 < H2) { float d = H2 - r2; sum += poly6_ * d * d * d; }
      }
    mass_ = cfg::kSphRestDens / sum;

    // Walls are treated as a half-space of virtual rest-density fluid on a
    // lattice that continues the fluid lattice.  Tabulate, versus distance d
    // from the wall, its density contribution and its summed kernel gradient.
    for (int t = 0; t <= kTab; ++t) {
      float d = H * (float)t / kTab, rho = 0, g = 0;
      for (int k = 0; k <= r + 1; ++k) {
        float xv = -(k + 0.5f) * s;
        for (int j = -r; j <= r; ++j) {
          float dx = d - xv, dy = -(j * s), r2 = dx * dx + dy * dy;
          if (r2 >= H2 || r2 < 1e-12f) continue;
          float rr = std::sqrt(r2), q = H2 - r2;
          rho += mass_ * poly6_ * q * q * q;
          g += mass_ * spiky_ * (H - rr) * (H - rr) * dx / rr;   // x-component of m*grad W (<0)
        }
      }
      wallRho_[t] = rho; wallGrad_[t] = g;
    }
  }

  // linear lookup of a wall table at distance d (0 beyond the support)
  static float lookup(const float* tab, float d, float H) {
    if (d >= H) return 0.0f;
    float f = std::max(0.0f, d) / H * kTab; int i = std::min(kTab - 1, (int)f);
    return tab[i] + (tab[i + 1] - tab[i]) * (f - (float)i);
  }

  int cellOf(float x, float y) const {
    int cx = std::min(cellsX_ - 1, std::max(0, (int)(x / H)));
    int cy = std::min(cellsY_ - 1, std::max(0, (int)(y / H)));
    return cy * cellsX_ + cx;
  }

  void buildGrid() {
    std::fill(cellStart_.begin(), cellStart_.end(), 0);
    cellOfP_.resize(n_);
    for (size_t i = 0; i < n_; ++i) { cellOfP_[i] = cellOf(px_[i], py_[i]); cellStart_[cellOfP_[i] + 1]++; }
    for (size_t c = 1; c < cellStart_.size(); ++c) cellStart_[c] += cellStart_[c - 1];
    fillPos_.assign(cellStart_.begin(), cellStart_.end() - 1);
    for (size_t i = 0; i < n_; ++i) order_[fillPos_[cellOfP_[i]]++] = (int)i;
  }

  template <class F>
  inline void forNeighbours(size_t i, F&& f) const {
    int c = cellOfP_[i], cx = c % cellsX_, cy = c / cellsX_;
    for (int y = std::max(0, cy - 1); y <= std::min(cellsY_ - 1, cy + 1); ++y)
      for (int x = std::max(0, cx - 1); x <= std::min(cellsX_ - 1, cx + 1); ++x) {
        int cc = y * cellsX_ + x;
        for (int k = cellStart_[cc]; k < cellStart_[cc + 1]; ++k) {
          int j = order_[k];
          if ((size_t)j != i) f((size_t)j);
        }
      }
  }

  void substep(float dt) {
    using namespace cfg;
    buildGrid();
    const int N = (int)n_;
    // 1) density + pressure
#pragma omp parallel for schedule(static)
    for (int i = 0; i < N; ++i) {
      float rho = mass_ * poly6_ * H2 * H2 * H2;  // self contribution
      forNeighbours(i, [&](size_t j) {
        float dx = px_[j] - px_[i], dy = py_[j] - py_[i], r2 = dx * dx + dy * dy;
        if (r2 < H2) { float d = H2 - r2; rho += mass_ * poly6_ * d * d * d; }
      });
      rho += lookup(wallRho_, px_[i], H) + lookup(wallRho_, kWorldW - px_[i], H) +
             lookup(wallRho_, py_[i], H) + lookup(wallRho_, kWorldH - py_[i], H);
      rho_[i] = rho;
      prs_[i] = std::max(0.0f, kSphStiffness * (rho - kSphRestDens));
    }
    // 2) forces (acceleration) + heat diffusion
#pragma omp parallel for schedule(static)
    for (int i = 0; i < N; ++i) {
      float fx = 0, fy = 0, dT = 0;
      forNeighbours(i, [&](size_t j) {
        float dx = px_[i] - px_[j], dy = py_[i] - py_[j], r2 = dx * dx + dy * dy;
        if (r2 >= H2 || r2 < 1e-12f) return;
        float r = std::sqrt(r2), inv = 1.0f / r;
        float pk = -mass_ * (prs_[i] + prs_[j]) / (2.0f * rho_[i] * rho_[j]) * spiky_ * (H - r) * (H - r);
        fx += pk * dx * inv; fy += pk * dy * inv;
        float w = mass_ / rho_[j] * lap_ * (H - r);
        fx += kSphViscosity * w * (vx_[j] - vx_[i]);
        fy += kSphViscosity * w * (vy_[j] - vy_[i]);
        dT += kSphThermalDiff * w * (T_[j] - T_[i]);
      });
      // wall pressure: virtual wall fluid with p_j = p_i, rho_j = rho0
      float wk = -prs_[i] / (rho_[i] * kSphRestDens);
      fx += wk * (lookup(wallGrad_, px_[i], H) - lookup(wallGrad_, kWorldW - px_[i], H));
      fy += wk * (lookup(wallGrad_, py_[i], H) - lookup(wallGrad_, kWorldH - py_[i], H));
      // gravity with thermal buoyancy: hot fluid feels less weight
      float g = kGravity * (1.0f - kBuoyancy * (T_[i] - kTCold));
      ax_[i] = fx;
      ay_[i] = fy - g;
      dT_[i] = dT;
    }
    // 3) integrate (symplectic Euler) + walls + heat sources
    const float s = kSphSpacing, margin = 0.5f * s, vmax = 12.0f;
    const float xMin = margin, xMax = kWorldW - margin, yMin = margin, yMax = kWorldH - margin;
    const float hx0 = kHeaterX0 * kWorldW, hx1 = kHeaterX1 * kWorldW;
    for (size_t i = 0; i < n_; ++i) {
      vx_[i] += ax_[i] * dt; vy_[i] += ay_[i] * dt;
      float sp = std::sqrt(vx_[i] * vx_[i] + vy_[i] * vy_[i]);
      if (sp > vmax) { vx_[i] *= vmax / sp; vy_[i] *= vmax / sp; }
      px_[i] += vx_[i] * dt; py_[i] += vy_[i] * dt;
      if (px_[i] < xMin) { px_[i] = xMin; if (vx_[i] < 0) vx_[i] *= -0.3f; }
      if (px_[i] > xMax) { px_[i] = xMax; if (vx_[i] > 0) vx_[i] *= -0.3f; }
      if (py_[i] < yMin) { py_[i] = yMin; if (vy_[i] < 0) vy_[i] *= -0.3f; }
      if (py_[i] > yMax) { py_[i] = yMax; if (vy_[i] > 0) vy_[i] *= -0.3f; }
      float T = T_[i] + dT_[i] * dt;
      if (py_[i] < 4.0f * s && px_[i] > hx0 && px_[i] < hx1) T += (kTHot - T) * std::min(1.0f, 6.0f * dt);  // heater
      T += (kTCold - T) * std::min(1.0f, 0.15f * dt);                                                  // slow ambient loss
      T_[i] = std::min(kTHot, std::max(kTCold, T));
      // smoothed diagnostics (per-sub-step EMA keeps the colours calm)
      float a = std::sqrt(ax_[i] * ax_[i] + ay_[i] * ay_[i]);
      accS_[i] += (a - accS_[i]) * 0.04f;
      prsS_[i] += (prs_[i] - prsS_[i]) * 0.04f;
    }
  }

  void publish() {
    for (size_t i = 0; i < n_; ++i) {
      out_.x[i] = px_[i]; out_.y[i] = py_[i];
      out_.speed[i] = std::sqrt(vx_[i] * vx_[i] + vy_[i] * vy_[i]);
      out_.heat[i] = T_[i]; out_.pressure[i] = prsS_[i]; out_.accel[i] = accS_[i];
    }
  }

  size_t n_ = 0;
  static constexpr int kTab = 64;
  float wallRho_[kTab + 1] = {}, wallGrad_[kTab + 1] = {};
  float mass_ = 1;
  std::vector<float> px_, py_, vx_, vy_, ax_, ay_, rho_, prs_, T_, dT_, accS_, prsS_;
  std::vector<int> order_, cellStart_, cellOfP_, fillPos_;
  int cellsX_ = 1, cellsY_ = 1;
  ParticleSet out_;
};
