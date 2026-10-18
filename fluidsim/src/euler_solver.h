#pragma once
// Grid-based Eulerian solver ("Stable Fluids", Stam 1999) on a staggered MAC
// grid: semi-Lagrangian advection, Gauss-Seidel/SOR pressure projection,
// Boussinesq thermal buoyancy.  A heated floor strip and a cooled ceiling
// drive Rayleigh-Benard style convection.
//
// Each cell field (pressure, temperature, acceleration) and the velocity are
// sampled by massless tracer particles, so every particle can be coloured
// independently - same interface as the SPH solver.
#include <algorithm>
#include <cmath>
#include <random>
#include "config.h"
#include "solver.h"

class EulerSolver : public Solver {
 public:
  EulerSolver() { reset(); }
  const char* name() const override { return "Eulerian grid"; }
  float particleRadius() const override { return 0.0055f * 2.0f; }
  const ParticleSet& particles() const override { return out_; }

  void reset() override {
    using namespace cfg;
    nx_ = kGridNX; ny_ = kGridNY; h_ = kWorldW / nx_;
    size_t N = (size_t)nx_ * ny_;
    u_.assign(N, 0); v_.assign(N, 0); nu_.assign(N, 0); nv_.assign(N, 0);
    p_.assign(N, 0); s_.assign(N, 1); T_.assign(N, kTCold); nT_.assign(N, kTCold);
    accC_.assign(N, 0); uc0_.assign(N, 0); vc0_.assign(N, 0);
    // solid border
    for (int i = 0; i < nx_; ++i) { s_[idx(i, 0)] = 0; s_[idx(i, ny_ - 1)] = 0; }
    for (int j = 0; j < ny_; ++j) { s_[idx(0, j)] = 0; s_[idx(nx_ - 1, j)] = 0; }
    std::mt19937 rng(7);
    std::uniform_real_distribution<float> noise(-0.5f, 0.5f), u01(0.0f, 1.0f);
    for (int j = 0; j < ny_; ++j)
      for (int i = 0; i < nx_; ++i) {
        float f = (float)j / (ny_ - 1);                         // 0 floor .. 1 ceiling
        T_[idx(i, j)] = kTHot + (kTCold - kTHot) * f + noise(rng);
      }
    // tracer particles (jittered lattice)
    int tx = kGridTracersX, ty = kGridTracersY;
    out_.resize((size_t)tx * ty);
    for (int j = 0; j < ty; ++j)
      for (int i = 0; i < tx; ++i) {
        size_t k = (size_t)j * tx + i;
        out_.x[k] = h_ + (kWorldW - 2 * h_) * (i + u01(rng)) / tx;
        out_.y[k] = h_ + (kWorldH - 2 * h_) * (j + u01(rng)) / ty;
      }
    sampleParticles();
  }

  void interact(float x, float y, float vx, float vy) override {
    const float R = 0.15f;
    for (int j = 1; j < ny_ - 1; ++j)
      for (int i = 1; i < nx_ - 1; ++i) {
        // u faces live at (i*h, (j+.5)h), v faces at ((i+.5)h, j*h)
        if (s_[idx(i, j)] && s_[idx(i - 1, j)]) {
          float d = std::hypot(i * h_ - x, (j + 0.5f) * h_ - y);
          if (d < R) u_[idx(i, j)] += 0.35f * (1 - d / R) * vx;
        }
        if (s_[idx(i, j)] && s_[idx(i, j - 1)]) {
          float d = std::hypot((i + 0.5f) * h_ - x, j * h_ - y);
          if (d < R) v_[idx(i, j)] += 0.35f * (1 - d / R) * vy;
        }
      }
  }

  void step() override {
    for (int k = 0; k < cfg::kGridSubsteps; ++k) substep(cfg::kGridDt);
    sampleParticles();
  }

 private:
  enum Field { FU, FV, FS };  // sample positions: u-face, v-face, cell centre
  int nx_ = 0, ny_ = 0;
  float h_ = 0;
  std::vector<float> u_, v_, nu_, nv_, p_, s_, T_, nT_, accC_, uc0_, vc0_;
  ParticleSet out_;

  int idx(int i, int j) const { return j * nx_ + i; }

  float sample(const std::vector<float>& f, float x, float y, Field kind) const {
    x = std::max(std::min(x, nx_ * h_), h_);
    y = std::max(std::min(y, ny_ * h_), h_);
    float dx = 0, dy = 0;
    if (kind == FU) dy = h_ * 0.5f;
    else if (kind == FV) dx = h_ * 0.5f;
    else dx = dy = h_ * 0.5f;
    int x0 = std::min((int)std::floor((x - dx) / h_), nx_ - 1);
    int y0 = std::min((int)std::floor((y - dy) / h_), ny_ - 1);
    float tx = ((x - dx) - x0 * h_) / h_, ty = ((y - dy) - y0 * h_) / h_;
    int x1 = std::min(x0 + 1, nx_ - 1), y1 = std::min(y0 + 1, ny_ - 1);
    float sx = 1 - tx, sy = 1 - ty;
    return sx * sy * f[idx(x0, y0)] + tx * sy * f[idx(x1, y0)] + tx * ty * f[idx(x1, y1)] + sx * ty * f[idx(x0, y1)];
  }

  float avgU(int i, int j) const { return (u_[idx(i, j - 1)] + u_[idx(i, j)] + u_[idx(i + 1, j - 1)] + u_[idx(i + 1, j)]) * 0.25f; }
  float avgV(int i, int j) const { return (v_[idx(i - 1, j)] + v_[idx(i, j)] + v_[idx(i - 1, j + 1)] + v_[idx(i, j + 1)]) * 0.25f; }

  void advect(float dt) {
    nu_ = u_; nv_ = v_; nT_ = T_;
    for (int j = 1; j < ny_ - 1; ++j)
      for (int i = 1; i < nx_ - 1; ++i) {
        if (s_[idx(i, j)] == 0) continue;
        if (s_[idx(i - 1, j)] != 0) {  // u face
          float x = i * h_, y = (j + 0.5f) * h_;
          x -= dt * u_[idx(i, j)]; y -= dt * avgV(i, j);
          nu_[idx(i, j)] = sample(u_, x, y, FU);
        }
        if (s_[idx(i, j - 1)] != 0) {  // v face
          float x = (i + 0.5f) * h_, y = j * h_;
          x -= dt * avgU(i, j); y -= dt * v_[idx(i, j)];
          nv_[idx(i, j)] = sample(v_, x, y, FV);
        }
        // temperature at the cell centre
        float uc = 0.5f * (u_[idx(i, j)] + u_[idx(i + 1, j)]);
        float vc = 0.5f * (v_[idx(i, j)] + v_[idx(i, j + 1)]);
        nT_[idx(i, j)] = sample(T_, (i + 0.5f) * h_ - dt * uc, (j + 0.5f) * h_ - dt * vc, FS);
      }
    u_ = nu_; v_ = nv_; T_ = nT_;
  }

  void thermalBoundaries() {
    using namespace cfg;
    for (int i = 0; i < nx_; ++i) {
      float fx = (float)i / (nx_ - 1);
      bool heater = fx > kHeaterX0 && fx < kHeaterX1;
      T_[idx(i, 0)] = heater ? kTHot : T_[idx(i, 1)];  // heated strip, rest insulated
      T_[idx(i, ny_ - 1)] = kTCold;                    // cooled ceiling
    }
    for (int j = 0; j < ny_; ++j) { T_[idx(0, j)] = T_[idx(1, j)]; T_[idx(nx_ - 1, j)] = T_[idx(nx_ - 2, j)]; }
  }

  void diffuseHeat(float dt) {
    thermalBoundaries();
    nT_ = T_;
    float k = cfg::kGridThermalDiff * dt / (h_ * h_);
    for (int j = 1; j < ny_ - 1; ++j)
      for (int i = 1; i < nx_ - 1; ++i)
        nT_[idx(i, j)] = T_[idx(i, j)] + k * (T_[idx(i - 1, j)] + T_[idx(i + 1, j)] + T_[idx(i, j - 1)] + T_[idx(i, j + 1)] - 4 * T_[idx(i, j)]);
    T_ = nT_;
    thermalBoundaries();
  }

  void centreVelocity(std::vector<float>& uc, std::vector<float>& vc) const {
    for (int j = 1; j < ny_ - 1; ++j)
      for (int i = 1; i < nx_ - 1; ++i) {
        uc[idx(i, j)] = 0.5f * (u_[idx(i, j)] + u_[idx(i + 1, j)]);
        vc[idx(i, j)] = 0.5f * (v_[idx(i, j)] + v_[idx(i, j + 1)]);
      }
  }

  void project(float dt) {
    using namespace cfg;
    float cp = kGridDensity * h_ / dt;
    // Warm start: re-apply last step's pressure gradient, then only the
    // remaining correction is solved. The hydrostatic field converges over time.
    float k = dt / (kGridDensity * h_);
    for (int j = 1; j < ny_ - 1; ++j)
      for (int i = 1; i < nx_ - 1; ++i) {
        if (!s_[idx(i, j)]) continue;
        if (s_[idx(i - 1, j)]) u_[idx(i, j)] -= k * (p_[idx(i, j)] - p_[idx(i - 1, j)]);
        if (s_[idx(i, j - 1)]) v_[idx(i, j)] -= k * (p_[idx(i, j)] - p_[idx(i, j - 1)]);
      }
    for (int it = 0; it < kGridPressureIters; ++it)
      for (int j = 1; j < ny_ - 1; ++j)
        for (int i = 1; i < nx_ - 1; ++i) {
          if (s_[idx(i, j)] == 0) continue;
          float sx0 = s_[idx(i - 1, j)], sx1 = s_[idx(i + 1, j)], sy0 = s_[idx(i, j - 1)], sy1 = s_[idx(i, j + 1)];
          float sum = sx0 + sx1 + sy0 + sy1;
          if (sum == 0) continue;
          float div = u_[idx(i + 1, j)] - u_[idx(i, j)] + v_[idx(i, j + 1)] - v_[idx(i, j)];
          float pp = -div / sum * kGridOverRelax;
          p_[idx(i, j)] += cp * pp;
          u_[idx(i, j)] -= sx0 * pp; u_[idx(i + 1, j)] += sx1 * pp;
          v_[idx(i, j)] -= sy0 * pp; v_[idx(i, j + 1)] += sy1 * pp;
        }
    // A closed box only determines pressure up to a constant: fix the gauge so
    // the ceiling row averages 0 Pa (differences, and so the flow, are unchanged).
    float ref = 0;
    for (int i = 1; i < nx_ - 1; ++i) ref += p_[idx(i, ny_ - 2)];
    ref /= (float)(nx_ - 2);
    for (auto& x : p_) x -= ref;
  }

  void substep(float dt) {
    using namespace cfg;
    advect(dt);
    centreVelocity(uc0_, vc0_);                    // velocity before forces / pressure
    // gravity + thermal buoyancy on v faces
    for (int j = 1; j < ny_ - 1; ++j)
      for (int i = 1; i < nx_ - 1; ++i)
        if (s_[idx(i, j)] && s_[idx(i, j - 1)]) {
          float Tf = 0.5f * (T_[idx(i, j)] + T_[idx(i, j - 1)]);
          float Tref = 0.5f * (kTHot + kTCold);
          v_[idx(i, j)] += dt * (-kGravity + kGravity * kBuoyancy * (Tf - Tref));
        }
    diffuseHeat(dt);
    project(dt);
    // material acceleration = velocity change caused by forces + pressure
    std::vector<float>& uc = nu_; std::vector<float>& vc = nv_;
    centreVelocity(uc, vc);
    for (int j = 1; j < ny_ - 1; ++j)
      for (int i = 1; i < nx_ - 1; ++i) {
        float ax = (uc[idx(i, j)] - uc0_[idx(i, j)]) / dt, ay = (vc[idx(i, j)] - vc0_[idx(i, j)]) / dt;
        accC_[idx(i, j)] += (std::sqrt(ax * ax + ay * ay) - accC_[idx(i, j)]) * 0.15f;
      }
    advectTracers(dt);
  }

  void velocityAt(float x, float y, float& vx, float& vy) const {
    vx = sample(u_, x, y, FU); vy = sample(v_, x, y, FV);
  }

  void advectTracers(float dt) {
    float lo = 1.001f * h_, hiX = cfg::kWorldW - 1.001f * h_, hiY = cfg::kWorldH - 1.001f * h_;
    for (size_t k = 0; k < out_.size(); ++k) {  // RK2 (midpoint)
      float x = out_.x[k], y = out_.y[k], vx, vy, mx, my;
      velocityAt(x, y, vx, vy);
      velocityAt(x + 0.5f * dt * vx, y + 0.5f * dt * vy, mx, my);
      out_.x[k] = std::min(hiX, std::max(lo, x + dt * mx));
      out_.y[k] = std::min(hiY, std::max(lo, y + dt * my));
    }
  }

  void sampleParticles() {
    for (size_t k = 0; k < out_.size(); ++k) {
      float x = out_.x[k], y = out_.y[k], vx, vy;
      velocityAt(x, y, vx, vy);
      out_.speed[k] = std::sqrt(vx * vx + vy * vy);
      out_.heat[k] = sample(T_, x, y, FS);
      out_.pressure[k] = sample(p_, x, y, FS);
      out_.accel[k] = sample(accC_, x, y, FS);
    }
  }
};
