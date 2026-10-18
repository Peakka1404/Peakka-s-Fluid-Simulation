#pragma once
#include <cstddef>
#include <memory>
#include <vector>

// Per-particle quantities that the renderer colours independently.
struct ParticleSet {
  std::vector<float> x, y;        // position (m)
  std::vector<float> speed;       // |v|       (m/s)
  std::vector<float> heat;        // temperature (K)
  std::vector<float> pressure;    // pressure  (Pa)
  std::vector<float> accel;       // |dv/dt|   (m/s^2)
  size_t size() const { return x.size(); }
  void resize(size_t n) {
    x.assign(n, 0); y.assign(n, 0); speed.assign(n, 0);
    heat.assign(n, 0); pressure.assign(n, 0); accel.assign(n, 0);
  }
};

class Solver {
 public:
  virtual ~Solver() = default;
  virtual void reset() = 0;
  virtual void step() = 0;  // advance by one rendered frame (several sub-steps)
  // Stir the fluid: impulse velocity (vx,vy) m/s around world point (x,y).
  virtual void interact(float x, float y, float vx, float vy) = 0;
  virtual const ParticleSet& particles() const = 0;
  virtual float particleRadius() const = 0;  // for drawing (m)
  virtual const char* name() const = 0;
};
