#pragma once
// ============================================================================
//  FLUID SIMULATION CONFIG
// ============================================================================
#define FLUID_SOLVER_SPH   0   // Smoothed Particle Hydrodynamics (Lagrangian)
#define FLUID_SOLVER_EULER 1   // Grid-based Eulerian (Stable Fluids, MAC grid)

// >>>>>>>>>>>>>>>>>>>>  PICK THE SOLVER HERE  <<<<<<<<<<<<<<<<<<<<
// (or override from the command line: cmake -DFLUID_SOLVER=EULER ..)
#ifndef FLUID_SOLVER
#define FLUID_SOLVER FLUID_SOLVER_SPH
#endif

namespace cfg {
// World (metres). Origin bottom-left, +y up.
constexpr float kWorldW = 2.0f;
constexpr float kWorldH = 1.5f;
constexpr float kGravity = 9.81f;

// Temperatures (Kelvin)
constexpr float kTCold = 293.0f;
constexpr float kTHot  = 353.0f;
constexpr float kBuoyancy = 0.003f;   // thermal expansion-like coefficient (1/K)

// Heater strip on the floor (fraction of world width)
constexpr float kHeaterX0 = 0.35f;
constexpr float kHeaterX1 = 0.65f;

// ---- SPH ------------------------------------------------------------------
constexpr float kSphSpacing   = 0.015f;          // initial particle spacing
constexpr float kSphH         = 2.0f * kSphSpacing; // kernel support radius
constexpr float kSphRestDens  = 1000.0f;
constexpr float kSphStiffness = 350.0f;          // p = k (rho - rho0)  (k = c^2)
constexpr float kSphViscosity = 0.02f;           // kinematic viscosity (m^2/s)
constexpr float kSphThermalDiff = 0.02f;         // heat diffusivity   (m^2/s)
constexpr float kSphDt        = 6.0e-4f;         // sub-step
constexpr int   kSphSubsteps  = 22;              // sub-steps per rendered frame
constexpr float kSphBlockW    = 0.6f;            // initial dam-break block
constexpr float kSphBlockH    = 1.0f;

// ---- Eulerian grid ---------------------------------------------------------
constexpr int   kGridNX = 128;                   // cell size = kWorldW / kGridNX
constexpr int   kGridNY = 96;
constexpr float kGridDensity = 1000.0f;
constexpr int   kGridPressureIters = 50;
constexpr float kGridOverRelax = 1.9f;
constexpr float kGridThermalDiff = 0.002f;
constexpr float kGridDt = 1.0f / 60.0f;
constexpr int   kGridSubsteps = 1;               // per rendered frame (1/60 s)
constexpr int   kGridTracersX = 100;             // tracer particles (visualisation)
constexpr int   kGridTracersY = 75;
}  // namespace cfg
