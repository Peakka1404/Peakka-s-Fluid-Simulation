#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <exception>
#include <memory>
#include <stdexcept>
#include <string>

#include "config.h"
#include "euler_solver.h"
#include "solver.h"
#include "spectrum.h"
#include "sph_solver.h"
#include "vk_renderer.h"

struct App {
  ViewMode mode = ViewMode::Speed;
  bool paused = false, resetReq = false;
};

static std::unique_ptr<Solver> makeSolver() {
#if FLUID_SOLVER == FLUID_SOLVER_EULER
  return std::make_unique<EulerSolver>();
#else
  return std::make_unique<SphSolver>();
#endif
}

static void onKey(GLFWwindow* w, int key, int, int action, int) {
  if (action != GLFW_PRESS) return;
  auto* app = static_cast<App*>(glfwGetWindowUserPointer(w));
  switch (key) {
    case GLFW_KEY_1: app->mode = ViewMode::Speed; break;
    case GLFW_KEY_2: app->mode = ViewMode::Heat; break;
    case GLFW_KEY_3: app->mode = ViewMode::Pressure; break;
    case GLFW_KEY_4: app->mode = ViewMode::Acceleration; break;
    case GLFW_KEY_SPACE: app->paused = !app->paused; break;
    case GLFW_KEY_R: app->resetReq = true; break;
    case GLFW_KEY_ESCAPE: glfwSetWindowShouldClose(w, GLFW_TRUE); break;
    default: break;
  }
}

int main() try {
  if (!glfwInit()) throw std::runtime_error("glfwInit failed");
  glfwWindowHint(GLFW_CLIENT_API, GLFW_NO_API);  // no OpenGL 
  GLFWwindow* win = glfwCreateWindow(1000, 750, "fluidsim", nullptr, nullptr);
  if (!win) throw std::runtime_error("window creation failed");
  App app;
  glfwSetWindowUserPointer(win, &app);
  glfwSetKeyCallback(win, onKey);

  auto solver = makeSolver();
  Renderer renderer(win, solver->particles().size());
  std::printf("Solver: %s | keys: 1 speed, 2 heat, 3 pressure, 4 accel | space pause | R reset | LMB stir\n",
              solver->name());

  Range ranges[4];
  std::vector<float> inst;
  double lastMx = 0, lastMy = 0, lastT = glfwGetTime(), fpsT = lastT;
  int fpsN = 0;

  while (!glfwWindowShouldClose(win)) {
    glfwPollEvents();
    if (app.resetReq) { solver->reset(); app.resetReq = false; }

    // mouse -> world, stir the fluid while LMB is held
    double mx, my; int ww, wh;
    glfwGetCursorPos(win, &mx, &my); glfwGetWindowSize(win, &ww, &wh);
    double now = glfwGetTime(), dtReal = std::max(1e-4, now - lastT);
    if (glfwGetMouseButton(win, GLFW_MOUSE_BUTTON_LEFT) == GLFW_PRESS && ww > 0 && wh > 0 && !app.paused) {
      Viewport4 lb = letterbox((float)ww, (float)wh, cfg::kWorldW / cfg::kWorldH);
      float wx = ((float)mx - lb.x) / lb.w * cfg::kWorldW;
      float wy = (1.0f - ((float)my - lb.y) / lb.h) * cfg::kWorldH;
      float vx = (float)((mx - lastMx) / dtReal) / lb.w * cfg::kWorldW;
      float vy = -(float)((my - lastMy) / dtReal) / lb.h * cfg::kWorldH;
      float sp = std::hypot(vx, vy), cap = 4.0f;
      if (sp > cap) { vx *= cap / sp; vy *= cap / sp; }
      solver->interact(wx, wy, vx, vy);
    }
    lastMx = mx; lastMy = my; lastT = now;

    if (!app.paused) solver->step();

    // ---- per-particle colouring: metric -> [0,1] -> purple..red ----
    const ParticleSet& ps = solver->particles();
    const std::vector<float>* metric = &ps.speed;
    bool zeroBased = true; float minSpan = 1.0f;
    switch (app.mode) {
      case ViewMode::Speed:        metric = &ps.speed;    zeroBased = true;  minSpan = 1.0f;   break;
      case ViewMode::Heat:         metric = &ps.heat;     zeroBased = false; minSpan = 5.0f;   break;
      case ViewMode::Pressure:     metric = &ps.pressure; zeroBased = false; minSpan = 500.0f; break;
      case ViewMode::Acceleration: metric = &ps.accel;    zeroBased = true;  minSpan = 5.0f;   break;
    }
    Range& rg = ranges[(int)app.mode];
    rg.update(*metric, zeroBased, minSpan);
    inst.resize(ps.size() * 5);
    for (size_t i = 0; i < ps.size(); ++i) {
      float rgb[3];
      spectrum(rg.map((*metric)[i]), rgb);
      float* o = &inst[i * 5];
      o[0] = ps.x[i]; o[1] = ps.y[i]; o[2] = rgb[0]; o[3] = rgb[1]; o[4] = rgb[2];
    }
    renderer.draw(inst.data(), ps.size(), solver->particleRadius(), cfg::kWorldW, cfg::kWorldH);

    ++fpsN;
    if (now - fpsT > 0.5) {
      char title[200];
      std::snprintf(title, sizeof(title), "fluidsim | %s | %zu particles | view: %s [%.4g .. %.4g] | %.0f fps%s",
                    solver->name(), ps.size(), viewName(app.mode), rg.lo, rg.hi, fpsN / (now - fpsT),
                    app.paused ? " | PAUSED" : "");
      glfwSetWindowTitle(win, title);
      fpsT = now; fpsN = 0;
    }
  }
  // renderer and solver are destroyed before the window
  return (glfwDestroyWindow(win), glfwTerminate(), 0);
} catch (const std::exception& e) {
  std::fprintf(stderr, "Fatal: %s\n", e.what());
  return 1;
}
