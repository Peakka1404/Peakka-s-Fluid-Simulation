#pragma once
#define GLFW_INCLUDE_VULKAN
#include <GLFW/glfw3.h>
#include <vector>

struct Viewport4 { float x, y, w, h; };
// Letterboxed viewport that keeps the world's aspect ratio.
inline Viewport4 letterbox(float fbW, float fbH, float worldAspect) {
  float w = fbW, h = fbW / worldAspect;
  if (h > fbH) { h = fbH; w = fbH * worldAspect; }
  return {(fbW - w) * 0.5f, (fbH - h) * 0.5f, w, h};
}

// Minimal Vulkan 1.0 renderer: draws N instanced, circular particles.
// Instance layout: 5 floats = x, y, r, g, b.
class Renderer {
 public:
  Renderer(GLFWwindow* window, size_t maxParticles);
  ~Renderer();
  Renderer(const Renderer&) = delete;
  Renderer& operator=(const Renderer&) = delete;
  void draw(const float* instances, size_t count, float radius, float worldW, float worldH);

 private:
  static constexpr int kFrames = 2;
  void createInstance();
  void pickPhysicalDevice();
  void createDevice();
  void createSwapchain();
  void destroySwapchain();
  void recreateSwapchain();
  void createRenderPass();
  void createPipeline();
  void createFrameResources(size_t maxParticles);
  uint32_t findMemoryType(uint32_t bits, VkMemoryPropertyFlags props) const;

  GLFWwindow* win_;
  VkInstance inst_ = VK_NULL_HANDLE;
  VkSurfaceKHR surface_ = VK_NULL_HANDLE;
  VkPhysicalDevice phys_ = VK_NULL_HANDLE;
  VkDevice dev_ = VK_NULL_HANDLE;
  VkQueue queue_ = VK_NULL_HANDLE;
  uint32_t family_ = 0;
  VkSwapchainKHR swap_ = VK_NULL_HANDLE;
  VkFormat format_ = VK_FORMAT_B8G8R8A8_UNORM;
  VkExtent2D extent_{};
  std::vector<VkImage> images_;
  std::vector<VkImageView> views_;
  std::vector<VkFramebuffer> fbs_;
  std::vector<VkSemaphore> renderDone_;  // one per swapchain image
  VkRenderPass pass_ = VK_NULL_HANDLE;
  VkPipelineLayout layout_ = VK_NULL_HANDLE;
  VkPipeline pipe_ = VK_NULL_HANDLE;
  VkCommandPool pool_ = VK_NULL_HANDLE;
  VkCommandBuffer cmd_[kFrames]{};
  VkSemaphore imageAvail_[kFrames]{};
  VkFence inFlight_[kFrames]{};
  VkBuffer buf_[kFrames]{};
  VkDeviceMemory mem_[kFrames]{};
  void* mapped_[kFrames]{};
  size_t maxParticles_ = 0;
  uint32_t frame_ = 0;
};
