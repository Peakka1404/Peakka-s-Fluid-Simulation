#include "vk_renderer.h"
#include <algorithm>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <stdexcept>
#include <string>

#ifndef SHADER_DIR
#define SHADER_DIR "shaders"
#endif

#define VK_CHECK(x)                                                                    \
  do {                                                                                 \
    VkResult r_ = (x);                                                                 \
    if (r_ != VK_SUCCESS) throw std::runtime_error(std::string(#x) + " failed: " + std::to_string(r_)); \
  } while (0)

static std::vector<char> readFile(const std::string& path) {
  std::ifstream f(path, std::ios::binary | std::ios::ate);
  if (!f) throw std::runtime_error("cannot open " + path + " (was glslc run?)");
  std::vector<char> d((size_t)f.tellg());
  f.seekg(0);
  f.read(d.data(), (std::streamsize)d.size());
  return d;
}

Renderer::Renderer(GLFWwindow* window, size_t maxParticles) : win_(window), maxParticles_(maxParticles) {
  createInstance();
  VK_CHECK(glfwCreateWindowSurface(inst_, win_, nullptr, &surface_));
  pickPhysicalDevice();
  createDevice();
  createSwapchain();
  createRenderPass();
  createPipeline();
  // framebuffers need the render pass
  fbs_.resize(views_.size());
  for (size_t i = 0; i < views_.size(); ++i) {
    VkFramebufferCreateInfo fi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
    fi.renderPass = pass_; fi.attachmentCount = 1; fi.pAttachments = &views_[i];
    fi.width = extent_.width; fi.height = extent_.height; fi.layers = 1;
    VK_CHECK(vkCreateFramebuffer(dev_, &fi, nullptr, &fbs_[i]));
  }
  createFrameResources(maxParticles);
}

Renderer::~Renderer() {
  if (dev_) vkDeviceWaitIdle(dev_);
  for (int i = 0; i < kFrames; ++i) {
    if (mapped_[i]) vkUnmapMemory(dev_, mem_[i]);
    vkDestroyBuffer(dev_, buf_[i], nullptr);
    vkFreeMemory(dev_, mem_[i], nullptr);
    vkDestroySemaphore(dev_, imageAvail_[i], nullptr);
    vkDestroyFence(dev_, inFlight_[i], nullptr);
  }
  vkDestroyCommandPool(dev_, pool_, nullptr);
  destroySwapchain();
  vkDestroyPipeline(dev_, pipe_, nullptr);
  vkDestroyPipelineLayout(dev_, layout_, nullptr);
  vkDestroyRenderPass(dev_, pass_, nullptr);
  vkDestroyDevice(dev_, nullptr);
  vkDestroySurfaceKHR(inst_, surface_, nullptr);
  vkDestroyInstance(inst_, nullptr);
}

void Renderer::createInstance() {
  VkApplicationInfo app{VK_STRUCTURE_TYPE_APPLICATION_INFO};
  app.pApplicationName = "fluidsim"; app.apiVersion = VK_API_VERSION_1_0;
  uint32_t n = 0;
  const char** ext = glfwGetRequiredInstanceExtensions(&n);
  if (!ext) throw std::runtime_error("GLFW: Vulkan not supported");
  std::vector<const char*> exts(ext, ext + n), layers;
#ifndef NDEBUG
  uint32_t lc = 0; vkEnumerateInstanceLayerProperties(&lc, nullptr);
  std::vector<VkLayerProperties> lp(lc); vkEnumerateInstanceLayerProperties(&lc, lp.data());
  for (auto& l : lp) if (!strcmp(l.layerName, "VK_LAYER_KHRONOS_validation")) layers.push_back("VK_LAYER_KHRONOS_validation");
#endif
  VkInstanceCreateInfo ci{VK_STRUCTURE_TYPE_INSTANCE_CREATE_INFO};
  ci.pApplicationInfo = &app;
  ci.enabledExtensionCount = (uint32_t)exts.size(); ci.ppEnabledExtensionNames = exts.data();
  ci.enabledLayerCount = (uint32_t)layers.size(); ci.ppEnabledLayerNames = layers.data();
  VK_CHECK(vkCreateInstance(&ci, nullptr, &inst_));
}

void Renderer::pickPhysicalDevice() {
  uint32_t n = 0; vkEnumeratePhysicalDevices(inst_, &n, nullptr);
  if (!n) throw std::runtime_error("no Vulkan devices");
  std::vector<VkPhysicalDevice> devs(n); vkEnumeratePhysicalDevices(inst_, &n, devs.data());
  int best = -1;
  for (auto d : devs) {
    uint32_t qn = 0; vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, nullptr);
    std::vector<VkQueueFamilyProperties> qf(qn); vkGetPhysicalDeviceQueueFamilyProperties(d, &qn, qf.data());
    for (uint32_t i = 0; i < qn; ++i) {
      VkBool32 present = VK_FALSE;
      vkGetPhysicalDeviceSurfaceSupportKHR(d, i, surface_, &present);
      if ((qf[i].queueFlags & VK_QUEUE_GRAPHICS_BIT) && present) {
        VkPhysicalDeviceProperties pr; vkGetPhysicalDeviceProperties(d, &pr);
        int score = pr.deviceType == VK_PHYSICAL_DEVICE_TYPE_DISCRETE_GPU ? 2 : 1;
        if (score > best) { best = score; phys_ = d; family_ = i; }
        break;
      }
    }
  }
  if (!phys_) throw std::runtime_error("no suitable GPU");
  VkPhysicalDeviceProperties pr; vkGetPhysicalDeviceProperties(phys_, &pr);
  std::printf("GPU: %s\n", pr.deviceName);
}

void Renderer::createDevice() {
  float prio = 1.0f;
  VkDeviceQueueCreateInfo qi{VK_STRUCTURE_TYPE_DEVICE_QUEUE_CREATE_INFO};
  qi.queueFamilyIndex = family_; qi.queueCount = 1; qi.pQueuePriorities = &prio;
  const char* ext[] = {VK_KHR_SWAPCHAIN_EXTENSION_NAME};
  VkDeviceCreateInfo ci{VK_STRUCTURE_TYPE_DEVICE_CREATE_INFO};
  ci.queueCreateInfoCount = 1; ci.pQueueCreateInfos = &qi;
  ci.enabledExtensionCount = 1; ci.ppEnabledExtensionNames = ext;
  VK_CHECK(vkCreateDevice(phys_, &ci, nullptr, &dev_));
  vkGetDeviceQueue(dev_, family_, 0, &queue_);
}

void Renderer::createSwapchain() {
  int fw = 0, fh = 0;
  glfwGetFramebufferSize(win_, &fw, &fh);
  while (fw == 0 || fh == 0) { glfwWaitEvents(); glfwGetFramebufferSize(win_, &fw, &fh); }
  VkSurfaceCapabilitiesKHR caps;
  vkGetPhysicalDeviceSurfaceCapabilitiesKHR(phys_, surface_, &caps);
  uint32_t fc = 0; vkGetPhysicalDeviceSurfaceFormatsKHR(phys_, surface_, &fc, nullptr);
  std::vector<VkSurfaceFormatKHR> fmts(fc); vkGetPhysicalDeviceSurfaceFormatsKHR(phys_, surface_, &fc, fmts.data());
  VkSurfaceFormatKHR sf = fmts[0];
  for (auto& f : fmts)  // UNORM: colours are written exactly as computed
    if ((f.format == VK_FORMAT_B8G8R8A8_UNORM || f.format == VK_FORMAT_R8G8B8A8_UNORM) &&
        f.colorSpace == VK_COLOR_SPACE_SRGB_NONLINEAR_KHR) { sf = f; break; }
  format_ = sf.format;
  extent_ = caps.currentExtent;
  if (extent_.width == UINT32_MAX) {
    extent_.width = std::clamp((uint32_t)fw, caps.minImageExtent.width, caps.maxImageExtent.width);
    extent_.height = std::clamp((uint32_t)fh, caps.minImageExtent.height, caps.maxImageExtent.height);
  }
  uint32_t count = caps.minImageCount + 1;
  if (caps.maxImageCount && count > caps.maxImageCount) count = caps.maxImageCount;
  VkSwapchainCreateInfoKHR ci{VK_STRUCTURE_TYPE_SWAPCHAIN_CREATE_INFO_KHR};
  ci.surface = surface_; ci.minImageCount = count; ci.imageFormat = sf.format; ci.imageColorSpace = sf.colorSpace;
  ci.imageExtent = extent_; ci.imageArrayLayers = 1; ci.imageUsage = VK_IMAGE_USAGE_COLOR_ATTACHMENT_BIT;
  ci.imageSharingMode = VK_SHARING_MODE_EXCLUSIVE; ci.preTransform = caps.currentTransform;
  ci.compositeAlpha = VK_COMPOSITE_ALPHA_OPAQUE_BIT_KHR; ci.presentMode = VK_PRESENT_MODE_FIFO_KHR;
  ci.clipped = VK_TRUE;
  VK_CHECK(vkCreateSwapchainKHR(dev_, &ci, nullptr, &swap_));
  uint32_t ic = 0; vkGetSwapchainImagesKHR(dev_, swap_, &ic, nullptr);
  images_.resize(ic); vkGetSwapchainImagesKHR(dev_, swap_, &ic, images_.data());
  views_.resize(ic); renderDone_.resize(ic);
  for (uint32_t i = 0; i < ic; ++i) {
    VkImageViewCreateInfo vi{VK_STRUCTURE_TYPE_IMAGE_VIEW_CREATE_INFO};
    vi.image = images_[i]; vi.viewType = VK_IMAGE_VIEW_TYPE_2D; vi.format = format_;
    vi.subresourceRange = {VK_IMAGE_ASPECT_COLOR_BIT, 0, 1, 0, 1};
    VK_CHECK(vkCreateImageView(dev_, &vi, nullptr, &views_[i]));
    VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VK_CHECK(vkCreateSemaphore(dev_, &si, nullptr, &renderDone_[i]));
  }
}

void Renderer::destroySwapchain() {
  for (auto f : fbs_) vkDestroyFramebuffer(dev_, f, nullptr);
  for (auto s : renderDone_) vkDestroySemaphore(dev_, s, nullptr);
  for (auto v : views_) vkDestroyImageView(dev_, v, nullptr);
  fbs_.clear(); renderDone_.clear(); views_.clear();
  if (swap_) vkDestroySwapchainKHR(dev_, swap_, nullptr);
  swap_ = VK_NULL_HANDLE;
}

void Renderer::recreateSwapchain() {
  vkDeviceWaitIdle(dev_);
  destroySwapchain();
  createSwapchain();
  fbs_.resize(views_.size());
  for (size_t i = 0; i < views_.size(); ++i) {
    VkFramebufferCreateInfo fi{VK_STRUCTURE_TYPE_FRAMEBUFFER_CREATE_INFO};
    fi.renderPass = pass_; fi.attachmentCount = 1; fi.pAttachments = &views_[i];
    fi.width = extent_.width; fi.height = extent_.height; fi.layers = 1;
    VK_CHECK(vkCreateFramebuffer(dev_, &fi, nullptr, &fbs_[i]));
  }
}

void Renderer::createRenderPass() {
  VkAttachmentDescription a{};
  a.format = format_; a.samples = VK_SAMPLE_COUNT_1_BIT;
  a.loadOp = VK_ATTACHMENT_LOAD_OP_CLEAR; a.storeOp = VK_ATTACHMENT_STORE_OP_STORE;
  a.stencilLoadOp = VK_ATTACHMENT_LOAD_OP_DONT_CARE; a.stencilStoreOp = VK_ATTACHMENT_STORE_OP_DONT_CARE;
  a.initialLayout = VK_IMAGE_LAYOUT_UNDEFINED; a.finalLayout = VK_IMAGE_LAYOUT_PRESENT_SRC_KHR;
  VkAttachmentReference ref{0, VK_IMAGE_LAYOUT_COLOR_ATTACHMENT_OPTIMAL};
  VkSubpassDescription sp{};
  sp.pipelineBindPoint = VK_PIPELINE_BIND_POINT_GRAPHICS; sp.colorAttachmentCount = 1; sp.pColorAttachments = &ref;
  VkSubpassDependency dep{};
  dep.srcSubpass = VK_SUBPASS_EXTERNAL; dep.dstSubpass = 0;
  dep.srcStageMask = dep.dstStageMask = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  dep.dstAccessMask = VK_ACCESS_COLOR_ATTACHMENT_WRITE_BIT;
  VkRenderPassCreateInfo ci{VK_STRUCTURE_TYPE_RENDER_PASS_CREATE_INFO};
  ci.attachmentCount = 1; ci.pAttachments = &a; ci.subpassCount = 1; ci.pSubpasses = &sp;
  ci.dependencyCount = 1; ci.pDependencies = &dep;
  VK_CHECK(vkCreateRenderPass(dev_, &ci, nullptr, &pass_));
}

static VkShaderModule makeShader(VkDevice d, const std::string& path) {
  auto code = readFile(path);
  VkShaderModuleCreateInfo ci{VK_STRUCTURE_TYPE_SHADER_MODULE_CREATE_INFO};
  ci.codeSize = code.size(); ci.pCode = reinterpret_cast<const uint32_t*>(code.data());
  VkShaderModule m; VK_CHECK(vkCreateShaderModule(d, &ci, nullptr, &m));
  return m;
}

void Renderer::createPipeline() {
  VkShaderModule vs = makeShader(dev_, std::string(SHADER_DIR) + "/particle.vert.spv");
  VkShaderModule fs = makeShader(dev_, std::string(SHADER_DIR) + "/particle.frag.spv");
  VkPipelineShaderStageCreateInfo st[2]{};
  st[0].sType = st[1].sType = VK_STRUCTURE_TYPE_PIPELINE_SHADER_STAGE_CREATE_INFO;
  st[0].stage = VK_SHADER_STAGE_VERTEX_BIT; st[0].module = vs; st[0].pName = "main";
  st[1].stage = VK_SHADER_STAGE_FRAGMENT_BIT; st[1].module = fs; st[1].pName = "main";

  VkVertexInputBindingDescription bind{0, 5 * sizeof(float), VK_VERTEX_INPUT_RATE_INSTANCE};
  VkVertexInputAttributeDescription attr[2] = {{0, 0, VK_FORMAT_R32G32_SFLOAT, 0},
                                               {1, 0, VK_FORMAT_R32G32B32_SFLOAT, 2 * sizeof(float)}};
  VkPipelineVertexInputStateCreateInfo vi{VK_STRUCTURE_TYPE_PIPELINE_VERTEX_INPUT_STATE_CREATE_INFO};
  vi.vertexBindingDescriptionCount = 1; vi.pVertexBindingDescriptions = &bind;
  vi.vertexAttributeDescriptionCount = 2; vi.pVertexAttributeDescriptions = attr;
  VkPipelineInputAssemblyStateCreateInfo ia{VK_STRUCTURE_TYPE_PIPELINE_INPUT_ASSEMBLY_STATE_CREATE_INFO};
  ia.topology = VK_PRIMITIVE_TOPOLOGY_TRIANGLE_STRIP;
  VkPipelineViewportStateCreateInfo vp{VK_STRUCTURE_TYPE_PIPELINE_VIEWPORT_STATE_CREATE_INFO};
  vp.viewportCount = 1; vp.scissorCount = 1;
  VkPipelineRasterizationStateCreateInfo rs{VK_STRUCTURE_TYPE_PIPELINE_RASTERIZATION_STATE_CREATE_INFO};
  rs.polygonMode = VK_POLYGON_MODE_FILL; rs.cullMode = VK_CULL_MODE_NONE; rs.lineWidth = 1.0f;
  VkPipelineMultisampleStateCreateInfo ms{VK_STRUCTURE_TYPE_PIPELINE_MULTISAMPLE_STATE_CREATE_INFO};
  ms.rasterizationSamples = VK_SAMPLE_COUNT_1_BIT;
  VkPipelineColorBlendAttachmentState cb{};
  cb.blendEnable = VK_TRUE; cb.colorWriteMask = 0xF;
  cb.srcColorBlendFactor = VK_BLEND_FACTOR_SRC_ALPHA; cb.dstColorBlendFactor = VK_BLEND_FACTOR_ONE_MINUS_SRC_ALPHA;
  cb.colorBlendOp = VK_BLEND_OP_ADD;
  cb.srcAlphaBlendFactor = VK_BLEND_FACTOR_ONE; cb.dstAlphaBlendFactor = VK_BLEND_FACTOR_ZERO; cb.alphaBlendOp = VK_BLEND_OP_ADD;
  VkPipelineColorBlendStateCreateInfo cbs{VK_STRUCTURE_TYPE_PIPELINE_COLOR_BLEND_STATE_CREATE_INFO};
  cbs.attachmentCount = 1; cbs.pAttachments = &cb;
  VkDynamicState dyn[] = {VK_DYNAMIC_STATE_VIEWPORT, VK_DYNAMIC_STATE_SCISSOR};
  VkPipelineDynamicStateCreateInfo ds{VK_STRUCTURE_TYPE_PIPELINE_DYNAMIC_STATE_CREATE_INFO};
  ds.dynamicStateCount = 2; ds.pDynamicStates = dyn;

  VkPushConstantRange pc{VK_SHADER_STAGE_VERTEX_BIT, 0, 4 * sizeof(float)};
  VkPipelineLayoutCreateInfo li{VK_STRUCTURE_TYPE_PIPELINE_LAYOUT_CREATE_INFO};
  li.pushConstantRangeCount = 1; li.pPushConstantRanges = &pc;
  VK_CHECK(vkCreatePipelineLayout(dev_, &li, nullptr, &layout_));

  VkGraphicsPipelineCreateInfo gp{VK_STRUCTURE_TYPE_GRAPHICS_PIPELINE_CREATE_INFO};
  gp.stageCount = 2; gp.pStages = st; gp.pVertexInputState = &vi; gp.pInputAssemblyState = &ia;
  gp.pViewportState = &vp; gp.pRasterizationState = &rs; gp.pMultisampleState = &ms;
  gp.pColorBlendState = &cbs; gp.pDynamicState = &ds; gp.layout = layout_; gp.renderPass = pass_;
  VK_CHECK(vkCreateGraphicsPipelines(dev_, VK_NULL_HANDLE, 1, &gp, nullptr, &pipe_));
  vkDestroyShaderModule(dev_, vs, nullptr);
  vkDestroyShaderModule(dev_, fs, nullptr);
}

uint32_t Renderer::findMemoryType(uint32_t bits, VkMemoryPropertyFlags props) const {
  VkPhysicalDeviceMemoryProperties mp; vkGetPhysicalDeviceMemoryProperties(phys_, &mp);
  for (uint32_t i = 0; i < mp.memoryTypeCount; ++i)
    if ((bits & (1u << i)) && (mp.memoryTypes[i].propertyFlags & props) == props) return i;
  throw std::runtime_error("no suitable memory type");
}

void Renderer::createFrameResources(size_t maxParticles) {
  VkCommandPoolCreateInfo pi{VK_STRUCTURE_TYPE_COMMAND_POOL_CREATE_INFO};
  pi.flags = VK_COMMAND_POOL_CREATE_RESET_COMMAND_BUFFER_BIT; pi.queueFamilyIndex = family_;
  VK_CHECK(vkCreateCommandPool(dev_, &pi, nullptr, &pool_));
  VkCommandBufferAllocateInfo ai{VK_STRUCTURE_TYPE_COMMAND_BUFFER_ALLOCATE_INFO};
  ai.commandPool = pool_; ai.level = VK_COMMAND_BUFFER_LEVEL_PRIMARY; ai.commandBufferCount = kFrames;
  VK_CHECK(vkAllocateCommandBuffers(dev_, &ai, cmd_));
  for (int i = 0; i < kFrames; ++i) {
    VkSemaphoreCreateInfo si{VK_STRUCTURE_TYPE_SEMAPHORE_CREATE_INFO};
    VK_CHECK(vkCreateSemaphore(dev_, &si, nullptr, &imageAvail_[i]));
    VkFenceCreateInfo fi{VK_STRUCTURE_TYPE_FENCE_CREATE_INFO};
    fi.flags = VK_FENCE_CREATE_SIGNALED_BIT;
    VK_CHECK(vkCreateFence(dev_, &fi, nullptr, &inFlight_[i]));
    VkBufferCreateInfo bi{VK_STRUCTURE_TYPE_BUFFER_CREATE_INFO};
    bi.size = maxParticles * 5 * sizeof(float); bi.usage = VK_BUFFER_USAGE_VERTEX_BUFFER_BIT;
    bi.sharingMode = VK_SHARING_MODE_EXCLUSIVE;
    VK_CHECK(vkCreateBuffer(dev_, &bi, nullptr, &buf_[i]));
    VkMemoryRequirements mr; vkGetBufferMemoryRequirements(dev_, buf_[i], &mr);
    VkMemoryAllocateInfo mi{VK_STRUCTURE_TYPE_MEMORY_ALLOCATE_INFO};
    mi.allocationSize = mr.size;
    mi.memoryTypeIndex = findMemoryType(mr.memoryTypeBits, VK_MEMORY_PROPERTY_HOST_VISIBLE_BIT | VK_MEMORY_PROPERTY_HOST_COHERENT_BIT);
    VK_CHECK(vkAllocateMemory(dev_, &mi, nullptr, &mem_[i]));
    VK_CHECK(vkBindBufferMemory(dev_, buf_[i], mem_[i], 0));
    VK_CHECK(vkMapMemory(dev_, mem_[i], 0, VK_WHOLE_SIZE, 0, &mapped_[i]));
  }
}

void Renderer::draw(const float* instances, size_t count, float radius, float worldW, float worldH) {
  uint32_t f = frame_;
  vkWaitForFences(dev_, 1, &inFlight_[f], VK_TRUE, UINT64_MAX);
  uint32_t img = 0;
  VkResult r = vkAcquireNextImageKHR(dev_, swap_, UINT64_MAX, imageAvail_[f], VK_NULL_HANDLE, &img);
  if (r == VK_ERROR_OUT_OF_DATE_KHR) { recreateSwapchain(); return; }
  if (r != VK_SUCCESS && r != VK_SUBOPTIMAL_KHR) throw std::runtime_error("acquire failed");
  vkResetFences(dev_, 1, &inFlight_[f]);

  count = std::min(count, maxParticles_);
  std::memcpy(mapped_[f], instances, count * 5 * sizeof(float));

  VkCommandBuffer cb = cmd_[f];
  vkResetCommandBuffer(cb, 0);
  VkCommandBufferBeginInfo bi{VK_STRUCTURE_TYPE_COMMAND_BUFFER_BEGIN_INFO};
  VK_CHECK(vkBeginCommandBuffer(cb, &bi));
  VkClearValue clear{}; clear.color = {{0.02f, 0.02f, 0.04f, 1.0f}};
  VkRenderPassBeginInfo rp{VK_STRUCTURE_TYPE_RENDER_PASS_BEGIN_INFO};
  rp.renderPass = pass_; rp.framebuffer = fbs_[img]; rp.renderArea = {{0, 0}, extent_};
  rp.clearValueCount = 1; rp.pClearValues = &clear;
  vkCmdBeginRenderPass(cb, &rp, VK_SUBPASS_CONTENTS_INLINE);
  vkCmdBindPipeline(cb, VK_PIPELINE_BIND_POINT_GRAPHICS, pipe_);
  Viewport4 lb = letterbox((float)extent_.width, (float)extent_.height, worldW / worldH);
  VkViewport vp{lb.x, lb.y, lb.w, lb.h, 0.0f, 1.0f};
  VkRect2D sc{{(int32_t)lb.x, (int32_t)lb.y}, {(uint32_t)lb.w, (uint32_t)lb.h}};
  vkCmdSetViewport(cb, 0, 1, &vp);
  vkCmdSetScissor(cb, 0, 1, &sc);
  float pc[4] = {worldW, worldH, radius, 0.0f};
  vkCmdPushConstants(cb, layout_, VK_SHADER_STAGE_VERTEX_BIT, 0, sizeof(pc), pc);
  VkDeviceSize off = 0;
  vkCmdBindVertexBuffers(cb, 0, 1, &buf_[f], &off);
  if (count) vkCmdDraw(cb, 4, (uint32_t)count, 0, 0);
  vkCmdEndRenderPass(cb);
  VK_CHECK(vkEndCommandBuffer(cb));

  VkPipelineStageFlags wait = VK_PIPELINE_STAGE_COLOR_ATTACHMENT_OUTPUT_BIT;
  VkSubmitInfo si{VK_STRUCTURE_TYPE_SUBMIT_INFO};
  si.waitSemaphoreCount = 1; si.pWaitSemaphores = &imageAvail_[f]; si.pWaitDstStageMask = &wait;
  si.commandBufferCount = 1; si.pCommandBuffers = &cb;
  si.signalSemaphoreCount = 1; si.pSignalSemaphores = &renderDone_[img];
  VK_CHECK(vkQueueSubmit(queue_, 1, &si, inFlight_[f]));

  VkPresentInfoKHR pi{VK_STRUCTURE_TYPE_PRESENT_INFO_KHR};
  pi.waitSemaphoreCount = 1; pi.pWaitSemaphores = &renderDone_[img];
  pi.swapchainCount = 1; pi.pSwapchains = &swap_; pi.pImageIndices = &img;
  r = vkQueuePresentKHR(queue_, &pi);
  if (r == VK_ERROR_OUT_OF_DATE_KHR || r == VK_SUBOPTIMAL_KHR) recreateSwapchain();
  else if (r != VK_SUCCESS) throw std::runtime_error("present failed");
  frame_ = (f + 1) % kFrames;
}
