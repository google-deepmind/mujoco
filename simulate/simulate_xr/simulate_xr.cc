#include "simulate_xr.h"
// Copyright 2024 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "simulate_xr.h"

// #pragma comment(lib, "opengl32.lib")

// #include "glad\glad.h"
// #include <GLFW/glfw3.h>

#include "glad\glad.h"
//// gfxwrapper will redefine these macros
// #undef XR_USE_PLATFORM_WIN32
// #undef XR_USE_PLATFORM_XLIB
// #undef XR_USE_PLATFORM_XCB
// #undef XR_USE_PLATFORM_WAYLAND
// #include <gfxwrapper_opengl.h>

#include <GLFW/glfw3.h>
#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#define GLFW_NATIVE_INCLUDE_NONE
#include <GLFW/glfw3native.h>

SimulateXr::SimulateXr() {}

SimulateXr::~SimulateXr() {}

void SimulateXr::init(/*void* window*/) {
  // window_ = (GLFWwindow *)window;
  // Only needed in _create_session?
  // but here created in the beginning after spawning the window, so should be
  // correct

  // Needed for textures
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize OpenGL context for OpenXR." << std::endl;
    return;
  } else {
    std::cout << "Initialized OpenGL context for OpenXR." << std::endl;
  }

  _create_instance();

  _get_instance_properties();

  _get_system_id();

  _get_view_configuration_views();

  _get_environment_blend_modes();

  _create_session();

  _create_reference_space();

  _create_swapchain();

  // there is also creation of CompositionLayerProjection in Python,
  // but it should be covered by the OpenXR RenderLayerInfo - done during rendering
}

void SimulateXr::deinit() {

  _destroy_swapchain();

  _destroy_reference_space();

  _destroy_session();

  _destroy_instance();
}

void SimulateXr::init_scene_vis(mjvScene *scn, mjModel *m) {
  if (scn)
    scn->stereo = mjSTEREO_SIDEBYSIDE;

  if (m) {
    m->vis.global.offwidth = width_render;
    m->vis.global.offheight = height;
    m->vis.quality.offsamples = 0;
  }
}

void SimulateXr::_create_instance() {
  // Fill out an XrApplicationInfo structure detailing the names and OpenXR
  // version. The application/engine name and version are user-definied. These
  // may help IHVs or runtimes.
  // AS change
  XrApplicationInfo AI;
  strncpy(AI.applicationName, "OpenXR MuJoCo", XR_MAX_APPLICATION_NAME_SIZE);
  AI.applicationVersion = 1;
  strncpy(AI.engineName, "OpenXR Engine", XR_MAX_ENGINE_NAME_SIZE);
  AI.engineVersion = 1;
  AI.apiVersion = XR_CURRENT_API_VERSION;

  // AS change Cannot get the debug extension, so only opengl
  m_instanceExtensions.push_back(XR_KHR_OPENGL_ENABLE_EXTENSION_NAME);

  // Get all the API Layers from the OpenXR runtime.
  uint32_t apiLayerCount = 0;
  std::vector<XrApiLayerProperties> apiLayerProperties;
  if (xrEnumerateApiLayerProperties(0, &apiLayerCount, nullptr) < 0)
    std::cerr << "Failed to enumerate ApiLayerProperties." << std::endl;
  apiLayerProperties.resize(apiLayerCount, {XR_TYPE_API_LAYER_PROPERTIES});
  if (xrEnumerateApiLayerProperties(apiLayerCount, &apiLayerCount,
                                    apiLayerProperties.data()) < 0)
    std::cerr << "Failed to enumerate ApiLayerProperties." << std::endl;

  // Check the requested API layers against the ones from the OpenXR. If found
  // add it to the Active API Layers.
  for (auto &requestLayer : m_apiLayers) {
    for (auto &layerProperty : apiLayerProperties) {
      // strcmp returns 0 if the strings match.
      if (strcmp(requestLayer.c_str(), layerProperty.layerName) != 0) {
        continue;
      } else {
        m_activeAPILayers.push_back(requestLayer.c_str());
        break;
      }
    }
  }

  // Get all the Instance Extensions from the OpenXR instance.
  uint32_t extensionCount = 0;
  std::vector<XrExtensionProperties> extensionProperties;
  if (xrEnumerateInstanceExtensionProperties(nullptr, 0, &extensionCount,
                                             nullptr) < 0)
    std::cerr << "Failed to enumerate InstanceExtensionProperties."
              << std::endl;
  extensionProperties.resize(extensionCount, {XR_TYPE_EXTENSION_PROPERTIES});
  if (xrEnumerateInstanceExtensionProperties(nullptr, extensionCount,
                                             &extensionCount,
                                             extensionProperties.data()) < 0)
    std::cerr << "Failed to enumerate InstanceExtensionProperties."
              << std::endl;

  // Check the requested Instance Extensions against the ones from the OpenXR
  // runtime. If an extension is found add it to Active Instance Extensions. Log
  // error if the Instance Extension is not found.
  for (auto &requestedInstanceExtension : m_instanceExtensions) {
    bool found = false;
    for (auto &extensionProperty : extensionProperties) {
      // strcmp returns 0 if the strings match.
      if (strcmp(requestedInstanceExtension.c_str(),
                 extensionProperty.extensionName) != 0) {
        continue;
      } else {
        m_activeInstanceExtensions.push_back(
            requestedInstanceExtension.c_str());
        found = true;
        break;
      }
    }
    if (!found) {
      std::cerr << "Failed to find OpenXR instance extension: "
                << requestedInstanceExtension;
    }
  }

  // Fill out an XrInstanceCreateInfo structure and create an XrInstance.
  XrInstanceCreateInfo instanceCI{XR_TYPE_INSTANCE_CREATE_INFO};
  instanceCI.createFlags = 0;
  instanceCI.applicationInfo = AI;
  instanceCI.enabledApiLayerCount =
      static_cast<uint32_t>(m_activeAPILayers.size());
  instanceCI.enabledApiLayerNames = m_activeAPILayers.data();
  instanceCI.enabledExtensionCount =
      static_cast<uint32_t>(m_activeInstanceExtensions.size());
  instanceCI.enabledExtensionNames = m_activeInstanceExtensions.data();

  if (xrCreateInstance(&instanceCI, &m_xrInstance) < 0) {
    std::cerr << "Failed to create XR instance." << std::endl;
  } else {
    std::cout << "Created XR instance." << std::endl;
  }
}

void SimulateXr::_destroy_instance() {
  // Destroy the XrInstance.
  if (xrDestroyInstance(m_xrInstance) < 0)
    std::cerr << "Failed to destroy Instance." << std::endl;
}

void SimulateXr::_get_instance_properties() {
  // Get the instance's properties and log the runtime name and version.
  XrInstanceProperties instanceProperties{XR_TYPE_INSTANCE_PROPERTIES};
  if (xrGetInstanceProperties(m_xrInstance, &instanceProperties) < 0)
    std::cerr << "Failed to get InstanceProperties." << std::endl;

  std::cout << "OpenXR Runtime: " << instanceProperties.runtimeName << " - "
            << XR_VERSION_MAJOR(instanceProperties.runtimeVersion) << "."
            << XR_VERSION_MINOR(instanceProperties.runtimeVersion) << "."
            << XR_VERSION_PATCH(instanceProperties.runtimeVersion) << std::endl;
}

void SimulateXr::_get_system_id() {
  // Get the XrSystemId from the instance and the supplied XrFormFactor.
  XrSystemGetInfo systemGI{XR_TYPE_SYSTEM_GET_INFO};
  systemGI.formFactor = m_formFactor;
  if (xrGetSystem(m_xrInstance, &systemGI, &m_systemID) < 0)
    std::cerr << "Failed to get SystemID." << std::endl;

  // Get the System's properties for some general information about the hardware
  // and the vendor.
  if (xrGetSystemProperties(m_xrInstance, m_systemID, &m_systemProperties) < 0)
    std::cerr << "Failed to get SystemProperties." << std::endl;
}

void SimulateXr::_get_view_configuration_views() {
  // Gets the View Configuration Types. The first call gets the count of the
  // array that will be returned. The next call fills out the array.
  uint32_t viewConfigurationCount = 0;
  if (xrEnumerateViewConfigurations(m_xrInstance, m_systemID, 0,
                                    &viewConfigurationCount, nullptr) < 0)
    std::cerr << "Failed to enumerate View Configurations." << std::endl;
  m_viewConfigurations.resize(viewConfigurationCount);
  if (xrEnumerateViewConfigurations(
          m_xrInstance, m_systemID, viewConfigurationCount,
          &viewConfigurationCount, m_viewConfigurations.data()) < 0)
    std::cerr << "Failed to enumerate View Configurations." << std::endl;

  // Pick the first application supported View Configuration Type con supported
  // by the hardware.
  for (const XrViewConfigurationType &viewConfiguration :
       m_applicationViewConfigurations) {
    if (std::find(m_viewConfigurations.begin(), m_viewConfigurations.end(),
                  viewConfiguration) != m_viewConfigurations.end()) {
      m_viewConfiguration = viewConfiguration;
      break;
    }
  }
  if (m_viewConfiguration == XR_VIEW_CONFIGURATION_TYPE_MAX_ENUM) {
    std::cerr << "Failed to find a view configuration type. Defaulting to "
                 "XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO."
              << std::endl;
    m_viewConfiguration = XR_VIEW_CONFIGURATION_TYPE_PRIMARY_STEREO;
  }

  // Gets the View Configuration Views. The first call gets the count of the
  // array that will be returned. The next call fills out the array.
  uint32_t viewConfigurationViewCount = 0;
  if (xrEnumerateViewConfigurationViews(
          m_xrInstance, m_systemID, m_viewConfiguration, 0,
          &viewConfigurationViewCount, nullptr) < 0)
    std::cerr << "Failed to enumerate ViewConfiguration Views." << std::endl;

  m_viewConfigurationViews.resize(viewConfigurationViewCount,
                                  {XR_TYPE_VIEW_CONFIGURATION_VIEW});
  if (xrEnumerateViewConfigurationViews(
          m_xrInstance, m_systemID, m_viewConfiguration,
          viewConfigurationViewCount, &viewConfigurationViewCount,
          m_viewConfigurationViews.data()) < 0)
    std::cerr << "Failed to enumerate ViewConfiguration Views." << std::endl;

  // set externally usable variables for rendering
  width = m_viewConfigurationViews[0].recommendedImageRectWidth;
  height = m_viewConfigurationViews[0].recommendedImageRectHeight;

  width_render = width * 2;
}

void SimulateXr::_get_environment_blend_modes() {
  // Retrieves the available blend modes. The first call gets the count of the
  // array that will be returned. The next call fills out the array.
  uint32_t environmentBlendModeCount = 0;
  if (xrEnumerateEnvironmentBlendModes(m_xrInstance, m_systemID,
                                       m_viewConfiguration, 0,
                                       &environmentBlendModeCount, nullptr) < 0)
    std::cerr << "Failed to enumerate EnvironmentBlend Modes." << std::endl;
  m_environmentBlendModes.resize(environmentBlendModeCount);
  if (xrEnumerateEnvironmentBlendModes(
          m_xrInstance, m_systemID, m_viewConfiguration,
          environmentBlendModeCount, &environmentBlendModeCount,
          m_environmentBlendModes.data()) < 0)
    std::cerr << "Failed to enumerate EnvironmentBlend Modes." << std::endl;

  // Pick the first application supported blend mode supported by the hardware.
  for (const XrEnvironmentBlendMode &environmentBlendMode :
       m_applicationEnvironmentBlendModes) {
    if (std::find(m_environmentBlendModes.begin(),
                  m_environmentBlendModes.end(),
                  environmentBlendMode) != m_environmentBlendModes.end()) {
      m_environmentBlendMode = environmentBlendMode;
      break;
    }
  }
  if (m_environmentBlendMode == XR_ENVIRONMENT_BLEND_MODE_MAX_ENUM) {
    std::cout << "Failed to find a compatible blend mode. Defaulting to "
                 "XR_ENVIRONMENT_BLEND_MODE_OPAQUE."
              << std::endl;
    m_environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
  }
}

void SimulateXr::_create_session() {
  // Create an XrSessionCreateInfo structure.
  XrSessionCreateInfo sessionCI{XR_TYPE_SESSION_CREATE_INFO};

  if (xrGetInstanceProcAddr(
          m_xrInstance, "xrGetOpenGLGraphicsRequirementsKHR",
          (PFN_xrVoidFunction *)&xrGetOpenGLGraphicsRequirementsKHR) < 0)
    std::cerr << "Failed to get InstanceProcAddr for "
                 "xrGetOpenGLGraphicsRequirementsKHR."
              << std::endl;

  XrGraphicsRequirementsOpenGLKHR graphicsRequirements{
      XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
  if (xrGetOpenGLGraphicsRequirementsKHR(m_xrInstance, m_systemID,
                                         &graphicsRequirements) < 0)
    std::cerr << "Failed to get Graphics Requirements for OpenGL." << std::endl;

  // get graphics binding
  graphicsBinding = {XR_TYPE_GRAPHICS_BINDING_OPENGL_WIN32_KHR};
  // get the DC and context from the window
  // TODO this also feels dirty
  GLFWwindow *window_ = glfwGetCurrentContext();
  graphicsBinding.hDC = GetDC(glfwGetWin32Window(window_));
  graphicsBinding.hGLRC = glfwGetWGLContext(window_);

  sessionCI.next = &graphicsBinding;
  sessionCI.createFlags = 0;
  sessionCI.systemId = m_systemID;

  if (xrCreateSession(m_xrInstance, &sessionCI, &m_session) < 0)
    std::cerr << "Failed to create XR Session." << std::endl;
  else
    std::cout << "Created XR Session." << std::endl;
}

void SimulateXr::_destroy_session() {
  if (xrDestroySession(m_session) < 0)
    std::cerr << "Failed to destroy Session." << std::endl;
}

void SimulateXr::_create_reference_space() {
  // Fill out an XrReferenceSpaceCreateInfo structure and create a reference
  // XrSpace, specifying a Local space with an identity pose as the origin.
  XrReferenceSpaceCreateInfo referenceSpaceCI{
      XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
  referenceSpaceCI.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
  referenceSpaceCI.poseInReferenceSpace = {{0.0f, 0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 0.0f}};
  if (xrCreateReferenceSpace(m_session, &referenceSpaceCI, &m_localSpace) < 0)
    std::cerr << "Failed to create ReferenceSpace." << std::endl;
  else
    std::cout << "Created ReferenceSpace." << std::endl;
}

void SimulateXr::_destroy_reference_space() {
  // Destroy the reference XrSpace.
  if (xrDestroySpace(m_localSpace) < 0)
    std::cerr << "Failed to destroy Space." << std::endl;
}

void SimulateXr::_create_swapchain() {
  // create swapchain and swapchain_images
  // only a single one, color

  // Get the supported swapchain formats as an array of int64_t and ordered by
  // runtime preference.
  uint32_t formatCount = 0;
  if (xrEnumerateSwapchainFormats(m_session, 0, &formatCount, nullptr) < 0)
    std::cerr << "Failed to enumerate Swapchain Formats";
  std::vector<int64_t> formats(formatCount);
  if (xrEnumerateSwapchainFormats(m_session, formatCount, &formatCount,
                                  formats.data()) < 0)
    std::cerr << "Failed to enumerate Swapchain Formats";
  std::cout << "Found Swapchain Formats:"; 
  for (size_t i = 0; i < formatCount; i++) {
    std::cout << " " << std::hex << formats[i];
  }
  std::cout << ". Compatible format: " << std::hex
            << SelectColorSwapchainFormat(formats)
            << "." << std::endl;
  // GL_RGBA16F is 0x881A or 34842
  // GL_RGBA8 is 0x8058 (unsupported)
  // GL_RGBA16 is 0x805b

  // TODO Making only 1
  size_t num_swapchains = 1;  // m_viewConfigurationViews.size()
  m_colorSwapchainInfos.resize(num_swapchains);

  // Per view, create a color and depth swapchain, and their associated image
  // views.
  for (size_t i = 0; i < num_swapchains; i++) {
    SwapchainInfo &colorSwapchainInfo = m_colorSwapchainInfos[i];
    // Fill out an XrSwapchainCreateInfo structure and create an XrSwapchain.
    // Color.
    XrSwapchainCreateInfo swapchainCI{XR_TYPE_SWAPCHAIN_CREATE_INFO};
    swapchainCI.createFlags = 0;
    swapchainCI.usageFlags = //XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT |
                             XR_SWAPCHAIN_USAGE_SAMPLED_BIT |
                             XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT;
    swapchainCI.format = SelectColorSwapchainFormat(formats);
    swapchainCI.sampleCount = 1;
    swapchainCI.width = width_render;
    swapchainCI.height = height;
    swapchainCI.faceCount = 1;
    swapchainCI.arraySize = 1;
    swapchainCI.mipCount = 1;
    int ret = xrCreateSwapchain(m_session, &swapchainCI,
                                 &colorSwapchainInfo.swapchain);
    if (ret < 0)
      std::cerr << "Failed to create Color Swapchain :" << ret << "."
                << std::endl;
    else
      std::cout << "Created Color Swapchain." << std::endl;
    // Save the swapchain format for later use.
    colorSwapchainInfo.swapchainFormat = swapchainCI.format;  

    // Get the number of images in the color/depth swapchain and allocate
    // Swapchain image data via GraphicsAPI to store the returned array.
    uint32_t colorSwapchainImageCount = 0;
    if (xrEnumerateSwapchainImages(colorSwapchainInfo.swapchain, 0,
                                   &colorSwapchainImageCount, nullptr) < 0)
      std::cerr << "Failed to enumerate Color Swapchain Images." << std::endl;
    // following could be simplified
    XrSwapchainImageBaseHeader *colorSwapchainImages =
        AllocateSwapchainImageData(colorSwapchainInfo.swapchain,
                                   SwapchainType::COLOR,
                                   colorSwapchainImageCount);
    if (xrEnumerateSwapchainImages(
            colorSwapchainInfo.swapchain, colorSwapchainImageCount,
            &colorSwapchainImageCount, colorSwapchainImages) < 0)
      std::cerr << "Failed to enumerate Color Swapchain Images." << std::endl;
    else
      std::cout << "Enumerated Color Swapchain Images: "
                << colorSwapchainImageCount
                << "." << std::endl;

    // Per image in the swapchains, fill out a GraphicsAPI::ImageViewCreateInfo
    // structure and create a color/depth image view.
    for (uint32_t j = 0; j < colorSwapchainImageCount; j++) {
      ImageViewCreateInfo imageViewCI;
      imageViewCI.image = GetSwapchainImage(colorSwapchainInfo.swapchain, j);
      imageViewCI.type = ImageViewCreateInfo::Type::RTV;
      imageViewCI.view = ImageViewCreateInfo::View::TYPE_2D;
      imageViewCI.format = colorSwapchainInfo.swapchainFormat;
      imageViewCI.aspect = ImageViewCreateInfo::Aspect::COLOR_BIT;
      imageViewCI.baseMipLevel = 0;
      imageViewCI.levelCount = 1;
      imageViewCI.baseArrayLayer = 0;
      imageViewCI.layerCount = 1;
      colorSwapchainInfo.imageViews.push_back(CreateImageView(imageViewCI));
    }
  }
  std::cout << "Created Swapchain." << std::endl;
}
void SimulateXr::_destroy_swapchain() {
  // TODO(AS)

  //// Destroy the color and depth image views from GraphicsAPI.
  // for (void *&imageView : colorSwapchainInfo.imageViews) {
  //   m_graphicsAPI->DestroyImageView(imageView);
  // }

  //// Free the Swapchain Image Data.
  // m_graphicsAPI->FreeSwapchainImageData(colorSwapchainInfo.swapchain);

  //// Destroy the swapchains.
  // OPENXR_CHECK(xrDestroySwapchain(colorSwapchainInfo.swapchain),
  //              "Failed to destroy Color Swapchain");
}
XrSwapchainImageBaseHeader *SimulateXr::AllocateSwapchainImageData(
    XrSwapchain swapchain, SwapchainType type, uint32_t count) {
  swapchainImagesMap[swapchain].first = type;
  swapchainImagesMap[swapchain].second.resize(
      count, {XR_TYPE_SWAPCHAIN_IMAGE_OPENGL_KHR});
  return reinterpret_cast<XrSwapchainImageBaseHeader *>(
      swapchainImagesMap[swapchain].second.data());
}

void *SimulateXr::CreateImageView(const ImageViewCreateInfo &imageViewCI) {
  GLuint framebuffer = 0;
  glGenFramebuffers(1, &framebuffer);

  GLenum attachment =
      imageViewCI.aspect == ImageViewCreateInfo::Aspect::COLOR_BIT
          ? GL_COLOR_ATTACHMENT0
          : GL_DEPTH_ATTACHMENT;

  glBindFramebuffer(GL_FRAMEBUFFER, framebuffer);
  if (imageViewCI.view == ImageViewCreateInfo::View::TYPE_2D_ARRAY) {
    glFramebufferTextureMultiviewOVR(
        GL_DRAW_FRAMEBUFFER, attachment, (GLuint)(uint64_t)imageViewCI.image,
        imageViewCI.baseMipLevel, imageViewCI.baseArrayLayer,
        imageViewCI.layerCount);
  } else if (imageViewCI.view == ImageViewCreateInfo::View::TYPE_2D) {
    glFramebufferTexture2D(GL_DRAW_FRAMEBUFFER, attachment, GL_TEXTURE_2D,
                           (GLuint)(uint64_t)imageViewCI.image,
                           imageViewCI.baseMipLevel);
  } else {
    std::cerr << "ERROR: OPENGL: Unknown ImageView View type." << std::endl;
  }

  GLenum result = glCheckFramebufferStatus(GL_DRAW_FRAMEBUFFER);
  if (result != GL_FRAMEBUFFER_COMPLETE) {
    std::cerr << "ERROR: OPENGL: Framebuffer is not complete." << std::endl;
  }
  glBindFramebuffer(GL_FRAMEBUFFER, 0);

  imageViews[framebuffer] = imageViewCI;
  return (void *)(uint64_t)framebuffer;
}
int64_t SimulateXr::SelectColorSwapchainFormat(
    const std::vector<int64_t> &formats) {
  const std::vector<int64_t> supportSwapchainFormats = {
      GL_RGB10_A2,
      GL_RGBA16F,
      // The two below should only be used as a fallback, as they are linear
      // color formats without enough bits for color depth, thus leading to
      // banding.
      GL_RGBA8,
      GL_RGBA8_SNORM,
  };

  const std::vector<int64_t>::const_iterator &swapchainFormatIt =
      std::find_first_of(formats.begin(), formats.end(),
                         std::begin(supportSwapchainFormats),
                         std::end(supportSwapchainFormats));
  if (swapchainFormatIt == formats.end()) {
    std::cerr << "ERROR: Unable to find supported Color Swapchain Format"
              << std::endl;
    return 0;
  }

  return *swapchainFormatIt;
}