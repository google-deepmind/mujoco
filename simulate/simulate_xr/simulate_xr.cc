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

// include order is VERY important:
// header, glad, glfw3, glfw3native
// autoformat will try to screw with this
#include "simulate_xr.h"
#include "glad/glad.h"

#include <GLFW/glfw3.h>

#define GLFW_EXPOSE_NATIVE_WIN32
#define GLFW_EXPOSE_NATIVE_WGL
#define GLFW_NATIVE_INCLUDE_NONE
#include <GLFW/glfw3native.h>

SimulateXr::SimulateXr() {}

SimulateXr::~SimulateXr() {}

void SimulateXr::init() {
  // Needed for textures
  if (!gladLoadGLLoader((GLADloadproc)glfwGetProcAddress)) {
    std::cerr << "Failed to initialize OpenGL context for OpenXR." << std::endl;
    return;
  } else if (verbose > 0)
    std::cout << "Initialized OpenGL context for OpenXR." << std::endl;

  if (_create_instance() < 0) {
    std::cerr << "Failed to create OpenXr instance." << std::endl;
    return;
  } else if (verbose > 0)
    std::cout << "Created OpenXr instance." << std::endl;

  _get_instance_properties();

  if (_get_system_id() < 0) {
    std::cerr << "Failed to get XR System ID." << std::endl;
    return;
  } else if (verbose > 1)
    std::cout << "Got XR System ID." << std::endl;

  if (_get_view_configuration_views() < 0) {
    std::cerr << "Failed to get XR view configuration." << std::endl;
    return;
  } else if (verbose > 1)
    std::cout << "Got XR view configuration." << std::endl;

  if (_get_environment_blend_modes() < 0) {
    std::cerr << "Failed to get XR blend modes." << std::endl;
    return;
  } else if (verbose > 1)
    std::cout << "Got XR blend modes." << std::endl;

  if (_create_session() < 0) {
    std::cerr << "Failed to create XR session." << std::endl;
    return;
  } else if (verbose > 0)
    std::cout << "Created XR session." << std::endl;

  if (_create_reference_space() < 0) {
    std::cerr << "Failed to create XR reference space." << std::endl;
    return;
  } else if (verbose > 1)
    std::cout << "Created reference space." << std::endl;

  if (_create_swapchain() < 0) {
    std::cerr << "Failed to create XR swapchain." << std::endl;
    return;
  } else if (verbose > 1)
    std::cout << "Created swapchain." << std::endl;

  m_initialized = true;
  if (verbose > 0) std::cout << "Initialized XR." << std::endl;
}

void SimulateXr::deinit() {
  m_initialized = false;

  _destroy_swapchain();

  _destroy_reference_space();

  _destroy_session();

  _destroy_instance();
}


void SimulateXr::set_scn_params(mjvScene *scn) {
  if (scn) scn->stereo = mjSTEREO_SIDEBYSIDE;
}

void SimulateXr::set_vis_params(mjModel *m) {
  if (m) {
    m->vis.global.offwidth = width_render;
    m->vis.global.offheight = height;
    m->vis.quality.offsamples = 0;
  }
}


bool SimulateXr::before_render(mjvScene *scn, mjModel *m) {
  rendered = false;

  // poll events
  _poll_events();
  if (!m_sessionRunning) return false;

  // Essentially, _render_frame_start()
  // Get the XrFrameState for timing and rendering info.
  XrFrameWaitInfo frameWaitInfo{XR_TYPE_FRAME_WAIT_INFO};
  if (xrWaitFrame(m_session, &frameWaitInfo, &frameState) < 0)
    std::cerr << "Failed to wait for XR Frame." << std::endl;

  // Tell the OpenXR compositor that the application is beginning the frame.
  XrFrameBeginInfo frameBeginInfo{XR_TYPE_FRAME_BEGIN_INFO};
  if (xrBeginFrame(m_session, &frameBeginInfo) < 0)
    std::cerr << "Failed to begin the XR Frame." << std::endl;

  // Variables for rendering and layer composition.
  renderLayerInfo.predictedDisplayTime = frameState.predictedDisplayTime;
  // has to be cleaned
  renderLayerInfo.layers.clear();
  renderLayerInfo.layerProjectionViews.clear();

  // Check that the session is active and that we should render.
  bool sessionActive = (m_sessionState == XR_SESSION_STATE_SYNCHRONIZED ||
                        m_sessionState == XR_SESSION_STATE_VISIBLE ||
                        m_sessionState == XR_SESSION_STATE_FOCUSED);

  if (!(sessionActive && frameState.shouldRender)) return false;

  // essentially, first part of RenderLayer

  // Locate the views from the view configuration within the (reference) space
  // at the display time.
  std::vector<XrView> views(m_viewConfigurationViews.size(), {XR_TYPE_VIEW});

  XrViewState viewState{XR_TYPE_VIEW_STATE};
  // Will contain information on whether the position
  // and/or orientation is valid and/or tracked.
  XrViewLocateInfo viewLocateInfo{XR_TYPE_VIEW_LOCATE_INFO};
  viewLocateInfo.viewConfigurationType = m_viewConfiguration;
  viewLocateInfo.displayTime = renderLayerInfo.predictedDisplayTime;
  viewLocateInfo.space = m_localSpace;
  uint32_t viewCount = 0;
  XrResult result = xrLocateViews(m_session, &viewLocateInfo, &viewState,
                                  static_cast<uint32_t>(views.size()),
                                  &viewCount, views.data());
  if (result != XR_SUCCESS) {
    std::cerr << ("Failed to locate Views.") << std::endl;
    return false;
  }

  // Resize the layer projection views to match the view count. The layer
  // projection views are used in the layer projection.
  renderLayerInfo.layerProjectionViews.resize(
      viewCount, {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW});

  // Acquire and wait for an image from the swapchains.
  // Get the image index of an image in the swapchains.
  // The timeout is infinite.
  uint32_t colorImageIndex = 0;
  XrSwapchainImageAcquireInfo acquireInfo{XR_TYPE_SWAPCHAIN_IMAGE_ACQUIRE_INFO};
  if (xrAcquireSwapchainImage(m_colorSwapchainInfo.swapchain, &acquireInfo,
                              &colorImageIndex) < 0)
    std::cerr << "Failed to acquire Image from the Color Swapchian"
              << std::endl;

  XrSwapchainImageWaitInfo waitInfo = {XR_TYPE_SWAPCHAIN_IMAGE_WAIT_INFO};
  waitInfo.timeout = XR_INFINITE_DURATION;
  if (xrWaitSwapchainImage(m_colorSwapchainInfo.swapchain, &waitInfo) < 0)
    std::cerr << "Failed to wait for Image from the Color Swapchain"
              << std::endl;

  // Per view in the view configuration:
  for (uint32_t i = 0; i < viewCount; i++) {
    _view_to_cam(scn->camera[i], views[i]);

    _fill_layer_proj_views(renderLayerInfo.layerProjectionViews[i], views[i],
                           static_cast<int32_t>(i * width));
  }

  scn->enabletransform = true;
  scn->rotate[0] = cos(0.25 * mjPI);
  scn->rotate[1] = sin(-0.25 * mjPI);
  scn->translate[1] = 0;  // TODO AS give user control?
  scn->translate[2] = -1;

  // RENDER
  // BeginRendering
  glBindFramebuffer(GL_FRAMEBUFFER, (GLuint)m_colorSwapchainInfo.imageViews[0]);
  glFramebufferTexture2D(
      GL_FRAMEBUFFER, GL_COLOR_ATTACHMENT0, GL_TEXTURE_2D,
      (GLuint)GetSwapchainImage(m_colorSwapchainInfo.swapchain, 0), 0);

  rendered = true;

  return true;
}

bool SimulateXr::is_initialized() { return m_initialized; }

void SimulateXr::after_render(mjrContext *con) {
  if (!m_sessionRunning) return;

  int answ;

  // We copy what MuJoCo rendered on our framebuffer object
  // for each imageview
  glBindFramebuffer(GL_READ_FRAMEBUFFER, con->offFBO);
  for (size_t i_imageview = 0;
       i_imageview < m_colorSwapchainInfo.imageViews.size(); i_imageview++) {
    glBindFramebuffer(GL_DRAW_FRAMEBUFFER,
                      (GLuint)m_colorSwapchainInfo.imageViews[i_imageview]);
    glBlitFramebuffer(0, 0, width_render, height, 0, 0, width_render, height,
                      GL_COLOR_BUFFER_BIT, GL_NEAREST);
  }

  // mirror to mujoco window
  glBindFramebuffer(GL_DRAW_FRAMEBUFFER, mjFB_WINDOW);
  //// TODO: pull window size from the system
  // glBlitFramebuffer(0, 0, width_render, height, 0, 0, width / 2, height / 2,
  //                   GL_COLOR_BUFFER_BIT, GL_LINEAR);
  _blit_to_mujoco();

  // here be other things
  //// EndRendering
  glBindFramebuffer(GL_FRAMEBUFFER, 0);
  // glDeleteFramebuffers(1, 0);

  // Give the swapchain image back to OpenXR, allowing the compositor to use
  // the image.
  XrSwapchainImageReleaseInfo releaseInfo{XR_TYPE_SWAPCHAIN_IMAGE_RELEASE_INFO};
  answ = xrReleaseSwapchainImage(m_colorSwapchainInfo.swapchain, &releaseInfo);
  if (answ < 0) {
    // OpenXR throws one XR_ERROR_CALL_ORDER_INVALID at the end
    if (answ != -37 || verbose > 2)
      std::cerr << "Failed to release Image back to the Color Swapchain. Code: "
                << answ << "." << std::endl;
  }

  // Fill out the XrCompositionLayerProjection structure for usage with
  // xrEndFrame().
  renderLayerInfo.layerProjection.layerFlags =
      XR_COMPOSITION_LAYER_BLEND_TEXTURE_SOURCE_ALPHA_BIT |
      XR_COMPOSITION_LAYER_CORRECT_CHROMATIC_ABERRATION_BIT;
  renderLayerInfo.layerProjection.space = m_localSpace;
  renderLayerInfo.layerProjection.viewCount =
      static_cast<uint32_t>(renderLayerInfo.layerProjectionViews.size());
  renderLayerInfo.layerProjection.views =
      renderLayerInfo.layerProjectionViews.data();

  if (rendered) {
    renderLayerInfo.layers.push_back(
        reinterpret_cast<XrCompositionLayerBaseHeader *>(
            &renderLayerInfo.layerProjection));
  }

  // Tell OpenXR that we are finished with this frame; specifying its display
  // time, environment blending and layers.
  XrFrameEndInfo frameEndInfo{XR_TYPE_FRAME_END_INFO};
  frameEndInfo.displayTime = frameState.predictedDisplayTime;
  frameEndInfo.environmentBlendMode = m_environmentBlendMode;
  frameEndInfo.layerCount =
      static_cast<uint32_t>(renderLayerInfo.layers.size());
  frameEndInfo.layers = renderLayerInfo.layers.data();
  answ = xrEndFrame(m_session, &frameEndInfo);
  if (answ < 0) {
    std::cerr << "Failed to end the XR Frame. Code: " << answ << "."
              << std::endl;
    std::cerr << "Layer count: " << frameEndInfo.layerCount << "." << std::endl;
  }
}

int SimulateXr::_create_instance() {
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
  if (xrEnumerateApiLayerProperties(0, &apiLayerCount, nullptr) < 0) {
    std::cerr << "Failed to enumerate ApiLayerProperties." << std::endl;
    return -1;
  }
  apiLayerProperties.resize(apiLayerCount, {XR_TYPE_API_LAYER_PROPERTIES});
  if (xrEnumerateApiLayerProperties(apiLayerCount, &apiLayerCount,
                                    apiLayerProperties.data()) < 0) {
    std::cerr << "Failed to enumerate ApiLayerProperties." << std::endl;
    return -1;
  }
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
                                             nullptr) < 0) {
    std::cerr << "Failed to enumerate InstanceExtensionProperties."
              << std::endl;
    return -2;
  }
  extensionProperties.resize(extensionCount, {XR_TYPE_EXTENSION_PROPERTIES});
  if (xrEnumerateInstanceExtensionProperties(nullptr, extensionCount,
                                             &extensionCount,
                                             extensionProperties.data()) < 0) {
    std::cerr << "Failed to enumerate InstanceExtensionProperties."
              << std::endl;
    return -2;
  }
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

  if (xrCreateInstance(&instanceCI, &m_xrInstance) < 0)
    return -3;

  return 0;
}

void SimulateXr::_destroy_instance() {
  // Destroy the XrInstance.
  if (xrDestroyInstance(m_xrInstance) < 0)
    std::cerr << "Failed to destroy Instance." << std::endl;
}

void SimulateXr::_view_to_cam(mjvGLCamera &cam, const XrView &view) {
  cam.pos[0] = view.pose.position.x;
  cam.pos[1] = view.pose.position.y;
  cam.pos[2] = view.pose.position.z;
  cam.frustum_near = nearZ;
  cam.frustum_far = farZ;
  cam.frustum_bottom = tan(view.fov.angleDown) * nearZ;
  cam.frustum_top = tan(view.fov.angleUp) * nearZ;
  cam.frustum_center =
      0.5 * (tan(view.fov.angleLeft) + tan(view.fov.angleRight)) * nearZ;

  mjtNum rot_quat[4] = {view.pose.orientation.w, view.pose.orientation.x,
                        view.pose.orientation.y, view.pose.orientation.z};

  mjtNum forward[3] = {0, 0, 0};
  const mjtNum forward_vec[3] = {0, 0, -1};
  mju_rotVecQuat(forward, forward_vec, rot_quat);
  cam.forward[0] = forward[0];
  cam.forward[1] = forward[1];
  cam.forward[2] = forward[2];

  mjtNum up[3] = {0, 0, 0};
  const mjtNum up_vec[3] = {0, 1, 0};
  mju_rotVecQuat(up, up_vec, rot_quat);
  cam.up[0] = up[0];
  cam.up[1] = up[1];
  cam.up[2] = up[2];
}

void SimulateXr::_fill_layer_proj_views(
    XrCompositionLayerProjectionView &xr_lpv, const XrView &view,
    const int32_t offset) {
  // Fill out the XrCompositionLayerProjectionView structure specifying
  // the pose and fov from the view. This also associates the swapchain
  // image with this layer projection view.
  xr_lpv = {XR_TYPE_COMPOSITION_LAYER_PROJECTION_VIEW};
  xr_lpv.pose = view.pose;
  xr_lpv.fov = view.fov;
  xr_lpv.subImage.swapchain = m_colorSwapchainInfo.swapchain;
  xr_lpv.subImage.imageRect.offset.x = offset;
  xr_lpv.subImage.imageRect.offset.y = 0;
  xr_lpv.subImage.imageRect.extent.width = static_cast<int32_t>(width);
  xr_lpv.subImage.imageRect.extent.height = static_cast<int32_t>(height);
  xr_lpv.subImage.imageArrayIndex = 0;
  // Useful for multiview rendering.
}

void SimulateXr::_get_instance_properties() {
  // Get the instance's properties and log the runtime name and version.
  XrInstanceProperties instanceProperties{XR_TYPE_INSTANCE_PROPERTIES};
  if (xrGetInstanceProperties(m_xrInstance, &instanceProperties) < 0)
    std::cerr << "Failed to get InstanceProperties." << std::endl;

  if (verbose > 0)
    std::cout << "OpenXR Runtime: " << instanceProperties.runtimeName << " - "
              << XR_VERSION_MAJOR(instanceProperties.runtimeVersion) << "."
              << XR_VERSION_MINOR(instanceProperties.runtimeVersion) << "."
              << XR_VERSION_PATCH(instanceProperties.runtimeVersion)
              << std::endl;
}

int SimulateXr::_get_system_id() {
  // Get the XrSystemId from the instance and the supplied XrFormFactor.
  XrSystemGetInfo systemGI{XR_TYPE_SYSTEM_GET_INFO};
  systemGI.formFactor = m_formFactor;
  if (xrGetSystem(m_xrInstance, &systemGI, &m_systemID) < 0) {
    std::cerr << "Failed to get SystemID." << std::endl;
    return -1;
  }

  // Get the System's properties for some general information about the hardware
  // and the vendor.
  if (xrGetSystemProperties(m_xrInstance, m_systemID, &m_systemProperties) <
      0) {
    std::cerr << "Failed to get SystemProperties." << std::endl;
    return -2;
  }

  return 0;
}

int SimulateXr::_get_view_configuration_views() {
  // Gets the View Configuration Types. The first call gets the count of the
  // array that will be returned. The next call fills out the array.
  uint32_t viewConfigurationCount = 0;
  if (xrEnumerateViewConfigurations(m_xrInstance, m_systemID, 0,
                                    &viewConfigurationCount, nullptr) < 0) {
    std::cerr << "Failed to enumerate View Configurations." << std::endl;
    return -1;
  }
  m_viewConfigurations.resize(viewConfigurationCount);
  if (xrEnumerateViewConfigurations(
          m_xrInstance, m_systemID, viewConfigurationCount,
          &viewConfigurationCount, m_viewConfigurations.data()) < 0) {
    std::cerr << "Failed to enumerate View Configurations." << std::endl;
    return -1;
  }

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
          &viewConfigurationViewCount, nullptr) < 0) {
    std::cerr << "Failed to enumerate ViewConfiguration Views." << std::endl;
    return -2;
  }

  m_viewConfigurationViews.resize(viewConfigurationViewCount,
                                  {XR_TYPE_VIEW_CONFIGURATION_VIEW});
  if (xrEnumerateViewConfigurationViews(
          m_xrInstance, m_systemID, m_viewConfiguration,
          viewConfigurationViewCount, &viewConfigurationViewCount,
          m_viewConfigurationViews.data()) < 0) {
    std::cerr << "Failed to enumerate ViewConfiguration Views." << std::endl;
    return -2;
  }

  // set externally usable variables for rendering
  width = m_viewConfigurationViews[0].recommendedImageRectWidth;
  height = m_viewConfigurationViews[0].recommendedImageRectHeight;

  width_render = width * 2;

  return 0;
}

int SimulateXr::_get_environment_blend_modes() {
  // Retrieves the available blend modes. The first call gets the count of the
  // array that will be returned. The next call fills out the array.
  uint32_t environmentBlendModeCount = 0;
  if (xrEnumerateEnvironmentBlendModes(
          m_xrInstance, m_systemID, m_viewConfiguration, 0,
          &environmentBlendModeCount, nullptr) < 0) {
    std::cerr << "Failed to enumerate EnvironmentBlend Modes." << std::endl;
    return -1;
  }
  m_environmentBlendModes.resize(environmentBlendModeCount);
  if (xrEnumerateEnvironmentBlendModes(
          m_xrInstance, m_systemID, m_viewConfiguration,
          environmentBlendModeCount, &environmentBlendModeCount,
          m_environmentBlendModes.data()) < 0) {
    std::cerr << "Failed to enumerate EnvironmentBlend Modes." << std::endl;
    return -1;
  }
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
    std::cerr << "Failed to find a compatible blend mode. Defaulting to "
                 "XR_ENVIRONMENT_BLEND_MODE_OPAQUE."
              << std::endl;
    m_environmentBlendMode = XR_ENVIRONMENT_BLEND_MODE_OPAQUE;
    return 1;
  }
  return 0;
}

int SimulateXr::_create_session() {
  // Create an XrSessionCreateInfo structure.
  XrSessionCreateInfo sessionCI{XR_TYPE_SESSION_CREATE_INFO};

  if (xrGetInstanceProcAddr(
          m_xrInstance, "xrGetOpenGLGraphicsRequirementsKHR",
          (PFN_xrVoidFunction *)&xrGetOpenGLGraphicsRequirementsKHR) < 0) {
    std::cerr << "Failed to get xrGetOpenGLGraphicsRequirementsKHR."
              << std::endl;
    return -1;
  }

  XrGraphicsRequirementsOpenGLKHR graphicsRequirements{
      XR_TYPE_GRAPHICS_REQUIREMENTS_OPENGL_KHR};
  if (xrGetOpenGLGraphicsRequirementsKHR(m_xrInstance, m_systemID,
                                         &graphicsRequirements) < 0) {
    std::cerr << "Failed to get Graphics Requirements for OpenGL." << std::endl;
    return -2;
  }

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

  if (xrCreateSession(m_xrInstance, &sessionCI, &m_session) < 0) return -3;

  return 0;
}

void SimulateXr::_destroy_session() {
  if (xrDestroySession(m_session) < 0)
    std::cerr << "Failed to destroy Session." << std::endl;
}

int SimulateXr::_create_reference_space() {
  // TODO Fill it here from the model file?

  // Fill out an XrReferenceSpaceCreateInfo structure and create a reference
  // XrSpace, specifying a Local space with an identity pose as the origin.
  XrReferenceSpaceCreateInfo referenceSpaceCI{
      XR_TYPE_REFERENCE_SPACE_CREATE_INFO};
  referenceSpaceCI.referenceSpaceType = XR_REFERENCE_SPACE_TYPE_LOCAL;
  referenceSpaceCI.poseInReferenceSpace = {{0.0f, 0.0f, 0.0f, 1.0f},
                                           {0.0f, 0.0f, 0.0f}};
  if (xrCreateReferenceSpace(m_session, &referenceSpaceCI, &m_localSpace) < 0)
    return -1;

  return 0;
}

void SimulateXr::_destroy_reference_space() {
  // Destroy the reference XrSpace.
  if (xrDestroySpace(m_localSpace) < 0)
    std::cerr << "Failed to destroy Space." << std::endl;
}

int SimulateXr::_create_swapchain() {
  // create swapchain and swapchain_images
  // only a single one, color

  // Get the supported swapchain formats as an array of int64_t and ordered by
  // runtime preference.
  uint32_t formatCount = 0;
  if (xrEnumerateSwapchainFormats(m_session, 0, &formatCount, nullptr) < 0) {
    std::cerr << "Failed to enumerate Swapchain Formats";
    return -1;
  }
  std::vector<int64_t> formats(formatCount);
  if (xrEnumerateSwapchainFormats(m_session, formatCount, &formatCount,
                                  formats.data()) < 0) {
    std::cerr << "Failed to enumerate Swapchain Formats";
    return -1;
  }

  if (this->verbose > 1) {
    std::cout << "Found Swapchain Formats:";
    for (size_t i = 0; i < formatCount; i++) {
      std::cout << " " << std::hex << formats[i] << std::dec;
    }
    std::cout << ". Compatible format: " << std::hex
              << SelectColorSwapchainFormat(formats) << std::dec << "."
              << std::endl;
    // GL_RGBA16F is 0x881A or 34842
    // GL_RGBA8 is 0x8058 (unsupported)
    // GL_RGBA16 is 0x805b
  }

  // Making only 1 swapchain
  // Fill out an XrSwapchainCreateInfo structure and create an XrSwapchain.
  // Color.
  XrSwapchainCreateInfo swapchainCI{XR_TYPE_SWAPCHAIN_CREATE_INFO};
  swapchainCI.createFlags = 0;
  swapchainCI.usageFlags = XR_SWAPCHAIN_USAGE_SAMPLED_BIT |
                           XR_SWAPCHAIN_USAGE_COLOR_ATTACHMENT_BIT |
                           XR_SWAPCHAIN_USAGE_TRANSFER_DST_BIT;
  swapchainCI.format = SelectColorSwapchainFormat(formats);
  swapchainCI.sampleCount = 1;
  swapchainCI.width = width_render;
  swapchainCI.height = height;
  swapchainCI.faceCount = 1;
  swapchainCI.arraySize = 1;
  swapchainCI.mipCount = 1;
  int ret = xrCreateSwapchain(m_session, &swapchainCI,
                              &m_colorSwapchainInfo.swapchain);
  if (ret < 0) {
    std::cerr << "Failed to create Color Swapchain:" << ret << "." << std::endl;
    return -2;
  } else if (this->verbose > 0)
    std::cout << "Created Color Swapchain." << std::endl;

  // Save the swapchain format for later use.
  m_colorSwapchainInfo.swapchainFormat = swapchainCI.format;

  // Get the number of images in the color/depth swapchain and allocate
  // Swapchain image data (buffer) to store the returned array.
  uint32_t colorSwapchainImageCount = 0;
  if (xrEnumerateSwapchainImages(m_colorSwapchainInfo.swapchain, 0,
                                 &colorSwapchainImageCount, nullptr) < 0) {
    std::cerr << "Failed to enumerate Color Swapchain Images." << std::endl;
    return -3;
  }
  XrSwapchainImageBaseHeader *colorSwapchainImages = AllocateSwapchainImageData(
      m_colorSwapchainInfo.swapchain, SwapchainType::COLOR,
      colorSwapchainImageCount);
  if (xrEnumerateSwapchainImages(
          m_colorSwapchainInfo.swapchain, colorSwapchainImageCount,
          &colorSwapchainImageCount, colorSwapchainImages) < 0) {
    std::cerr << "Failed to enumerate Color Swapchain Images." << std::endl;
    return -3;
  } else if (this->verbose > 1)
    std::cout << "Enumerated Color Swapchain Images: "
              << colorSwapchainImageCount << "." << std::endl;

  // Per image in the swapchains, fill out a GraphicsAPI::ImageViewCreateInfo
  // structure and create a color/depth image view.
  for (uint32_t j = 0; j < colorSwapchainImageCount; j++) {
    ImageViewCreateInfo imageViewCI;
    imageViewCI.image = GetSwapchainImage(m_colorSwapchainInfo.swapchain, j);
    imageViewCI.type = ImageViewCreateInfo::Type::RTV;
    imageViewCI.view = ImageViewCreateInfo::View::TYPE_2D;
    imageViewCI.format = m_colorSwapchainInfo.swapchainFormat;
    imageViewCI.aspect = ImageViewCreateInfo::Aspect::COLOR_BIT;
    imageViewCI.baseMipLevel = 0;
    imageViewCI.levelCount = 1;
    imageViewCI.baseArrayLayer = 0;
    imageViewCI.layerCount = 1;
    m_colorSwapchainInfo.imageViews.push_back(CreateImageView(imageViewCI));
  }

  return 0;
}

void SimulateXr::_destroy_swapchain() {
  // Destroy the color image view from GraphicsAPI.
  for (void *&imageView : m_colorSwapchainInfo.imageViews) {
    GLuint framebuffer = (GLuint)(uint64_t)imageView;
    imageViews.erase(framebuffer);
    glDeleteFramebuffers(1, &framebuffer);
  }

  // Free the Swapchain Image Data.
  swapchainImagesMap[m_colorSwapchainInfo.swapchain].second.clear();
  swapchainImagesMap.erase(m_colorSwapchainInfo.swapchain);

  // Destroy the swapchains.
  if (xrDestroySwapchain(m_colorSwapchainInfo.swapchain) < 0)
    std::cerr << "Failed to destroy Color Swapchain." << std::endl;
}

void SimulateXr::_poll_events() {
  // Poll OpenXR for a new event.
  XrEventDataBuffer eventData{XR_TYPE_EVENT_DATA_BUFFER};
  auto XrPollEvents = [&]() -> bool {
    eventData = {XR_TYPE_EVENT_DATA_BUFFER};
    return xrPollEvent(m_xrInstance, &eventData) == XR_SUCCESS;
  };

  while (XrPollEvents()) {
    switch (eventData.type) {
      // Log the number of lost events from the runtime.
      case XR_TYPE_EVENT_DATA_EVENTS_LOST: {
        XrEventDataEventsLost *eventsLost =
            reinterpret_cast<XrEventDataEventsLost *>(&eventData);
        std::cerr << "OPENXR: Events Lost: " << eventsLost->lostEventCount
                  << std::endl;
        break;
      }
      // Log that an instance loss is pending and shutdown the application.
      case XR_TYPE_EVENT_DATA_INSTANCE_LOSS_PENDING: {
        XrEventDataInstanceLossPending *instanceLossPending =
            reinterpret_cast<XrEventDataInstanceLossPending *>(&eventData);
        std::cout << "OPENXR: Instance Loss Pending at: "
                  << instanceLossPending->lossTime << std::endl;
        m_sessionRunning = false;
        break;
      }
      // Log that the interaction profile has changed.
      case XR_TYPE_EVENT_DATA_INTERACTION_PROFILE_CHANGED: {
        XrEventDataInteractionProfileChanged *interactionProfileChanged =
            reinterpret_cast<XrEventDataInteractionProfileChanged *>(
                &eventData);
        std::cout << "OPENXR: Interaction Profile changed for Session: "
                  << interactionProfileChanged->session << std::endl;
        if (interactionProfileChanged->session != m_session) {
          std::cout
              << "XrEventDataInteractionProfileChanged for unknown Session"
              << std::endl;
          break;
        }
        break;
      }
      // Log that there's a reference space change pending.
      case XR_TYPE_EVENT_DATA_REFERENCE_SPACE_CHANGE_PENDING: {
        XrEventDataReferenceSpaceChangePending *referenceSpaceChangePending =
            reinterpret_cast<XrEventDataReferenceSpaceChangePending *>(
                &eventData);
        std::cout << "OPENXR: Reference Space Change pending for Session: "
                  << referenceSpaceChangePending->session << std::endl;
        if (referenceSpaceChangePending->session != m_session) {
          std::cout
              << "XrEventDataReferenceSpaceChangePending for unknown Session"
              << std::endl;
          break;
        }
        break;
      }
      // Session State changes:
      case XR_TYPE_EVENT_DATA_SESSION_STATE_CHANGED: {
        XrEventDataSessionStateChanged *sessionStateChanged =
            reinterpret_cast<XrEventDataSessionStateChanged *>(&eventData);
        if (sessionStateChanged->session != m_session) {
          std::cout << "XrEventDataSessionStateChanged for unknown Session"
                    << std::endl;
          break;
        }

        if (sessionStateChanged->state == XR_SESSION_STATE_READY) {
          // SessionState is ready. Begin the XrSession using the
          // XrViewConfigurationType.
          XrSessionBeginInfo sessionBeginInfo{XR_TYPE_SESSION_BEGIN_INFO};
          sessionBeginInfo.primaryViewConfigurationType = m_viewConfiguration;
          if (xrBeginSession(m_session, &sessionBeginInfo) < 0)
            std::cerr << "Failed to begin Session." << std::endl;
          m_sessionRunning = true;
        }
        if (sessionStateChanged->state == XR_SESSION_STATE_STOPPING) {
          // SessionState is stopping. End the XrSession.
          if (xrEndSession(m_session) < 0)
            std::cerr << "Failed to end Session." << std::endl;
          m_sessionRunning = false;
        }
        if (sessionStateChanged->state == XR_SESSION_STATE_EXITING) {
          // SessionState is exiting. Exit the application.
          m_sessionRunning = false;
        }
        if (sessionStateChanged->state == XR_SESSION_STATE_LOSS_PENDING) {
          // SessionState is loss pending. Exit the application.
          // It's possible to try a reestablish an XrInstance and XrSession, but
          // we will simply exit here.
          m_sessionRunning = false;
        }
        // Store state for reference across the application.
        m_sessionState = sessionStateChanged->state;
        break;
      }
      default: {
        break;
      }
    }
  }
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

void SimulateXr::_blit_to_mujoco() {
  // make the background black
  glClearColor(0, 0, 0, 1);

  // get window size
  int dst_x = 0, dst_y = 0;
  int dst_width, dst_height;
  GLFWwindow *window_ = glfwGetCurrentContext();
  glfwGetWindowSize(window_, &dst_width, &dst_height);

  int src_width = width;
  int src_height = width;  // height / 2;
  int src_x = 0;
  int src_y = (height - src_height) / 2;

  // aspect ratio
  float dst_ar = ((float)dst_width) / dst_height;
  float src_ar = ((float)src_width) / src_height;

  // bound dst_ar to 2.5:1 and 1:2.5, bc otherwise weird visual stuff happens
  if (dst_ar < 0.4) dst_ar = 0.4;
  if (dst_ar > 2.5) dst_ar = 2.5;

  // adjust the widths and height to preserve proportions
  if (src_ar > dst_ar) {
    // source is wider than destination, take the middle subsection of src
    // proportional to the dst
    src_width = dst_ar * src_height;
    src_x = 0.5 * ((src_ar * src_height) - src_width);
  } else {
    //// source is narrower than destination, blit to the middle part of the dst
    // does not perform properly
    // dst_width = src_ar * dst_height;
    // dst_x = 0.5 * ((dst_ar * dst_height) - dst_width);

    // the other way around
    src_height = ((float)src_width) / dst_ar;
    src_y += 0.5 * ((((float)src_width) / src_ar) - src_height);
  }
  if (verbose > 3) {
    std::cout << "Blit to window information:" << std::endl;
    std::cout << "Src: " << width << " " << height << " " << src_ar
              << std::endl;
    std::cout << "Dst: " << dst_width << " " << dst_height << " " << dst_ar
              << std::endl;
    std::cout << "Chg: " << src_x << " " << src_y << " " << src_width << " "
              << src_height << std::endl;
  }

  glBlitFramebuffer(src_x, src_y, src_width, src_height, dst_x, dst_y,
                    dst_width, dst_height, GL_COLOR_BUFFER_BIT, GL_LINEAR);
}
