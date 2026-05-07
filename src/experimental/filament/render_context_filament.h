// Copyright 2025 DeepMind Technologies Limited
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

#ifndef MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_H_
#define MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_H_

#include <cstdint>

#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjvisualize.h>
#include <mujoco/mujoco.h>

#if defined(__cplusplus)
extern "C" {
#endif

// IMPORTANT: This API should still be considered experimental and is likely
// change frequently.

// Opaque types.
struct mjrTexture {};
struct mjrMesh {};
struct mjrScene {};
struct mjrLight {};
struct mjrRenderable {};
struct mjrRenderTarget {};

// Opaque type for the filament rendering context.
struct mjrfContext {};

// The different modes that can be used to render a scene.
typedef enum mjrDrawMode_ {
  // Render the scene with "normal" colors and lighting.
  mjDRAW_MODE_COLOR,
  // Render the scene as a grayscale depth map.
  mjDRAW_MODE_DEPTH,
  // Render each object with a unique, uniform (flat) color regardless of
  // lighting and texture.
  mjDRAW_MODE_SEGMENTATION,
} mjrDrawMode;

enum { mjNUM_DRAW_MODES = 3 };

// The type of data stored in an index buffer.
typedef enum mjrIndexType_ {
  mjINDEX_TYPE_U16 = 0,
  mjINDEX_TYPE_U32,
} mjrIndexType;

// The type of primitive to be drawn by vertex data.
typedef enum mjrMeshPrimitiveType_ {
  mjMESH_PRIMITIVE_TYPE_TRIANGLES = 0,
  mjMESH_PRIMITIVE_TYPE_LINES,
} mjrMeshPrimitiveType;

// The usage/purpose of an attribute of a vertex.
typedef enum mjrVertexAttributeUsage_ {
  mjVERTEX_ATTRIBUTE_USAGE_POSITION = 0,
  mjVERTEX_ATTRIBUTE_USAGE_NORMAL,
  mjVERTEX_ATTRIBUTE_USAGE_TANGENTS,
  mjVERTEX_ATTRIBUTE_USAGE_UV,
  mjVERTEX_ATTRIBUTE_USAGE_COLOR,
} mjrVertexAttributeUsage;

// The data format of an attribute of a vertex.
typedef enum mjrVertexAttributeType_ {
  mjVERTEX_ATTRIBUTE_TYPE_FLOAT2 = 0,
  mjVERTEX_ATTRIBUTE_TYPE_FLOAT3,
  mjVERTEX_ATTRIBUTE_TYPE_FLOAT4,
  mjVERTEX_ATTRIBUTE_TYPE_UBYTE4,
} mjrVertexAttributeType;

// Pixel formats for textures.
typedef enum mjrPixelFormat_ {
  mjPIXEL_FORMAT_UNKNOWN = 0,
  mjPIXEL_FORMAT_R8,
  mjPIXEL_FORMAT_RGB8,
  mjPIXEL_FORMAT_RGBA8,
  mjPIXEL_FORMAT_R32F,
  mjPIXEL_FORMAT_DEPTH32F,
  mjPIXEL_FORMAT_KTX,
} mjrPixelFormat;

typedef enum mjrGraphicsApi_ {  // backend graphics API to use
  mjGRAPHICS_API_DEFAULT = 0,   // default based on platform
  mjGRAPHICS_API_OPENGL,        // OpenGL (desktop) / WebGL
  mjGRAPHICS_API_VULKAN         // Vulkan
} mjrGraphicsApi;


// Rendering is asynchronous by nature. Each render request is assigned a
// unique Handle which can be used to query the status of the request. The
// Handle can also be used to block until the request is completed.
typedef std::uint64_t mjrFrameHandle;

// Bring some legacy mjt types into the mjr namespace.
typedef mjtTexture mjrSamplerType;
typedef mjtColorSpace mjrColorSpace;
typedef mjtLightType mjrLightType;
typedef mjvGLCamera mjrCamera;

// The material to be applied to a renderable.
struct mjrMaterial {
  // The color of the object. Defaults to white.
  float color[4];

  // The color to use for segmentation rendering. Defaults to white.
  float segmentation_color[4];

  // Applies an addition scale to the UV coordinates of the object. Defaults to
  // (1, 1, 1).
  float uv_scale[3];

  // Applies an offset to the UV coordinates of the object. Defaults to (0, 0,
  // 0).
  float uv_offset[3];

  // Applies a scissor test to the object.
  float scissor[4];

  // Factors for PBR metallic-roughness materials.
  float metallic;
  float roughness;

  // Factors for (non-PBR) specular-glossiness materials.
  float specular;
  float glossiness;

  // The emissive (glow) factor of the object.
  float emissive;

  // Whether or not the object is a reflective surface. Only applies to planes.
  mjtByte reflective;
  // The blend factor to use for reflective surfaces. A value of 1.0 means that
  // the surface is fully reflective (i.e. a mirror).
  float reflectance;

  // If true, does not apply any lighting to the object. (Assumes the object is
  // used for UX or decorative elements like contact forces and labels.)
  mjtByte decor_ux;

  // The texture containing the base color of the object.
  const mjrTexture* color_texture;

  // The normal map of the object.
  const mjrTexture* normal_texture;

  // The metallic map of the object.
  const mjrTexture* metallic_texture;

  // The roughness map of the object.
  const mjrTexture* roughness_texture;

  // The occlusion map of the object.
  const mjrTexture* occlusion_texture;

  // A texture containing the occlusion, roughness, and metallic maps packed
  // into the R, G, B channels, respectively.
  const mjrTexture* orm_texture;

  // An emissive texture for the object.
  const mjrTexture* emissive_texture;

  // The reflection texture to use for the object. For internal use only.
  const mjrTexture* reflection_texture;
};

// Initializes the mjrMaterial to default values.
void mjr_defaultMaterial(mjrMaterial* material);

// The binary contents of a texture.
struct mjrTextureData {
  // Pointer to the image data. If null, an empty texture will be created.
  const void* bytes;

  // The number of bytes in the image data.
  mjtSize nbytes;

  // Because rendering may be multithreaded, we cannot make assumptions about
  // when the image data will finish uploading to the GPU. As such, we will use
  // this callback to notify callers when it is safe to free the image data.
  void (*release_callback)(void* user_data);

  // User data to pass to the release callback.
  void* user_data;
};

// Initializes the mjrTextureData to default values.
void mjr_defaultTextureData(mjrTextureData* data);

// Defines the basic properties of a texture.
struct mjrTextureConfig {
  // The width of the texture. For compressed textures (e.g. KTX), this is the
  // number of bytes in the compressed data.
  int width;

  // The height of the texture. For compressed textures (e.g. KTX), this should
  // be 0.
  int height;

  // The target of the texture (e.g. 2D, cube, etc.)
  mjrSamplerType sampler_type;

  // The format of the pixels in the texture (e.g. RGB8, RGBA8, KTX, etc.)
  mjrPixelFormat format;

  // The color space of the texture (e.g. LINEAR, sRGB, etc.)
  mjrColorSpace color_space;
};

// Initializes the mjrTextureConfig to default values.
void mjr_defaultTextureConfig(mjrTextureConfig* config);

// Configuration parameters for a Renderable.
struct mjrRenderableParams {
  // Whether or not the Renderable casts shadows.
  mjtByte cast_shadows;
  // Whether or not the Renderable receives shadows.
  mjtByte receive_shadows;
  // The layers to which the Renderable belongs. This mask is used in
  // conjunction with the layer mask in the Scene to determine which
  // Renderables to render. Defaults to 0xff.
  uint8_t layer_mask;
  // Controls the order in which the Renderable is drawn relative to other
  // Renderables; defaults to 4.
  uint8_t priority;
  // Similar to priority, but provides finer-grained control for Renderables
  // with transparency; defaults to 0.
  uint16_t blend_order;
};

// Initializes the mjrRenderableParams to default values.
void mjr_defaultRenderableParams(mjrRenderableParams* params);

// Information about a single attribute of a vertex.
struct mjrVertexAttribute {
  // The data for the attribute.
  const void* bytes;

  // The usage/purpose of the attribute.
  mjrVertexAttributeUsage usage;

  // The data format of the attribute.
  mjrVertexAttributeType type;
};

// Maximum number of vertex attributes in a mesh.
enum { mjMAX_VERTEX_ATTRIBUTES = 16 };

// The binary contents of a mesh.
struct mjrMeshData {
  // The number of vertices in the mesh. Each of the vertex arrays below is
  // assumed to have this number of elements.
  mjtSize nvertices;

  // The number of attributes for each vertex in the mesh.
  int nattributes;

  // Information about each attribute of a vertex in the mesh. See `interleaved`
  // for more details.
  mjrVertexAttribute attributes[mjMAX_VERTEX_ATTRIBUTES];

  // Whether the vertex attributes are interleaved or not.
  //
  // If true, assumes that the attributes are packed in the order specified in
  // the attributes array, with no padding in-between. Additionally, the
  // `data` pointer for each attribute is assumed to point to the first element
  // of that type.
  //
  // If false, assume each attribute is stored in a separate array as defined
  // by the `data` field of the attribute.
  mjtByte interleaved;

  // The number of indices in the mesh. The indices array is assumed to have
  // this number of elements.
  mjtSize nindices;

  // The indices of the mesh, stored as either ushort or uint depending on the
  // index type.
  const void* indices;

  // The type of data stored in the indices array.
  mjrIndexType index_type;

  // The type of primitive to be drawn by vertex data.
  mjrMeshPrimitiveType primitive_type;

  // Whether to compute the bounds of the mesh using the vertex positions.
  mjtByte compute_bounds;

  // The bounds of the mesh. If bounds_min == bounds_max, then we assume that
  // that the bounds are not set (i.e. the bounds is empty).
  float bounds_min[3];
  float bounds_max[3];

  // Because rendering may be multithreaded, we cannot make assumptions about
  // when the mesh data will finish uploading to the GPU. As such, we will use
  // this callback to notify callers when it is safe to free the mesh data.
  void (*release_callback)(void* user_data);

  // User data to pass to the release callback.
  void* user_data;
};

// Initializes the mjrMeshData to default values.
void mjr_defaultMeshData(mjrMeshData* data);

// Configuration parameters for a Scene.
struct mjrSceneParams {
  // Whether or not to enable post processing; enabled by default.
  mjtByte enable_post_processing;
  // Whether or not to enable reflections; enabled by default.
  mjtByte enable_reflections;
  // Whether or not to enable shadows; enabled by default.
  mjtByte enable_shadows;
  // This mask, in conjunction with the layer mask in the Renderable, determines
  // which Renderables to render within the Scene.
  uint8_t layer_mask;
  // The layer mask to use for reflections.
  uint8_t reflection_layer_mask;
};

// Initializes the mjrSceneParams to default values.
void mjr_defaultSceneParams(mjrSceneParams* params);

// Configuration parameters for a light.
struct mjrLightParams {
  // The type of light (e.g. spot, point, directional, etc.)
  mjrLightType type;
  // The texture to use for image lights.
  const mjrTexture* texture;
  // The color of the light.
  float color[3];
  // The intensity of the light, in candela.
  float intensity;
  // Whether or not the light casts shadows.
  mjtByte cast_shadows;
  // The range/distance in which the light is effective, in meters.
  float range;
  // The angle of the spot light cone, in degrees.
  float spot_cone_angle;
  // The radius of the bulb used for soft shadows.
  float bulb_radius;
  // The size of the shadow map.
  int shadow_map_size;
  // Blur width for EL VSM.
  float vsm_blur_width;
};

// Initializes the mjrLightParams to default values.
void mjr_defaultLightParams(mjrLightParams* params);

// Defines the basic properties of a render target.
struct mjrRenderTargetConfig {
  // The width of the render target.
  int width;
  // The height of the render target.
  int height;
  // The format of the color buffer in the render target.
  mjrPixelFormat color_format;
  // The format of the depth buffer in the render target.
  mjrPixelFormat depth_format;
};

// Initializes the RenderTargetConfig to default values.
void mjr_defaultRenderTargetConfig(mjrRenderTargetConfig* config);

// Information needed to render a single image of a scene.
struct mjrRenderRequest {
  // The scene to render.
  mjrScene* scene;

  // The method (e.g. Color, Depth, Segmentation, etc.) to use for rendering.
  mjrDrawMode draw_mode;

  // The camera from which to render the scene.
  mjrCamera camera;

  // The viewport into which to render the image.
  mjrRect viewport;

  // The render target into which to render the image. If nullptr, the image
  // will be rendered to the window (as previously configured in
  // mjrFilamentConfig::native_window).
  mjrRenderTarget* target;
};

// Initializes the mjrRenderRequest to default values.
void mjr_defaultRenderRequest(mjrRenderRequest* request);

// Information needed to read pixels from a render target.
struct mjrReadPixelsRequest {
  mjrRenderTarget* target;

  // The buffer into which the read pixels will be written.
  void* output;

  // The number of bytes in the output buffer. This should match the size of
  // the render target texture.
  mjtSize num_bytes;

  // Callback when the read pixels operation is complete. This will be called
  // during WaitForFrame() or in a subsequent call to Render(). This function
  // can optionally be used to free the output buffer if needed.
  void (*read_completed_callback)(void* user_data);

  // User data to pass to the completion callback.
  void* user_data;
};

// Initializes the mjrReadPixelsRequest to default values.
void mjr_defaultReadPixelsRequest(mjrReadPixelsRequest* request);

// Information about a single frame of rendering.
struct mjrFrameStats {
  // The frame rate of the renderer, in frames per second.
  double frame_rate;
};

// Initializes the mjrFrameStats to default values.
void mjr_defaultFrameStats(mjrFrameStats* stats);

// Configuration parameters for the filament rendering context.
struct mjrFilamentConfig {
  // The native window handle into which we can render directly.
  void* native_window;

  // The initial width and height of the offscreen framebuffer.
  int width;
  int height;

  // The backend graphics API to use.
  int graphics_api;

  // Use software rendering even if the platform supports hardware rendering.
  bool force_software_rendering;
};

// Initializes the mjrFilamentConfig to default values.
void mjrf_defaultFilamentConfig(mjrFilamentConfig* config);

// Creates a filament rendering context.
mjrfContext* mjrf_createContext(const mjrFilamentConfig* config);

// Destroys the filament rendering context.
void mjrf_destroyContext(mjrfContext* ctx);

// Creates a texture for the filament renderer.
mjrTexture* mjrf_createTexture(mjrfContext* ctx, const mjrTextureConfig* cfg);

// Destroys the texture.
void mjrf_destroyTexture(mjrTexture* texture);

// Creates a mesh for the filament renderer.
mjrMesh* mjrf_createMesh(mjrfContext* ctx, const mjrMeshData* data);

// Destroys the mesh.
void mjrf_destroyMesh(mjrMesh* mesh);

// Creates a scene for the filament renderer.
mjrScene* mjrf_createScene(mjrfContext* ctx, const mjrSceneParams* params);

// Destroys the scene.
void mjrf_destroyScene(mjrScene* scene);

// Creates a light for the filament renderer.
mjrLight* mjrf_createLight(mjrfContext* ctx, const mjrLightParams* params);

// Destroys the light.
void mjrf_destroyLight(mjrLight* light);

// Creates a renderable for the filament renderer.
mjrRenderable* mjrf_createRenderable(mjrfContext* ctx, const mjrRenderableParams* params);

// Destroys the renderable.
void mjrf_destroyRenderable(mjrRenderable* renderable);

// Creates a render target for the filament renderer.
mjrRenderTarget* mjrf_createRenderTarget(mjrfContext* ctx,
                                         const mjrRenderTargetConfig* config);

// Destroys the render target.
void mjrf_destroyRenderTarget(mjrRenderTarget* render_target);

// Uploads the given texture data to the texture.
void mjrf_setTextureData(mjrTexture* texture, const mjrTextureData* data);

// Returns the width of the texture.
int mjrf_getTextureWidth(const mjrTexture* texture);

// Returns the height of the texture.
int mjrf_getTextureHeight(const mjrTexture* texture);

// Returns the sampler type of the texture.
mjrSamplerType mjrf_getSamplerType(const mjrTexture* texture);

// Enables or disables the light.
void mjrf_setLightEnabled(mjrLight* light, bool enabled);

// Sets the intensity of the light, in candela.
void mjrf_setLightIntensity(mjrLight* light, float intensity);

// Sets the RGB color of the light.
void mjrf_setLightColor(mjrLight* light, const float color[3]);

// Sets the position and direction of the light.
void mjrf_setLightTransform(mjrLight* light, const float position[3],
                            const float direction[3]);

// Returns the type of the light.
mjrLightType mjrf_getLightType(const mjrLight* light);

// Sets the mesh of the renderable.
void mjrf_setRenderableMesh(mjrRenderable* renderable, const mjrMesh* mesh,
                            int elem_offset, int elem_count);

// Sets the mesh of the renderable to a built-in mesh based on the geom type.
// Note: using the same parameters (nstack, nslice, nquad) will have better
// performance as the internal mesh data can be shared across renderables.
void mjrf_setRenderableGeomMesh(mjrRenderable* renderable, mjtGeom type,
                                int nstack, int nslice, int nquad);

// Sets the material properties and textures of the renderable.
void mjrf_setRenderableMaterial(mjrRenderable* renderable,
                                const mjrMaterial* material);

// Sets the transform (position, rotation, and size) of the renderable.
void mjrf_setRenderableTransform(mjrRenderable* renderable,
                                 const float position[3],
                                 const float rotation[9], const float size[3]);

// Sets whether the renderable casts shadows or not.
void mjrf_setRenderableCastShadows(mjrRenderable* renderable,
                                   bool cast_shadows);

// Sets whether the renderable receives shadows or not.
void mjrf_setRenderableReceiveShadows(mjrRenderable* renderable,
                                      bool receive_shadows);

// Forces the renderable to be rendered using lines.
void mjrf_setRenderableWireframe(mjrRenderable* renderable, bool wireframe);

// Sets the layer mask of the renderable. See mjrRenderableParams for details.
void mjrf_setRenderableLayerMask(mjrRenderable* renderable, uint8_t layer_mask);

// Adds the light to the scene.
void mjrf_addLightToScene(mjrScene* scene, mjrLight* light);

// Removes the light from the scene.
void mjrf_removeLightFromScene(mjrScene* scene, mjrLight* light);

// Adds the renderable to the scene.
void mjrf_addRenderableToScene(mjrScene* scene, mjrRenderable* renderable);

// Removes the renderable from the scene.
void mjrf_removeRenderableFromScene(mjrScene* scene, mjrRenderable* renderable);

// Sets the skybox texture of the scene.
void mjrf_setSceneSkybox(mjrScene* scene, const mjrTexture* texture);

// Enables (or disables) shadows in the scene..
void mjrf_setSceneShadowsEnabled(mjrScene* scene, bool enabled);

// Enables (or disables) reflections in the scene.
void mjrf_setSceneReflectionsEnabled(mjrScene* scene, bool enabled);

// Configures the scene based on the parameters in the model.
void mjrf_configureSceneFromModel(mjrScene* scene, const mjModel* model);

// Submits the given requests for rendering.
mjrFrameHandle mjrf_render(mjrfContext* ctx, const mjrRenderRequest* req,
                           int nreq, const mjrReadPixelsRequest* read_req,
                           int nread_req);

// Waits for the rendering to complete for the given frame handle.
void mjrf_waitForFrame(mjrfContext* ctx, mjrFrameHandle frame);

// Returns the stats for the given frame but updating the given `stats_out`.
void mjrf_getFrameStats(mjrfContext* ctx, mjrFrameHandle frame,
                        mjrFrameStats* stats_out);

// Draws an ImGui editor for the given scene, exposing filament-specific
// settings.
void mjrf_DEBUG_drawImguiEditor(mjrScene* scene);


// Legacy API, to be deprecated.

void mjrf_defaultFilamentConfig(mjrFilamentConfig* config);

void mjrf_makeFilamentContext(const mjModel* m, mjrContext* con,
                              const mjrFilamentConfig* config);

void mjrf_defaultContext(mjrContext* con);

void mjrf_makeContext(const mjModel* m, mjrContext* con, int fontscale);

void mjrf_freeContext(mjrContext* con);

void mjrf_renderScene(mjrRect viewport, mjvScene* scn, const mjrContext* con);

void mjrf_uploadMesh(const mjModel* m, const mjrContext* con, int meshid);

void mjrf_uploadTexture(const mjModel* m, const mjrContext* con, int texid);

void mjrf_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid);

void mjrf_setBuffer(int framebuffer, mjrContext* con);

void mjrf_readPixels(unsigned char* rgb, float* depth, mjrRect viewport,
                     const mjrContext* con);

double mjrf_getFrameRate(const mjrContext* con);

uintptr_t mjrf_uploadGuiImage(uintptr_t tex_id, const unsigned char* pixels,
                              int width, int height, int bpp,
                              const mjrContext* con);

void mjrf_updateGui(const mjrContext* con);

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // MUJOCO_SRC_EXPERIMENTAL_FILAMENT_RENDER_CONTEXT_FILAMENT_H_
