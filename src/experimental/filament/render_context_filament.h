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

// This library provides a C API for the filament rendering library
// (https://github.com/google/filament) that is designed to work with the
// MuJoCo library for visualizing simulations.
//
// The filament renderer is a real-time physically based rendering (PBR) engine
// developed by Google. It is designed to be as small as possible and as
// efficient as possible, while still providing high-quality results. It works
// across all major platforms (Linux, Windows, macOS, Android, iOS, Web) and
// supports OpenGL, Vulkan, and Metal.
//
// For the purposes of this API, we assume the reader has a basic understanding
// of rendering concepts (e.g. textures, vertices, cameras, framebuffers, etc.).
// We will also highlight some of the key differences between this renderer and
// the legacy/classic MuJoCo (mjr) renderer.
//
// ## API Overview
//
// There are seven key components: Context, Texture, Mesh, Scene, Light,
// Renderable, and RenderTarget. We'll describe these in detail further below.
//
// Each object is created using a `create` function and destroyed using a
// `destroy` function, e.g. `mjrf_createTexture` and `mjrf_destroyTexture`.
// The `create` functions accept a pointer to a configuration struct (e.g.
// `mjrTextureConfig`) which describes the parameters for the object to be
// created. Each of these structs has a corresponding `default` function (e.g.
// `mjr_defaultTextureConfig`) which can be used to initialize the struct to
// default values. Default values are assumed to be 0/NULL unless otherwise
// specified.
//
// For now, we'll just define opaque handles for each of our components.
struct mjrfContext {};
struct mjrTexture {};
struct mjrMesh {};
struct mjrScene {};
struct mjrLight {};
struct mjrRenderable {};
struct mjrRenderTarget {};


// ## Rendering Context (mjrfContext)
//
// The Context is the main entry point for the library. It manages all the
// core filament objects that are responsible for the rendering of an image.
//
// Filament uses a separate thread for doing the actual rendering. However,
// despite that, this API is not thread-safe; calls are expected to be made
// from a single thread. Also, due to the asynchronous nature of filament,
// some APIs provide handles or callbacks to signal when an operation is
// complete. (Note: for WASM builds, filament does not use a separate thread.)
//
// All other objects (e.g. Textures, Meshes, Scenes, etc.) need a Context in
// order to be created. Otherwise, the main function to use with the Context is
// `mjrf_render()` which does the actual rendering.
//
// There are two key differences between the mjrfContext and the classic
// mjrContext. Firstly, the filament context will manage the underlying graphics
// context itself. This means users do not need to initialize EGL or similar
// libraries beforehand. Secondly, the filament context is independent of a
// MuJoCo model. That means you can use a single mjrfContext to render images
// for multiple models.

// Underlying graphics API library to use for the Context.
typedef enum mjrGraphicsApi_ {
  // Default, based on current platform.
  mjGRAPHICS_API_DEFAULT = 0,
  // OpenGL (desktop), GLES (mobile), WebGL (web)
  mjGRAPHICS_API_OPENGL,
  // Vulkan
  mjGRAPHICS_API_VULKAN,
} mjrGraphicsApi;

// Configuration parameters for the filament rendering context.
struct mjrFilamentConfig {
  // The native window handle into which we can render directly. If nullptr,
  // rendering will be done to an offscreen framebuffer.
  void* native_window;

  // The initial width and height of the offscreen framebuffer.
  int width;
  int height;

  // The backend graphics API to use.
  mjrGraphicsApi graphics_api;

  // Use software rendering even if the platform supports hardware rendering.
  mjtByte force_software_rendering;
};

// Initializes the mjrFilamentConfig to default values.
void mjrf_defaultFilamentConfig(mjrFilamentConfig* config);

// Creates a filament rendering context.
mjrfContext* mjrf_createContext(const mjrFilamentConfig* config);

// Destroys the filament rendering context.
void mjrf_destroyContext(mjrfContext* ctx);

// Describes the look/intention of the final rendered image.
typedef enum mjrDrawMode_ {
  // Render the scene with "normal" colors and lighting.
  mjDRAW_MODE_COLOR,
  // Render the scene as a grayscale depth map.
  mjDRAW_MODE_DEPTH,
  // Render each object with a unique, uniform (flat) color regardless of
  // lighting and texture.
  mjDRAW_MODE_SEGMENTATION,
} mjrDrawMode;

enum { mjNUM_DRAW_MODES = 3 };  // Number of modes in `mjrDrawMode`.

// Parameters describing the camera to use for rendering an image.
typedef mjvGLCamera mjrCamera;

// Describes a single rendering operation; used by `mjrf_render()`.
struct mjrRenderRequest {
  // The scene to render.
  mjrScene* scene;

  // The camera from which to render the scene.
  mjrCamera camera;

  // The viewport into which to render the image.
  mjrRect viewport;

  // The render target into which to render the image. If nullptr, the image
  // will be rendered to the window (as previously configured in
  // mjrFilamentConfig::native_window).
  mjrRenderTarget* target;

  // The method (e.g. Color, Depth, Segmentation, etc.) to use for rendering.
  mjrDrawMode draw_mode;

  // Whether or not to enable post processing; enabled by default.
  mjtByte enable_post_processing;

  // Whether or not to enable reflections; enabled by default.
  mjtByte enable_reflections;

  // Whether or not to enable shadows; enabled by default.
  mjtByte enable_shadows;
};

// Initializes the mjrRenderRequest to default values.
void mjr_defaultRenderRequest(mjrRenderRequest* request);

// Information needed to read pixels; used by `mjrf_render()`.
struct mjrReadPixelsRequest {
  // The render target from which to read the image pixels.
  mjrRenderTarget* target;

  // The buffer into which the read pixels will be written.
  void* output;

  // The number of bytes in the output buffer. This should match the size of
  // the render target texture.
  mjtSize num_bytes;

  // Callback when the read pixels operation is complete. This function can
  // optionally be used to free the output buffer if needed.
  void (*read_completed_callback)(void* user_data);

  // User data to pass to the completion callback.
  void* user_data;
};

// Initializes the mjrReadPixelsRequest to default values.
void mjr_defaultReadPixelsRequest(mjrReadPixelsRequest* request);

// Because rendering is asynchronous, each render request is assigned a
// unique Handle which can be used to query the status of the request. The
// Handle can also be used to block until the request is completed.
typedef std::uint64_t mjrFrameHandle;

// Submits the given requests for rendering. Because rendering may happen
// asynchronously, we have to submit both the render and read requests in the
// same call. This function is also when any callbacks will be triggered,
// though there is no guarantee on when exactly that will be done.
//
// Multiple requests and reads can be submitted in a single call. These
// requests will be processed in order, so some care must be taken. Firstly,
// requests should be grouped by target. Next, the combined area of the
// viewports for all requests for a given target must be contained within the
// dimensions of the target itself.
mjrFrameHandle mjrf_render(mjrfContext* ctx, const mjrRenderRequest* req,
                           int nreq, const mjrReadPixelsRequest* read_req,
                           int nread_req);

// Waits for all rendering operations to complete for the given frame handle,
// triggering any callbacks as needed.
void mjrf_waitForFrame(mjrfContext* ctx, mjrFrameHandle frame);

// Sets the clear color for the renderer.
void mjrf_setClearColor(mjrfContext* ctx, const float color[3]);

// Information about a single frame of rendering.
struct mjrFrameStats {
  // The frame rate of the renderer, in frames per second.
  double frame_rate;
};

// Initializes the mjrFrameStats to default values.
void mjr_defaultFrameStats(mjrFrameStats* stats);

// Returns the stats for the given frame but updating the given `stats_out`.
void mjrf_getFrameStats(mjrfContext* ctx, mjrFrameHandle frame,
                        mjrFrameStats* stats_out);

// ## Textures (mjrTexture)
//
// A texture is a 2D or 3D (cubemap) image that adds visual detail to a rendered
// model, such as color or bumpiness, without increasing geometric complexity.
//
// For textures intended to be used for image-based lights (see `mjrLight`
// below), you should use filament's `cmgen` tool to generate a KTX image from
// your source image. This tool will calculate additional data (i.e. the
// spherical harmonics) and encode that information into the KTX file.

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

// Type of texture.
typedef mjtTexture mjrSamplerType;

// Type of color space encoding.
typedef mjtColorSpace mjrColorSpace;

// Defines the basic properties of a texture.
struct mjrTextureConfig {
  // The width of the texture. For compressed textures (e.g. KTX), this is the
  // number of bytes in the compressed data.
  int width;

  // The height of the texture. For compressed textures (e.g. KTX), this should
  // be 0.
  int height;

  // How the texture will be interpreted by the renderer (e.g. 2D, cube, etc.).
  mjrSamplerType sampler_type;

  // The format of the pixels in the texture (e.g. RGB8, RGBA8, KTX, etc.)
  mjrPixelFormat format;

  // The color space of the texture (e.g. LINEAR, sRGB, etc.)
  mjrColorSpace color_space;
};

// Initializes the mjrTextureConfig to default values.
void mjr_defaultTextureConfig(mjrTextureConfig* config);

// Creates a texture with the given configuration. Note that the texture will
// not be created on the GPU until `mjrf_setTextureData()` is called.
mjrTexture* mjrf_createTexture(mjrfContext* ctx, const mjrTextureConfig* config);

// Destroys the texture.
void mjrf_destroyTexture(mjrTexture* texture);

// The binary data for a texture.
struct mjrTextureData {
  // Pointer to the data. If null, an empty texture will be created.
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

// Uploads the given texture data to the texture.
void mjrf_setTextureData(mjrTexture* texture, const mjrTextureData* data);

// Returns the width of the texture.
int mjrf_getTextureWidth(const mjrTexture* texture);

// Returns the height of the texture.
int mjrf_getTextureHeight(const mjrTexture* texture);

// Returns the target type of the texture.
mjrSamplerType mjrf_getSamplerType(const mjrTexture* texture);

// ## Meshes (mjrMesh)
//
// A mesh describes the surface geometry of an object to be rendered. It is
// defined as a collection of vertices (i.e. a VertexBuffer), a set of indices
// (i.e. an IndexBuffer) that describes the order in which the vertices should
// be processed, and a primitive type that defined how the vertices are to be
// interpreted (e.g. triangles, lines, etc.) when rendering the surface.
//
// Filament does not directly support normals. Instead, it encodes the normal,
// tangen, and bitangent into a 4-component quaternion describing the
// "orientation" of the vertex. Ideally, you should preprocess your assets
// to generate this data offline, but we will compute it on the fly if needed
// (at a performance cost).
//
// We also suggest precomputing the bounds of the mesh, otherwise we will also
// compute it on the fly.

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

// Creates a mesh with the given data.
mjrMesh* mjrf_createMesh(mjrfContext* ctx, const mjrMeshData* data);

// Destroys the mesh.
void mjrf_destroyMesh(mjrMesh* mesh);

// ## Scenes (mjrScene)
//
// A scene is a collection of entities (Lights and Renderables) that defines
// what is to be rendered. It also specifies the various effects that are to be
// applied to the rendering (e.g. shadows, reflections, post-processing, etc.)

// Configuration parameters for a Scene.
struct mjrSceneParams {
  // This mask, in conjunction with the layer mask in the Renderable, determines
  // which Renderables to render within the Scene.
  uint8_t layer_mask;

  // The layer mask to use for reflections.
  uint8_t reflection_layer_mask;
};

// Initializes the mjrSceneParams to default values.
void mjr_defaultSceneParams(mjrSceneParams* params);

// Creates a scene with the given parameters.
mjrScene* mjrf_createScene(mjrfContext* ctx, const mjrSceneParams* params);

// Destroys the scene.
void mjrf_destroyScene(mjrScene* scene);

// Adds a light to the scene.
void mjrf_addLightToScene(mjrScene* scene, mjrLight* light);

// Removes the light from the scene.
void mjrf_removeLightFromScene(mjrScene* scene, mjrLight* light);

// Adds a renderable to the scene.
void mjrf_addRenderableToScene(mjrScene* scene, mjrRenderable* renderable);

// Removes the renderable from the scene.
void mjrf_removeRenderableFromScene(mjrScene* scene, mjrRenderable* renderable);

// Sets the skybox (cube texture) for the scene.
void mjrf_setSceneSkybox(mjrScene* scene, const mjrTexture* texture);

// Configures the scene based on the parameters in an mjModel.
void mjrf_configureSceneFromModel(mjrScene* scene, const mjModel* model);

// ## Lights (mjrLight)
//
// A light is a source of illumination in the scene. (Without lights, a scene
// will be completely black.) There are several different types of lights such
// as directional, spot, point, and image lights.
//
// The primary light in a scene is the image light (also sometimes known as the
// environment light). This is a light that "surrounds" the entire scene and
// is defined as a 3D texture. Each "pixel" of the cubemap is interpreted as the
// color of projected into the scene from a particular direction.
//
// Directional lights are the next most common type of light and is usually
// used to simulate the sun; a uniformly colored light that is emitted in a
// single direction.
//
// Filament only supports a single image and directional light. You can define
// as many point or spot lights as you want. Each light source (except image
// based lights) may or may not cast shadows. Each shadow-casting light incurs a
// performance cost.

// The type of light (spot, directional, image, etc.).
typedef mjtLightType mjrLightType;

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

// Creates a light for the filament renderer.
mjrLight* mjrf_createLight(mjrfContext* ctx, const mjrLightParams* params);

// Destroys the light.
void mjrf_destroyLight(mjrLight* light);

// Enables or disables the light.
void mjrf_setLightEnabled(mjrLight* light, mjtByte enabled);

// Sets the intensity of the light, in candela.
void mjrf_setLightIntensity(mjrLight* light, float intensity);

// Sets the RGB color of the light.
void mjrf_setLightColor(mjrLight* light, const float color[3]);

// Sets the position and direction of the light.
void mjrf_setLightTransform(mjrLight* light, const float position[3],
                            const float direction[3]);

// Returns the type of the light.
mjrLightType mjrf_getLightType(const mjrLight* light);

// ## Renderables (mjrRenderable)
//
// A renderable is a single drawable object in the scene. It is defined as a
// combination of a mesh (i.e. surface geometry) and a material (i.e. surface
// appearance and properties).
//
// In terms of materials, there are three lighting models currently supported:
//
// 1. Metallic-roughness (PBR): this is the preferred model for rendering
//    models based standard metallic-roughness workflows.
// 2. Specular-glossiness (non-PBR): this is a legacy model designed to be
//    compatible with classic mjr renderer, though it is not 100% identical.
// 3. Unlit: this model ignores lighting and used for rendering UX or decorative
//    elements like contact forces and labels.
//
// Which lighting model is used is determined by the mjrMaterial properties.

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

  // If true, does not apply any lighting to the object. Assumes the object is
  // used for UX or decorative elements like contact forces and labels.
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

// Creates a renderable with the given parameters.
mjrRenderable* mjrf_createRenderable(mjrfContext* ctx,
                                     const mjrRenderableParams* params);

// Destroys the renderable.
void mjrf_destroyRenderable(mjrRenderable* renderable);

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

// Sets the transform (position, rotation, and size) of the renderable. Note
// that `size` is not the same as `scale`. For example, the z-size of a capsule
// only scales the tubular-portion of its geometry, but not the spherical caps.
void mjrf_setRenderableTransform(mjrRenderable* renderable,
                                 const float position[3],
                                 const float rotation[9], const float size[3]);

// Sets whether the renderable casts shadows or not.
void mjrf_setRenderableCastShadows(mjrRenderable* renderable,
                                   mjtByte cast_shadows);

// Sets whether the renderable receives shadows or not.
void mjrf_setRenderableReceiveShadows(mjrRenderable* renderable,
                                      mjtByte receive_shadows);

// Forces the renderable to be rendered using lines.
void mjrf_setRenderableWireframe(mjrRenderable* renderable, mjtByte wireframe);

// Sets the layer mask of the renderable. See mjrRenderableParams for details.
void mjrf_setRenderableLayerMask(mjrRenderable* renderable, uint8_t layer_mask);

// ## Render Targets (mjrRenderTarget)
//
// A render target is a memory buffer that holds the results of a rendering
// operation. (This is an alternative to rendering directly to the screen.)
// See mjrf_render for more details.

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

// Creates a render target for the filament renderer.
mjrRenderTarget* mjrf_createRenderTarget(mjrfContext* ctx,
                                         const mjrRenderTargetConfig* config);

// Destroys the render target.
void mjrf_destroyRenderTarget(mjrRenderTarget* render_target);

// ## Debug-only functions.

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
