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

#ifndef MUJOCO_MJRFILAMENT_H_
#define MUJOCO_MJRFILAMENT_H_

#include <stdint.h>

#include <mujoco/mjmodel.h>
#include <mujoco/mjrender.h>
#include <mujoco/mjtype.h>

#if defined(__cplusplus)
extern "C" {
#endif

// IMPORTANT: This API should still be considered experimental and is likely change frequently.

// This library provides a C API for the filament rendering library (github.com/google/filament)
// that is designed to work with the MuJoCo library for visualizing simulations.
//
// The filament renderer is a real-time physically based rendering (PBR) engine developed by Google.
// It is designed to be as small as possible and as efficient as possible, while still providing
// high-quality results. It works across all major platforms (Linux, Windows, macOS, Android, iOS,
// Web) and supports OpenGL, Vulkan, and Metal.
//
// For the purposes of this API, we assume the reader has a basic understanding of rendering
// concepts (e.g. textures, vertices, cameras, framebuffers, etc.). We will also highlight some of
// the key differences between this renderer and the legacy/classic MuJoCo (mjr) renderer.
//
// ## API Overview
//
// There are seven key components: Context, Texture, Mesh, Scene, Light, Renderable, and
// RenderTarget. We'll describe these in detail further below.
//
// Each object is created using a `create` function and destroyed using a `destroy` function, e.g.
// `mjrf_createTexture` and `mjrf_destroyTexture`. The `create` functions accept a pointer to a
// configuration struct (e.g. `mjrTextureConfig`) which describes the parameters for the object to
// be created. Each of these structs has a corresponding `default` function (e.g.
// `mjrf_defaultTextureConfig`) which can be used to initialize the struct to default values. Default
// values are assumed to be 0/NULL unless otherwise specified.
//
// For now, we'll just define opaque handles for each of our components.
typedef struct mjrfContext_ {} mjrfContext;
typedef struct mjrfTexture_ {} mjrfTexture;
typedef struct mjrfMesh_ {} mjrfMesh;
typedef struct mjrfScene_ {} mjrfScene;
typedef struct mjrfLight_ {} mjrfLight;
typedef struct mjrfRenderable_ {} mjrfRenderable;
typedef struct mjrfRenderTarget_ {} mjrfRenderTarget;

// ## Rendering Context (mjrfContext)
//
// The Context is the main entry point for the library. It manages all the core filament objects
// that are responsible for the rendering of an image.
//
// All other objects (e.g. Textures, Meshes, Scenes, etc.) need a Context in order to be created.
// Otherwise, the main function to use with the Context is `mjrf_render()` which does the actual
// rendering.
//
// Filament uses a separate thread for doing the actual rendering. However, despite that, this API
// is not thread-safe; calls are expected to be made from a single thread. Also, due to the
// asynchronous nature of filament, some APIs provide handles or callbacks to signal when an
// operation is complete. (Note: for WASM builds, filament does not use a separate thread.)
//
// There are two key differences between the mjrfContext and the classic mjrContext. Firstly, the
// filament context will manage the underlying graphics context itself. This means users do not need
// to initialize EGL or similar libraries beforehand. Secondly, the filament context is independent
// of a MuJoCo model. That means you can use a single mjrfContext to render images for multiple
// models.

// Callback function type for rendering operations.
typedef void (*mjrfCallback)(void* user_data);

typedef enum mjrGraphicsApi_ {  // underlying graphics API to use for rendering
  mjGRAPHICS_API_DEFAULT = 0,   // default (platform-dependent)
  mjGRAPHICS_API_OPENGL,        // desktop, mobile (GLES), web (WebGL)
  mjGRAPHICS_API_VULKAN,        // vulkan
} mjrGraphicsApi;

typedef struct mjrfContextConfig_ {  // parameters for creating filament context (mjrfContext)
  int graphics_api;                  // rendering graphics API [mjrGraphicsApi]
  mjtBool force_software_rendering;  // force backend to use software rendering
  void* native_window;               // platform-dependent window handle (or nullptr for windowless)
} mjrfContextConfig;

// Initializes the mjrfContextConfig to default values.
void mjrf_defaultContextConfig(mjrfContextConfig* config);

// Creates a filament rendering context.
mjrfContext* mjrf_createContext(const mjrfContextConfig* config);

// Destroys the filament rendering context.
void mjrf_destroyContext(mjrfContext* ctx);

typedef enum mjrDrawMode_ {           // how to draw objects in the scene
  mjDRAW_MODE_DEFAULT,                // default colors and lighting
  mjDRAW_MODE_DEFAULT_NO_TEXTURES,    // default, but without textures
  mjDRAW_MODE_WIREFRAME,              // wireframe rendering
  mjDRAW_MODE_DEPTH,                  // grayscale depth map
  mjDRAW_MODE_ISLANDS,                // color objects based on island and sleep state
  mjDRAW_MODE_SEGMENTATION_BY_ID,     // color objects based on segmentation id
  mjDRAW_MODE_SEGMENTATION_BY_COLOR,  // generate visually distinct colors using segmentation id
} mjrDrawMode;

typedef struct mjrfRenderRequest_ {  // a single rendering operation
  mjrfScene* scene;                  // scene to render
  mjrCamera camera;                  // camera (viewpoint) from which to render scene
  mjrRect viewport;                  // viewport (rect area) into which to render
  mjrfRenderTarget* target;          // target used for rendering (or nullptr for window rendering)
  int draw_mode;                     // method to use for drawing objects [mjrDrawMode]
  mjtBool enable_post_processing;    // enable post processing, enabled by default
  mjtBool enable_reflections;        // enable reflections, enabled by default
  mjtBool enable_shadows;            // enable shadows, enabled by default
} mjrfRenderRequest;

// Initializes the mjrfRenderRequest to default values.
void mjrf_defaultRenderRequest(mjrfRenderRequest* request);

typedef struct mjrfReadPixelsRequest_ {  // a single read operation
  mjrfRenderTarget* target;              // render target from which to read the image pixels
  void* output;                          // buffer into which the pixels will be stored
  mjtSize num_bytes;                     // size of output buffer
  mjrfCallback read_completed;           // callback when read is complete; can use to free output
  void* user_data;                       // user data for read_completed_callback
} mjrfReadPixelsRequest;

// Initializes the mjrfReadPixelsRequest to default values.
void mjrf_defaultReadPixelsRequest(mjrfReadPixelsRequest* request);

// Unique handle assigned to each render request; used to block until request is completed or query
// the status of the request.
typedef uint64_t mjrfFrameHandle;

// Submits the given requests for rendering. Because rendering happens asynchronously, callers have
// to submit both the render and read requests in the same call. Multiple requests and reads can be
// submitted in a single call. These requests will be processed in order, so some care must be
// taken. Firstly, requests should be grouped by target. Next, the combined area of the viewports
// for all requests for a given target must be contained within the dimensions of the target itself.
//
// Callbacks will be invoked from within this function, though there is no guarantee on when exactly
// that will be done.
mjrfFrameHandle mjrf_render(mjrfContext* ctx, const mjrfRenderRequest* req, int nreq,
                            const mjrfReadPixelsRequest* read_req, int nread_req);

// Waits for all rendering operations to complete for the given frame handle,
// triggering any callbacks as needed.
void mjrf_waitForFrame(mjrfContext* ctx, mjrfFrameHandle frame);

// Sets the clear color for the renderer.
void mjrf_setClearColor(mjrfContext* ctx, const float color[3]);

typedef struct mjrfFrameStats_ {  // stats for a single frame of rendering
  double frame_rate;              // frame rate, in frames per second
} mjrfFrameStats;

// Initializes the mjrFrameStats to default values.
void mjrf_defaultFrameStats(mjrfFrameStats* stats);

// Returns the stats for the given frame but updating the given `stats_out`.
void mjrf_getFrameStats(mjrfContext* ctx, mjrfFrameHandle frame, mjrfFrameStats* stats_out);

// ## Textures (mjrfTexture)
//
// A texture is a 2D or 3D (cubemap) image that adds visual detail to a rendered model, such as
// color or bumpiness, without increasing geometric complexity.
//
// For textures intended to be used for image-based lights (see `mjrfLight` below), you should use
// filament's `cmgen` tool to generate a KTX image from your source image. This tool will calculate
// additional data (i.e. the spherical harmonics) and encode that information into the KTX file.

typedef struct mjrfTextureConfig_ {  // parameters for creating a texture (mjrfTexture)
  int width;                         // texture width, or number of bytes for compressed data (e.g. KTX)
  int height;                        // texture height, or 0 for compressed data (e.g. KTX)
  int format;                        // pixel format (e.g. RGB8, RGBA8, KTX, etc.) [mjrPixelFormat]
  int color_space;                   // color space (e.g. LINEAR, sRGB, etc.) [mjrColorSpace]
  int sampler_type;                  // texture sampler (e.g. 2D, cube, etc.) [mjrSamplerType]
} mjrfTextureConfig;

// Initializes the mjrfTextureConfig to default values.
void mjrf_defaultTextureConfig(mjrfTextureConfig* config);

// Creates a filament texture. Note that the texture will not be created on the
// GPU until `mjrf_setTextureData()` is called.
mjrfTexture* mjrf_createTexture(mjrfContext* ctx,
                                const mjrfTextureConfig* config);

// Destroys the texture.
void mjrf_destroyTexture(mjrfTexture* texture);

typedef struct mjrfTextureData_ {  // binary data for a texture (mjrfTexture)
  const void* bytes;               // pointer to image data, or nullptr for empty texture
  mjtSize num_bytes;               // number of bytes in the image data
  mjrfCallback release;            // callback when data has finished uploading
  void* user_data;                 // user data for release callback
} mjrfTextureData;

// Initializes the mjrfTextureData to default values.
void mjrf_defaultTextureData(mjrfTextureData* data);

// Uploads the given texture data to the texture.
void mjrf_setTextureData(mjrfTexture* texture, const mjrfTextureData* data);

// Returns the width of the texture.
int mjrf_getTextureWidth(const mjrfTexture* texture);

// Returns the height of the texture.
int mjrf_getTextureHeight(const mjrfTexture* texture);

// Returns the sampler type (mjrSamplerType) used by the texture.
// [returns: mjrSamplerType]
int mjrf_getTextureSamplerType(const mjrfTexture* texture);

// ## Meshes (mjrfMesh)
//
// A mesh describes the surface geometry of an object to be rendered. It is defined as a collection
// of vertices (i.e. a VertexBuffer), a set of indices (i.e. an IndexBuffer) that describes the
// order in which the vertices should be processed, and a primitive type that defined how the
// vertices are to be interpreted (e.g. triangles, lines, etc.) when rendering the surface.
//
// Filament does not directly support normals. Instead, it encodes the normal, tangent, and
// bitangent into a 4-component quaternion describing the "orientation" of the vertex. Ideally, you
// should preprocess your assets to generate this data offline, but we will compute it on the fly if
// needed (at a performance cost).
//
// Vertex data may or may not be interleaved. Interleaved data assumes that the attributes are
// packed in the order specified in the attributes array, with no padding in-between. Additionally,
// the `data` pointer for each attribute is assumed to point to the first element of that type. For
// non-interleaved data, each attribute is assumed to be stored in a separate array.
//
// Additionally, the bounds of the mesh should be computed in order to allow the filament renderer
// to perform frustum-based culling. Alternatively, the bounds can be computed at runtime (though
// there is a small performance cost). If no bounds are provided (or calculated), then frustum
// culling will not be performed.

// Maximum number of vertex attributes in a mesh.
enum { mjMAX_VERTEX_ATTRIBUTES = 16 };

typedef struct mjrfMeshData_ {  // binary data for a mesh (mjrfMesh)
  mjtSize num_vertices;         // number of vertices; all vertex attributes share this size
  int num_attributes;           // number of attributes defined
  mjrVertexAttribute attributes[mjMAX_VERTEX_ATTRIBUTES];  // per-vertex attribute information
  mjtBool interleaved;          // true if vertex attributes are interleaved
  mjtSize num_indices;          // number of indices
  const void* indices;          // indices data array
  int index_type;               // index data format (e.g. UINT16 or UINT32) [mjrIndexType]
  int primitive_type;           // index interpretation (e.g. TRIANGLES, etc.) [mjrMeshPrimitiveType]
  mjtBool compute_bounds;       // if true, compute bounds from vertex positions
  float bounds_min[3];          // min/max bounds; assume unset if bounds_min == bounds_max
  float bounds_max[3];
  mjrfCallback release;         // callback when data has finished uploading
  void* user_data;              // user data for release callback
} mjrfMeshData;

// Initializes the mjrfMeshData to default values.
void mjrf_defaultMeshData(mjrfMeshData* data);

// Creates a mesh with the given data.
mjrfMesh* mjrf_createMesh(mjrfContext* ctx, const mjrfMeshData* data);

// Destroys the mesh.
void mjrf_destroyMesh(mjrfMesh* mesh);

// ## Scenes (mjrfScene)
//
// A scene is a collection of entities (Lights and Renderables) that describes what is to be
// rendered.

typedef struct mjrfSceneParams_ {  // parameters for creating a scene (mjrfScene)
} mjrfSceneParams;

// Initializes the mjrfSceneParams to default values.
void mjrf_defaultSceneParams(mjrfSceneParams* params);

// Creates a scene with the given parameters.
mjrfScene* mjrf_createScene(mjrfContext* ctx, const mjrfSceneParams* params);

// Destroys the scene.
void mjrf_destroyScene(mjrfScene* scene);

// Adds a light to the scene.
void mjrf_addLightToScene(mjrfScene* scene, mjrfLight* light);

// Removes the light from the scene.
void mjrf_removeLightFromScene(mjrfScene* scene, mjrfLight* light);

// Adds a renderable to the scene.
void mjrf_addRenderableToScene(mjrfScene* scene, mjrfRenderable* renderable);

// Removes the renderable from the scene.
void mjrf_removeRenderableFromScene(mjrfScene* scene, mjrfRenderable* renderable);

// Sets the skybox (cube texture) for the scene.
void mjrf_setSceneSkybox(mjrfScene* scene, const mjrfTexture* texture);

// Configures the scene based on the parameters in an mjModel.
void mjrf_configureSceneFromModel(mjrfScene* scene, const mjModel* model);

// ## Lights (mjrfLight)
//
// A light is a source of illumination in the scene. (Without lights, a scene will be completely
// black.) There are several different types of lights such as directional, spot, point, and image
// lights.
//
// The primary light in a scene is the image light (also sometimes known as the environment light).
// This is a light that "surrounds" the entire scene and is defined as a 3D texture. Each "pixel" of
// the cubemap is interpreted as the color of projected into the scene from a particular direction.
//
// Directional lights are the next most common type of light and is usually used to simulate the
// sun; a uniformly colored light that is emitted in a single direction.
//
// Filament only supports a single image and directional light. You can define as many point or spot
// lights as you want. Each light source (except image based lights) may or may not cast shadows.
// Each shadow-casting light incurs a performance cost.

typedef struct mjrfLightParams_ {  // parameters for creating a light (mjrfLight)
  int type;                        // type of light (e.g. spot, point, image, etc.) [mjrLightType]
  const mjrfTexture* texture;      // texture; only for image lights
  float color[3];                  // RGB color
  float intensity;                 // light intensity, in candela
  mjtBool cast_shadows;            // if true, cast shadows
  float range;                     // effective range of light, in meters
  float spot_cone_angle;           // spot light cone angle, in degrees
  int shadow_map_size;             // size of shadow map texture, 0 to use default size
  float bulb_radius;               // bulb radius, used for soft shadows
  float vsm_blur_width;            // variance shadow map blur width
} mjrfLightParams;

// Initializes the mjrfLightParams to default values.
void mjrf_defaultLightParams(mjrfLightParams* params);

// Creates a light for the filament renderer.
mjrfLight* mjrf_createLight(mjrfContext* ctx, const mjrfLightParams* params);

// Destroys the light.
void mjrf_destroyLight(mjrfLight* light);

// Enables or disables the light.
void mjrf_setLightEnabled(mjrfLight* light, mjtBool enabled);

// Sets the intensity of the light, in candela.
void mjrf_setLightIntensity(mjrfLight* light, float intensity);

// Sets the RGB color of the light.
void mjrf_setLightColor(mjrfLight* light, const float color[3]);

// Sets the position and direction of the light.
void mjrf_setLightTransform(mjrfLight* light, const float position[3], const float direction[3]);

// Returns the type of the light (mjrLightType).
int mjrf_getLightType(const mjrfLight* light);

// ## Renderables (mjrfRenderable)
//
// A renderable is a single drawable object in the scene. It is defined as a combination of a mesh
// (i.e. surface geometry) and a material (i.e. surface appearance and properties).
//
// In terms of materials, there are three lighting models currently supported:
//
// 1. Metallic-roughness (PBR): this is the preferred model for rendering models based standard
//    metallic-roughness workflows.
// 2. Specular-glossiness (non-PBR): this is a legacy model designed to be compatible with classic
//    mjr renderer, though it is not 100% identical.
// 3. Unlit: this model ignores lighting and used for rendering UX or decorative elements like
//    contact forces and labels.
//
// Which lighting model is used is determined by the mjrfMaterial properties.

typedef struct mjrfMaterial_ {  // material properties for a renderable (mjrfMaterial)
  float color[4];               // object color; defaults to white
  int32_t segmentation_id;      // ID for segmentation rendering; maps to RGB8 color (i.e. 24 bits)
  int32_t island_id;            // ID to which the renderable belongs
  int sleep_state;              // sleep state of the renderable [mjtSleepState]
  float uv_scale[3];            // scale applied to UV coordinates; defaults to (1,1,1)
  float uv_offset[3];           // offset applied to UV coordinates; defaults to (0,0,0)
  float scissor[4];             // if non-zero, applies scissor testing when rendering
  float metallic;               // metallic factory [0, 1]; disabled if < 0
  float roughness;              // roughness factor [0, 1]; disabled if < 0
  float specular;               // specular factor [0, 1]; disabled if < 0
  float glossiness;             // glossiness factor [0, 1]; disabled if < 0
  float emissive;               // emissive/glow factor [0, 1]; disabled if < 0
  float reflectance;            // blend factor for reflective surfaces [0, 1]; applies only to planes
  mjtBool decor_ux;             // for ux elements, does not apply any lighting
  mjtBool selected;             // for "selected" ux elements, adds additional styling
  const mjrfTexture* color_texture;       // color/albedo texture (RGB8)
  const mjrfTexture* opacity_texture;     // opacity texture (A8)
  const mjrfTexture* normal_texture;      // normal map texture (RGB8)
  const mjrfTexture* metallic_texture;    // metallic map texture (R8)
  const mjrfTexture* roughness_texture;   // roughness map texture (R8)
  const mjrfTexture* occlusion_texture;   // ambient occlusion texture (R8)
  const mjrfTexture* orm_texture;         // occlusion/roughness/metallic texture (RGB8)
  const mjrfTexture* emissive_texture;    // emissive texture (RGB8)
  const mjrfTexture* reflection_texture;  // reflection texture, for internal use only
} mjrfMaterial;

// Initializes the mjrfMaterial to default values.
void mjrf_defaultMaterial(mjrfMaterial* material);

typedef struct mjrfRenderableParams_ {  // parameters for creating a renderable (mjrfRenderable)
  mjtBool cast_shadows;                 // if true, casts shadows
  mjtBool receive_shadows;              // if true, receives shadows
  uint16_t blend_order;                 // controls draw order for transparent objects [0, 8]
} mjrfRenderableParams;

// Initializes the mjrfRenderableParams to default values.
void mjrf_defaultRenderableParams(mjrfRenderableParams* params);

// Creates a renderable with the given parameters.
mjrfRenderable* mjrf_createRenderable(mjrfContext* ctx, const mjrfRenderableParams* params);

// Destroys the renderable.
void mjrf_destroyRenderable(mjrfRenderable* renderable);

// Sets the mesh of the renderable.
void mjrf_setRenderableMesh(mjrfRenderable* renderable, const mjrfMesh* mesh, int elem_offset,
                            int elem_count);

// Sets the mesh of the renderable to a built-in mesh based on the geom type. Note: using the same
// parameters (nstack, nslice, nquad) will have better performance as the internal mesh data can be
// shared across renderables.
// [type: mjtGeom]
void mjrf_setRenderableGeomMesh(mjrfRenderable* renderable, int type, int nstack, int nslice,
                                int nquad);

// Sets the material properties and textures of the renderable.
void mjrf_setRenderableMaterial(mjrfRenderable* renderable, const mjrfMaterial* material);

// Copies the material properties of the renderable into the given mjrfMaterial.
void mjrf_getRenderableMaterial(mjrfRenderable* renderable, mjrfMaterial* material);

// Sets the transform position and rotation of the renderable.
void mjrf_setRenderableTransform(mjrfRenderable* renderable, const float position[3],
                                 const float rotation[9]);

// Sets the size of the renderable. Note that, for most renderables, this is equivalent to setting
// the scale. However, for some geom-based renderables, the size scale is not applied uniformly
// (e.g. the spherical ends of a capsule are scaled such that they always remain spherical).
void mjrf_setRenderableSize(mjrfRenderable* renderable, const float size[3]);

// ## Render Targets (mjrfRenderTarget)
//
// A render target is a memory buffer that holds the results of a rendering operation. (This is an
// alternative to rendering directly to the screen.) See mjrf_render for more details.

typedef struct mjrfRenderTargetConfig_ {  // parameters for creating a render target (mjrfRenderTarget)
  int width;                              // texture width
  int height;                             // texture height
  int color_format;                       // pixel format for color buffer [mjrPixelFormat]
  int depth_format;                       // pixel format for depth buffer [mjrPixelFormat]
} mjrfRenderTargetConfig;

// Initializes the RenderTargetConfig to default values.
void mjrf_defaultRenderTargetConfig(mjrfRenderTargetConfig* config);

// Creates a render target for the filament renderer.
mjrfRenderTarget* mjrf_createRenderTarget(mjrfContext* ctx, const mjrfRenderTargetConfig* config);

// Destroys the render target.
void mjrf_destroyRenderTarget(mjrfRenderTarget* render_target);

// Resizes the render target to the given width and height.
void mjrf_resizeRenderTarget(mjrfRenderTarget* render_target, int width, int height);

// ## Debug-only functions.

// Draws an ImGui editor for the given scene, exposing filament-specific settings.
void mjrf_DEBUG_drawImguiEditor(mjrfScene* scene);

#if defined(__cplusplus)
}  // extern "C"
#endif

#endif  // MUJOCO_MJRFILAMENT_H_
