// Copyright 2026 DeepMind Technologies Limited
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

// AUTO-GENERATED. DO NOT EDIT.

py::enum_<mjrLightType> mjr_light_type(m, "LightType", py::module_local());
mjr_light_type.value("LIGHT_SPOT", mjLIGHT_SPOT);
mjr_light_type.value("LIGHT_DIRECTIONAL", mjLIGHT_DIRECTIONAL);
mjr_light_type.value("LIGHT_POINT", mjLIGHT_POINT);
mjr_light_type.value("LIGHT_IMAGE", mjLIGHT_IMAGE);

py::enum_<mjrSamplerType> mjr_sampler_type(m, "SamplerType", py::module_local());
mjr_sampler_type.value("TEXTURE_2D", mjTEXTURE_2D);
mjr_sampler_type.value("TEXTURE_CUBE", mjTEXTURE_CUBE);
mjr_sampler_type.value("TEXTURE_SKYBOX", mjTEXTURE_SKYBOX);

py::enum_<mjrColorSpace> mjr_color_space(m, "ColorSpace", py::module_local());
mjr_color_space.value("COLORSPACE_AUTO", mjCOLORSPACE_AUTO);
mjr_color_space.value("COLORSPACE_LINEAR", mjCOLORSPACE_LINEAR);
mjr_color_space.value("COLORSPACE_SRGB", mjCOLORSPACE_SRGB);

py::enum_<mjrPixelFormat> mjr_pixel_format(m, "PixelFormat", py::module_local());
mjr_pixel_format.value("PIXEL_FORMAT_UNKNOWN", mjPIXEL_FORMAT_UNKNOWN);
mjr_pixel_format.value("PIXEL_FORMAT_R8", mjPIXEL_FORMAT_R8);
mjr_pixel_format.value("PIXEL_FORMAT_RGB8", mjPIXEL_FORMAT_RGB8);
mjr_pixel_format.value("PIXEL_FORMAT_RGBA8", mjPIXEL_FORMAT_RGBA8);
mjr_pixel_format.value("PIXEL_FORMAT_R32F", mjPIXEL_FORMAT_R32F);
mjr_pixel_format.value("PIXEL_FORMAT_DEPTH32F", mjPIXEL_FORMAT_DEPTH32F);
mjr_pixel_format.value("PIXEL_FORMAT_KTX", mjPIXEL_FORMAT_KTX);

py::enum_<mjrVertexAttributeUsage> mjr_vertex_attribute_usage(m, "VertexAttributeUsage", py::module_local());
mjr_vertex_attribute_usage.value("VERTEX_ATTRIBUTE_USAGE_POSITION", mjVERTEX_ATTRIBUTE_USAGE_POSITION);
mjr_vertex_attribute_usage.value("VERTEX_ATTRIBUTE_USAGE_NORMAL", mjVERTEX_ATTRIBUTE_USAGE_NORMAL);
mjr_vertex_attribute_usage.value("VERTEX_ATTRIBUTE_USAGE_TANGENTS", mjVERTEX_ATTRIBUTE_USAGE_TANGENTS);
mjr_vertex_attribute_usage.value("VERTEX_ATTRIBUTE_USAGE_UV", mjVERTEX_ATTRIBUTE_USAGE_UV);
mjr_vertex_attribute_usage.value("VERTEX_ATTRIBUTE_USAGE_COLOR", mjVERTEX_ATTRIBUTE_USAGE_COLOR);

py::enum_<mjrVertexAttributeType> mjr_vertex_attribute_type(m, "VertexAttributeType", py::module_local());
mjr_vertex_attribute_type.value("VERTEX_ATTRIBUTE_TYPE_FLOAT2", mjVERTEX_ATTRIBUTE_TYPE_FLOAT2);
mjr_vertex_attribute_type.value("VERTEX_ATTRIBUTE_TYPE_FLOAT3", mjVERTEX_ATTRIBUTE_TYPE_FLOAT3);
mjr_vertex_attribute_type.value("VERTEX_ATTRIBUTE_TYPE_FLOAT4", mjVERTEX_ATTRIBUTE_TYPE_FLOAT4);
mjr_vertex_attribute_type.value("VERTEX_ATTRIBUTE_TYPE_UBYTE4", mjVERTEX_ATTRIBUTE_TYPE_UBYTE4);

py::enum_<mjrIndexType> mjr_index_type(m, "IndexType", py::module_local());
mjr_index_type.value("INDEX_TYPE_U16", mjINDEX_TYPE_U16);
mjr_index_type.value("INDEX_TYPE_U32", mjINDEX_TYPE_U32);

py::enum_<mjrMeshPrimitiveType> mjr_mesh_primitive_type(m, "MeshPrimitiveType", py::module_local());
mjr_mesh_primitive_type.value("MESH_PRIMITIVE_TYPE_TRIANGLES", mjMESH_PRIMITIVE_TYPE_TRIANGLES);
mjr_mesh_primitive_type.value("MESH_PRIMITIVE_TYPE_LINES", mjMESH_PRIMITIVE_TYPE_LINES);

py::enum_<mjrGraphicsApi> mjr_graphics_api(m, "GraphicsApi", py::module_local());
mjr_graphics_api.value("GRAPHICS_API_DEFAULT", mjGRAPHICS_API_DEFAULT);
mjr_graphics_api.value("GRAPHICS_API_OPENGL", mjGRAPHICS_API_OPENGL);
mjr_graphics_api.value("GRAPHICS_API_VULKAN", mjGRAPHICS_API_VULKAN);

py::enum_<mjrDrawMode> mjr_draw_mode(m, "DrawMode", py::module_local());
mjr_draw_mode.value("DRAW_MODE_DEFAULT", mjDRAW_MODE_DEFAULT);
mjr_draw_mode.value("DRAW_MODE_DEFAULT_NO_TEXTURES", mjDRAW_MODE_DEFAULT_NO_TEXTURES);
mjr_draw_mode.value("DRAW_MODE_WIREFRAME", mjDRAW_MODE_WIREFRAME);
mjr_draw_mode.value("DRAW_MODE_DEPTH", mjDRAW_MODE_DEPTH);
mjr_draw_mode.value("DRAW_MODE_ISLANDS", mjDRAW_MODE_ISLANDS);
mjr_draw_mode.value("DRAW_MODE_SEGMENTATION_BY_ID", mjDRAW_MODE_SEGMENTATION_BY_ID);
mjr_draw_mode.value("DRAW_MODE_SEGMENTATION_BY_COLOR", mjDRAW_MODE_SEGMENTATION_BY_COLOR);

py::class_<mjrCamera> mjr_camera(m, "Camera", py::module_local());
mjr_camera.def(py::init([](
  std::optional<Float3::Ref> pos,
  std::optional<Float3::Ref> forward,
  std::optional<Float3::Ref> up,
  std::optional<float> frustum_center,
  std::optional<float> frustum_width,
  std::optional<float> frustum_bottom,
  std::optional<float> frustum_top,
  std::optional<float> frustum_near,
  std::optional<float> frustum_far,
  std::optional<bool> orthographic
) {
  mjrCamera obj;
  Init(obj);
  if (pos) Setter(obj.pos, *pos);
  if (forward) Setter(obj.forward, *forward);
  if (up) Setter(obj.up, *up);
  if (frustum_center) Setter(obj.frustum_center, *frustum_center);
  if (frustum_width) Setter(obj.frustum_width, *frustum_width);
  if (frustum_bottom) Setter(obj.frustum_bottom, *frustum_bottom);
  if (frustum_top) Setter(obj.frustum_top, *frustum_top);
  if (frustum_near) Setter(obj.frustum_near, *frustum_near);
  if (frustum_far) Setter(obj.frustum_far, *frustum_far);
  if (orthographic) Setter(obj.orthographic, *orthographic);
  return obj;
})
, py::arg("pos").none(true) = py::none()
, py::arg("forward").none(true) = py::none()
, py::arg("up").none(true) = py::none()
, py::arg("frustum_center").none(true) = py::none()
, py::arg("frustum_width").none(true) = py::none()
, py::arg("frustum_bottom").none(true) = py::none()
, py::arg("frustum_top").none(true) = py::none()
, py::arg("frustum_near").none(true) = py::none()
, py::arg("frustum_far").none(true) = py::none()
, py::arg("orthographic").none(true) = py::none()
);
DefProperty<Float3>(mjr_camera, "pos", &mjrCamera::pos);
DefProperty<Float3>(mjr_camera, "forward", &mjrCamera::forward);
DefProperty<Float3>(mjr_camera, "up", &mjrCamera::up);
DefProperty<float>(mjr_camera, "frustum_center", &mjrCamera::frustum_center);
DefProperty<float>(mjr_camera, "frustum_width", &mjrCamera::frustum_width);
DefProperty<float>(mjr_camera, "frustum_bottom", &mjrCamera::frustum_bottom);
DefProperty<float>(mjr_camera, "frustum_top", &mjrCamera::frustum_top);
DefProperty<float>(mjr_camera, "frustum_near", &mjrCamera::frustum_near);
DefProperty<float>(mjr_camera, "frustum_far", &mjrCamera::frustum_far);
DefProperty<bool>(mjr_camera, "orthographic", &mjrCamera::orthographic);

py::class_<mjrRect> mjr_rect(m, "Rect", py::module_local());
mjr_rect.def(py::init([](
  std::optional<int> left,
  std::optional<int> bottom,
  std::optional<int> width,
  std::optional<int> height
) {
  mjrRect obj;
  Init(obj);
  if (left) Setter(obj.left, *left);
  if (bottom) Setter(obj.bottom, *bottom);
  if (width) Setter(obj.width, *width);
  if (height) Setter(obj.height, *height);
  return obj;
})
, py::arg("left").none(true) = py::none()
, py::arg("bottom").none(true) = py::none()
, py::arg("width").none(true) = py::none()
, py::arg("height").none(true) = py::none()
);
DefProperty<int>(mjr_rect, "left", &mjrRect::left);
DefProperty<int>(mjr_rect, "bottom", &mjrRect::bottom);
DefProperty<int>(mjr_rect, "width", &mjrRect::width);
DefProperty<int>(mjr_rect, "height", &mjrRect::height);

py::class_<mjrVertexAttribute> mjr_vertex_attribute(m, "VertexAttribute", py::module_local());
mjr_vertex_attribute.def(py::init([](
  std::optional<mjrVertexAttributeUsage> usage,
  std::optional<mjrVertexAttributeType> type
) {
  mjrVertexAttribute obj;
  Init(obj);
  if (usage) Setter(obj.usage, *usage);
  if (type) Setter(obj.type, *type);
  return obj;
})
, py::arg("usage").none(true) = py::none()
, py::arg("type").none(true) = py::none()
);
DefProperty<mjrVertexAttributeUsage>(mjr_vertex_attribute, "usage", &mjrVertexAttribute::usage);
DefProperty<mjrVertexAttributeType>(mjr_vertex_attribute, "type", &mjrVertexAttribute::type);

py::class_<mjrfContextConfig> mjrf_context_config(m, "ContextConfig", py::module_local());
mjrf_context_config.def(py::init([](
  std::optional<mjrGraphicsApi> graphics_api,
  std::optional<bool> force_software_rendering,
  std::optional<uint64_t> native_window
) {
  mjrfContextConfig obj;
  Init(obj);
  if (graphics_api) Setter(obj.graphics_api, *graphics_api);
  if (force_software_rendering) Setter(obj.force_software_rendering, *force_software_rendering);
  if (native_window) Setter(obj.native_window, *native_window);
  return obj;
})
, py::arg("graphics_api").none(true) = py::none()
, py::arg("force_software_rendering").none(true) = py::none()
, py::arg("native_window").none(true) = py::none()
);
DefProperty<mjrGraphicsApi>(mjrf_context_config, "graphics_api", &mjrfContextConfig::graphics_api);
DefProperty<bool>(mjrf_context_config, "force_software_rendering", &mjrfContextConfig::force_software_rendering);
DefProperty<uint64_t>(mjrf_context_config, "native_window", &mjrfContextConfig::native_window);

py::class_<mjrfRenderRequest> mjrf_render_request(m, "RenderRequest", py::module_local());
mjrf_render_request.def(py::init([](
  PyScene* scene,
  std::optional<mjrCamera> camera,
  std::optional<mjrRect> viewport,
  PyRenderTarget* target,
  std::optional<mjrDrawMode> draw_mode,
  std::optional<bool> enable_post_processing,
  std::optional<bool> enable_reflections,
  std::optional<bool> enable_shadows
) {
  mjrfRenderRequest obj;
  Init(obj);
  if (scene) Setter(obj.scene, *scene);
  if (camera) Setter(obj.camera, *camera);
  if (viewport) Setter(obj.viewport, *viewport);
  if (target) Setter(obj.target, *target);
  if (draw_mode) Setter(obj.draw_mode, *draw_mode);
  if (enable_post_processing) Setter(obj.enable_post_processing, *enable_post_processing);
  if (enable_reflections) Setter(obj.enable_reflections, *enable_reflections);
  if (enable_shadows) Setter(obj.enable_shadows, *enable_shadows);
  return obj;
})
, py::arg("scene").none(true) = py::none()
, py::arg("camera").none(true) = py::none()
, py::arg("viewport").none(true) = py::none()
, py::arg("target").none(true) = py::none()
, py::arg("draw_mode").none(true) = py::none()
, py::arg("enable_post_processing").none(true) = py::none()
, py::arg("enable_reflections").none(true) = py::none()
, py::arg("enable_shadows").none(true) = py::none()
);
DefProperty<PyScene>(mjrf_render_request, "scene", &mjrfRenderRequest::scene);
DefProperty<mjrCamera>(mjrf_render_request, "camera", &mjrfRenderRequest::camera);
DefProperty<mjrRect>(mjrf_render_request, "viewport", &mjrfRenderRequest::viewport);
DefProperty<PyRenderTarget>(mjrf_render_request, "target", &mjrfRenderRequest::target);
DefProperty<mjrDrawMode>(mjrf_render_request, "draw_mode", &mjrfRenderRequest::draw_mode);
DefProperty<bool>(mjrf_render_request, "enable_post_processing", &mjrfRenderRequest::enable_post_processing);
DefProperty<bool>(mjrf_render_request, "enable_reflections", &mjrfRenderRequest::enable_reflections);
DefProperty<bool>(mjrf_render_request, "enable_shadows", &mjrfRenderRequest::enable_shadows);

py::class_<mjrfReadPixelsRequest> mjrf_read_pixels_request(m, "ReadPixelsRequest", py::module_local());
mjrf_read_pixels_request.def(py::init([](
  PyRenderTarget* target
) {
  mjrfReadPixelsRequest obj;
  Init(obj);
  if (target) Setter(obj.target, *target);
  return obj;
})
, py::arg("target").none(true) = py::none()
);
mjrf_read_pixels_request.def("__del__", [](mjrfReadPixelsRequest& obj) { Free(obj); });
mjrf_read_pixels_request.def("free", [](mjrfReadPixelsRequest& obj) { Free(obj); });
DefProperty<PyRenderTarget>(mjrf_read_pixels_request, "target", &mjrfReadPixelsRequest::target);

py::class_<mjrfFrameStats> mjrf_frame_stats(m, "FrameStats", py::module_local());
mjrf_frame_stats.def(py::init([](
  std::optional<double> frame_rate
) {
  mjrfFrameStats obj;
  Init(obj);
  if (frame_rate) Setter(obj.frame_rate, *frame_rate);
  return obj;
})
, py::arg("frame_rate").none(true) = py::none()
);
DefProperty<double>(mjrf_frame_stats, "frame_rate", &mjrfFrameStats::frame_rate);

py::class_<mjrfTextureConfig> mjrf_texture_config(m, "TextureConfig", py::module_local());
mjrf_texture_config.def(py::init([](
  std::optional<int> width,
  std::optional<int> height,
  std::optional<mjrPixelFormat> format,
  std::optional<mjrColorSpace> color_space,
  std::optional<mjrSamplerType> sampler_type
) {
  mjrfTextureConfig obj;
  Init(obj);
  if (width) Setter(obj.width, *width);
  if (height) Setter(obj.height, *height);
  if (format) Setter(obj.format, *format);
  if (color_space) Setter(obj.color_space, *color_space);
  if (sampler_type) Setter(obj.sampler_type, *sampler_type);
  return obj;
})
, py::arg("width").none(true) = py::none()
, py::arg("height").none(true) = py::none()
, py::arg("format").none(true) = py::none()
, py::arg("color_space").none(true) = py::none()
, py::arg("sampler_type").none(true) = py::none()
);
DefProperty<int>(mjrf_texture_config, "width", &mjrfTextureConfig::width);
DefProperty<int>(mjrf_texture_config, "height", &mjrfTextureConfig::height);
DefProperty<mjrPixelFormat>(mjrf_texture_config, "format", &mjrfTextureConfig::format);
DefProperty<mjrColorSpace>(mjrf_texture_config, "color_space", &mjrfTextureConfig::color_space);
DefProperty<mjrSamplerType>(mjrf_texture_config, "sampler_type", &mjrfTextureConfig::sampler_type);

py::class_<mjrfTextureData> mjrf_texture_data(m, "TextureData", py::module_local());
mjrf_texture_data.def(py::init([](

) {
  mjrfTextureData obj;
  Init(obj);

  return obj;
})
);
mjrf_texture_data.def("__del__", [](mjrfTextureData& obj) { Free(obj); });
mjrf_texture_data.def("free", [](mjrfTextureData& obj) { Free(obj); });

py::class_<mjrfMeshConfig> mjrf_mesh_config(m, "MeshConfig", py::module_local());
mjrf_mesh_config.def(py::init([](
  std::optional<int> max_vertices,
  std::optional<int> max_indices,
  std::optional<std::vector<mjrVertexAttribute>> attributes,
  std::optional<bool> interleaved,
  std::optional<mjrIndexType> index_type,
  std::optional<mjrMeshPrimitiveType> primitive_type
) {
  mjrfMeshConfig obj;
  Init(obj);
  if (max_vertices) Setter(obj.max_vertices, *max_vertices);
  if (max_indices) Setter(obj.max_indices, *max_indices);
  if (attributes) { Setter(obj.attributes, *attributes); obj.num_attributes = attributes->size(); }
  if (interleaved) Setter(obj.interleaved, *interleaved);
  if (index_type) Setter(obj.index_type, *index_type);
  if (primitive_type) Setter(obj.primitive_type, *primitive_type);
  return obj;
})
, py::arg("max_vertices").none(true) = py::none()
, py::arg("max_indices").none(true) = py::none()
, py::arg("attributes").none(true) = py::none()
, py::arg("interleaved").none(true) = py::none()
, py::arg("index_type").none(true) = py::none()
, py::arg("primitive_type").none(true) = py::none()
);
DefProperty<int>(mjrf_mesh_config, "max_vertices", &mjrfMeshConfig::max_vertices);
DefProperty<int>(mjrf_mesh_config, "max_indices", &mjrfMeshConfig::max_indices);
DefProperty<bool>(mjrf_mesh_config, "interleaved", &mjrfMeshConfig::interleaved);
DefProperty<mjrIndexType>(mjrf_mesh_config, "index_type", &mjrfMeshConfig::index_type);
DefProperty<mjrMeshPrimitiveType>(mjrf_mesh_config, "primitive_type", &mjrfMeshConfig::primitive_type);

py::class_<mjrfMeshData> mjrf_mesh_data(m, "MeshData", py::module_local());
mjrf_mesh_data.def(py::init([](
  std::optional<bool> compute_bounds,
  std::optional<Float3::Ref> bounds_min,
  std::optional<Float3::Ref> bounds_max
) {
  mjrfMeshData obj;
  Init(obj);
  if (compute_bounds) Setter(obj.compute_bounds, *compute_bounds);
  if (bounds_min) Setter(obj.bounds_min, *bounds_min);
  if (bounds_max) Setter(obj.bounds_max, *bounds_max);
  return obj;
})
, py::arg("compute_bounds").none(true) = py::none()
, py::arg("bounds_min").none(true) = py::none()
, py::arg("bounds_max").none(true) = py::none()
);
mjrf_mesh_data.def("__del__", [](mjrfMeshData& obj) { Free(obj); });
mjrf_mesh_data.def("free", [](mjrfMeshData& obj) { Free(obj); });
DefProperty<bool>(mjrf_mesh_data, "compute_bounds", &mjrfMeshData::compute_bounds);
DefProperty<Float3>(mjrf_mesh_data, "bounds_min", &mjrfMeshData::bounds_min);
DefProperty<Float3>(mjrf_mesh_data, "bounds_max", &mjrfMeshData::bounds_max);

py::class_<mjrfSceneParams> mjrf_scene_params(m, "SceneParams", py::module_local());
mjrf_scene_params.def(py::init([](

) {
  mjrfSceneParams obj;
  Init(obj);

  return obj;
})
);

py::class_<mjrfLightParams> mjrf_light_params(m, "LightParams", py::module_local());
mjrf_light_params.def(py::init([](
  std::optional<mjrLightType> type,
  PyTexture* texture,
  std::optional<Float3::Ref> color,
  std::optional<float> intensity,
  std::optional<bool> cast_shadows,
  std::optional<float> range,
  std::optional<float> spot_cone_angle,
  std::optional<float> spot_softness,
  std::optional<int> shadow_map_size,
  std::optional<float> bulb_radius,
  std::optional<float> vsm_blur_width
) {
  mjrfLightParams obj;
  Init(obj);
  if (type) Setter(obj.type, *type);
  if (texture) Setter(obj.texture, *texture);
  if (color) Setter(obj.color, *color);
  if (intensity) Setter(obj.intensity, *intensity);
  if (cast_shadows) Setter(obj.cast_shadows, *cast_shadows);
  if (range) Setter(obj.range, *range);
  if (spot_cone_angle) Setter(obj.spot_cone_angle, *spot_cone_angle);
  if (spot_softness) Setter(obj.spot_softness, *spot_softness);
  if (shadow_map_size) Setter(obj.shadow_map_size, *shadow_map_size);
  if (bulb_radius) Setter(obj.bulb_radius, *bulb_radius);
  if (vsm_blur_width) Setter(obj.vsm_blur_width, *vsm_blur_width);
  return obj;
})
, py::arg("type").none(true) = py::none()
, py::arg("texture").none(true) = py::none()
, py::arg("color").none(true) = py::none()
, py::arg("intensity").none(true) = py::none()
, py::arg("cast_shadows").none(true) = py::none()
, py::arg("range").none(true) = py::none()
, py::arg("spot_cone_angle").none(true) = py::none()
, py::arg("spot_softness").none(true) = py::none()
, py::arg("shadow_map_size").none(true) = py::none()
, py::arg("bulb_radius").none(true) = py::none()
, py::arg("vsm_blur_width").none(true) = py::none()
);
DefProperty<mjrLightType>(mjrf_light_params, "type", &mjrfLightParams::type);
DefProperty<PyTexture>(mjrf_light_params, "texture", &mjrfLightParams::texture);
DefProperty<Float3>(mjrf_light_params, "color", &mjrfLightParams::color);
DefProperty<float>(mjrf_light_params, "intensity", &mjrfLightParams::intensity);
DefProperty<bool>(mjrf_light_params, "cast_shadows", &mjrfLightParams::cast_shadows);
DefProperty<float>(mjrf_light_params, "range", &mjrfLightParams::range);
DefProperty<float>(mjrf_light_params, "spot_cone_angle", &mjrfLightParams::spot_cone_angle);
DefProperty<float>(mjrf_light_params, "spot_softness", &mjrfLightParams::spot_softness);
DefProperty<int>(mjrf_light_params, "shadow_map_size", &mjrfLightParams::shadow_map_size);
DefProperty<float>(mjrf_light_params, "bulb_radius", &mjrfLightParams::bulb_radius);
DefProperty<float>(mjrf_light_params, "vsm_blur_width", &mjrfLightParams::vsm_blur_width);

py::class_<mjrfMaterial> mjrf_material(m, "Material", py::module_local());
mjrf_material.def(py::init([](
  std::optional<Float4::Ref> color,
  std::optional<int32_t> segmentation_id,
  std::optional<int32_t> island_id,
  std::optional<mjtSleepState> sleep_state,
  std::optional<Float3::Ref> uv_scale,
  std::optional<Float3::Ref> uv_offset,
  std::optional<Float4::Ref> scissor,
  std::optional<float> metallic,
  std::optional<float> roughness,
  std::optional<float> specular,
  std::optional<float> glossiness,
  std::optional<float> emissive,
  std::optional<float> reflectance,
  std::optional<bool> decor_ux,
  std::optional<bool> selected,
  PyTexture* color_texture,
  PyTexture* opacity_texture,
  PyTexture* normal_texture,
  PyTexture* metallic_texture,
  PyTexture* roughness_texture,
  PyTexture* occlusion_texture,
  PyTexture* orm_texture,
  PyTexture* emissive_texture,
  PyTexture* reflection_texture
) {
  mjrfMaterial obj;
  Init(obj);
  if (color) Setter(obj.color, *color);
  if (segmentation_id) Setter(obj.segmentation_id, *segmentation_id);
  if (island_id) Setter(obj.island_id, *island_id);
  if (sleep_state) Setter(obj.sleep_state, *sleep_state);
  if (uv_scale) Setter(obj.uv_scale, *uv_scale);
  if (uv_offset) Setter(obj.uv_offset, *uv_offset);
  if (scissor) Setter(obj.scissor, *scissor);
  if (metallic) Setter(obj.metallic, *metallic);
  if (roughness) Setter(obj.roughness, *roughness);
  if (specular) Setter(obj.specular, *specular);
  if (glossiness) Setter(obj.glossiness, *glossiness);
  if (emissive) Setter(obj.emissive, *emissive);
  if (reflectance) Setter(obj.reflectance, *reflectance);
  if (decor_ux) Setter(obj.decor_ux, *decor_ux);
  if (selected) Setter(obj.selected, *selected);
  if (color_texture) Setter(obj.color_texture, *color_texture);
  if (opacity_texture) Setter(obj.opacity_texture, *opacity_texture);
  if (normal_texture) Setter(obj.normal_texture, *normal_texture);
  if (metallic_texture) Setter(obj.metallic_texture, *metallic_texture);
  if (roughness_texture) Setter(obj.roughness_texture, *roughness_texture);
  if (occlusion_texture) Setter(obj.occlusion_texture, *occlusion_texture);
  if (orm_texture) Setter(obj.orm_texture, *orm_texture);
  if (emissive_texture) Setter(obj.emissive_texture, *emissive_texture);
  if (reflection_texture) Setter(obj.reflection_texture, *reflection_texture);
  return obj;
})
, py::arg("color").none(true) = py::none()
, py::arg("segmentation_id").none(true) = py::none()
, py::arg("island_id").none(true) = py::none()
, py::arg("sleep_state").none(true) = py::none()
, py::arg("uv_scale").none(true) = py::none()
, py::arg("uv_offset").none(true) = py::none()
, py::arg("scissor").none(true) = py::none()
, py::arg("metallic").none(true) = py::none()
, py::arg("roughness").none(true) = py::none()
, py::arg("specular").none(true) = py::none()
, py::arg("glossiness").none(true) = py::none()
, py::arg("emissive").none(true) = py::none()
, py::arg("reflectance").none(true) = py::none()
, py::arg("decor_ux").none(true) = py::none()
, py::arg("selected").none(true) = py::none()
, py::arg("color_texture").none(true) = py::none()
, py::arg("opacity_texture").none(true) = py::none()
, py::arg("normal_texture").none(true) = py::none()
, py::arg("metallic_texture").none(true) = py::none()
, py::arg("roughness_texture").none(true) = py::none()
, py::arg("occlusion_texture").none(true) = py::none()
, py::arg("orm_texture").none(true) = py::none()
, py::arg("emissive_texture").none(true) = py::none()
, py::arg("reflection_texture").none(true) = py::none()
);
DefProperty<Float4>(mjrf_material, "color", &mjrfMaterial::color);
DefProperty<int32_t>(mjrf_material, "segmentation_id", &mjrfMaterial::segmentation_id);
DefProperty<int32_t>(mjrf_material, "island_id", &mjrfMaterial::island_id);
DefProperty<mjtSleepState>(mjrf_material, "sleep_state", &mjrfMaterial::sleep_state);
DefProperty<Float3>(mjrf_material, "uv_scale", &mjrfMaterial::uv_scale);
DefProperty<Float3>(mjrf_material, "uv_offset", &mjrfMaterial::uv_offset);
DefProperty<Float4>(mjrf_material, "scissor", &mjrfMaterial::scissor);
DefProperty<float>(mjrf_material, "metallic", &mjrfMaterial::metallic);
DefProperty<float>(mjrf_material, "roughness", &mjrfMaterial::roughness);
DefProperty<float>(mjrf_material, "specular", &mjrfMaterial::specular);
DefProperty<float>(mjrf_material, "glossiness", &mjrfMaterial::glossiness);
DefProperty<float>(mjrf_material, "emissive", &mjrfMaterial::emissive);
DefProperty<float>(mjrf_material, "reflectance", &mjrfMaterial::reflectance);
DefProperty<bool>(mjrf_material, "decor_ux", &mjrfMaterial::decor_ux);
DefProperty<bool>(mjrf_material, "selected", &mjrfMaterial::selected);
DefProperty<PyTexture>(mjrf_material, "color_texture", &mjrfMaterial::color_texture);
DefProperty<PyTexture>(mjrf_material, "opacity_texture", &mjrfMaterial::opacity_texture);
DefProperty<PyTexture>(mjrf_material, "normal_texture", &mjrfMaterial::normal_texture);
DefProperty<PyTexture>(mjrf_material, "metallic_texture", &mjrfMaterial::metallic_texture);
DefProperty<PyTexture>(mjrf_material, "roughness_texture", &mjrfMaterial::roughness_texture);
DefProperty<PyTexture>(mjrf_material, "occlusion_texture", &mjrfMaterial::occlusion_texture);
DefProperty<PyTexture>(mjrf_material, "orm_texture", &mjrfMaterial::orm_texture);
DefProperty<PyTexture>(mjrf_material, "emissive_texture", &mjrfMaterial::emissive_texture);
DefProperty<PyTexture>(mjrf_material, "reflection_texture", &mjrfMaterial::reflection_texture);

py::class_<mjrfRenderableParams> mjrf_renderable_params(m, "RenderableParams", py::module_local());
mjrf_renderable_params.def(py::init([](
  std::optional<bool> cast_shadows,
  std::optional<bool> receive_shadows,
  std::optional<uint16_t> blend_order
) {
  mjrfRenderableParams obj;
  Init(obj);
  if (cast_shadows) Setter(obj.cast_shadows, *cast_shadows);
  if (receive_shadows) Setter(obj.receive_shadows, *receive_shadows);
  if (blend_order) Setter(obj.blend_order, *blend_order);
  return obj;
})
, py::arg("cast_shadows").none(true) = py::none()
, py::arg("receive_shadows").none(true) = py::none()
, py::arg("blend_order").none(true) = py::none()
);
DefProperty<bool>(mjrf_renderable_params, "cast_shadows", &mjrfRenderableParams::cast_shadows);
DefProperty<bool>(mjrf_renderable_params, "receive_shadows", &mjrfRenderableParams::receive_shadows);
DefProperty<uint16_t>(mjrf_renderable_params, "blend_order", &mjrfRenderableParams::blend_order);

py::class_<mjrfRenderTargetConfig> mjrf_render_target_config(m, "RenderTargetConfig", py::module_local());
mjrf_render_target_config.def(py::init([](
  std::optional<int> width,
  std::optional<int> height,
  std::optional<mjrPixelFormat> color_format,
  std::optional<mjrPixelFormat> depth_format
) {
  mjrfRenderTargetConfig obj;
  Init(obj);
  if (width) Setter(obj.width, *width);
  if (height) Setter(obj.height, *height);
  if (color_format) Setter(obj.color_format, *color_format);
  if (depth_format) Setter(obj.depth_format, *depth_format);
  return obj;
})
, py::arg("width").none(true) = py::none()
, py::arg("height").none(true) = py::none()
, py::arg("color_format").none(true) = py::none()
, py::arg("depth_format").none(true) = py::none()
);
DefProperty<int>(mjrf_render_target_config, "width", &mjrfRenderTargetConfig::width);
DefProperty<int>(mjrf_render_target_config, "height", &mjrfRenderTargetConfig::height);
DefProperty<mjrPixelFormat>(mjrf_render_target_config, "color_format", &mjrfRenderTargetConfig::color_format);
DefProperty<mjrPixelFormat>(mjrf_render_target_config, "depth_format", &mjrfRenderTargetConfig::depth_format);

py::class_<PyContext> mjrf_context(m, "Context");

py::class_<PyTexture> mjrf_texture(m, "Texture");

py::class_<PyMesh> mjrf_mesh(m, "Mesh");

py::class_<PyScene> mjrf_scene(m, "Scene");

py::class_<PyLight> mjrf_light(m, "Light");

py::class_<PyRenderable> mjrf_renderable(m, "Renderable");

py::class_<PyRenderTarget> mjrf_render_target(m, "RenderTarget");

mjrf_context.def("wait_for_frame", [](
  PyContext& ctx,
  mjrfFrameHandle frame
) {
    return mjrf_waitForFrame(ctx.get(), frame);
}
, py::arg("frame")
);

mjrf_context.def("set_clear_color", [](
  PyContext& ctx,
  Float3::Ref& color
) {
    return mjrf_setClearColor(ctx.get(), color.data());
}
, py::arg("color")
);

mjrf_context.def("get_frame_stats", [](
  PyContext& ctx,
  mjrfFrameHandle frame,
  mjrfFrameStats * stats_out
) {
    return mjrf_getFrameStats(ctx.get(), frame, stats_out);
}
, py::arg("frame")
, py::arg("stats_out")
);

mjrf_texture.def("set_data", [](
  PyTexture& texture,
  const mjrfTextureData * data
) {
    return mjrf_setTextureData(texture.get(), data);
}
, py::arg("data")
);

mjrf_texture.def("get_width", [](
  PyTexture& texture
) {
    return mjrf_getTextureWidth(texture.get());
}
);

mjrf_texture.def("get_height", [](
  PyTexture& texture
) {
    return mjrf_getTextureHeight(texture.get());
}
);

mjrf_texture.def("get_sampler_type", [](
  PyTexture& texture
) {
    return (mjrSamplerType)mjrf_getTextureSamplerType(texture.get());
}
);

mjrf_mesh.def("set_data", [](
  PyMesh& mesh,
  const mjrfMeshData* data
) {
    return mjrf_setMeshData(mesh.get(), data);
}
, py::arg("data")
);

mjrf_scene.def("add_light", [](
  PyScene& scene,
  PyLight& light
) {
    return mjrf_addLightToScene(scene.get(), light.get());
}
, py::arg("light")
);

mjrf_scene.def("remove_light", [](
  PyScene& scene,
  PyLight& light
) {
    return mjrf_removeLightFromScene(scene.get(), light.get());
}
, py::arg("light")
);

mjrf_scene.def("add_renderable", [](
  PyScene& scene,
  PyRenderable& renderable
) {
    return mjrf_addRenderableToScene(scene.get(), renderable.get());
}
, py::arg("renderable")
);

mjrf_scene.def("remove_renderable", [](
  PyScene& scene,
  PyRenderable& renderable
) {
    return mjrf_removeRenderableFromScene(scene.get(), renderable.get());
}
, py::arg("renderable")
);

mjrf_scene.def("set_skybox", [](
  PyScene& scene,
  PyTexture& texture
) {
    return mjrf_setSceneSkybox(scene.get(), texture.get());
}
, py::arg("texture")
);

mjrf_light.def("set_enabled", [](
  PyLight& light,
  bool enabled
) {
    return mjrf_setLightEnabled(light.get(), enabled);
}
, py::arg("enabled")
);

mjrf_light.def("set_shadows_enabled", [](
  PyLight& light,
  bool enabled
) {
    return mjrf_setLightShadowsEnabled(light.get(), enabled);
}
, py::arg("enabled")
);

mjrf_light.def("set_intensity", [](
  PyLight& light,
  float intensity
) {
    return mjrf_setLightIntensity(light.get(), intensity);
}
, py::arg("intensity")
);

mjrf_light.def("set_shadow_map_size", [](
  PyLight& light,
  int map_size
) {
    return mjrf_setLightShadowMapSize(light.get(), map_size);
}
, py::arg("map_size")
);

mjrf_light.def("set_color", [](
  PyLight& light,
  Float3::Ref& color
) {
    return mjrf_setLightColor(light.get(), color.data());
}
, py::arg("color")
);

//---
mjrf_light.def("set_range", [](
  PyLight& light,
  float range
) {
    return mjrf_setLightRange(light.get(), range);
}
, py::arg("range")
);

mjrf_light.def("set_cutoff_angle", [](
  PyLight& light,
  float cutoff
) {
    return mjrf_setLightCutoffAngle(light.get(), cutoff);
}
, py::arg("cutoff")
);

mjrf_light.def("set_softness", [](
  PyLight& light,
  float softness
) {
    return mjrf_setLightSoftness(light.get(), softness);
}
, py::arg("softness")
);

mjrf_light.def("set_bulb_radius", [](
  PyLight& light,
  float radius
) {
    return mjrf_setLightBulbRadius(light.get(), radius);
}
, py::arg("radius")
);

mjrf_light.def("set_blur_width", [](
  PyLight& light,
  float blur_width
) {
    return mjrf_setLightBlurWidth(light.get(), blur_width);
}
, py::arg("blur_width")
);

mjrf_light.def("set_transform", [](
  PyLight& light,
  Float3::Ref& position,
  Float3::Ref& direction
) {
    return mjrf_setLightTransform(light.get(), position.data(), direction.data());
}
, py::arg("position")
, py::arg("direction")
);

mjrf_light.def("get_type", [](
  PyLight& light
) {
    return mjrf_getLightType(light.get());
}
);

mjrf_renderable.def("set_mesh", [](
  PyRenderable& renderable,
  PyMesh& mesh,
  int elem_offset,
  int elem_count
) {
    return mjrf_setRenderableMesh(renderable.get(), mesh.get(), elem_offset, elem_count);
}
, py::arg("mesh")
, py::arg("elem_offset")
, py::arg("elem_count")
);

mjrf_renderable.def("set_geom_mesh", [](
  PyRenderable& renderable,
  mjtGeom type,
  int nstack,
  int nslice,
  int nquad
) {
    return mjrf_setRenderableGeomMesh(renderable.get(), type, nstack, nslice, nquad);
}
, py::arg("type")
, py::arg("nstack")
, py::arg("nslice")
, py::arg("nquad")
);

mjrf_renderable.def("set_material", [](
  PyRenderable& renderable,
  const mjrfMaterial * material
) {
    return mjrf_setRenderableMaterial(renderable.get(), material);
}
, py::arg("material")
);

mjrf_renderable.def("set_transform", [](
  PyRenderable& renderable,
  Float3::Ref& position,
  Float9::Ref& rotation
) {
    return mjrf_setRenderableTransform(renderable.get(), position.data(), rotation.data());
}
, py::arg("position")
, py::arg("rotation")
);

mjrf_renderable.def("set_size", [](
  PyRenderable& renderable,
  Float3::Ref& size
) {
    return mjrf_setRenderableSize(renderable.get(), size.data());
}
, py::arg("size")
);

mjrf_render_target.def("resize", [](
  PyRenderTarget& render_target,
  int width,
  int height
) {
    return mjrf_resizeRenderTarget(render_target.get(), width, height);
}
, py::arg("width")
, py::arg("height")
);
