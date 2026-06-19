// Vector basemap line shader. Vertices are in Web-Mercator-relative-to-origin
// metres (f32); the view uniform maps them to NDC. Drawn as a LineList, so
// each consecutive pair of vertices is one 1px segment.

struct ViewUniform {
    // ndc_x = world_x * x_scale + x_offset
    // ndc_y = world_y * y_scale + y_offset   (y_scale > 0: Mercator north is up)
    x_scale: f32,
    x_offset: f32,
    y_scale: f32,
    y_offset: f32,
};

@group(0) @binding(0) var<uniform> view: ViewUniform;

struct Vsout {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(@location(0) pos: vec2<f32>, @location(1) color: vec3<f32>) -> Vsout {
    var out: Vsout;
    out.clip = vec4<f32>(pos.x * view.x_scale + view.x_offset,
                         pos.y * view.y_scale + view.y_offset,
                         0.0, 1.0);
    out.color = color;
    return out;
}

@fragment
fn fs_main(in: Vsout) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
