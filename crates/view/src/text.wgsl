// Label text shader. Vertices are screen pixels (origin top-left); the uniform
// carries the viewport so we can map to NDC. Samples the R8 coverage atlas as
// alpha and paints dark text. Alpha-blended.

struct U {
    vw: f32,
    vh: f32,
    _p0: f32,
    _p1: f32,
};
@group(0) @binding(0) var<uniform> u: U;
@group(1) @binding(0) var atlas: texture_2d<f32>;
@group(1) @binding(1) var samp: sampler;

struct Out {
    @builtin(position) clip: vec4<f32>,
    @location(0) uv: vec2<f32>,
    @location(1) color: vec3<f32>,
};

@vertex
fn vs_main(@location(0) pos: vec2<f32>, @location(1) uv: vec2<f32>, @location(2) color: vec3<f32>) -> Out {
    var o: Out;
    let ndc = vec2<f32>(pos.x / u.vw * 2.0 - 1.0, 1.0 - pos.y / u.vh * 2.0);
    o.clip = vec4<f32>(ndc, 0.0, 1.0);
    o.uv = uv;
    o.color = color;
    return o;
}

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    let a = textureSample(atlas, samp, in.uv).r;
    return vec4<f32>(in.color, a);
}
