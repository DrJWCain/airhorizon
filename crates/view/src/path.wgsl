// Footpath shader: screen-space line expansion. Each segment is two triangles;
// every vertex carries the segment endpoint (Mercator-relative), the segment
// direction, and a side (+1/-1). The vertex is offset perpendicular to the
// segment by a constant pixel half-width — so paths keep the same on-screen
// thickness at every zoom, with no per-zoom rebuild.

struct PathU {
    x_scale: f32,
    x_offset: f32,
    y_scale: f32,
    y_offset: f32,
    vw: f32,
    vh: f32,
    half_px: f32,
    _pad: f32,
};
@group(0) @binding(0) var<uniform> u: PathU;

struct In {
    @location(0) pos: vec2<f32>,   // segment endpoint, Mercator-relative
    @location(1) dir: vec2<f32>,   // segment direction (world; need not be unit)
    @location(2) side: f32,        // +1 / -1
    @location(3) color: vec3<f32>,
};
struct Out {
    @builtin(position) clip: vec4<f32>,
    @location(0) color: vec3<f32>,
};

@vertex
fn vs_main(in: In) -> Out {
    let base = vec2<f32>(in.pos.x * u.x_scale + u.x_offset, in.pos.y * u.y_scale + u.y_offset);
    // Segment direction expressed in pixels (account for the view scale + aspect).
    let dpx = vec2<f32>(in.dir.x * u.x_scale * u.vw * 0.5, in.dir.y * u.y_scale * u.vh * 0.5);
    let udir = dpx / max(length(dpx), 1e-6);
    let nrm = vec2<f32>(-udir.y, udir.x);              // perpendicular, in pixels
    let off_px = nrm * u.half_px * in.side;
    let off_ndc = vec2<f32>(off_px.x * 2.0 / u.vw, off_px.y * 2.0 / u.vh);
    var out: Out;
    out.clip = vec4<f32>(base + off_ndc, 0.0, 1.0);
    out.color = in.color;
    return out;
}

@fragment
fn fs_main(in: Out) -> @location(0) vec4<f32> {
    return vec4<f32>(in.color, 1.0);
}
