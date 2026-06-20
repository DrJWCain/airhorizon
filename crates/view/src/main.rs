//! AirHorizon vector map viewer.
//!
//! Live-renders OS Open Zoomstack MVT tiles in Web Mercator: polygon area fills
//! (earcut) under tessellated thick line strokes (lyon). Mouse drag / wheel and
//! touch pan + pinch-zoom, OSM footpaths (P), and place-name labels (L).
//!
//!   cargo run -p view --release --offline -- [mbtiles] [lat] [lon]

mod text;

use std::collections::HashMap;
use std::sync::Arc;

use text::{FontAtlas, TextVertex};

use basemap::{GeomKind, Mbtiles};
use bytemuck::{Pod, Zeroable};
use dem::Dem;
use geodesy::{LatLon, Mercator, Tile, MERCATOR_MAX};
use horizon::{cast, visible_peaks, visible_water, HorizonParams, Visibility, WaterMask};
use mapdata::{load_paths, PathKind};
use peaks::Peaks;
use lyon::math::point;
use lyon::path::Path;
use lyon::tessellation::{BuffersBuilder, StrokeOptions, StrokeTessellator, StrokeVertex, VertexBuffers};
use wgpu::util::DeviceExt;
use winit::application::ApplicationHandler;
use winit::dpi::{PhysicalPosition, PhysicalSize};
use winit::event::{ElementState, MouseButton, MouseScrollDelta, Touch, TouchPhase, WindowEvent};
use winit::event_loop::{ActiveEventLoop, EventLoop};
use winit::keyboard::{KeyCode, PhysicalKey};
use winit::window::{Window, WindowId};

/// New tiles decoded+uploaded per frame; the rest stream over later frames
/// (keeps the window responsive — the qct-viewer progressive-fill lesson).
const TILE_BUDGET_PER_FRAME: usize = 4;
const MAX_SLIPPY_ZOOM: u8 = 14;

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct Vertex {
    pos: [f32; 2], // Web Mercator metres, relative to the world origin
    color: [f32; 3],
}

#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct ViewUniform {
    x_scale: f32,
    x_offset: f32,
    y_scale: f32,
    y_offset: f32,
}

/// Footpath vertex for screen-space line expansion (see path.wgsl).
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct PathVertex {
    pos: [f32; 2],   // segment endpoint, Mercator-relative
    dir: [f32; 2],   // segment direction (world)
    side: f32,       // +1 / -1
    color: [f32; 3],
}

/// Uniform for the path pipeline: view transform + viewport + pixel half-width.
#[repr(C)]
#[derive(Copy, Clone, Pod, Zeroable)]
struct PathUniform {
    x_scale: f32,
    x_offset: f32,
    y_scale: f32,
    y_offset: f32,
    vw: f32,
    vh: f32,
    half_px: f32,
    _pad: f32,
}

/// Roads stroke width in tile-local units. Footpaths match roads by computing
/// the equivalent on-screen pixel width each frame from the current zoom/level.
const ROAD_WIDTH_TILE: f32 = 22.0;

/// Label text colours.
const PLACE_LABEL_COLOR: [f32; 3] = [0.12, 0.10, 0.10]; // settlements/water: near-black
const PEAK_LABEL_COLOR: [f32; 3] = [0.60, 0.22, 0.0]; // visible summits: burnt orange
const SLOPE_LABEL_COLOR: [f32; 3] = [0.05, 0.18, 0.55]; // summit hidden, slopes in view: dark blue (reads on both sky and terrain)

const DEM_DIR: &str = r"C:\maps\OS Terrain 50";

/// What the viewer is showing.
#[derive(Clone, Copy, PartialEq)]
enum Mode {
    Map,
    Panorama,
    Compass,
}

/// A peak placed on the synthetic panorama.
struct PanoPeak {
    name: String,
    bearing_deg: f64,
    elev_deg: f64,
    dist_km: f64,
    /// Summit hidden behind a nearer ridge but the fell's slopes are in view.
    obscured: bool,
}

/// State for the synthetic-panorama mode: the cast skyline + visible peaks from
/// a fixed viewpoint, with the azimuth the view is currently centred on.
struct PanoState {
    viewpoint: LatLon,
    ground_m: f64,
    center_az: f64,
    fov_deg: f64,
    skyline: Vec<f32>, // horizon::AZIMUTH_BUCKETS elevation angles (radians)
    edge_lines: Vec<Vec<(f32, f32)>>, // linked occlusion edges: (azimuth_deg, elev_rad)
    edge_top: Vec<f32>, // per bucket: highest occlusion-edge elev (rad), or -inf
    water: Vec<Vec<(f32, f32)>>, // per bucket: visible-water segments (top, bottom) elev (rad)
    peaks: Vec<PanoPeak>,
}

/// How a Zoomstack layer is drawn: a filled area, or a stroked line of a given
/// width. Stroke width is in tile-local units (0..4096); since `slippy_zoom`
/// keeps a tile ~512 px on screen, `width_px ~= width / 8`.
#[derive(Clone, Copy)]
enum Style {
    Fill([f32; 3]),
    Stroke([f32; 3], f32),
}

/// Per-layer style, or `None` to skip. Fills are pale so the strokes read on top.
fn style(name: &str) -> Option<Style> {
    use Style::{Fill, Stroke};
    Some(match name {
        // Area fills (polygons), drawn underneath.
        "woodland" => Fill([0.80, 0.88, 0.73]),
        "greenspaces" => Fill([0.86, 0.91, 0.80]),
        "surfacewater" => Fill([0.69, 0.82, 0.92]),
        "buildings" => Fill([0.84, 0.79, 0.76]),
        // Line strokes, drawn on top (width in tile units; ~/8 = screen px).
        "roads" => Stroke([0.28, 0.26, 0.24], ROAD_WIDTH_TILE),
        "waterlines" => Stroke([0.20, 0.45, 0.80], 13.0),
        "contours" => Stroke([0.78, 0.66, 0.50], 6.0),
        // NB: don't stroke tile-clipped AREA polygons (e.g. national_parks) — the
        // clip edges show up as square outlines along every tile boundary.
        _ => return None, // names (points), sites, national_parks, etc.
    })
}

/// Tessellate a tile-local polyline into triangle vertices (tile-local coords)
/// of a stroke `width` wide, using lyon. Reuses one tessellator across calls.
fn stroke_to_tris(tess: &mut StrokeTessellator, points: &[[f32; 2]], width: f32) -> Vec<[f32; 2]> {
    if points.len() < 2 {
        return Vec::new();
    }
    let mut pb = Path::builder();
    pb.begin(point(points[0][0], points[0][1]));
    for p in &points[1..] {
        pb.line_to(point(p[0], p[1]));
    }
    pb.end(false);
    let path = pb.build();

    let mut buffers: VertexBuffers<[f32; 2], u32> = VertexBuffers::new();
    let opts = StrokeOptions::default().with_line_width(width);
    let ok = tess.tessellate_path(
        &path,
        &opts,
        &mut BuffersBuilder::new(&mut buffers, |v: StrokeVertex| {
            let p = v.position();
            [p.x, p.y]
        }),
    );
    if ok.is_err() {
        return Vec::new();
    }
    buffers.indices.iter().map(|&i| buffers.vertices[i as usize]).collect()
}

/// Drop a ring's explicit closing vertex (our decoder repeats first==last).
fn open_ring(ring: &[[f32; 2]]) -> &[[f32; 2]] {
    if ring.len() >= 2 && ring.first() == ring.last() {
        &ring[..ring.len() - 1]
    } else {
        ring
    }
}

/// Shoelace signed area in tile coordinates (y-down). MVT exterior rings are
/// clockwise => positive area; interior rings (holes) => negative.
fn signed_area(r: &[[f32; 2]]) -> f32 {
    let n = r.len();
    if n < 3 {
        return 0.0;
    }
    let mut a = 0.0f32;
    for i in 0..n {
        let j = (i + 1) % n;
        a += r[i][0] * r[j][1] - r[j][0] * r[i][1];
    }
    a * 0.5
}

/// View state: a centre point and zoom in Web Mercator space, stored relative
/// to a fixed origin so f32 vertex coords keep their precision.
struct Camera {
    origin_x: f64,
    origin_y: f64,
    center_x: f64, // relative to origin
    center_y: f64,
    zoom: f64, // screen px per Mercator metre
    vw: f64,
    vh: f64,
}

impl Camera {
    fn new(center: LatLon, vw: f64, vh: f64) -> Self {
        let m = center.to_mercator();
        Self {
            origin_x: m.x,
            origin_y: m.y,
            center_x: 0.0,
            center_y: 0.0,
            zoom: vw / 25_000.0, // ~25 km across the window to start
            vw,
            vh,
        }
    }

    fn uniform(&self) -> ViewUniform {
        // ndc = (world_rel - center) * 2 * zoom / viewport
        ViewUniform {
            x_scale: (2.0 * self.zoom / self.vw) as f32,
            x_offset: (-self.center_x * 2.0 * self.zoom / self.vw) as f32,
            y_scale: (2.0 * self.zoom / self.vh) as f32,
            y_offset: (-self.center_y * 2.0 * self.zoom / self.vh) as f32,
        }
    }

    /// Screen pixel -> Mercator coordinate (absolute), origin top-left.
    fn screen_to_merc(&self, sx: f64, sy: f64) -> (f64, f64) {
        let wx = self.center_x + (sx - self.vw * 0.5) / self.zoom;
        let wy = self.center_y - (sy - self.vh * 0.5) / self.zoom; // screen y down, world y up
        (self.origin_x + wx, self.origin_y + wy)
    }

    fn pan_screen(&mut self, dx: f64, dy: f64) {
        self.center_x -= dx / self.zoom;
        self.center_y += dy / self.zoom;
    }

    fn zoom_about(&mut self, factor: f64, ax: f64, ay: f64) {
        let (mx, my) = self.screen_to_merc(ax, ay);
        self.zoom = (self.zoom * factor).clamp(1.0e-4, 5.0);
        // Re-place centre so the anchored Mercator point stays under (ax, ay).
        self.center_x = (mx - self.origin_x) - (ax - self.vw * 0.5) / self.zoom;
        self.center_y = (my - self.origin_y) + (ay - self.vh * 0.5) / self.zoom;
    }

    /// Slippy zoom level that keeps tiles ~512 px on screen.
    fn slippy_zoom(&self, maxz: u8) -> u8 {
        let target_px = 512.0;
        let z = (2.0 * MERCATOR_MAX * self.zoom / target_px).log2().round();
        (z.clamp(0.0, maxz as f64)) as u8
    }

    /// Inclusive tile-index range covering the viewport at zoom `z`.
    fn visible_tiles(&self, z: u8) -> ((u32, u32), (u32, u32)) {
        let n = 1u32 << z;
        let to_tile = |mx: f64, my: f64| -> (i64, i64) {
            let fx = (mx + MERCATOR_MAX) / (2.0 * MERCATOR_MAX) * n as f64;
            let fy = (MERCATOR_MAX - my) / (2.0 * MERCATOR_MAX) * n as f64;
            (fx.floor() as i64, fy.floor() as i64)
        };
        let (ax, ay) = self.screen_to_merc(0.0, 0.0);
        let (bx, by) = self.screen_to_merc(self.vw, self.vh);
        let (tx0, ty0) = to_tile(ax.min(bx), ay.max(by)); // NW
        let (tx1, ty1) = to_tile(ax.max(bx), ay.min(by)); // SE
        let clamp = |v: i64| v.clamp(0, n as i64 - 1) as u32;
        ((clamp(tx0), clamp(ty0)), (clamp(tx1), clamp(ty1)))
    }
}

/// GPU geometry for one tile: area-fill triangles and stroke triangles (both
/// TriangleList). Either may be empty; an all-empty mesh is still cached so the
/// tile isn't re-decoded.
struct TileMesh {
    area_buf: Option<wgpu::Buffer>,
    area_count: u32,
    stroke_buf: Option<wgpu::Buffer>,
    stroke_count: u32,
    /// Place-name labels: (name1, type, Mercator-relative point).
    labels: Vec<(String, String, [f32; 2])>,
}

/// Lowest slippy zoom at which a label of this Zoomstack `type` is shown — keeps
/// minor names off the map until you zoom in.
fn label_min_zoom(kind: &str) -> u8 {
    match kind {
        "City" => 7,
        "Town" => 9,
        "Village" | "Suburban Area" => 11,
        "Hamlet" | "Small Settlements" => 13,
        "Water" => 13,
        _ => 12,
    }
}

/// Placement priority (lower = more important, placed first under collision).
fn label_priority(kind: &str) -> u8 {
    match kind {
        "City" => 0,
        "Town" => 1,
        "Village" | "Suburban Area" => 2,
        "Water" => 3,
        "Hamlet" | "Small Settlements" => 4,
        _ => 3,
    }
}

/// Lowest slippy zoom at which a peak of this prominence (m) is labelled.
fn peak_min_zoom(prom: f64) -> u8 {
    if prom >= 600.0 {
        8
    } else if prom >= 300.0 {
        10
    } else if prom >= 150.0 {
        11
    } else if prom >= 100.0 {
        12
    } else {
        13
    }
}

/// Peak label collision priority by prominence (lower = placed first).
fn peak_priority(prom: f64) -> u8 {
    if prom >= 300.0 {
        1
    } else if prom >= 150.0 {
        2
    } else {
        3
    }
}

struct Gpu {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    pipeline: wgpu::RenderPipeline,
    view_buf: wgpu::Buffer,
    view_bg: wgpu::BindGroup,
    path_pipeline: wgpu::RenderPipeline,
    path_ubuf: wgpu::Buffer,
    path_bg: wgpu::BindGroup,
}

impl Gpu {
    async fn new(window: Arc<Window>) -> Self {
        let size = window.inner_size();
        let instance = wgpu::Instance::new(wgpu::InstanceDescriptor::default());
        let surface = instance.create_surface(window.clone()).expect("surface");
        let adapter = instance
            .request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                compatible_surface: Some(&surface),
                force_fallback_adapter: false,
            })
            .await
            .expect("adapter");
        let (device, queue) = adapter
            .request_device(
                &wgpu::DeviceDescriptor {
                    label: Some("airhorizon device"),
                    required_features: wgpu::Features::empty(),
                    required_limits: wgpu::Limits::default(),
                    memory_hints: wgpu::MemoryHints::Performance,
                },
                None,
            )
            .await
            .expect("device");
        let device = Arc::new(device);
        let queue = Arc::new(queue);

        let caps = surface.get_capabilities(&adapter);
        let format = caps
            .formats
            .iter()
            .find(|f| !f.is_srgb())
            .copied()
            .unwrap_or(caps.formats[0]);
        let config = wgpu::SurfaceConfiguration {
            usage: wgpu::TextureUsages::RENDER_ATTACHMENT,
            format,
            width: size.width.max(1),
            height: size.height.max(1),
            present_mode: wgpu::PresentMode::Fifo,
            desired_maximum_frame_latency: 2,
            alpha_mode: caps.alpha_modes[0],
            view_formats: vec![],
        };
        surface.configure(&device, &config);

        let view_buf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("view uniform"),
            size: std::mem::size_of::<ViewUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let view_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("view bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let view_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("view bg"),
            layout: &view_bgl,
            entries: &[wgpu::BindGroupEntry {
                binding: 0,
                resource: view_buf.as_entire_binding(),
            }],
        });

        let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("line shader"),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!("line.wgsl"))),
        });
        let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("line pl layout"),
            bind_group_layouts: &[&view_bgl],
            push_constant_ranges: &[],
        });
        // One TriangleList pipeline: both area fills and tessellated strokes are
        // triangles, sharing the shader, layout and vertex format.
        let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("tri pipeline"),
            layout: Some(&layout),
            vertex: wgpu::VertexState {
                module: &shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<Vertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x2,
                            offset: 0,
                            shader_location: 0,
                        },
                        wgpu::VertexAttribute {
                            format: wgpu::VertexFormat::Float32x3,
                            offset: 8,
                            shader_location: 1,
                        },
                    ],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        // Path pipeline: its own uniform (adds viewport + half-width) and shader
        // that expands segments to a constant pixel width.
        let path_ubuf = device.create_buffer(&wgpu::BufferDescriptor {
            label: Some("path uniform"),
            size: std::mem::size_of::<PathUniform>() as u64,
            usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
            mapped_at_creation: false,
        });
        let path_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
            label: Some("path bgl"),
            entries: &[wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::VERTEX,
                ty: wgpu::BindingType::Buffer {
                    ty: wgpu::BufferBindingType::Uniform,
                    has_dynamic_offset: false,
                    min_binding_size: None,
                },
                count: None,
            }],
        });
        let path_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
            label: Some("path bg"),
            layout: &path_bgl,
            entries: &[wgpu::BindGroupEntry { binding: 0, resource: path_ubuf.as_entire_binding() }],
        });
        let path_shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
            label: Some("path shader"),
            source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!("path.wgsl"))),
        });
        let path_layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
            label: Some("path pl layout"),
            bind_group_layouts: &[&path_bgl],
            push_constant_ranges: &[],
        });
        let path_pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
            label: Some("path pipeline"),
            layout: Some(&path_layout),
            vertex: wgpu::VertexState {
                module: &path_shader,
                entry_point: Some("vs_main"),
                compilation_options: Default::default(),
                buffers: &[wgpu::VertexBufferLayout {
                    array_stride: std::mem::size_of::<PathVertex>() as u64,
                    step_mode: wgpu::VertexStepMode::Vertex,
                    attributes: &[
                        wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x2, offset: 0, shader_location: 0 },
                        wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x2, offset: 8, shader_location: 1 },
                        wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32, offset: 16, shader_location: 2 },
                        wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 20, shader_location: 3 },
                    ],
                }],
            },
            fragment: Some(wgpu::FragmentState {
                module: &path_shader,
                entry_point: Some("fs_main"),
                compilation_options: Default::default(),
                targets: &[Some(wgpu::ColorTargetState {
                    format,
                    blend: Some(wgpu::BlendState::REPLACE),
                    write_mask: wgpu::ColorWrites::ALL,
                })],
            }),
            primitive: wgpu::PrimitiveState {
                topology: wgpu::PrimitiveTopology::TriangleList,
                ..Default::default()
            },
            depth_stencil: None,
            multisample: wgpu::MultisampleState::default(),
            multiview: None,
            cache: None,
        });

        Self {
            device,
            queue,
            surface,
            config,
            pipeline,
            view_buf,
            view_bg,
            path_pipeline,
            path_ubuf,
            path_bg,
        }
    }

    fn resize(&mut self, w: u32, h: u32) {
        if w == 0 || h == 0 {
            return;
        }
        self.config.width = w;
        self.config.height = h;
        self.surface.configure(&self.device, &self.config);
    }
}

struct AppState {
    window: Arc<Window>,
    gpu: Gpu,
    mbt: Mbtiles,
    maxzoom: u8,
    cam: Camera,
    tiles: HashMap<Tile, TileMesh>,
    cursor: Option<(f64, f64)>,
    dragging: bool,
    last_cursor: Option<(f64, f64)>,
    /// Active touch fingers: id -> last position. While non-empty, mouse drag
    /// is ignored (Windows synthesises mouse events from touch).
    touches: HashMap<u64, (f64, f64)>,
    /// Two-finger reference (midpoint, separation) for the pinch delta.
    pinch_prev: Option<((f64, f64), f64)>,
    /// Layer visibility toggles (R toggles roads).
    show_roads: bool,
    /// OSM footpaths overlay, lazily loaded on first P press.
    paths_pbf: std::path::PathBuf,
    paths_loaded: bool,
    show_paths: bool,
    path_buf: Option<wgpu::Buffer>,
    path_count: u32,
    /// Place-name labels (L toggles).
    atlas: FontAtlas,
    show_labels: bool,
    text_pipeline: wgpu::RenderPipeline,
    text_ubuf: wgpu::Buffer,
    text_ubg: wgpu::BindGroup,
    text_tex_bg: wgpu::BindGroup,
    /// DoBIH summits (K toggles markers+height, N toggles the peak name text).
    peaks: Peaks,
    show_peaks: bool,
    show_peak_names: bool,
    /// Map vs synthetic panorama (V toggles). DEM loads lazily on first entry.
    mode: Mode,
    dem: Option<Dem>,
    pano: Option<PanoState>,
    show_edges: bool,
    show_water: bool,
    /// Panorama render style: false = clean/natural, true = pen-and-ink artist view.
    artist: bool,
}

/// Cheap deterministic 0..1 jitter from an integer, for hand-drawn variation.
fn jitter(i: u32) -> f32 {
    let x = i.wrapping_mul(2_654_435_761);
    ((x >> 8) & 0xffff) as f32 / 65_535.0
}

/// Push a thick screen-space line segment (two triangles) sampling the solid
/// texel — used by the compass view for rings, arms and cardinals.
fn push_seg(tv: &mut Vec<TextVertex>, uv: [f32; 2], x0: f32, y0: f32, x1: f32, y1: f32, t: f32, c: [f32; 3]) {
    let (dx, dy) = (x1 - x0, y1 - y0);
    let l = (dx * dx + dy * dy).sqrt().max(1e-3);
    let (nx, ny) = (-dy / l * t, dx / l * t);
    tv.push(TextVertex { pos: [x0 + nx, y0 + ny], uv, color: c });
    tv.push(TextVertex { pos: [x0 - nx, y0 - ny], uv, color: c });
    tv.push(TextVertex { pos: [x1 + nx, y1 + ny], uv, color: c });
    tv.push(TextVertex { pos: [x1 + nx, y1 + ny], uv, color: c });
    tv.push(TextVertex { pos: [x0 - nx, y0 - ny], uv, color: c });
    tv.push(TextVertex { pos: [x1 - nx, y1 - ny], uv, color: c });
}

fn push_circle(tv: &mut Vec<TextVertex>, uv: [f32; 2], cx: f32, cy: f32, rad: f32, t: f32, c: [f32; 3]) {
    let n = 96;
    let mut prev: Option<(f32, f32)> = None;
    for k in 0..=n {
        let th = k as f32 / n as f32 * std::f32::consts::TAU;
        let (x, y) = (cx + rad * th.cos(), cy + rad * th.sin());
        if let Some((px, py)) = prev {
            push_seg(tv, uv, px, py, x, y, t, c);
        }
        prev = Some((x, y));
    }
}

/// Build the text rendering resources: upload the atlas as an R8 texture and
/// make the pipeline (screen-px quads sampling coverage as alpha). Returns
/// (pipeline, viewport-uniform buffer, uniform bind group, texture bind group).
fn build_text(
    device: &wgpu::Device,
    queue: &wgpu::Queue,
    format: wgpu::TextureFormat,
    atlas: &FontAtlas,
) -> (wgpu::RenderPipeline, wgpu::Buffer, wgpu::BindGroup, wgpu::BindGroup) {
    let tex = device.create_texture(&wgpu::TextureDescriptor {
        label: Some("font atlas"),
        size: wgpu::Extent3d { width: atlas.width, height: atlas.height, depth_or_array_layers: 1 },
        mip_level_count: 1,
        sample_count: 1,
        dimension: wgpu::TextureDimension::D2,
        format: wgpu::TextureFormat::R8Unorm,
        usage: wgpu::TextureUsages::TEXTURE_BINDING | wgpu::TextureUsages::COPY_DST,
        view_formats: &[],
    });
    queue.write_texture(
        wgpu::ImageCopyTexture {
            texture: &tex,
            mip_level: 0,
            origin: wgpu::Origin3d::ZERO,
            aspect: wgpu::TextureAspect::All,
        },
        &atlas.pixels,
        wgpu::ImageDataLayout {
            offset: 0,
            bytes_per_row: Some(atlas.width), // R8, and width=512 is 256-aligned
            rows_per_image: Some(atlas.height),
        },
        wgpu::Extent3d { width: atlas.width, height: atlas.height, depth_or_array_layers: 1 },
    );
    let view = tex.create_view(&wgpu::TextureViewDescriptor::default());
    let sampler = device.create_sampler(&wgpu::SamplerDescriptor {
        label: Some("font sampler"),
        mag_filter: wgpu::FilterMode::Linear,
        min_filter: wgpu::FilterMode::Linear,
        ..Default::default()
    });

    let u_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("text u bgl"),
        entries: &[wgpu::BindGroupLayoutEntry {
            binding: 0,
            visibility: wgpu::ShaderStages::VERTEX,
            ty: wgpu::BindingType::Buffer {
                ty: wgpu::BufferBindingType::Uniform,
                has_dynamic_offset: false,
                min_binding_size: None,
            },
            count: None,
        }],
    });
    let tex_bgl = device.create_bind_group_layout(&wgpu::BindGroupLayoutDescriptor {
        label: Some("text tex bgl"),
        entries: &[
            wgpu::BindGroupLayoutEntry {
                binding: 0,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Texture {
                    sample_type: wgpu::TextureSampleType::Float { filterable: true },
                    view_dimension: wgpu::TextureViewDimension::D2,
                    multisampled: false,
                },
                count: None,
            },
            wgpu::BindGroupLayoutEntry {
                binding: 1,
                visibility: wgpu::ShaderStages::FRAGMENT,
                ty: wgpu::BindingType::Sampler(wgpu::SamplerBindingType::Filtering),
                count: None,
            },
        ],
    });

    let ubuf = device.create_buffer(&wgpu::BufferDescriptor {
        label: Some("text uniform"),
        size: 16,
        usage: wgpu::BufferUsages::UNIFORM | wgpu::BufferUsages::COPY_DST,
        mapped_at_creation: false,
    });
    let ubg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("text ubg"),
        layout: &u_bgl,
        entries: &[wgpu::BindGroupEntry { binding: 0, resource: ubuf.as_entire_binding() }],
    });
    let tex_bg = device.create_bind_group(&wgpu::BindGroupDescriptor {
        label: Some("text tex bg"),
        layout: &tex_bgl,
        entries: &[
            wgpu::BindGroupEntry { binding: 0, resource: wgpu::BindingResource::TextureView(&view) },
            wgpu::BindGroupEntry { binding: 1, resource: wgpu::BindingResource::Sampler(&sampler) },
        ],
    });

    let shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
        label: Some("text shader"),
        source: wgpu::ShaderSource::Wgsl(std::borrow::Cow::Borrowed(include_str!("text.wgsl"))),
    });
    let layout = device.create_pipeline_layout(&wgpu::PipelineLayoutDescriptor {
        label: Some("text pl layout"),
        bind_group_layouts: &[&u_bgl, &tex_bgl],
        push_constant_ranges: &[],
    });
    let pipeline = device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
        label: Some("text pipeline"),
        layout: Some(&layout),
        vertex: wgpu::VertexState {
            module: &shader,
            entry_point: Some("vs_main"),
            compilation_options: Default::default(),
            buffers: &[wgpu::VertexBufferLayout {
                array_stride: std::mem::size_of::<TextVertex>() as u64,
                step_mode: wgpu::VertexStepMode::Vertex,
                attributes: &[
                    wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x2, offset: 0, shader_location: 0 },
                    wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x2, offset: 8, shader_location: 1 },
                    wgpu::VertexAttribute { format: wgpu::VertexFormat::Float32x3, offset: 16, shader_location: 2 },
                ],
            }],
        },
        fragment: Some(wgpu::FragmentState {
            module: &shader,
            entry_point: Some("fs_main"),
            compilation_options: Default::default(),
            targets: &[Some(wgpu::ColorTargetState {
                format,
                blend: Some(wgpu::BlendState::ALPHA_BLENDING),
                write_mask: wgpu::ColorWrites::ALL,
            })],
        }),
        primitive: wgpu::PrimitiveState {
            topology: wgpu::PrimitiveTopology::TriangleList,
            ..Default::default()
        },
        depth_stencil: None,
        multisample: wgpu::MultisampleState::default(),
        multiview: None,
        cache: None,
    });

    (pipeline, ubuf, ubg, tex_bg)
}

impl AppState {
    /// Tessellate one decoded tile's styled layers: polygons -> filled triangles
    /// (earcut), lines/polygons -> stroked segments. All in Mercator-relative
    /// f32 coordinates.
    fn build_mesh(&self, tile: Tile, vt: &basemap::VectorTile) -> TileMesh {
        let (ox, oy) = (self.cam.origin_x, self.cam.origin_y);
        let to_vertex = |p: [f32; 2], color: [f32; 3]| {
            let m = tile.mvt_to_mercator(p[0] as f64, p[1] as f64);
            Vertex { pos: [(m.x - ox) as f32, (m.y - oy) as f32], color }
        };

        let mut areas: Vec<Vertex> = Vec::new();
        let mut strokes: Vec<Vertex> = Vec::new();
        let mut tess = StrokeTessellator::new();

        for layer in &vt.layers {
            let Some(st) = style(&layer.name) else { continue };
            if !self.show_roads && layer.name == "roads" {
                continue;
            }
            match st {
                Style::Stroke(color, width) => {
                    for feat in &layer.features {
                        if feat.kind == GeomKind::Point {
                            continue;
                        }
                        for part in &feat.parts {
                            for p in stroke_to_tris(&mut tess, part, width) {
                                strokes.push(to_vertex(p, color));
                            }
                        }
                    }
                }
                Style::Fill(color) => {
                    for feat in &layer.features {
                        if feat.kind != GeomKind::Polygon {
                            continue;
                        }
                        // Group rings into polygons: a positive-area (exterior)
                        // ring starts a polygon; negative-area rings are its holes.
                        let mut polys: Vec<Vec<&[[f32; 2]]>> = Vec::new();
                        for ring in &feat.parts {
                            let r = open_ring(ring);
                            if r.len() < 3 {
                                continue;
                            }
                            if signed_area(r) > 0.0 {
                                polys.push(vec![r]);
                            } else if let Some(last) = polys.last_mut() {
                                last.push(r);
                            } else {
                                polys.push(vec![r]);
                            }
                        }
                        for poly in &polys {
                            let mut data: Vec<f32> = Vec::new();
                            let mut holes: Vec<usize> = Vec::new();
                            let mut flat: Vec<[f32; 2]> = Vec::new();
                            for (ri, ring) in poly.iter().enumerate() {
                                if ri > 0 {
                                    holes.push(flat.len());
                                }
                                for &p in ring.iter() {
                                    data.push(p[0]);
                                    data.push(p[1]);
                                    flat.push(p);
                                }
                            }
                            if let Ok(idx) = earcutr::earcut(&data, &holes, 2) {
                                for i in idx {
                                    areas.push(to_vertex(flat[i], color));
                                }
                            }
                        }
                    }
                }
            }
        }

        let mk = |verts: &[Vertex], label| -> (Option<wgpu::Buffer>, u32) {
            if verts.is_empty() {
                (None, 0)
            } else {
                let buf = self.gpu.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                    label: Some(label),
                    contents: bytemuck::cast_slice(verts),
                    usage: wgpu::BufferUsages::VERTEX,
                });
                (Some(buf), verts.len() as u32)
            }
        };
        // Place-name labels from the names layer (point features), as
        // Mercator-relative anchor points projected per frame at draw time.
        let mut labels: Vec<(String, String, [f32; 2])> = Vec::new();
        if let Some(nl) = vt.layer("names") {
            for feat in &nl.features {
                if feat.kind != GeomKind::Point {
                    continue;
                }
                if let (Some(name), Some(pt)) =
                    (feat.attr("name1"), feat.parts.first().and_then(|p| p.first()))
                {
                    let kind = feat.attr("type").unwrap_or("").to_string();
                    let m = tile.mvt_to_mercator(pt[0] as f64, pt[1] as f64);
                    labels.push((name.to_string(), kind, [(m.x - ox) as f32, (m.y - oy) as f32]));
                }
            }
        }

        let (area_buf, area_count) = mk(&areas, "tile areas");
        let (stroke_buf, stroke_count) = mk(&strokes, "tile strokes");
        TileMesh { area_buf, area_count, stroke_buf, stroke_count, labels }
    }

    /// Reset the pinch reference from the current touch set (called whenever a
    /// finger lands or lifts), so the next two-finger move has a clean baseline.
    fn refresh_pinch(&mut self) {
        if self.touches.len() == 2 {
            let pts: Vec<(f64, f64)> = self.touches.values().copied().collect();
            let mid = ((pts[0].0 + pts[1].0) * 0.5, (pts[0].1 + pts[1].1) * 0.5);
            let dist = (pts[0].0 - pts[1].0).hypot(pts[0].1 - pts[1].1);
            self.pinch_prev = Some((mid, dist));
        } else {
            self.pinch_prev = None;
        }
    }

    /// Load the OSM footpath overlay once: extract paths, project to
    /// Mercator-relative, and emit one screen-space-expansion quad per segment
    /// (constant pixel width at any zoom — the shader does the widening).
    fn ensure_paths_loaded(&mut self) {
        if self.paths_loaded {
            return;
        }
        self.paths_loaded = true;
        let t = std::time::Instant::now();
        let ways = match load_paths(&self.paths_pbf) {
            Ok(w) => w,
            Err(e) => {
                eprintln!("paths: load failed: {e}");
                return;
            }
        };
        let (ox, oy) = (self.cam.origin_x, self.cam.origin_y);
        let mut verts: Vec<PathVertex> = Vec::new();
        for w in &ways {
            let color = match w.kind {
                PathKind::Foot => [0.10, 0.45, 0.18],
                PathKind::Bridleway => [0.55, 0.25, 0.62],
                PathKind::Track => [0.45, 0.32, 0.16],
            };
            let merc: Vec<[f32; 2]> = w
                .points
                .iter()
                .map(|&(lat, lon)| {
                    let m = LatLon::new(lat, lon).to_mercator();
                    [(m.x - ox) as f32, (m.y - oy) as f32]
                })
                .collect();
            for s in merc.windows(2) {
                let (p0, p1) = (s[0], s[1]);
                let dir = [p1[0] - p0[0], p1[1] - p0[1]];
                let v = |pos, side| PathVertex { pos, dir, side, color };
                // two triangles forming the segment quad
                verts.push(v(p0, 1.0));
                verts.push(v(p0, -1.0));
                verts.push(v(p1, 1.0));
                verts.push(v(p1, 1.0));
                verts.push(v(p0, -1.0));
                verts.push(v(p1, -1.0));
            }
        }
        if !verts.is_empty() {
            self.path_buf = Some(self.gpu.device.create_buffer_init(
                &wgpu::util::BufferInitDescriptor {
                    label: Some("paths"),
                    contents: bytemuck::cast_slice(&verts),
                    usage: wgpu::BufferUsages::VERTEX,
                },
            ));
            self.path_count = verts.len() as u32;
        }
        println!(
            "paths: {} ways -> {} verts in {:.1}s",
            ways.len(),
            verts.len(),
            t.elapsed().as_secs_f32()
        );
    }

    /// Gather Zoomstack water within `radius_m` of `vp` as BNG geometry: lake
    /// polygons (`surfacewater`) and river centrelines (`waterlines`).
    fn gather_water_bng(&self, vp: LatLon, radius_m: f64) -> (Vec<Vec<[f64; 2]>>, Vec<Vec<[f64; 2]>>) {
        let z = 12u8;
        let dlat = radius_m / 111_320.0;
        let dlon = radius_m / (111_320.0 * vp.lat.to_radians().cos().abs().max(0.01));
        let t_nw = Tile::containing(LatLon::new(vp.lat + dlat, vp.lon - dlon), z);
        let t_se = Tile::containing(LatLon::new(vp.lat - dlat, vp.lon + dlon), z);
        let (mut polys, mut lines) = (Vec::new(), Vec::new());
        for ty in t_nw.y..=t_se.y {
            for tx in t_nw.x..=t_se.x {
                let tile = Tile::new(z, tx, ty);
                let Ok(Some(vt)) = self.mbt.decode_tile(tile) else { continue };
                let to_bng = |p: [f32; 2]| {
                    let m = tile.mvt_to_mercator(p[0] as f64, p[1] as f64);
                    let ll = m.to_latlon();
                    let (e, n) = geodesy::wgs84_to_bng(ll.lat, ll.lon);
                    [e, n]
                };
                if let Some(layer) = vt.layer("surfacewater") {
                    for f in &layer.features {
                        if f.kind == GeomKind::Polygon {
                            for ring in &f.parts {
                                if ring.len() >= 4 {
                                    polys.push(ring.iter().map(|&p| to_bng(p)).collect());
                                }
                            }
                        }
                    }
                }
                if let Some(layer) = vt.layer("waterlines") {
                    for f in &layer.features {
                        if f.kind == GeomKind::Line {
                            for part in &f.parts {
                                if part.len() >= 2 {
                                    lines.push(part.iter().map(|&p| to_bng(p)).collect());
                                }
                            }
                        }
                    }
                }
            }
        }
        (polys, lines)
    }

    /// Switch view mode, casting the horizon from the current map centre when
    /// entering a horizon view (panorama/compass) fresh from the map.
    fn set_mode(&mut self, target: Mode) {
        if target != Mode::Map && (self.mode == Mode::Map || self.pano.is_none()) {
            self.compute_pano();
        }
        if target == Mode::Map || self.pano.is_some() {
            self.mode = target;
        }
        self.window.request_redraw();
    }

    /// Cycle Map -> Panorama -> Compass -> Map (the on-screen button / no arg).
    fn cycle_mode(&mut self) {
        let next = match self.mode {
            Mode::Map => Mode::Panorama,
            Mode::Panorama => Mode::Compass,
            Mode::Compass => Mode::Map,
        };
        self.set_mode(next);
    }

    /// Cast the horizon + visible peaks/water from the current map centre into
    /// `self.pano`. DEM loads lazily on first use.
    fn compute_pano(&mut self) {
        let m = Mercator::new(
            self.cam.origin_x + self.cam.center_x,
            self.cam.origin_y + self.cam.center_y,
        )
        .to_latlon();
        let vp = LatLon::new(m.lat, m.lon);

        if self.dem.is_none() {
            println!("panorama: loading DEM (first use)...");
            match Dem::open(std::path::Path::new(DEM_DIR)) {
                Ok(d) => self.dem = Some(d),
                Err(e) => {
                    eprintln!("panorama: DEM load failed: {e}");
                    return;
                }
            }
        }
        let dem = self.dem.as_ref().unwrap();
        let params = HorizonParams::default();
        let Some(h) = cast(dem, vp, &params) else {
            eprintln!("panorama: ({:.4},{:.4}) is outside DEM coverage", vp.lat, vp.lon);
            return;
        };
        // Distance-scaled prominence cut: keep low-prominence fells when they're
        // near (e.g. Lingmell, a 72 m-drop shoulder, looms large from Wasdale),
        // but require real prominence for distant ones to avoid clutter.
        let mut peaks: Vec<PanoPeak> = visible_peaks(&h, vp, &self.peaks, &params)
            .into_iter()
            .filter(|v| v.peak.prominence_m >= 25.0 + 7.0 * (v.dist_m / 1000.0))
            .map(|v| PanoPeak {
                name: v.peak.name.clone(),
                bearing_deg: v.bearing_deg,
                elev_deg: v.elev_deg,
                dist_km: v.dist_m / 1000.0,
                obscured: v.visibility == Visibility::Slopes,
            })
            .collect();
        // Visible water: rasterise nearby lakes to a BNG mask, then cast. (Rivers
        // are deliberately NOT stamped — every Zoomstack beck floods the valleys.)
        let radius = 22_000.0;
        let (polys, _lines) = self.gather_water_bng(vp, radius);
        let (eye_e, eye_n) = geodesy::wgs84_to_bng(vp.lat, vp.lon);
        let cell = 50.0;
        let ncells = ((2.0 * radius) / cell) as usize;
        let mask =
            WaterMask::from_polygons(eye_e - radius, eye_n - radius, cell, ncells, ncells, &polys);
        let water = visible_water(dem, vp, &params, &mask);
        println!("panorama: {} lakes in range", polys.len());

        // Visible summits first, then nearer peaks, win the name-collision pass.
        peaks.sort_by(|a, b| {
            a.obscured
                .cmp(&b.obscured)
                .then(a.dist_km.partial_cmp(&b.dist_km).unwrap_or(std::cmp::Ordering::Equal))
        });
        let center_az = h.highest().0;
        println!(
            "panorama from ({:.4},{:.4}) ground {:.0} m: {} peaks visible",
            vp.lat,
            vp.lon,
            h.eye_ground_m,
            peaks.len()
        );
        self.pano = Some(PanoState {
            viewpoint: vp,
            ground_m: h.eye_ground_m,
            center_az,
            fov_deg: 90.0,
            edge_top: h
                .edges
                .iter()
                .map(|v| v.iter().map(|&(e, _)| e).fold(f32::NEG_INFINITY, f32::max))
                .collect(),
            edge_lines: h.edge_polylines(),
            water,
            skyline: h.elev_rad,
            peaks,
        });
    }

    /// Render the synthetic panorama: sky, terrain fill under the skyline, the
    /// skyline line, an eye-level reference, cardinal marks, and visible peaks
    /// (marker + name). All drawn through the text pipeline via the solid texel.
    fn render_panorama(&mut self) {
        let Some(pano) = self.pano.as_ref() else { return };
        let (vw, vh) = (self.cam.vw, self.cam.vh);
        let (vwf, vhf) = (vw as f32, vh as f32);
        let px_per_deg = vw / pano.fov_deg;
        let eye_y = (vh * 0.5) as f32; // 0 deg elevation sits mid-screen
        let suv = self.atlas.solid_uv();

        // Palette: natural (map-like) vs artist (pen-and-ink on paper).
        let artist = self.artist;
        let pal_sky = if artist {
            wgpu::Color { r: 0.93, g: 0.89, b: 0.80, a: 1.0 } // paper
        } else {
            wgpu::Color { r: 0.62, g: 0.74, b: 0.86, a: 1.0 } // sky blue
        };
        let pal_terrain = if artist { [0.86, 0.81, 0.71] } else { [0.34, 0.37, 0.31] };
        let pal_skyline = if artist { [0.16, 0.11, 0.05] } else { [0.12, 0.13, 0.13] };
        let pal_edge = if artist { [0.30, 0.22, 0.12] } else { [0.18, 0.18, 0.20] };
        let pal_eye = if artist { [0.62, 0.52, 0.36] } else { [0.5, 0.55, 0.6] };
        let pal_text = if artist { [0.20, 0.13, 0.06] } else { [0.1, 0.1, 0.1] };
        let pal_peak = if artist { [0.45, 0.20, 0.04] } else { PEAK_LABEL_COLOR };
        let pal_slope = if artist { [0.40, 0.30, 0.18] } else { SLOPE_LABEL_COLOR };
        let elev_to_y = |e_deg: f64| eye_y - (e_deg * px_per_deg) as f32;
        let az_to_x = |az: f64| {
            let d = (az - pano.center_az + 540.0).rem_euclid(360.0) - 180.0;
            (vw * 0.5 + d * px_per_deg) as f32
        };
        let mut tv: Vec<TextVertex> = Vec::new();
        let mut tri = |x0: f32, y0: f32, x1: f32, y1: f32, x2: f32, y2: f32, c: [f32; 3]| {
            tv.push(TextVertex { pos: [x0, y0], uv: suv, color: c });
            tv.push(TextVertex { pos: [x1, y1], uv: suv, color: c });
            tv.push(TextVertex { pos: [x2, y2], uv: suv, color: c });
        };

        let terrain = pal_terrain;
        let sky_line = pal_skyline;
        let half = pano.fov_deg * 0.5 + 2.0;
        let start = ((pano.center_az - half) * 10.0).floor() as i32;
        let end = ((pano.center_az + half) * 10.0).ceil() as i32;
        let buckets = horizon::AZIMUTH_BUCKETS as i32;
        let mut prev: Option<(f32, f32)> = None;
        for b in start..=end {
            let bucket = b.rem_euclid(buckets) as usize;
            let x = az_to_x(b as f64 * 0.1);
            let y = elev_to_y((pano.skyline[bucket] as f64).to_degrees());
            if let Some((px, py)) = prev {
                // terrain quad from the skyline down to the bottom of the screen
                tri(px, py, px, vhf, x, y, terrain);
                tri(x, y, px, vhf, x, vhf, terrain);
                // skyline line (vertical thickness)
                let t = 1.5;
                tri(px, py - t, px, py + t, x, y - t, sky_line);
                tri(x, y - t, px, py + t, x, y + t, sky_line);
            }
            prev = Some((x, y));
        }

        // Visible water surface (lakes) as a blue band sitting in the valley,
        // drawn over the terrain fill.
        if self.show_water {
            let water_c = if artist { [0.55, 0.63, 0.74] } else { [0.28, 0.50, 0.80] };
            let strip_w = (px_per_deg * 0.1) as f32 + 1.0; // one bucket wide, slight overlap
            for b in start..=end {
                let bucket = b.rem_euclid(buckets) as usize;
                if pano.water[bucket].is_empty() {
                    continue;
                }
                let x = az_to_x(b as f64 * 0.1);
                for &(wt, wb) in &pano.water[bucket] {
                    let yt = elev_to_y((wt as f64).to_degrees());
                    let yb = (elev_to_y((wb as f64).to_degrees())).max(yt + 1.0); // >=1px tall
                    // vertical strip [x, x+strip_w] x [yt, yb]
                    tri(x, yt, x, yb, x + strip_w, yt, water_c);
                    tri(x + strip_w, yt, x, yb, x + strip_w, yb, water_c);
                }
            }
        }

        // Eye-level (0 deg) reference line.
        let y0 = elev_to_y(0.0);
        tri(0.0, y0 - 0.6, 0.0, y0 + 0.6, vwf, y0 - 0.6, pal_eye);
        tri(vwf, y0 - 0.6, 0.0, y0 + 0.6, vwf, y0 + 0.6, pal_eye);

        // Artist view: hatch a fringe of short, length-varied ink strokes hanging
        // from the skyline — the shaded near-slope, hand-drawn feel.
        if artist {
            let hatch_c = [0.42, 0.34, 0.22];
            let mut b = start;
            while b <= end {
                let x = az_to_x(b as f64 * 0.1);
                if x >= -2.0 && x <= vwf + 2.0 {
                    let bucket = b.rem_euclid(buckets) as usize;
                    let sy = elev_to_y((pano.skyline[bucket] as f64).to_degrees());
                    let mut len = 5.0 + jitter(b as u32) * 10.0;
                    // If a ridge line sits close below the skyline here, pull the
                    // hatch up to stop above it so the line stays clean.
                    let te = pano.edge_top[bucket];
                    if te.is_finite() {
                        let edge_y = elev_to_y((te as f64).to_degrees());
                        let allowed = (edge_y - 3.0) - (sy + 1.0);
                        if allowed < len {
                            len = allowed;
                        }
                    }
                    if len > 1.0 {
                        let xw = 0.5;
                        tri(x - xw, sy + 1.0, x - xw, sy + 1.0 + len, x + xw, sy + 1.0, hatch_c);
                        tri(x + xw, sy + 1.0, x - xw, sy + 1.0 + len, x + xw, sy + 1.0 + len, hatch_c);
                    }
                }
                b += 4; // ~every 0.4 deg
            }
        }

        // Wainwright-style occlusion edges, linked into lines: where a near ridge
        // ends and a farther fell shows behind it, traced across azimuths.
        if self.show_edges {
            let edge_c = pal_edge;
            for line in &pano.edge_lines {
                for w in line.windows(2) {
                    let x0 = az_to_x(w[0].0 as f64);
                    let x1 = az_to_x(w[1].0 as f64);
                    if (x1 - x0).abs() > vwf {
                        continue; // crosses the ±180° seam
                    }
                    if x0.max(x1) < -2.0 || x0.min(x1) > vwf + 2.0 {
                        continue; // off-screen
                    }
                    let y0 = elev_to_y((w[0].1 as f64).to_degrees());
                    let y1 = elev_to_y((w[1].1 as f64).to_degrees());
                    let t = 0.8;
                    tri(x0, y0 - t, x0, y0 + t, x1, y1 - t, edge_c);
                    tri(x1, y1 - t, x0, y0 + t, x1, y1 + t, edge_c);
                }
            }
        }

        // Cardinal marks along the bottom.
        for (az, lbl) in [
            (0.0, "N"), (45.0, "NE"), (90.0, "E"), (135.0, "SE"),
            (180.0, "S"), (225.0, "SW"), (270.0, "W"), (315.0, "NW"),
        ] {
            let x = az_to_x(az);
            if x < -20.0 || x > vwf + 20.0 {
                continue;
            }
            let w = self.atlas.measure(lbl);
            self.atlas.layout(lbl, x - w * 0.5, vhf - 8.0, pal_text, &mut tv);
        }

        // Visible peaks: marker at the summit point, name above (nearer wins).
        let mut placed: Vec<(f32, f32)> = Vec::new();
        for pk in &pano.peaks {
            let x = az_to_x(pk.bearing_deg);
            if x < 0.0 || x > vwf {
                continue;
            }
            let y = elev_to_y(pk.elev_deg);
            let color = if pk.obscured { pal_slope } else { pal_peak };
            self.atlas.marker(x, y, 4.0, color, &mut tv);
            if self.show_peak_names {
                let w = self.atlas.measure(&pk.name);
                let hw = w * 0.5;
                if placed.iter().any(|(qx, qhw)| (qx - x).abs() < qhw + hw) {
                    continue;
                }
                placed.push((x, hw));
                self.atlas.layout(&pk.name, x - hw, y - 8.0, color, &mut tv);
            }
        }

        // HUD hint.
        let hud = format!(
            "PANORAMA {} ({:.4}, {:.4})  ground {:.0} m   (V:map  drag:scrub  wheel:zoom  N:names  E:edges  W:water  A:style)",
            if artist { "[artist]" } else { "[natural]" },
            pano.viewpoint.lat,
            pano.viewpoint.lon,
            pano.ground_m
        );
        self.atlas.layout(&hud, 8.0, 22.0, pal_text, &mut tv);
        self.draw_mode_button(&mut tv);

        // Upload + draw through the text pipeline; clear to a sky gradient base.
        let tu = [vwf, vhf, 0.0, 0.0];
        self.gpu.queue.write_buffer(&self.text_ubuf, 0, bytemuck::cast_slice(&tu));
        let buf = self.gpu.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("panorama"),
            contents: bytemuck::cast_slice(&tv),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let frame = match self.gpu.surface.get_current_texture() {
            Ok(f) => f,
            Err(_) => {
                self.gpu.surface.configure(&self.gpu.device, &self.gpu.config);
                return;
            }
        };
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut enc = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("pano enc") });
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("pano pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(pal_sky),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            if !tv.is_empty() {
                pass.set_pipeline(&self.text_pipeline);
                pass.set_bind_group(0, &self.text_ubg, &[]);
                pass.set_bind_group(1, &self.text_tex_bg, &[]);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..tv.len() as u32, 0..1);
            }
        }
        self.gpu.queue.submit(Some(enc.finish()));
        frame.present();
    }

    /// Compass / plan view: you at the centre, each visible fell on its bearing
    /// (north up, clockwise) at a radius set by its distance in miles, with an
    /// arm and a labelled dot. Distance rings in miles.
    fn render_compass(&mut self) {
        let Some(pano) = self.pano.as_ref() else { return };
        let (vw, vh) = (self.cam.vw, self.cam.vh);
        let (vwf, vhf) = (vw as f32, vh as f32);
        let (cx, cy) = (vwf * 0.5, vhf * 0.5);
        let uv = self.atlas.solid_uv();
        let artist = self.artist;

        let bg = if artist {
            wgpu::Color { r: 0.93, g: 0.89, b: 0.80, a: 1.0 }
        } else {
            wgpu::Color { r: 0.96, g: 0.96, b: 0.95, a: 1.0 }
        };
        let ink = if artist { [0.20, 0.13, 0.06] } else { [0.15, 0.15, 0.18] };
        let ring_c = if artist { [0.62, 0.54, 0.40] } else { [0.70, 0.72, 0.76] };
        let arm_c = if artist { [0.55, 0.47, 0.34] } else { [0.62, 0.66, 0.72] };
        let peak_c = if artist { [0.45, 0.20, 0.04] } else { PEAK_LABEL_COLOR };
        let slope_c = if artist { [0.40, 0.30, 0.18] } else { SLOPE_LABEL_COLOR };

        let r_px = (vw.min(vh) as f32) * 0.44;
        let max_mi = pano
            .peaks
            .iter()
            .map(|p| p.dist_km * 0.621_371)
            .fold(1.0_f64, f64::max)
            .min(25.0) as f32;
        let scale = r_px / max_mi.max(1.0);

        let mut tv: Vec<TextVertex> = Vec::new();

        // Distance rings (miles) + ring labels.
        for &mi in &[1.0f32, 2.0, 3.0, 5.0, 10.0, 15.0, 20.0, 25.0] {
            if mi > max_mi + 0.01 {
                break;
            }
            push_circle(&mut tv, uv, cx, cy, mi * scale, 0.8, ring_c);
        }
        // Cardinal radials + letters (north up, clockwise).
        for (deg, lbl) in [(0.0f32, "N"), (90.0, "E"), (180.0, "S"), (270.0, "W")] {
            let a = deg.to_radians();
            let (ex, ey) = (cx + r_px * a.sin(), cy - r_px * a.cos());
            push_seg(&mut tv, uv, cx, cy, ex, ey, 0.5, ring_c);
            let (lx, ly) = (cx + (r_px + 14.0) * a.sin(), cy - (r_px + 14.0) * a.cos());
            let w = self.atlas.measure(lbl);
            self.atlas.layout(lbl, lx - w * 0.5, ly + 5.0, ink, &mut tv);
        }

        // Arms + dots to each visible fell (peaks are pre-sorted: summits then
        // nearer first, so they win the label-collision pass).
        let mut placed: Vec<[f32; 4]> = Vec::new();
        let h = self.atlas.px;
        for pk in &pano.peaks {
            let mi = (pk.dist_km * 0.621_371) as f32;
            let r = (mi * scale).min(r_px);
            let a = (pk.bearing_deg as f32).to_radians();
            let (px, py) = (cx + r * a.sin(), cy - r * a.cos());
            let c = if pk.obscured { slope_c } else { peak_c };
            push_seg(&mut tv, uv, cx, cy, px, py, 0.6, arm_c);
            self.atlas.rect(px - 2.0, py - 2.0, px + 2.0, py + 2.0, c, &mut tv);

            // Label just outside the dot, pushed radially outward; collision-skip.
            let label = format!("{} {:.1}mi", pk.name, mi);
            let lw = self.atlas.measure(&label);
            let outward = if a.sin() >= 0.0 { 6.0 } else { -6.0 - lw };
            let lx = px + outward;
            let ly = py + 4.0;
            let box_ = [lx, ly - h, lx + lw, ly];
            if placed.iter().any(|b| lx < b[2] && box_[2] > b[0] && box_[1] < b[3] && ly > b[1]) {
                continue;
            }
            placed.push(box_);
            self.atlas.layout(&label, lx, ly, c, &mut tv);
        }
        // You-are-here marker.
        self.atlas.rect(cx - 3.0, cy - 3.0, cx + 3.0, cy + 3.0, ink, &mut tv);

        self.atlas.layout(
            &format!(
                "COMPASS  ({:.4}, {:.4})  ground {:.0} m   (rings = miles, N up;  C/V/button: views)",
                pano.viewpoint.lat, pano.viewpoint.lon, pano.ground_m
            ),
            8.0,
            22.0,
            ink,
            &mut tv,
        );
        self.draw_mode_button(&mut tv);

        // Upload + draw through the text pipeline.
        let tu = [vwf, vhf, 0.0, 0.0];
        self.gpu.queue.write_buffer(&self.text_ubuf, 0, bytemuck::cast_slice(&tu));
        let buf = self.gpu.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
            label: Some("compass"),
            contents: bytemuck::cast_slice(&tv),
            usage: wgpu::BufferUsages::VERTEX,
        });
        let frame = match self.gpu.surface.get_current_texture() {
            Ok(f) => f,
            Err(_) => {
                self.gpu.surface.configure(&self.gpu.device, &self.gpu.config);
                return;
            }
        };
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut enc = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("compass enc") });
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("compass pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations { load: wgpu::LoadOp::Clear(bg), store: wgpu::StoreOp::Store },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            if !tv.is_empty() {
                pass.set_pipeline(&self.text_pipeline);
                pass.set_bind_group(0, &self.text_ubg, &[]);
                pass.set_bind_group(1, &self.text_tex_bg, &[]);
                pass.set_vertex_buffer(0, buf.slice(..));
                pass.draw(0..tv.len() as u32, 0..1);
            }
        }
        self.gpu.queue.submit(Some(enc.finish()));
        frame.present();
    }

    /// Draw the bottom-right key legend onto the text vertex list: a dark panel
    /// plus one line per toggle, white when on / grey when off.
    fn draw_legend(&self, tv: &mut Vec<TextVertex>) {
        let lines: [(&str, bool); 5] = [
            ("R  roads", self.show_roads),
            ("P  paths", self.show_paths),
            ("L  labels", self.show_labels),
            ("K  peaks", self.show_peaks),
            ("N  peak names", self.show_peak_names),
        ];
        let (line_h, pad, margin) = (18.0f32, 7.0f32, 8.0f32);
        let text_w = lines.iter().map(|(t, _)| self.atlas.measure(t)).fold(0.0f32, f32::max);
        let panel_w = text_w + pad * 2.0;
        let panel_h = line_h * lines.len() as f32 + pad * 2.0;
        // Bottom-right corner.
        let x1 = self.cam.vw as f32 - margin;
        let y1 = self.cam.vh as f32 - margin;
        let (x0, y0) = (x1 - panel_w, y1 - panel_h);
        self.atlas.rect(x0, y0, x1, y1, [0.08, 0.08, 0.10], tv);
        for (i, (t, on)) in lines.iter().enumerate() {
            let baseline = y0 + pad + line_h * i as f32 + 13.0;
            let color = if *on { [0.95, 0.95, 0.95] } else { [0.45, 0.45, 0.45] };
            self.atlas.layout(t, x0 + pad, baseline, color, tv);
        }
    }

    /// Bottom-left mode-toggle button: returns its screen rect and label.
    fn mode_button(&self) -> ([f32; 4], &'static str) {
        // Shows the next mode in the cycle (what a tap switches to).
        let label = match self.mode {
            Mode::Map => "PANORAMA",
            Mode::Panorama => "COMPASS",
            Mode::Compass => "MAP",
        };
        let w = self.atlas.measure(label) + 20.0;
        let h = 32.0;
        let (x0, y1) = (10.0, self.cam.vh as f32 - 10.0);
        ([x0, y1 - h, x0 + w, y1], label)
    }

    fn draw_mode_button(&self, tv: &mut Vec<TextVertex>) {
        let ([x0, y0, x1, y1], label) = self.mode_button();
        self.atlas.rect(x0, y0, x1, y1, [0.08, 0.08, 0.10], tv);
        let tw = self.atlas.measure(label);
        self.atlas.layout(label, x0 + ((x1 - x0) - tw) * 0.5, y0 + 21.0, [0.95, 0.95, 0.95], tv);
    }

    fn hit_mode_button(&self, x: f64, y: f64) -> bool {
        let ([x0, y0, x1, y1], _) = self.mode_button();
        (x as f32) >= x0 && (x as f32) <= x1 && (y as f32) >= y0 && (y as f32) <= y1
    }

    fn toggle_mode(&mut self) {
        self.cycle_mode();
    }

    fn render(&mut self) {
        if self.mode == Mode::Panorama {
            return self.render_panorama();
        }
        if self.mode == Mode::Compass {
            return self.render_compass();
        }
        let z = self.cam.slippy_zoom(self.maxzoom);
        let ((tx0, ty0), (tx1, ty1)) = self.cam.visible_tiles(z);

        // Centre-out order so the middle of the view fills first.
        let cx = (tx0 + tx1) as f64 * 0.5;
        let cy = (ty0 + ty1) as f64 * 0.5;
        let mut want: Vec<Tile> = Vec::new();
        for ty in ty0..=ty1 {
            for tx in tx0..=tx1 {
                want.push(Tile::new(z, tx, ty));
            }
        }
        want.sort_by(|a, b| {
            let da = (a.x as f64 - cx).powi(2) + (a.y as f64 - cy).powi(2);
            let db = (b.x as f64 - cx).powi(2) + (b.y as f64 - cy).powi(2);
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        });

        // Decode up to the per-frame budget of not-yet-cached tiles.
        let mut loaded = 0usize;
        let mut more = false;
        for &t in &want {
            if self.tiles.contains_key(&t) {
                continue;
            }
            if loaded >= TILE_BUDGET_PER_FRAME {
                more = true;
                break;
            }
            let mesh = match self.mbt.decode_tile(t) {
                Ok(Some(vt)) => self.build_mesh(t, &vt),
                Ok(None) => TileMesh { area_buf: None, area_count: 0, stroke_buf: None, stroke_count: 0, labels: Vec::new() },
                Err(e) => {
                    eprintln!("decode {:?} failed: {e}", t);
                    TileMesh { area_buf: None, area_count: 0, stroke_buf: None, stroke_count: 0, labels: Vec::new() }
                }
            };
            self.tiles.insert(t, mesh);
            loaded += 1;
        }

        let vu = self.cam.uniform();
        self.gpu.queue.write_buffer(&self.gpu.view_buf, 0, bytemuck::bytes_of(&vu));
        if self.show_paths {
            // Match the roads' current on-screen width: a tile of 4096 units
            // spans `tile_span * zoom` px, so a road of ROAD_WIDTH_TILE units is
            // this many px wide right now.
            let tile_span = 2.0 * MERCATOR_MAX / ((1u32 << z) as f64);
            let road_px = ROAD_WIDTH_TILE as f64 * tile_span * self.cam.zoom
                / geodesy::TILE_EXTENT as f64;
            let pu = PathUniform {
                x_scale: vu.x_scale,
                x_offset: vu.x_offset,
                y_scale: vu.y_scale,
                y_offset: vu.y_offset,
                vw: self.cam.vw as f32,
                vh: self.cam.vh as f32,
                half_px: (road_px * 0.5) as f32,
                _pad: 0.0,
            };
            self.gpu.queue.write_buffer(&self.gpu.path_ubuf, 0, bytemuck::bytes_of(&pu));
        }

        // Build screen-space label geometry for visible tiles (deduped by name).
        let mut label_buf: Option<wgpu::Buffer> = None;
        let mut label_count = 0u32;
        let mut tv: Vec<TextVertex> = Vec::new();
        if self.show_labels || self.show_peaks {
            // Gather on-screen candidates passing the per-type zoom threshold.
            struct Cand {
                pr: u8,
                text: String,
                color: [f32; 3],
                marker: bool, // peak: draw a summit triangle and offset text up
                sx: f64,
                sy: f64,
                w: f64,
            }
            let mut cands: Vec<Cand> = Vec::new();
            let project = |mx: f64, my: f64| {
                (
                    (mx - self.cam.center_x) * self.cam.zoom + self.cam.vw * 0.5,
                    self.cam.vh * 0.5 - (my - self.cam.center_y) * self.cam.zoom,
                )
            };
            let on_screen = |sx: f64, sy: f64| {
                sx > -100.0 && sx < self.cam.vw + 100.0 && sy > -20.0 && sy < self.cam.vh + 20.0
            };
            if self.show_labels {
                for t in &want {
                    let Some(m) = self.tiles.get(t) else { continue };
                    for (name, kind, mp) in &m.labels {
                        if z < label_min_zoom(kind) {
                            continue;
                        }
                        let (sx, sy) = project(mp[0] as f64, mp[1] as f64);
                        if !on_screen(sx, sy) {
                            continue;
                        }
                        cands.push(Cand {
                            pr: label_priority(kind),
                            color: PLACE_LABEL_COLOR,
                            marker: false,
                            sx,
                            sy,
                            w: self.atlas.measure(name) as f64,
                            text: name.clone(),
                        });
                    }
                }
            }
            if self.show_peaks {
                // Peaks in the visible lon/lat box, filtered by prominence vs zoom.
                let (ax, ay) = self.cam.screen_to_merc(0.0, 0.0);
                let (bx, by) = self.cam.screen_to_merc(self.cam.vw, self.cam.vh);
                let nw = Mercator::new(ax.min(bx), ay.max(by)).to_latlon();
                let se = Mercator::new(ax.max(bx), ay.min(by)).to_latlon();
                let (ox, oy) = (self.cam.origin_x, self.cam.origin_y);
                for pk in self.peaks.in_bbox(nw.lon, se.lat, se.lon, nw.lat) {
                    if z < peak_min_zoom(pk.prominence_m) {
                        continue;
                    }
                    let m = LatLon::new(pk.lat, pk.lon).to_mercator();
                    let (sx, sy) = project(m.x - ox, m.y - oy);
                    if !on_screen(sx, sy) {
                        continue;
                    }
                    // Always show the height; the name is toggleable (N).
                    let h_m = pk.height_m.round() as i32;
                    let text = if self.show_peak_names {
                        format!("{} {}m", pk.name, h_m)
                    } else {
                        format!("{}m", h_m)
                    };
                    let w = self.atlas.measure(&text) as f64;
                    cands.push(Cand {
                        pr: peak_priority(pk.prominence_m),
                        color: PEAK_LABEL_COLOR,
                        marker: true,
                        sx,
                        sy,
                        w,
                        text,
                    });
                }
            }
            // Peaks first (so summits win collisions over place-names), then by
            // priority, then a stable key so the same labels win frame to frame.
            cands.sort_by(|a, b| {
                b.marker
                    .cmp(&a.marker) // marker=true (peaks) sort first
                    .then(a.pr.cmp(&b.pr))
                    .then(a.text.cmp(&b.text))
                    .then(a.sx.partial_cmp(&b.sx).unwrap_or(std::cmp::Ordering::Equal))
            });
            // Greedy collision: place a label only if its box is clear. This also
            // collapses the duplicate label points Zoomstack repeats per tile.
            let h = self.atlas.px as f64;
            let mut boxes: Vec<[f64; 4]> = Vec::new();
            for c in &cands {
                // Peaks put their text above the summit marker; place-names sit on
                // the point. Collision box covers the text (and the marker below).
                let baseline = if c.marker { c.sy - 9.0 } else { c.sy };
                let box_cy = if c.marker { c.sy - 4.0 } else { c.sy };
                let box_h = if c.marker { h + 12.0 } else { h };
                let (l, t, r, b) =
                    (c.sx - c.w * 0.5, box_cy - box_h * 0.5, c.sx + c.w * 0.5, box_cy + box_h * 0.5);
                if boxes.iter().any(|x| l < x[2] && r > x[0] && t < x[3] && b > x[1]) {
                    continue;
                }
                boxes.push([l, t, r, b]);
                if c.marker {
                    self.atlas.marker(c.sx as f32, c.sy as f32, 5.0, c.color, &mut tv);
                }
                self.atlas.layout(&c.text, (c.sx - c.w * 0.5) as f32, baseline as f32, c.color, &mut tv);
            }
        }

        // Always-on bottom-right key legend (drawn through the text pipeline).
        self.draw_legend(&mut tv);

        // POV crosshair at screen centre — the viewpoint the panorama (V) casts from.
        {
            let (cx, cy) = (self.cam.vw as f32 * 0.5, self.cam.vh as f32 * 0.5);
            let red = [0.85, 0.12, 0.12];
            self.atlas.rect(cx - 9.0, cy - 1.5, cx + 9.0, cy + 1.5, red, &mut tv);
            self.atlas.rect(cx - 1.5, cy - 9.0, cx + 1.5, cy + 9.0, red, &mut tv);
        }
        self.draw_mode_button(&mut tv);

        if !tv.is_empty() {
            label_buf = Some(self.gpu.device.create_buffer_init(&wgpu::util::BufferInitDescriptor {
                label: Some("labels+hud"),
                contents: bytemuck::cast_slice(&tv),
                usage: wgpu::BufferUsages::VERTEX,
            }));
            label_count = tv.len() as u32;
            let tu = [self.cam.vw as f32, self.cam.vh as f32, 0.0, 0.0];
            self.gpu.queue.write_buffer(&self.text_ubuf, 0, bytemuck::cast_slice(&tu));
        }

        let frame = match self.gpu.surface.get_current_texture() {
            Ok(f) => f,
            Err(_) => {
                self.gpu.surface.configure(&self.gpu.device, &self.gpu.config);
                return;
            }
        };
        let view = frame.texture.create_view(&wgpu::TextureViewDescriptor::default());
        let mut enc = self
            .gpu
            .device
            .create_command_encoder(&wgpu::CommandEncoderDescriptor { label: Some("enc") });
        {
            let mut pass = enc.begin_render_pass(&wgpu::RenderPassDescriptor {
                label: Some("map pass"),
                color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                    view: &view,
                    resolve_target: None,
                    ops: wgpu::Operations {
                        load: wgpu::LoadOp::Clear(wgpu::Color {
                            r: 0.93,
                            g: 0.93,
                            b: 0.90,
                            a: 1.0,
                        }),
                        store: wgpu::StoreOp::Store,
                    },
                })],
                depth_stencil_attachment: None,
                timestamp_writes: None,
                occlusion_query_set: None,
            });
            pass.set_bind_group(0, &self.gpu.view_bg, &[]);
            pass.set_pipeline(&self.gpu.pipeline);
            // Area fills underneath...
            for &t in &want {
                if let Some(TileMesh { area_buf: Some(buf), area_count, .. }) = self.tiles.get(&t) {
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..*area_count, 0..1);
                }
            }
            // ...stroked lines on top.
            for &t in &want {
                if let Some(TileMesh { stroke_buf: Some(buf), stroke_count, .. }) = self.tiles.get(&t) {
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..*stroke_count, 0..1);
                }
            }
            // OSM footpaths overlay, above everything, via the screen-space
            // expansion pipeline (constant pixel width at any zoom).
            if self.show_paths {
                if let Some(buf) = &self.path_buf {
                    if self.path_count > 0 {
                        pass.set_pipeline(&self.gpu.path_pipeline);
                        pass.set_bind_group(0, &self.gpu.path_bg, &[]);
                        pass.set_vertex_buffer(0, buf.slice(..));
                        pass.draw(0..self.path_count, 0..1);
                    }
                }
            }
            // Place-name labels, screen-aligned, on top of everything.
            if let Some(buf) = &label_buf {
                if label_count > 0 {
                    pass.set_pipeline(&self.text_pipeline);
                    pass.set_bind_group(0, &self.text_ubg, &[]);
                    pass.set_bind_group(1, &self.text_tex_bg, &[]);
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..label_count, 0..1);
                }
            }
        }
        self.gpu.queue.submit(Some(enc.finish()));
        frame.present();

        if more {
            self.window.request_redraw();
        }
    }
}

struct App {
    path: std::path::PathBuf,
    paths_pbf: std::path::PathBuf,
    peaks_csv: std::path::PathBuf,
    at: LatLon,
    state: Option<AppState>,
}

impl ApplicationHandler for App {
    fn resumed(&mut self, event_loop: &ActiveEventLoop) {
        if self.state.is_some() {
            return;
        }
        let mbt = Mbtiles::open(&self.path).expect("open mbtiles");
        let maxzoom = mbt.metadata().ok().and_then(|m| m.maxzoom).unwrap_or(MAX_SLIPPY_ZOOM);
        println!("opened {} (maxzoom {maxzoom})", self.path.display());

        let attrs = Window::default_attributes()
            .with_title("airhorizon-view — Zoomstack")
            .with_inner_size(PhysicalSize::new(1280, 800));
        let window = Arc::new(event_loop.create_window(attrs).expect("window"));
        let gpu = pollster::block_on(Gpu::new(window.clone()));
        let size = window.inner_size();
        let cam = Camera::new(self.at, size.width as f64, size.height as f64);

        // Build the label font atlas from a system font.
        let font_bytes = std::fs::read(r"C:\Windows\Fonts\segoeui.ttf")
            .or_else(|_| std::fs::read(r"C:\Windows\Fonts\arial.ttf"))
            .expect("load a system font");
        let atlas = FontAtlas::build(&font_bytes, 18.0);
        let (text_pipeline, text_ubuf, text_ubg, text_tex_bg) =
            build_text(&gpu.device, &gpu.queue, gpu.config.format, &atlas);

        let peaks = match Peaks::load_csv(&self.peaks_csv) {
            Ok(p) => {
                println!("loaded {} peaks", p.len());
                p
            }
            Err(e) => {
                eprintln!("peaks: load failed ({e}); continuing without");
                Peaks::empty()
            }
        };

        self.state = Some(AppState {
            window,
            gpu,
            mbt,
            maxzoom,
            cam,
            tiles: HashMap::new(),
            cursor: None,
            dragging: false,
            last_cursor: None,
            touches: HashMap::new(),
            pinch_prev: None,
            show_roads: false,
            paths_pbf: self.paths_pbf.clone(),
            paths_loaded: false,
            show_paths: false,
            path_buf: None,
            path_count: 0,
            atlas,
            show_labels: false,
            text_pipeline,
            text_ubuf,
            text_ubg,
            text_tex_bg,
            peaks,
            show_peaks: true,
            show_peak_names: true,
            mode: Mode::Map,
            dem: None,
            pano: None,
            show_edges: true,
            show_water: true,
            artist: false,
        });
        self.state.as_ref().unwrap().window.request_redraw();
    }

    fn window_event(&mut self, event_loop: &ActiveEventLoop, _id: WindowId, event: WindowEvent) {
        let Some(s) = self.state.as_mut() else { return };
        match event {
            WindowEvent::CloseRequested => event_loop.exit(),
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::Escape) =>
            {
                // Esc returns to the map first, otherwise quits.
                if s.mode != Mode::Map {
                    s.set_mode(Mode::Map);
                } else {
                    event_loop.exit();
                }
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyV) =>
            {
                s.set_mode(if s.mode == Mode::Panorama { Mode::Map } else { Mode::Panorama });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyC) =>
            {
                s.set_mode(if s.mode == Mode::Compass { Mode::Map } else { Mode::Compass });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyE) =>
            {
                s.show_edges = !s.show_edges;
                s.window.request_redraw();
                println!("ridge edges: {}", if s.show_edges { "on" } else { "off" });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyA) =>
            {
                s.artist = !s.artist;
                s.window.request_redraw();
                println!("panorama style: {}", if s.artist { "artist" } else { "natural" });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyW) =>
            {
                s.show_water = !s.show_water;
                s.window.request_redraw();
                println!("water: {}", if s.show_water { "on" } else { "off" });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyR) =>
            {
                s.show_roads = !s.show_roads;
                s.tiles.clear(); // rebuild meshes with/without the roads layer
                s.window.request_redraw();
                println!("roads: {}", if s.show_roads { "on" } else { "off" });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyP) =>
            {
                if !s.paths_loaded {
                    println!("paths: loading...");
                    s.ensure_paths_loaded();
                    s.show_paths = true;
                } else {
                    s.show_paths = !s.show_paths;
                }
                s.window.request_redraw();
                println!("paths: {}", if s.show_paths { "on" } else { "off" });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyL) =>
            {
                s.show_labels = !s.show_labels;
                s.window.request_redraw();
                println!("labels: {}", if s.show_labels { "on" } else { "off" });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyK) =>
            {
                s.show_peaks = !s.show_peaks;
                s.window.request_redraw();
                println!("peaks: {}", if s.show_peaks { "on" } else { "off" });
            }
            WindowEvent::KeyboardInput { event, .. }
                if event.state == ElementState::Pressed
                    && event.physical_key == PhysicalKey::Code(KeyCode::KeyN) =>
            {
                s.show_peak_names = !s.show_peak_names;
                s.window.request_redraw();
                println!("peak names: {}", if s.show_peak_names { "on" } else { "off" });
            }
            WindowEvent::Resized(new) => {
                s.gpu.resize(new.width, new.height);
                s.cam.vw = new.width as f64;
                s.cam.vh = new.height as f64;
                s.window.request_redraw();
            }
            WindowEvent::MouseInput { state, button: MouseButton::Left, .. } => {
                // A click on the mode button toggles map/panorama.
                if state == ElementState::Pressed {
                    if let Some((cx, cy)) = s.cursor {
                        if s.hit_mode_button(cx, cy) {
                            s.toggle_mode();
                            return;
                        }
                    }
                }
                // Ignore mouse press while a finger is down (touch synthesises it).
                s.dragging = state == ElementState::Pressed && s.touches.is_empty();
                s.last_cursor = s.cursor;
            }
            WindowEvent::CursorMoved { position, .. } => {
                let pos = (position.x, position.y);
                s.cursor = Some(pos);
                if s.dragging && s.touches.is_empty() {
                    if let Some((lx, ly)) = s.last_cursor {
                        match s.mode {
                            Mode::Map => s.cam.pan_screen(pos.0 - lx, pos.1 - ly),
                            Mode::Panorama => {
                                if let Some(p) = s.pano.as_mut() {
                                    // Drag right -> look left (azimuth decreases).
                                    p.center_az =
                                        (p.center_az - (pos.0 - lx) * p.fov_deg / s.cam.vw).rem_euclid(360.0);
                                }
                            }
                            Mode::Compass => {}
                        }
                        s.window.request_redraw();
                    }
                }
                s.last_cursor = Some(pos);
            }
            WindowEvent::Touch(Touch { id, phase, location, .. }) => {
                let pos = (location.x, location.y);
                match phase {
                    TouchPhase::Started => {
                        // A tap on the mode button toggles map/panorama.
                        if s.touches.is_empty() && s.hit_mode_button(pos.0, pos.1) {
                            s.toggle_mode();
                            return;
                        }
                        s.dragging = false; // a touch cancels any mouse drag
                        s.touches.insert(id, pos);
                        s.refresh_pinch();
                    }
                    TouchPhase::Moved => {
                        let prev = s.touches.insert(id, pos);
                        match s.touches.len() {
                            1 => {
                                if let Some((px, py)) = prev {
                                    let (dx, dy) = (pos.0 - px, pos.1 - py);
                                    match s.mode {
                                        Mode::Map => s.cam.pan_screen(dx, dy),
                                        Mode::Panorama => {
                                            if let Some(p) = s.pano.as_mut() {
                                                p.center_az = (p.center_az - dx * p.fov_deg / s.cam.vw)
                                                    .rem_euclid(360.0);
                                            }
                                        }
                                        Mode::Compass => {}
                                    }
                                    s.window.request_redraw();
                                }
                            }
                            2 => {
                                let pts: Vec<(f64, f64)> = s.touches.values().copied().collect();
                                let mid =
                                    ((pts[0].0 + pts[1].0) * 0.5, (pts[0].1 + pts[1].1) * 0.5);
                                let dist = (pts[0].0 - pts[1].0).hypot(pts[0].1 - pts[1].1);
                                if let Some((pmid, pdist)) = s.pinch_prev {
                                    match s.mode {
                                        Mode::Map => {
                                            // Zoom about the midpoint by the finger-spread
                                            // ratio, and pan by the midpoint's movement.
                                            s.cam.zoom_about(dist / pdist.max(1.0), mid.0, mid.1);
                                            s.cam.pan_screen(mid.0 - pmid.0, mid.1 - pmid.1);
                                        }
                                        Mode::Panorama => {
                                            if let Some(p) = s.pano.as_mut() {
                                                // Pinch apart -> narrower FOV (zoom in);
                                                // midpoint drag scrubs azimuth.
                                                p.fov_deg = (p.fov_deg * pdist.max(1.0) / dist.max(1.0))
                                                    .clamp(20.0, 160.0);
                                                p.center_az = (p.center_az
                                                    - (mid.0 - pmid.0) * p.fov_deg / s.cam.vw)
                                                    .rem_euclid(360.0);
                                            }
                                        }
                                        Mode::Compass => {}
                                    }
                                    s.window.request_redraw();
                                }
                                s.pinch_prev = Some((mid, dist));
                            }
                            _ => {} // 3+ fingers: ignore
                        }
                    }
                    TouchPhase::Ended | TouchPhase::Cancelled => {
                        s.touches.remove(&id);
                        s.refresh_pinch();
                    }
                }
            }
            WindowEvent::MouseWheel { delta, .. } => {
                let scroll = match delta {
                    MouseScrollDelta::LineDelta(_, y) => y as f64,
                    MouseScrollDelta::PixelDelta(PhysicalPosition { y, .. }) => y / 120.0,
                };
                match s.mode {
                    Mode::Map => {
                        let (ax, ay) = s.cursor.unwrap_or((s.cam.vw * 0.5, s.cam.vh * 0.5));
                        s.cam.zoom_about(1.2f64.powf(scroll), ax, ay);
                    }
                    Mode::Panorama => {
                        if let Some(p) = s.pano.as_mut() {
                            p.fov_deg = (p.fov_deg * 1.1f64.powf(-scroll)).clamp(20.0, 160.0);
                        }
                    }
                    Mode::Compass => {}
                }
                s.window.request_redraw();
            }
            WindowEvent::RedrawRequested => s.render(),
            _ => {}
        }
    }
}

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .unwrap_or_else(|| r"C:\maps\airhorizon\data\OS_Open_Zoomstack.mbtiles".to_string());
    // Default to Wasdale Head Inn (NY187088, valley floor ~81 m).
    let lat: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(54.4679);
    let lon: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(-3.2559);

    let mut app = App {
        path: path.into(),
        paths_pbf: r"C:\maps\airhorizon\data\cumbria-latest.osm.pbf".into(),
        peaks_csv: r"C:\maps\airhorizon\data\DoBIH_v18_4.csv".into(),
        at: LatLon::new(lat, lon),
        state: None,
    };
    let event_loop = EventLoop::new().expect("event loop");
    event_loop.run_app(&mut app).expect("run");
}
