//! AirHorizon vector map viewer (B3 MVP).
//!
//! Live-renders OS Open Zoomstack MVT tiles as 1px lines in Web Mercator.
//! Pan with left-drag, zoom with the wheel. No fills/labels yet (B4/B5).
//!
//!   cargo run -p view --release --offline -- [mbtiles] [lat] [lon]

use std::collections::HashMap;
use std::sync::Arc;

use basemap::{GeomKind, Mbtiles};
use bytemuck::{Pod, Zeroable};
use geodesy::{LatLon, Tile, MERCATOR_MAX};
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

/// How a Zoomstack layer is drawn: a filled area or a stroked line.
#[derive(Clone, Copy)]
enum Style {
    Fill([f32; 3]),
    Stroke([f32; 3]),
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
        // Line strokes, drawn on top.
        "roads" => Stroke([0.28, 0.26, 0.24]),
        "waterlines" => Stroke([0.20, 0.45, 0.80]),
        "contours" => Stroke([0.78, 0.66, 0.50]),
        // NB: don't stroke tile-clipped AREA polygons (e.g. national_parks) — the
        // clip edges show up as square outlines along every tile boundary. A
        // proper park-edge line needs a real boundary feature; skip for now.
        _ => return None, // names (points), sites, national_parks, etc.
    })
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

/// GPU geometry for one tile: a triangle-list fill mesh and a line-list stroke
/// mesh. Either may be empty; an all-empty mesh is still cached so the tile
/// isn't re-decoded.
struct TileMesh {
    fill_buf: Option<wgpu::Buffer>,
    fill_count: u32,
    line_buf: Option<wgpu::Buffer>,
    line_count: u32,
}

struct Gpu {
    device: Arc<wgpu::Device>,
    queue: Arc<wgpu::Queue>,
    surface: wgpu::Surface<'static>,
    config: wgpu::SurfaceConfiguration,
    line_pipeline: wgpu::RenderPipeline,
    fill_pipeline: wgpu::RenderPipeline,
    view_buf: wgpu::Buffer,
    view_bg: wgpu::BindGroup,
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
        // Both pipelines share the shader, layout and vertex format; only the
        // primitive topology differs (lines vs filled triangles).
        let make_pipeline = |topology: wgpu::PrimitiveTopology, label: &str| {
            device.create_render_pipeline(&wgpu::RenderPipelineDescriptor {
                label: Some(label),
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
                primitive: wgpu::PrimitiveState { topology, ..Default::default() },
                depth_stencil: None,
                multisample: wgpu::MultisampleState::default(),
                multiview: None,
                cache: None,
            })
        };
        let fill_pipeline =
            make_pipeline(wgpu::PrimitiveTopology::TriangleList, "fill pipeline");
        let line_pipeline = make_pipeline(wgpu::PrimitiveTopology::LineList, "line pipeline");

        Self { device, queue, surface, config, line_pipeline, fill_pipeline, view_buf, view_bg }
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

        let mut fills: Vec<Vertex> = Vec::new();
        let mut lines: Vec<Vertex> = Vec::new();

        for layer in &vt.layers {
            let Some(st) = style(&layer.name) else { continue };
            match st {
                Style::Stroke(color) => {
                    for feat in &layer.features {
                        if feat.kind == GeomKind::Point {
                            continue;
                        }
                        for part in &feat.parts {
                            for seg in part.windows(2) {
                                lines.push(to_vertex(seg[0], color));
                                lines.push(to_vertex(seg[1], color));
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
                                    fills.push(to_vertex(flat[i], color));
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
        let (fill_buf, fill_count) = mk(&fills, "tile fills");
        let (line_buf, line_count) = mk(&lines, "tile lines");
        TileMesh { fill_buf, fill_count, line_buf, line_count }
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

    fn render(&mut self) {
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
                Ok(None) => TileMesh { fill_buf: None, fill_count: 0, line_buf: None, line_count: 0 },
                Err(e) => {
                    eprintln!("decode {:?} failed: {e}", t);
                    TileMesh { fill_buf: None, fill_count: 0, line_buf: None, line_count: 0 }
                }
            };
            self.tiles.insert(t, mesh);
            loaded += 1;
        }

        self.gpu
            .queue
            .write_buffer(&self.gpu.view_buf, 0, bytemuck::bytes_of(&self.cam.uniform()));

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
            // Fills underneath...
            pass.set_pipeline(&self.gpu.fill_pipeline);
            for &t in &want {
                if let Some(TileMesh { fill_buf: Some(buf), fill_count, .. }) = self.tiles.get(&t) {
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..*fill_count, 0..1);
                }
            }
            // ...strokes on top.
            pass.set_pipeline(&self.gpu.line_pipeline);
            for &t in &want {
                if let Some(TileMesh { line_buf: Some(buf), line_count, .. }) = self.tiles.get(&t) {
                    pass.set_vertex_buffer(0, buf.slice(..));
                    pass.draw(0..*line_count, 0..1);
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
                event_loop.exit();
            }
            WindowEvent::Resized(new) => {
                s.gpu.resize(new.width, new.height);
                s.cam.vw = new.width as f64;
                s.cam.vh = new.height as f64;
                s.window.request_redraw();
            }
            WindowEvent::MouseInput { state, button: MouseButton::Left, .. } => {
                // Ignore mouse press while a finger is down (touch synthesises it).
                s.dragging = state == ElementState::Pressed && s.touches.is_empty();
                s.last_cursor = s.cursor;
            }
            WindowEvent::CursorMoved { position, .. } => {
                let pos = (position.x, position.y);
                s.cursor = Some(pos);
                if s.dragging && s.touches.is_empty() {
                    if let Some((lx, ly)) = s.last_cursor {
                        s.cam.pan_screen(pos.0 - lx, pos.1 - ly);
                        s.window.request_redraw();
                    }
                }
                s.last_cursor = Some(pos);
            }
            WindowEvent::Touch(Touch { id, phase, location, .. }) => {
                let pos = (location.x, location.y);
                match phase {
                    TouchPhase::Started => {
                        s.dragging = false; // a touch cancels any mouse drag
                        s.touches.insert(id, pos);
                        s.refresh_pinch();
                    }
                    TouchPhase::Moved => {
                        let prev = s.touches.insert(id, pos);
                        match s.touches.len() {
                            1 => {
                                if let Some((px, py)) = prev {
                                    s.cam.pan_screen(pos.0 - px, pos.1 - py);
                                    s.window.request_redraw();
                                }
                            }
                            2 => {
                                let pts: Vec<(f64, f64)> = s.touches.values().copied().collect();
                                let mid =
                                    ((pts[0].0 + pts[1].0) * 0.5, (pts[0].1 + pts[1].1) * 0.5);
                                let dist = (pts[0].0 - pts[1].0).hypot(pts[0].1 - pts[1].1);
                                if let Some((pmid, pdist)) = s.pinch_prev {
                                    // Zoom about the midpoint by the finger-spread
                                    // ratio, and pan by the midpoint's movement.
                                    s.cam.zoom_about(dist / pdist.max(1.0), mid.0, mid.1);
                                    s.cam.pan_screen(mid.0 - pmid.0, mid.1 - pmid.1);
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
                let (ax, ay) = s.cursor.unwrap_or((s.cam.vw * 0.5, s.cam.vh * 0.5));
                s.cam.zoom_about(1.2f64.powf(scroll), ax, ay);
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
    let lat: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(54.6012);
    let lon: f64 = args.next().and_then(|s| s.parse().ok()).unwrap_or(-3.1399);

    let mut app = App {
        path: path.into(),
        at: LatLon::new(lat, lon),
        state: None,
    };
    let event_loop = EventLoop::new().expect("event loop");
    event_loop.run_app(&mut app).expect("run");
}
