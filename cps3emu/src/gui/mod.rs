use std::{iter, time::Instant};

use anyhow::Result;
use imgui::{FontConfig, FontSource};
use imgui_wgpu::{Renderer, RendererConfig};
use imgui_winit_support::HiDpiMode;
use log::debug;
use wgpu::{
    Adapter, Backends, CommandEncoderDescriptor, CompositeAlphaMode, Device, DeviceDescriptor,
    Features, Instance, InstanceDescriptor, Limits, PresentMode, Queue, RenderPassDescriptor,
    RequestAdapterOptions, Surface, SurfaceConfiguration, TextureFormat, TextureUsages,
};
use winit::{
    dpi::PhysicalSize,
    event::{Event, WindowEvent},
    event_loop::EventLoop,
    window::{Window, WindowBuilder},
};

pub struct RenderCtx {
    pub instance: Instance,
    pub adapter: Adapter,
    pub device: Device,
    pub queue: Queue,
}

impl RenderCtx {
    pub async fn with_window(window: &Window) -> Result<(Self, Surface)> {
        let instance = Instance::new(InstanceDescriptor {
            backends: Backends::DX12 | Backends::VULKAN | Backends::METAL,
            dx12_shader_compiler: wgpu::Dx12Compiler::Dxc {
                dxil_path: None,
                dxc_path: None,
            }, // todo
        });

        let window_surface = unsafe { instance.create_surface(window) }?;

        let adapter = instance
            .request_adapter(&RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::HighPerformance,
                force_fallback_adapter: false,
                compatible_surface: Some(&window_surface),
            })
            .await
            .expect("adapter");

        let (device, queue) = adapter
            .request_device(
                &DeviceDescriptor {
                    label: Some("device0"),
                    features: Features::empty(),
                    limits: Limits::default(),
                },
                None,
            )
            .await?;

        Ok((
            Self {
                instance,
                adapter,
                device,
                queue,
            },
            window_surface,
        ))
    }
}

struct OutputSurface {
    surface: Surface,
    config: SurfaceConfiguration,
    // depth_buffer: Texture,
    format: TextureFormat,
    dims: PhysicalSize<u32>,
}

impl OutputSurface {
    pub fn from_window_surface(
        rctx: &RenderCtx,
        window_surface: Surface,
        dims: PhysicalSize<u32>,
    ) -> Self {
        // find a surface format
        let surface_format = window_surface
            .get_capabilities(&rctx.adapter)
            .formats
            .iter()
            .filter(|fmt| fmt.is_srgb())
            .copied()
            .next()
            .expect("no srgb surface format");
        log::debug!(
            "create_window_surface: surface_formats={:?}",
            surface_format
        );

        let config = SurfaceConfiguration {
            usage: TextureUsages::RENDER_ATTACHMENT,
            format: surface_format,
            width: dims.width,
            height: dims.height,
            present_mode: PresentMode::Fifo,
            alpha_mode: CompositeAlphaMode::Opaque,
            view_formats: vec![],
        };

        window_surface.configure(&rctx.device, &config);

        Self {
            surface: window_surface,
            config,
            format: surface_format,
            dims,
        }
    }

    fn resize(&mut self, rctx: &RenderCtx, inner_size: PhysicalSize<u32>) {
        debug!("resizing OutputSurface to {:?}", inner_size);
        self.dims = inner_size;
        self.config.width = inner_size.width;
        self.config.height = inner_size.height;
        self.reconfigure(rctx);
    }

    fn reconfigure(&self, rctx: &RenderCtx) {
        self.surface.configure(&rctx.device, &self.config);
    }
}

pub async fn run() -> Result<()> {
    let evloop = EventLoop::new();
    let window = WindowBuilder::new().build(&evloop)?;
    let (mut rctx, window_surface) = RenderCtx::with_window(&window).await?;
    let mut output_surface =
        OutputSurface::from_window_surface(&rctx, window_surface, window.inner_size());

    let hidpi_factor = window.scale_factor();

    let mut imgui = imgui::Context::create();
    let mut platform = imgui_winit_support::WinitPlatform::init(&mut imgui);
    platform.attach_window(imgui.io_mut(), &window, HiDpiMode::Default);
    imgui.set_ini_filename(None);

    let fontsize = (13. * hidpi_factor) as f32;
    imgui.io_mut().font_global_scale = (1. / hidpi_factor) as f32;
    imgui.fonts().add_font(&[FontSource::DefaultFontData {
        config: Some(FontConfig {
            oversample_h: 1,
            pixel_snap_h: true,
            size_pixels: fontsize,
            ..Default::default()
        }),
    }]);

    let mut imgui_renderer = Renderer::new(
        &mut imgui,
        &rctx.device,
        &rctx.queue,
        RendererConfig {
            texture_format: output_surface.format,
            ..Default::default()
        },
    );

    let mut last_frame = Instant::now();
    let mut last_cursor = None;

    evloop.run(move |ev, tgt, flow| {
        flow.set_poll();
        match ev {
            Event::WindowEvent {
                window_id,
                event: ref window_event,
            } => match window_event {
                WindowEvent::CloseRequested => flow.set_exit(),
                WindowEvent::Resized(_) => {
                    output_surface.resize(&rctx, window.inner_size());
                }
                _ => (),
            },

            Event::MainEventsCleared => {
                window.request_redraw();
            }

            Event::RedrawRequested(_) => {
                let delta_s = last_frame.elapsed();
                let now = Instant::now();
                imgui.io_mut().update_delta_time(now - last_frame);
                last_frame = now;

                let output_tex = output_surface.surface.get_current_texture().expect("uhh");
                platform
                    .prepare_frame(imgui.io_mut(), &window)
                    .expect("prepare frame");

                let ui = imgui.frame();
                {
                    // ui.show_demo_window(&mut demo_open);
                    ui.main_menu_bar(|| {
                        ui.menu("File", || {});
                        ui.menu("Debug", || {});
                    });
                }

                let mut encoder = rctx
                    .device
                    .create_command_encoder(&CommandEncoderDescriptor {
                        label: Some("encoder0"),
                    });

                if last_cursor != Some(ui.mouse_cursor()) {
                    last_cursor = Some(ui.mouse_cursor());
                    platform.prepare_render(ui, &window);
                }

                let output_view = output_tex.texture.create_view(&Default::default());

                {
                    let mut rp = encoder.begin_render_pass(&RenderPassDescriptor {
                        label: Some("pass:imgui"),
                        color_attachments: &[Some(wgpu::RenderPassColorAttachment {
                            view: &output_view,
                            resolve_target: None,
                            ops: wgpu::Operations {
                                load: wgpu::LoadOp::Clear(wgpu::Color {
                                    r: 0.1,
                                    g: 0.2,
                                    b: 0.3,
                                    a: 1.0,
                                }),
                                store: true,
                            },
                        })],
                        depth_stencil_attachment: None,
                    });

                    imgui_renderer
                        .render(imgui.render(), &rctx.queue, &rctx.device, &mut rp)
                        .expect("imgui render");
                }

                rctx.queue.submit(iter::once(encoder.finish()));
                output_tex.present();
            }

            _ => (),
        }

        platform.handle_event(imgui.io_mut(), &window, &ev);
    });

    Ok(())
}
