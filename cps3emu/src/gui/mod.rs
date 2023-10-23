use std::time::Instant;

use anyhow::Result;
use imgui::{FontConfig, FontSource, MouseCursor};
use imgui_wgpu::RendererConfig;
use imgui_winit_support::HiDpiMode;
use pixels::{Pixels, PixelsContext, SurfaceTexture};
use wgpu::RenderPassColorAttachment;
use winit::{
    dpi::LogicalSize,
    event::{Event, WindowEvent},
    event_loop::EventLoop,
    window::{Window, WindowBuilder},
};

pub struct GuiCtx {
    imgui: imgui::Context,
    platform: imgui_winit_support::WinitPlatform,
    renderer: imgui_wgpu::Renderer,
    last_frame: Instant,
    last_cursor: Option<MouseCursor>,

    // ...
    about_open: bool,
}

impl GuiCtx {
    pub fn new(window: &Window, pixels: &Pixels) -> Self {
        let mut imgui = imgui::Context::create();
        imgui.set_ini_filename(None);

        let mut platform = imgui_winit_support::WinitPlatform::init(&mut imgui);
        platform.attach_window(imgui.io_mut(), window, HiDpiMode::Default);

        let hidpi_factor = window.scale_factor();
        let font_size = (13. * hidpi_factor) as f32;
        imgui.io_mut().font_global_scale = (1. / hidpi_factor) as f32;
        imgui.fonts().add_font(&[FontSource::DefaultFontData {
            config: Some(FontConfig {
                oversample_h: 1,
                pixel_snap_h: true,
                size_pixels: font_size,
                ..Default::default()
            }),
        }]);

        let device = pixels.device();
        let queue = pixels.queue();
        let renderer = imgui_wgpu::Renderer::new(
            &mut imgui,
            device,
            queue,
            RendererConfig {
                texture_format: pixels.render_texture_format(),
                ..Default::default()
            },
        );

        Self {
            imgui,
            platform,
            renderer,
            last_frame: Instant::now(),
            last_cursor: None,

            // ...
            about_open: true,
        }
    }

    pub fn prepare(&mut self, window: &Window) -> Result<(), winit::error::ExternalError> {
        let now = Instant::now();
        let dt = now - self.last_frame;
        self.last_frame = now;
        self.imgui.io_mut().update_delta_time(dt);
        self.platform.prepare_frame(self.imgui.io_mut(), window)
    }

    pub fn render(
        &mut self,
        window: &Window,
        encoder: &mut wgpu::CommandEncoder,
        render_target: &wgpu::TextureView,
        pxctx: &PixelsContext,
    ) -> imgui_wgpu::RendererResult<()> {
        let ui = self.imgui.new_frame();
        let mouse_cursor = ui.mouse_cursor();
        if self.last_cursor != mouse_cursor {
            self.last_cursor = mouse_cursor;
            self.platform.prepare_render(ui, window);
        }

        // -- gui start --
        ui.show_about_window(&mut self.about_open);
        // -- gui end --

        let mut rpass = encoder.begin_render_pass(&wgpu::RenderPassDescriptor {
            label: Some("imgui"),
            color_attachments: &[Some(RenderPassColorAttachment {
                view: render_target,
                resolve_target: None,
                ops: wgpu::Operations {
                    load: wgpu::LoadOp::Load, // overdraw
                    store: true,
                },
            })],
            depth_stencil_attachment: None,
        });

        self.renderer
            .render(self.imgui.render(), &pxctx.queue, &pxctx.device, &mut rpass)
    }

    fn handle_event(&mut self, window: &Window, event: &winit::event::Event<()>) {
        self.platform
            .handle_event(self.imgui.io_mut(), window, event)
    }
}

fn draw(w: usize, h: usize, fb: &mut [u8]) {
    for x in 0..w {
        for y in 0..h {
            // rgba8
            fb[x + w + y] = 0xff;
        }
    }
}

pub fn run() -> Result<()> {
    let [fb_width, fb_height] = [640, 480];

    let evloop = EventLoop::new();
    let window = WindowBuilder::new()
        .with_title("cps3emu")
        .with_min_inner_size(LogicalSize::new(fb_width, fb_height))
        .build(&evloop)?;

    let window_size = window.inner_size();

    let mut pixels = {
        let surface_texture = SurfaceTexture::new(window_size.width, window_size.height, &window);
        Pixels::new(fb_width, fb_height, surface_texture)?
    };

    let mut gui = GuiCtx::new(&window, &pixels);

    evloop.run(move |ev, _tgt, flow| {
        flow.set_poll();
        match ev {
            Event::RedrawRequested(_) => {
                draw(fb_width as usize, fb_height as usize, pixels.frame_mut());

                gui.prepare(&window).expect("prepare");

                pixels
                    .render_with(|encoder, render_target, pxctx| {
                        pxctx.scaling_renderer.render(encoder, render_target);
                        gui.render(&window, encoder, render_target, pxctx)?;

                        Ok(())
                    })
                    .expect("woag pixels error !"); // TODO: proper error handling
            }

            Event::MainEventsCleared => {
                window.request_redraw();
            }

            Event::WindowEvent {
                event: ref window_event,
                ..
            } => match window_event {
                WindowEvent::CloseRequested => flow.set_exit(),
                WindowEvent::Resized(size) => {
                    pixels
                        .resize_surface(size.width, size.height)
                        .expect("can't resize");
                }
                _ => (),
            },

            _ => (),
        }

        gui.handle_event(&window, &ev);
    });
}
