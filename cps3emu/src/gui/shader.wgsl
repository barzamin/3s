// cs: clip space

struct VertexInput {
    @location(0) position: vec2<f32>,
    @location(1) uv: vec2<f32>,
}

struct VertexOut {
    @builtin(position) cs_pos: vec4<f32>,
}

@vertex
fn vert_main(@builtin(vertex_index) vidx: u32) -> VertexOut {
    var o: VertexOut;

    let uv = vec2<f32>(f32((vidx << 1u) & 2u), f32(vidx & 2u));
    o.cs_pos = vec4<f32>(uv * vec2(2., -2.) + vec2(-1., 1.), 0., 1.);

    return o;
}

@fragment
fn frag_main(v: VertexOut) -> @location(0) vec4<f32> {
    return vec4<f32>(1., 0., 0., 1.);
}
