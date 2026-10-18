#version 450
// Instanced quad per particle: per-instance position (world metres) + colour.
layout(location = 0) in vec2 inPos;
layout(location = 1) in vec3 inColor;
layout(push_constant) uniform PC { vec2 world; float radius; float pad; } pc;
layout(location = 0) out vec3 vColor;
layout(location = 1) out vec2 vUV;
void main() {
  vec2 corner = vec2(float(gl_VertexIndex & 1), float(gl_VertexIndex >> 1)) * 2.0 - 1.0;
  vec2 p = inPos + corner * pc.radius;
  gl_Position = vec4(p.x / pc.world.x * 2.0 - 1.0, 1.0 - p.y / pc.world.y * 2.0, 0.0, 1.0);
  vColor = inColor;
  vUV = corner;
}
