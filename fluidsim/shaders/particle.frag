#version 450
layout(location = 0) in vec3 vColor;
layout(location = 1) in vec2 vUV;
layout(location = 0) out vec4 outColor;
void main() {
  float d = dot(vUV, vUV);
  if (d > 1.0) discard;
  outColor = vec4(vColor, 1.0 - smoothstep(0.7, 1.0, d));
}
