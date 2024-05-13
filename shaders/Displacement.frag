#version 330

uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

uniform vec4 u_color;

uniform sampler2D u_texture_2;
uniform vec2 u_texture_2_size;

uniform float u_normal_scaling;
uniform float u_height_scaling;

in vec4 v_position;
in vec4 v_normal;
in vec4 v_tangent;
in vec2 v_uv;

out vec4 out_color;

float h(vec2 uv) {
  // You may want to use this helper function...
  return texture(u_texture_2, uv).r * 15;
}

void main() {
  // YOUR CODE HERE
  float kd = 0.7;
  float ks = 0.2;
  float ka = 0.0;
  vec3 Ia = vec3(1.0, 1.0, 1.0);
  float p = 32.0;

  // Perform bump mapping
  vec3 n = normalize(v_normal.xyz);
  vec3 t = normalize(v_tangent.xyz);
  vec3 b = cross(n, t);
  mat3 TBN = mat3(t, b, n);
  float du = (h(v_uv + vec2(1.0/u_texture_2_size.x, 0.0)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  float dv = (h(v_uv + vec2(0.0, 1.0/u_texture_2_size.y)) - h(v_uv)) * u_height_scaling * u_normal_scaling;
  vec3 no = vec3(-du, -dv, 1.0);
  vec3 nd = TBN * no;

  vec3 light_diff = u_light_pos - v_position.xyz;
  vec3 cam_diff = u_cam_pos - v_position.xyz;
  float r2 = length(light_diff) * length(light_diff);
  vec3 h = normalize(normalize(cam_diff) + normalize(light_diff));
  vec3 light_d = kd * (u_light_intensity / r2) * max(0.0, dot(nd, normalize(light_diff)));
  vec3 light_s = ks * (u_light_intensity / r2) * max(0.0, pow(dot(nd, h), p));
  out_color = vec4(light_d + light_s + ka * Ia, 1.0);
}

