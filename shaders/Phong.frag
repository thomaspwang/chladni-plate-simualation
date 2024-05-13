#version 330

uniform vec4 u_color;
uniform vec3 u_cam_pos;
uniform vec3 u_light_pos;
uniform vec3 u_light_intensity;

in vec4 v_position;
in vec4 v_normal;
in vec2 v_uv;

out vec4 out_color;

void main() {
  // YOUR CODE HERE
  float kd = 0.5;
  float ks = 100.0;
  float ka = 2.0;
  vec3 Ia = vec3(0.1, 0.1, 0.1);
  float p = 80;
  vec3 light_diff = u_light_pos - v_position.xyz;
  vec3 cam_diff = u_cam_pos - v_position.xyz;
  float r2 = length(light_diff) * length(light_diff);
  vec3 h = normalize(normalize(cam_diff) + normalize(light_diff));
  vec3 light_d = kd * (u_light_intensity / r2) * max(0.0, dot(v_normal.xyz, normalize(light_diff)));
  vec3 light_s = ks * (u_light_intensity / r2) * max(0.0, pow(dot(v_normal.xyz, h), p));
  out_color = vec4(light_d + light_s + ka * Ia, 1.0);
  
  // (Placeholder code. You will want to replace it.)
  // out_color = (vec4(1, 1, 1, 0) + v_normal) / 2;
  // out_color.a = 1;
}

