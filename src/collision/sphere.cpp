#include <nanogui/nanogui.h>

#include "../clothMesh.h"
#include "../cloth.h"
#include "../misc/sphere_drawing.h"
#include "sphere.h"

using namespace nanogui;
using namespace CGL;

void Sphere::collide(PointMass &pm) {
  // TODO (Part 3): Handle collisions with spheres.
  if((pm.position - origin).norm2() < radius2) {
    auto tangent_point = (pm.position - origin).unit() * radius + origin;
    pm.position = pm.last_position + (tangent_point - pm.last_position) * (1 - friction);
  }
}

void Sphere::render(GLShader &shader) {
  // We decrease the radius here so flat triangles don't behave strangely
  // and intersect with the sphere when rendered
  m_sphere_mesh.draw_sphere(shader, origin, radius * 0.92);
}

void Sphere::simulate(double frames_per_sec, double simulation_steps,
                     vector<Vector3D> external_accelerations,
                     
                     Cloth* cloth) {
  // move sphere down
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  last_origin.x = origin.x;
  last_origin.y = origin.y;
  last_origin.z = origin.z;

  double tol = 0.03;
	if (origin.x < tol || origin.x > cloth->width - tol || origin.z < tol || origin.z > cloth->height - tol) {
        origin.x = cloth->width*((double)rand() / (RAND_MAX));
        origin.z = cloth->height*((double)rand() / (RAND_MAX));
	}

  double dx = cloth->width / (cloth->num_width_points - 1);
  double dy = cloth->height / (cloth->num_height_points - 1);
  double x = origin.x / dx;
  double y = origin.z / dy;
  int i = static_cast<int>(x);
  int j = static_cast<int>(y);

  Vector3D p00 = cloth->sim_point_masses[j * cloth->num_width_points + i].position;
  Vector3D p01 = cloth->sim_point_masses[j * cloth->num_width_points + (i+1)].position;
  Vector3D p10 = cloth->sim_point_masses[(j+1) * cloth->num_width_points + i].position;
  Vector3D p11 = cloth->sim_point_masses[(j+1) * cloth->num_width_points + (i+1)].position;

  double tx = x - i;
  double ty = y - j;
  double h0 = p00.y * ty + p01.y * (1 - ty);
  double h1 = p10.y * ty + p11.y * (1 - ty);
  double h = h0 * tx + h1 * (1 - tx);

  double random_theta = 2*3.1415926*((double)rand() / (RAND_MAX));
  Vector3D step{ std::cos(random_theta), 0, std::sin(random_theta)};

  double movement = std::abs(h * h * h); // a proxy for energy
  double max_movement = 0.05; // This value can be adjusted as needed
  if (movement > max_movement) {
      step *= max_movement;
  } else {
      step *= movement;
  }

  origin += step;
}
