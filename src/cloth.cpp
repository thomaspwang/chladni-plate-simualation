#include <iostream>
#include <math.h>
#include <random>
#include <vector>

#include "cloth.h"
#include "collision/plane.h"
#include "collision/sphere.h"

using namespace std;

Cloth::Cloth(double width, double height, int num_width_points,
             int num_height_points, float thickness) {
  this->width = width;
  this->height = height;
  this->num_width_points = num_width_points;
  this->num_height_points = num_height_points;
  this->thickness = thickness;

  // NOTE: THIS IS FAKE CONSTRUCTOR, SEE loadObjectsFromFile in main.cpp

  buildGrid();
  buildClothMesh();
}

Cloth::~Cloth() {
  point_masses.clear();
  springs.clear();
  sim_point_masses.clear();
  sim_springs.clear();

  if (clothMesh) {
    delete clothMesh;
  }
}

void Cloth::buildGrid() {
  // TODO (Part 1): Build a grid of masses and springs.
  // simple grid for rendering
  point_masses.emplace_back(Vector3D{ 0, 0, 0 }, true);
  point_masses.emplace_back(Vector3D{ width, 0, 0 }, true);
  point_masses.emplace_back(Vector3D{ 0, 0, height }, true);
  point_masses.emplace_back(Vector3D{ width, 0, height }, true);
  springs.emplace_back(&point_masses[0], &point_masses[1], STRUCTURAL);
  springs.emplace_back(&point_masses[0], &point_masses[2], STRUCTURAL);
  springs.emplace_back(&point_masses[3], &point_masses[1], STRUCTURAL);
  springs.emplace_back(&point_masses[3], &point_masses[2], STRUCTURAL);

  double dx = width / (num_width_points - 1);
  double dy = height / (num_height_points - 1);
  for (int j = 0; j < num_height_points; ++j) {
    for (int i = 0; i < num_width_points; ++i) {
      Vector3D pos;
      if (orientation == e_orientation::VERTICAL) {
        pos = {i*dx, j*dy, ((static_cast<double>(rand()) / RAND_MAX) * (2./1000)) - (1./1000)};
      } else if (orientation == e_orientation::HORIZONTAL) {
        pos = {i*dx, 0, j*dy};
      }
      bool is_pinned = false;
      for (auto xy : pinned) {
        if (xy[0] == i && xy[1] == j) {
          is_pinned = true;
          break;
        }
      }
      sim_point_masses.emplace_back(pos, is_pinned);
    }
  }
  // Add springs
  for(int x =0; x<num_width_points; ++x) { 
    for(int y=0; y<num_height_points; ++y) {
      // Connect springs down and to the right
      if (x < num_width_points - 1) {
        sim_springs.emplace_back(&sim_point_masses[y*num_width_points + x], &sim_point_masses[y*num_width_points + x + 1], STRUCTURAL);
      }
      if (y < num_height_points - 1) {
        sim_springs.emplace_back(&sim_point_masses[y*num_width_points + x], &sim_point_masses[(y+1)*num_width_points + x], STRUCTURAL);
      }
      // diagonal springs
      if (x < num_width_points - 1 && y < num_height_points - 1) {
        sim_springs.emplace_back(&sim_point_masses[y*num_width_points + x], &sim_point_masses[(y+1)*num_width_points + x + 1], SHEARING);
      }
      if (x > 0 && y < num_height_points - 1) {
        sim_springs.emplace_back(&sim_point_masses[y*num_width_points + x], &sim_point_masses[(y+1)*num_width_points + x - 1], SHEARING);
      }
      // bending springs
      if (x < num_width_points - 2) {
        sim_springs.emplace_back(&sim_point_masses[y*num_width_points + x], &sim_point_masses[y*num_width_points + x + 2], BENDING);
      }
      if (y < num_height_points - 2) {
        sim_springs.emplace_back(&sim_point_masses[y*num_width_points + x], &sim_point_masses[(y+2)*num_width_points + x], BENDING);
      }
    }
  }
}

void Cloth::simulate(double frames_per_sec, double simulation_steps, ClothParameters *cp,
                     vector<Vector3D> external_accelerations,
                     vector<CollisionObject *> *collision_objects) {
  // double mass = width * height * cp->density / num_width_points / num_height_points;
  double delta_t = 1.0f / frames_per_sec / simulation_steps;
  static double t = 0;
  t += delta_t;

  // TODO (Part 2): Compute total force acting on each point mass.
  Vector3D external_forces{};
  // for (auto& a : external_accelerations) {
  //     external_forces += a;
  // }
  // external_forces *= mass;
  for (auto& point_mass : sim_point_masses) {
    point_mass.forces = Vector3D{external_forces.x, external_forces.y, external_forces.z};
  }


  // Chladni Wave Function Simulation Step
  sim_p[num_width_points / 2][num_height_points / 2] = sim_amplitude * sin(2 * 3.1415926 * sim_frequency * t); // TODO: use t? or an integer step?
  for (int i = 1; i < num_width_points - 1; i++) {
      for (int j = 1; j < num_height_points - 1; j++) {
          sim_p_next[i][j] = 0.25 *
                              (sim_p[i + 1][j] + sim_p[i - 1][j] +
                              sim_p[i][j + 1] + sim_p[i][j - 1] - 4 * sim_p[i][j])
                              + 2 * sim_p[i][j] - sim_p_prev[i][j];
      }
  }

  // Reflective boundary conditions with damping (ASSUMES SQUARE)
  int size = num_width_points;
  for (int k = 0; k < size; k++) {
      sim_p_next[0][k] = sim_p_next[1][k] * sim_damper;
      sim_p_next[size - 1][k] = sim_p_next[size - 2][k] * sim_damper;
      sim_p_next[k][0] = sim_p_next[k][1] * sim_damper;
      sim_p_next[k][size - 1] = sim_p_next[k][size - 2] * sim_damper;
  }

  // Makes it so the wave form simulation happens slower
  // if (sim_curr_steplag != 0) {
  //   sim_curr_steplag--;
  //   return;
  // }

  // sim_curr_steplag = sim_max_steplag;
  
  for (int i = 0; i < num_width_points; i++) {
    for (int j = 0; j < num_height_points; j++) {
        PointMass &pm = sim_point_masses[j * num_width_points + i];
        pm.position.y = sim_p[i][j];
    }
  }

  // Swap the matrices for the next time step
  std::swap(sim_p_prev, sim_p);
  std::swap(sim_p, sim_p_next);



  // for (auto& spring : sim_springs) {
  //   if ((spring.spring_type == STRUCTURAL && cp->enable_structural_constraints)
  //    || (spring.spring_type == SHEARING && cp->enable_shearing_constraints)
  //    || (spring.spring_type == BENDING && cp->enable_bending_constraints)) {
  //     auto spring_diff = spring.pm_a->position - spring.pm_b->position;
  //     auto rest_diff = spring.rest_length * spring_diff.unit();
  //     auto spring_force = (spring.spring_type == BENDING ? 0.2 * cp->ks : cp->ks) * (spring_diff - rest_diff);
  //     spring.pm_b->forces += spring_force;
  //     spring.pm_a->forces -= spring_force;
  //   }
  // }

  // TODO (Part 2): Use Verlet integration to compute new point mass positions
  // for (auto& point_mass : sim_point_masses) {
  //   if (!point_mass.pinned) {
  //     Vector3D temp = point_mass.position;
  //     point_mass.position = point_mass.position + (1-cp->damping) * (point_mass.position - point_mass.last_position) + (delta_t*delta_t/mass)*point_mass.forces;
  //     point_mass.last_position = temp;
  //   }
  //   else { // if it is pinned, move it up and down in an oscillatory fashion
  //     Vector3D temp = point_mass.position;
  //     static double frequency = 1000;
  //     static double amplitude = 0.01;
  //     point_mass.position = Vector3D{ point_mass.position.x, amplitude * std::sin(t * (2 * 3.1415926 * frequency)), point_mass.position.z };
  //     point_mass.last_position = temp;
  //   }
  // }
  
  // TODO (Part 4): Handle self-collisions.
  // build_spatial_map();
  // for (auto& point_mass : point_masses) {
  //   self_collide(point_mass, simulation_steps);
  // }

  // TODO (Part 3): Handle collisions with other primitives.
  // for (auto& point_mass : point_masses) {
  //   for (auto& coll : *collision_objects) {
  //     coll->collide(point_mass);
  //   }
  // }

  // TODO (Part 2): Constrain the changes to be such that the spring does not change
  // in length more than 10% per timestep [Provot 1995].
  // for (auto& spring : sim_springs) {
  //   auto length = (spring.pm_a->position - spring.pm_b->position).norm();
  //   if (length > 1.1 * spring.rest_length) {
  //     if(spring.pm_a->pinned && spring.pm_b->pinned) {
  //       continue;
  //     } else if (spring.pm_a->pinned) {
  //       // move pm_b closer to a 
  //       spring.pm_b->position = spring.pm_a->position + (spring.pm_b->position - spring.pm_a->position).unit() * spring.rest_length * 1.1;
  //     } else if (spring.pm_b->pinned) {
  //       spring.pm_a->position = spring.pm_b->position + (spring.pm_a->position - spring.pm_b->position).unit() * spring.rest_length * 1.1;
  //     } else {
  //       // move both masses
  //       auto center = (spring.pm_a->position + spring.pm_b->position) / 2.;
  //       spring.pm_a->position = center + (spring.pm_a->position - spring.pm_b->position).unit() * spring.rest_length * 1.1 / 2.0;
  //       spring.pm_b->position = center + (spring.pm_b->position - spring.pm_a->position).unit() * spring.rest_length * 1.1 / 2.0;
  //     }
  //   }
  // }
}

void Cloth::build_spatial_map() {
  for (const auto &entry : map) {
    delete(entry.second);
  }
  map.clear();

  // TODO (Part 4): Build a spatial map out of all of the point masses.
  for (auto& point_mass : sim_point_masses) {
    auto hash = hash_position(point_mass.position);
    auto curr_pair = map.find(hash);
    if (curr_pair != map.end()) {
      curr_pair->second->push_back(&point_mass);
    } else {
      vector<PointMass*>* pair = new vector<PointMass*>;
      pair->push_back(&point_mass);
      map.emplace(hash, pair);
    }
  }
}

void Cloth::self_collide(PointMass &pm, double simulation_steps) {
  // TODO (Part 4): Handle self-collision for a given point mass.
  float hash = hash_position(pm.position);
  if (map.count(hash) > 0) {
    Vector3D corrections = Vector3D();
    int num_corrections = 0;
    for (auto other_pm : *(map[hash])) {
      if (&pm == other_pm) {
        continue;
      }
      auto diff = pm.position - other_pm->position;
      auto dist = diff.norm();
      if (dist < 2 * thickness) {
        num_corrections++;
        corrections += (2 * thickness - dist) * diff.unit();
      }
    }
    pm.position += corrections / simulation_steps / std::max(num_corrections, 1);
  }
}

float Cloth::hash_position(Vector3D pos) {
  // TODO (Part 4): Hash a 3D position into a unique float identifier that represents membership in some 3D box volume.
  float resolution = 25.0; // Needs to be greater than thickness 
  auto w = 3 * width / num_width_points;
  auto h = 3 * height / num_height_points;
  auto t = std::max(h, w);
  // truncate x, y, z
  int x_truncated = std::floor(pos.x * resolution);
  int y_truncated = std::floor(pos.y * resolution);
  int z_truncated = std::floor(pos.z * resolution);
  // mod with w, h, t
  double x_truncated_mod = std::fmod(x_truncated, w);
  double y_truncated_mod = std::fmod(y_truncated, h);
  double z_truncated_mod = std::fmod(z_truncated, t);
  // combine into a single float
  return x_truncated_mod + y_truncated_mod * w + z_truncated_mod * w * h;
}

///////////////////////////////////////////////////////
/// YOU DO NOT NEED TO REFER TO ANY CODE BELOW THIS ///
///////////////////////////////////////////////////////

void Cloth::reset() {
  PointMass *pm = &point_masses[0];
  for (int i = 0; i < point_masses.size(); i++) {
    pm->position = pm->start_position;
    pm->last_position = pm->start_position;
    pm++;
  }
}

void Cloth::buildClothMesh() {
  if (sim_point_masses.size() == 0) return;

  ClothMesh *clothMesh = new ClothMesh();
  vector<Triangle *> triangles;

  // Create vector of triangles
  for (int y = 0; y < num_height_points - 1; y++) {
    for (int x = 0; x < num_width_points - 1; x++) {
      PointMass *pm = &sim_point_masses[y * num_width_points + x];
      // Get neighboring point masses:
      /*                      *
       * pm_A -------- pm_B   *
       *             /        *
       *  |         /   |     *
       *  |        /    |     *
       *  |       /     |     *
       *  |      /      |     *
       *  |     /       |     *
       *  |    /        |     *
       *      /               *
       * pm_C -------- pm_D   *
       *                      *
       */
      
      float u_min = x;
      u_min /= num_width_points - 1;
      float u_max = x + 1;
      u_max /= num_width_points - 1;
      float v_min = y;
      v_min /= num_height_points - 1;
      float v_max = y + 1;
      v_max /= num_height_points - 1;
      
      PointMass *pm_A = pm                       ;
      PointMass *pm_B = pm                    + 1;
      PointMass *pm_C = pm + num_width_points    ;
      PointMass *pm_D = pm + num_width_points + 1;
      
      Vector3D uv_A = Vector3D(u_min, v_min, 0);
      Vector3D uv_B = Vector3D(u_max, v_min, 0);
      Vector3D uv_C = Vector3D(u_min, v_max, 0);
      Vector3D uv_D = Vector3D(u_max, v_max, 0);
      
      
      // Both triangles defined by vertices in counter-clockwise orientation
      triangles.push_back(new Triangle(pm_A, pm_C, pm_B, 
                                       uv_A, uv_C, uv_B));
      triangles.push_back(new Triangle(pm_B, pm_C, pm_D, 
                                       uv_B, uv_C, uv_D));
    }
  }

  // For each triangle in row-order, create 3 edges and 3 internal halfedges
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    // Allocate new halfedges on heap
    Halfedge *h1 = new Halfedge();
    Halfedge *h2 = new Halfedge();
    Halfedge *h3 = new Halfedge();

    // Allocate new edges on heap
    Edge *e1 = new Edge();
    Edge *e2 = new Edge();
    Edge *e3 = new Edge();

    // Assign a halfedge pointer to the triangle
    t->halfedge = h1;

    // Assign halfedge pointers to point masses
    t->pm1->halfedge = h1;
    t->pm2->halfedge = h2;
    t->pm3->halfedge = h3;

    // Update all halfedge pointers
    h1->edge = e1;
    h1->next = h2;
    h1->pm = t->pm1;
    h1->triangle = t;

    h2->edge = e2;
    h2->next = h3;
    h2->pm = t->pm2;
    h2->triangle = t;

    h3->edge = e3;
    h3->next = h1;
    h3->pm = t->pm3;
    h3->triangle = t;
  }

  // Go back through the cloth mesh and link triangles together using halfedge
  // twin pointers

  // Convenient variables for math
  int num_height_tris = (num_height_points - 1) * 2;
  int num_width_tris = (num_width_points - 1) * 2;

  bool topLeft = true;
  for (int i = 0; i < triangles.size(); i++) {
    Triangle *t = triangles[i];

    if (topLeft) {
      // Get left triangle, if it exists
      if (i % num_width_tris != 0) { // Not a left-most triangle
        Triangle *temp = triangles[i - 1];
        t->pm1->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm1->halfedge->twin = nullptr;
      }

      // Get triangle above, if it exists
      if (i >= num_width_tris) { // Not a top-most triangle
        Triangle *temp = triangles[i - num_width_tris + 1];
        t->pm3->halfedge->twin = temp->pm2->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle to bottom right; guaranteed to exist
      Triangle *temp = triangles[i + 1];
      t->pm2->halfedge->twin = temp->pm1->halfedge;
    } else {
      // Get right triangle, if it exists
      if (i % num_width_tris != num_width_tris - 1) { // Not a right-most triangle
        Triangle *temp = triangles[i + 1];
        t->pm3->halfedge->twin = temp->pm1->halfedge;
      } else {
        t->pm3->halfedge->twin = nullptr;
      }

      // Get triangle below, if it exists
      if (i + num_width_tris - 1 < 1.0f * num_width_tris * num_height_tris / 2.0f) { // Not a bottom-most triangle
        Triangle *temp = triangles[i + num_width_tris - 1];
        t->pm2->halfedge->twin = temp->pm3->halfedge;
      } else {
        t->pm2->halfedge->twin = nullptr;
      }

      // Get triangle to top left; guaranteed to exist
      Triangle *temp = triangles[i - 1];
      t->pm1->halfedge->twin = temp->pm2->halfedge;
    }

    topLeft = !topLeft;
  }

  clothMesh->triangles = triangles;
  this->clothMesh = clothMesh;
}
