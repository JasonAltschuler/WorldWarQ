// Source file for the particle system



// Include files

#include "R2/R2.h"
#include "R3/R3.h"
#include "R3Scene.h"
#include "particle.h"
#include <vector>
#include <iostream>
#include <deque>


#define PI 3.14159


////////////////////////////////////////////////////////////
// Random Number Generator
////////////////////////////////////////////////////////////

static double 
RandomNumber(void) 
{
#if defined(_WIN32)
  int r1 = rand();
  double r2 = ((double) rand()) / ((double) (RAND_MAX + 1));
  return (r1 + r2) / ((double) (RAND_MAX + 1));
#else
  return drand48();
#endif
}


void PrintDebuggingInfo(R3Scene * scene)
{
  for (int i = 0; i < scene->NParticles(); i++)
  {
    R3Particle * particle = scene->Particle(i);
    cout << particle->position[0] << " " << particle->position[1] << " " << particle->position[2] << ' '
        << particle->velocity[0] << " " << particle->velocity[1] << " " << particle->velocity[2] << endl;
  }
}



////////////////////////////////////////////////////////////
// Rendering Particles
////////////////////////////////////////////////////////////

void RenderParticles(R3Scene *scene, double current_time, double delta_time)
{
  // Draw every particle
  glDisable(GL_LIGHTING);
  glPointSize(3);
  glBegin(GL_POINTS);
  for (int i = 0; i < scene->NParticles(); i++) {
      R3Particle *particle = scene->Particle(i);
      if (particle->is_bullet)
          glColor3d(1, .6, 0);
      else
          glColor3d(particle->material->kd[0], particle->material->kd[1], particle->material->kd[2]);
      const R3Point& position = particle->position;
      glVertex3d(position[0], position[1], position[2]);
  }
//  glEnd();

  // Trails (implemented for one point)

  // remember the positions of the particles for the last K time steps
  const int K_TRAIL_SIZE = 30; // hardcoded, but can be changed easily

  glBegin(GL_POINTS);
  for (int i = 0; i < scene->NParticles(); i++)
  {
    // update trails
    R3Particle *particle = scene->Particle(i);
    deque<R3Point> &trail = particle->trail;
    trail.push_front(particle->position);
    if (trail.size() > K_TRAIL_SIZE)
      trail.pop_back();

    // draw trails
    R3Rgb material_color = particle->material->kd;
    for (int j = 0; j < trail.size(); j++)
    {
      double c = (double) j / (double) trail.size();
      R3Rgb interpolated_color = material_color * c;

      glColor3d(interpolated_color.Red(), interpolated_color.Green(), interpolated_color.Blue());
      glLineWidth(2);

      glVertex3d(trail[j].X(), trail[j].Y(), trail[j].Z());
    }

  }
  glEnd();


//  for (int j = 0; j < deque.size() - 1; j++) {
//    // interpolate between black and (0.0, 0.0, 1.0) blue for color of line,
//    // with the end of the trail closer to black because it is fading out
//    R3Rgb white(1.0, 1.0, 1.0, 1.0);
//    R3Rgb black(0.0, 0.0, 0.0, 1.0);
//
//    double c = (double) j / (double) deque.size();
//    R3Rgb interpolated_color = white * (1 - c) + c * black;
//    interpolated_color.Clamp();
//
//    vector<R3Point> vec_after = deque[j];
//    vector<R3Point> vec_before = deque[j + 1];
//    int num_particles = min(vec_after.size(), vec_before.size());
//
//    glColor3d(interpolated_color.Red(), interpolated_color.Green(), interpolated_color.Blue());
//    glLineWidth(2);
//    for (int i = 0; i < num_particles; i++)
//    {
//      glVertex3d(vec_after[i].X(), vec_after[i].Y(), vec_after[i].Z());
////      glVertex3d(vec_before[i].X(), vec_before[i].Y(), vec_before[i].Z());
//    }
//  }
//
//  glEnd();
}



////////////////////////////////////////////////////////////
// Generating Particles
////////////////////////////////////////////////////////////


void GenerateParticle(R3Scene *scene, R3ParticleSource *source)
{
  if (source->shape->type == R3_SPHERE_SHAPE)
    {
      R3Sphere *sphere = source->shape->sphere;
      R3Point center = sphere->Center();
      double radius = sphere->Radius();

      // calculate point of emanation
      double z = RandomNumber() * 2 * radius - radius;
      double phi = RandomNumber() * 2 * PI;
      double d = sqrt(radius * radius - z * z);
      double px = center.X() + d * cos(phi);
      double py = center.Y() + d * sin(phi);
      double pz = center.Z() + z;
      R3Point point_emanate(px, py, pz);

      // calculate direction of emanation
      R3Vector N = (point_emanate - center); // surface normal
      N.Normalize();
      R3Vector direction_emanate = (N.X() == 0 && N.Y() == 0) ?
          R3Vector(1.0, 0.0, 0.0) : R3Vector(N.Y(), -N.X(), 0.0); // vector on tangent plane (handle edge cases)
      double t1 = RandomNumber() * 2 * PI;
      double t2 = RandomNumber() * sin(source->angle_cutoff);
      direction_emanate.Rotate(N, t1);
      R3Vector cross = direction_emanate;
      cross.Cross(N);
      direction_emanate.Rotate(cross, acos(t2));
      direction_emanate.Normalize();

      // generate a particle
      R3Particle * new_particle = new R3Particle(point_emanate, direction_emanate, source);
      scene->particles.push_back(new_particle);
    }

    else if (source->shape->type == R3_CIRCLE_SHAPE)
    {
      R3Circle *circle = source->shape->circle;
      R3Point center = circle->Center();
      double radius = circle->Radius();
      R3Plane plane = circle->Plane();

      // first, find two perpendicular basis vectors for the circle's plane
      double A = plane.A();
      double B = plane.B();
      double C = plane.C();
      double D = plane.D();
      R3Vector vec1(0, 0, 0);
      if (C != 0)
        vec1 = R3Vector(1, 1, -(A + B + D) / C); // x = 1, y = 1, z = ? --> -(A + B + D) = C*z
      else if (B != 0)
        vec1 = R3Vector(1, -(A + C+ D) / B, 1); // x = 1, y = ?, z = 1 --> -(A + C + D) = B*y
      else
        vec1 = R3Vector(-(B + C + D) / A, 1, 1); // x = ?, y = 1, z = 1 --> -(B + C + D) = A*x
      R3Vector vec2 = vec1;
      vec2.Cross(plane.Normal());
      assert(vec1.Length() != 0);
      assert(vec2.Length() != 0);
      vec1.Normalize();
      vec2.Normalize();

      // Note: used this source for picking a point: http://mathworld.wolfram.com/DiskPointPicking.html
      // calculate point of emanation
      double r = sqrt(RandomNumber() * radius);
      double theta = RandomNumber() * 2 * PI;
      double cartesian_x = r * cos(theta);
      double cartesian_y = r * sin(theta);
      R3Point point_emanate = center + cartesian_x * vec1 + cartesian_y * vec2;

      // calculate direction of emanation
      R3Vector N = plane.Normal();
      double t1 = RandomNumber() * 2 * PI;
      double t2 = RandomNumber() * sin(source->angle_cutoff);
      vec1.Rotate(N, t1);
      R3Vector cross = vec1;
      cross.Cross(N);
      vec1.Rotate(cross, acos(t2));
      vec1.Normalize();
      R3Vector direction_emanate = vec1;

      // generate a particle
      R3Particle * new_particle = new R3Particle(point_emanate, direction_emanate, source);
      scene->particles.push_back(new_particle);
    }

    else if (source->shape->type == R3_MESH_SHAPE)
    {
      R3Mesh * mesh = source->shape->mesh;
      vector<R3MeshFace *> faces = mesh->faces;

      // calculate total surface area of mesh
      double total_mesh_area = 0;
      for (int i = 0; i < faces.size(); i++)
      {
       total_mesh_area += faces[i]->Area();
      }

      // select a face from which to generate emanating particle, with distribution
      // proportional to each face's area
      double rand1 = RandomNumber() * total_mesh_area;
      double temp1 = faces[0]->Area();
      int generating_face = 0;
      while (rand1 >= temp1)
      {
        generating_face++;
        temp1 += faces[generating_face]->Area();
      }

      assert(generating_face < faces.size());
      vector<R3MeshVertex *> vertices = faces[generating_face]->vertices;
      assert(vertices.size() >= 3);

      // select a triangle from which to generate the emanating particle, with distribution
      // proportional to its area
      vector<R3MeshFace *> triangles(vertices.size() - 2);

      double total_face_area = 0;
      for (int i = 2; i < vertices.size(); i++)
      {
        vector<R3MeshVertex *> new_triangle_vertices(3);
        new_triangle_vertices[0] = vertices[0];
        new_triangle_vertices[1] = vertices[1];
        new_triangle_vertices[2] = vertices[i];

        R3MeshFace * new_triangle = new R3MeshFace(new_triangle_vertices);
        triangles[i - 2] = new_triangle;

        total_face_area += new_triangle->Area();
      }

      // select a triangle from the face
      double rand2 = RandomNumber() * total_face_area;
      double temp2 = triangles[0]->Area();
      int generating_triangle = 0;
      while (rand2 >= temp2)
      {
        generating_triangle++;
        temp2 += triangles[generating_triangle]->Area();
      }

      assert(generating_triangle < triangles.size());

      R3MeshFace *triangle = triangles[generating_triangle];
      R3Vector v1 = triangle->vertices[1]->position - triangle->vertices[0]->position;
      R3Vector v2 = triangle->vertices[2]->position - triangle->vertices[0]->position;

      double a1 = 1;
      double a2 = 1;
      while (a1 + a2 >= 1)
      {
        a1 = RandomNumber();
        a2 = RandomNumber();
      }

      R3Point point_emanate = triangle->vertices[0]->position + a1 * v1 + a2 * v2;

      // calculate direction of emanation
      R3Vector N = triangle->plane.Normal();
      double t1 = RandomNumber() * 2 * PI;
      double t2 = RandomNumber() * sin(source->angle_cutoff);
      v1.Rotate(N, t1);
      R3Vector cross = v1;
      cross.Cross(N);
      v1.Rotate(cross, acos(t2));
      v1.Normalize();
      R3Vector direction_emanate = v1;

      // generate a particle
      R3Particle * new_particle = new R3Particle(point_emanate, direction_emanate, source);
      scene->particles.push_back(new_particle);


      // deallocate memory allocated with new
      for (int i = 0; i < triangles.size(); i++)
        delete triangles[i];
    }

    else if (source->shape->type == R3_BOX_SHAPE)
    {
      R3Box * box = source->shape->box;

      // we are going to be lazy here and reuse the code I wrote directly above for R3Meshes
      vector<R3MeshFace *> faces(6);

      R3Point a_point(box->XMax(), box->YMin(), box->ZMin());
      R3Point b_point(box->XMax(), box->YMax(), box->ZMin());
      R3Point c_point(box->XMin(), box->YMax(), box->ZMin());
      R3Point d_point(box->XMin(), box->YMin(), box->ZMin());
      R3Point e_point(box->XMax(), box->YMin(), box->ZMax());
      R3Point f_point(box->XMax(), box->YMax(), box->ZMax());
      R3Point g_point(box->XMin(), box->YMax(), box->ZMax());
      R3Point h_point(box->XMin(), box->YMin(), box->ZMax());

      R3MeshVertex *a = new R3MeshVertex(a_point, R3null_vector, R2null_point);
      R3MeshVertex *b = new R3MeshVertex(b_point, R3null_vector, R2null_point);
      R3MeshVertex *c = new R3MeshVertex(c_point, R3null_vector, R2null_point);
      R3MeshVertex *d = new R3MeshVertex(d_point, R3null_vector, R2null_point);
      R3MeshVertex *e = new R3MeshVertex(e_point, R3null_vector, R2null_point);
      R3MeshVertex *f = new R3MeshVertex(f_point, R3null_vector, R2null_point);
      R3MeshVertex *g = new R3MeshVertex(g_point, R3null_vector, R2null_point);
      R3MeshVertex *h = new R3MeshVertex(h_point, R3null_vector, R2null_point);

      vector<R3MeshVertex *> face_vertices(4);

      face_vertices[0] = a;
      face_vertices[1] = b;
      face_vertices[2] = c;
      face_vertices[3] = d;
      R3MeshFace * face0 = new R3MeshFace(face_vertices);
      faces[0] = face0;

      face_vertices[0] = a;
      face_vertices[1] = b;
      face_vertices[2] = f;
      face_vertices[3] = e;
      R3MeshFace * face1 = new R3MeshFace(face_vertices);
      faces[1] = face1;

      face_vertices[0] = b;
      face_vertices[1] = c;
      face_vertices[2] = g;
      face_vertices[3] = f;
      R3MeshFace * face2 = new R3MeshFace(face_vertices);
      faces[2] = face2;

      face_vertices[0] = c;
      face_vertices[1] = d;
      face_vertices[2] = h;
      face_vertices[3] = g;
      R3MeshFace * face3 = new R3MeshFace(face_vertices);
      faces[3] = face3;

      face_vertices[0] = a;
      face_vertices[1] = e;
      face_vertices[2] = h;
      face_vertices[3] = d;
      R3MeshFace * face4 = new R3MeshFace(face_vertices);
      faces[4] = face4;

      face_vertices[0] = f;
      face_vertices[1] = g;
      face_vertices[2] = h;
      face_vertices[3] = e;
      R3MeshFace * face5 = new R3MeshFace(face_vertices);
      faces[5] = face5;

      // create mesh out of the box's faces
      R3Mesh *box_mesh = new R3Mesh();
      box_mesh->faces = faces;

      R3Shape *mesh_shape = new R3Shape();
      mesh_shape->type = R3_MESH_SHAPE;
      mesh_shape->mesh = box_mesh;

      // create a R3ParticleSource identical to mesh_source, except with a box
      R3ParticleSource mesh_source;
      mesh_source.shape = mesh_shape;
      mesh_source.rate = source->rate;
      mesh_source.velocity = source->velocity;
      mesh_source.angle_cutoff = source->angle_cutoff;
      mesh_source.mass = source->mass;
      mesh_source.fixed = source->fixed;
      mesh_source.drag = source->drag;
      mesh_source.elasticity = source->elasticity;
      mesh_source.lifetime = source->lifetime;
      mesh_source.material = source->material;

      GenerateParticle(scene, &mesh_source);

      // deallocate memory
      delete a;
      delete b;
      delete c;
      delete d;
      delete e;
      delete f;
      delete g;
      delete h;
      for (int i = 0; i < faces.size(); i++)
        delete faces[i];

    }


    else
    {
      // haven't implemented for other source shapes
    }
}

void GenerateParticles(R3Scene *scene, double current_time, double delta_time)
{
  // Generate particles for exhaust for each airplane's engines (sources)
  for (int i = 0; i < scene->NAircrafts(); i++)
  {
    if (i != 0 || scene->Aircraft(0)->freeze_time <= 0)
    {

      vector<R3ParticleSource *> sources = scene->Aircraft(i)->sources;
      for (int j = 0; j < sources.size(); j++)
      {
        R3ParticleSource * source = sources[j];

        // calculate number of particles to generate (see https://piazza.com/class/hqsinyn8h2i6h2?cid=252)
        double mult_rate = source->rate * delta_time;
        int num_to_generate = floor(mult_rate);
        if (RandomNumber() < (mult_rate - num_to_generate))
          num_to_generate++;

        for (int k = 0; k < num_to_generate; k++)
          GenerateParticle(scene, source);
      }
    }
  }


  // Generate new particles for every source
//  for (int i = 0; i < scene->NParticleSources(); i++)
//  {
//    R3ParticleSource * source = scene->ParticleSource(i);
//
//    // calculate number of particles to generate (see https://piazza.com/class/hqsinyn8h2i6h2?cid=252)
//    double mult_rate = source->rate * delta_time;
//    int num_to_generate = floor(mult_rate);
//    if (RandomNumber() < (mult_rate - num_to_generate))
//      num_to_generate++;
//
//    for (int j = 0; j < num_to_generate; j++)
//    {
//      GenerateParticle(scene, source);
//    }
//  }
}


////////////////////////////////////////////////////////////
// Updating Particles
////////////////////////////////////////////////////////////

void EndParticles(R3Scene *scene, double delta_time)
{
  // account for time taken in delta time step. Then delete the particles if necessary.
  for (size_t i = 0; i < scene->NParticles(); )
  {
    scene->particles[i]->lifetime -= delta_time;
    if (scene->particles[i]->lifetime <= 0) {
//      R3Particle *particle = scene->particles[i]; // TODO: delete later
      scene->particles.erase(scene->particles.begin() + i);
//      delete particle;
    }
    else
      i++;
  }
}

R3Vector ForceOnParticle(R3Particle const * particle, int index, R3Scene * scene)
{
  if (particle->fixed)
    return R3zero_vector;

  R3Vector net_force(0, 0, 0);

  // calculate gravity between particles
  for (int i = 0; i < scene->NParticles(); i++)
  {
    R3Particle * other = scene->Particle(i);
    if (i == index)
      continue;

    static double GRAVITATIONAL_CONSTANT = 6.674e-11;

    double distance = (particle->position - other->position).Length();
    R3Vector direction = other->position - particle->position;
    direction.Normalize();

    R3Vector g_force = ((GRAVITATIONAL_CONSTANT * particle->mass * other->mass) / (distance * distance)) * direction;
    net_force += g_force;
  }

  // Account for scene gravity
  net_force += (particle->mass * scene->gravity);

  // Account for drag
  net_force += (-particle->velocity * particle->drag);

  // Account for springs
  vector<R3ParticleSpring *> springs = particle->springs;
  for (int i = 0; i < springs.size(); i++)
  {
    R3ParticleSpring *spring = springs[i];
    R3Particle *other = spring->OtherParticle(particle);
    double d = (particle->position - other->position).Length();
    R3Vector direction = other->position - particle->position;
    direction.Normalize();

    R3Vector spring_force = direction * (spring->ks * (d - spring->rest_length) +
        spring->kd * (other->velocity - particle->velocity).Dot(direction));

    net_force += spring_force;
  }


  // Account for sinks
  for (int i = 0; i < scene->NParticleSinks(); i++)
  {
    R3ParticleSink * sink = scene->ParticleSink(i);

    if (sink->shape->type == R3_SPHERE_SHAPE)
    {
      R3Ray ray(particle->position, sink->shape->sphere->Center());
      R3Intersection intersection = IntersectRayWithSphere(ray, sink->shape->sphere, scene->Root());
      double d = intersection.distance;
      assert(abs((intersection.position - particle->position).Length() - d) < 0.01);

      double denom = sink->constant_attenuation + sink->linear_attenuation * d +
          sink->quadratic_attenuation * d * d;
      R3Vector sink_direction = ray.Vector();
      sink_direction.Normalize();
      R3Vector sink_force = (sink->intensity / denom) * sink_direction;

      net_force += sink_force;
    }


    else if (sink->shape->type == R3_CIRCLE_SHAPE)
    {
      // let P denote particle->position. let O denote the center of the circle. let 'plane' denote
      // the plane that the circle is. I will sketch a proof that to find the distance
      // between a point and circle, we need only intersect P' with the circle, where
      // P' is the projection of P onto the plane. Proof: consider point A constructed in the method just
      // stated and point B a different point on the circle s.t. B != A. Let d denote the distance
      // from P to P' (equals the distance of P to the plane). Then:
      // |PA|^2 = d^2 + |AP'|^2 <= d^2 + |BP'|^2 = |PB|^2. QED

      R3Circle *circle = sink->shape->circle;
      R3Plane plane = circle->Plane();
      R3Point P_projected = particle->position;
      P_projected.Project(plane);

      R3Ray ray(P_projected, circle->Center());
      R3Sphere fake_sphere(circle->Center(), circle->Radius());
      R3Intersection intersection = IntersectRayWithSphere(ray, &fake_sphere, scene->Root());

      double proj_distance = (particle->position - P_projected).Length();
      double d = sqrt(intersection.distance * intersection.distance + proj_distance * proj_distance);

      double denom = sink->constant_attenuation + sink->linear_attenuation * d +
          sink->quadratic_attenuation * d * d;
      R3Vector sink_direction = intersection.position - particle->position;
      sink_direction.Normalize();
      R3Vector sink_force = (sink->intensity / denom) * sink_direction;

      net_force += sink_force;
    }
  }

  return net_force;
}





void ComputeUpdatedParticle(int i, R3Scene *scene, double current_time, double delta_time,
    int integration_type, vector<R3Point>& positions, vector<R3Vector>& velocities)
{

  // Get pointer to i'th particle
  R3Particle *particle = scene->Particle(i);

  if (particle->fixed)
  {
    positions[i] = particle->position;
    velocities[i] = R3zero_vector;
    return;
  }

  if (integration_type == EULER_INTEGRATION)
  {
    // Get total force
    R3Vector force(0, 0, 0);

    // Advance simulation
    R3Vector acceleration = force / particle->mass;
    R3Vector velocity = particle->velocity + acceleration * delta_time;
    R3Point position = particle->position + particle->velocity * delta_time;

    velocities[i] = velocity;
    positions[i] = position;
  }


  if (integration_type == MIDPOINT_INTEGRATION)
  {
    // Compute an Euler step
    R3Vector force_begin = ForceOnParticle(particle, i, scene);
    R3Vector acceleration_begin = force_begin / particle->mass;
    R3Point position_mid = particle->position + particle->velocity * delta_time / 2.0;
    R3Vector velocity_mid = particle->velocity + acceleration_begin * delta_time / 2.0;

    // Evaluate f at the midpoint of Euler step
    R3Particle particle_mid;
    particle_mid.mass = particle->mass;
    particle_mid.position = position_mid;
    particle_mid.velocity - velocity_mid;
    R3Vector force_mid = ForceOnParticle(&particle_mid, i, scene);
    R3Vector acceleration_mid = force_mid / particle->mass;

    // Compute new position / velocity using midpoint velocity / acceleration
    R3Vector velocity = particle->velocity + acceleration_mid * delta_time;
    R3Point position = particle->position + velocity_mid * delta_time;

    velocities[i] = velocity;
    positions[i] = position;
  }
}

// Kyle on Wednesday:
// this method is very slow for bullets, because every bullet every frame must compute intersection with the ground.
// easy fix: just don't do collisions for bullets
// hard fix: because bullets just move in a straight line, compute the time until it will intersect with the ground just once.
//          Then, set the particle's lifetime to be that time. Or do something more clever so we can animate bullets hitting the ground.

void UpdateParticles(R3Scene *scene, double current_time, double delta_time, int integration_type)
{
  // lifetime <= 0 to start with means the particle lives "forever"
  static bool first_time = true;
  if (first_time)
  {
    first_time = false;
    for (int i = 0; i < scene->NParticles(); i++)
    {
      if (scene->particles[i]->lifetime <= 0)
        scene->particles[i]->lifetime = DOUBLE_MAX; // not "forever" technically, but close enough, since
                                                    // delta_time_step is so small
    }
  }


  // update particles (includes particle-scene collision detection)
  for (int i = 0; i < scene->NParticles(); i++)
  {
    R3Particle *particle = scene->Particle(i);

    if (particle->fixed)
      particle->velocity = R3zero_vector;

    // modified by Jason so bullets intersect with airplanes, but not mesh (on May 10)
    // modified by Kyle so bullets are not computing intersections (before May 10)
    else if (particle->is_bullet)
    {
      R3Point new_position = particle->position + particle->velocity * delta_time;

      // calculate intersections with other aircrafts
      R3Ray ray(particle->position, new_position);
      R3Intersection closest_intersection = ComputeIntersectionWithAircrafts(scene, ray, particle);

      // make sure intersection is actually an intersection
      if (closest_intersection.IsHit())
      {
        double t = ray.T(new_position);
        double t_collision = ray.T(closest_intersection.position);
        if (t_collision < 0 || t_collision > t)
          closest_intersection.SetMiss();
      }

      if (!closest_intersection.IsHit())
      {
        particle->position = new_position;
        // velocity of bullet is constant because we assume no forces (fast enough)
      }

      else
      {
        particle->position = new_position;
        particle->velocity = R3zero_vector;
        particle->lifetime = 3;

        cout << "HIT" << endl;
        closest_intersection.aircraft->HitAircraft(scene);
      }
    }


    // only executed for schrapnel from aircraft explosions: don't check for intersections with scene here
    else
    {
      particle->position += particle->velocity * delta_time;
    }
  }


  EndParticles(scene, delta_time);


//  PrintDebuggingInfo(scene);
}














//    else
//    {
//      // check for collisions iteratively (if one collision, then "reverse" velocity, and
//      // check for more collisions in that direction etc.)
//      double remaining_time = delta_time;
//      R3Point old_position = particle->position;
//      R3Point new_position = positions[i];
//      R3Vector new_velocity = velocities[i];
//
//      while (remaining_time > 0)
//      {
//        double delta_distance = (old_position - new_position).Length();
//        R3Ray ray(old_position, new_position);
//
//        // check for intersection with scene graph (note that sinks and sources are NOT in the scene graph)
//        R3Intersection closest_intersection = ComputeIntersection(scene, scene->Root(), ray);
//
//        // check for intersections with sinks
//        //        for (int j = 0; j < scene->NParticleSinks(); j++)
//        //        {
//        //          R3ParticleSink *sink = scene->ParticleSink(j);
//        //          R3Intersection sink_intersection;
//        //
//        //          if (sink->shape->type == R3_SPHERE_SHAPE)
//        //          {
//        //            sink_intersection = IntersectRayWithSphere(ray, sink->shape->sphere, scene->Root());
//        //          }
//        //
//        //          else if (sink->shape->type == R3_CIRCLE_SHAPE)
//        //          {
//        //            R3Circle *circle = sink->shape->circle;
//        //            R3Plane plane = circle->Plane();
//        //            R3Point P = old_position;
//        //            R3Point P_projected = old_position;
//        //            P_projected.Project(plane);
//        //
//        //            // edge case of ray is on the circle's plane:
//        //            if (((P - P_projected).Length() < 0.0001) &&
//        //                abs(ray.Vector().Dot(plane.Normal())) < 0.0001)
//        //            {
//        //              R3Sphere fake_sphere(circle->Center(), circle->Radius());
//        //              sink_intersection = IntersectRayWithSphere(ray, &fake_sphere, scene->Root());
//        //            }
//        //
//        //            // normal case: ray is not on the circle's plane:
//        //            else
//        //            {
//        //              double t = -(R3Vector(old_position.X(), old_position.Y(), old_position.Z()).Dot(plane.Normal()) + plane.D()) /
//        //                  (ray.Vector().Dot(plane.Normal()));
//        //              R3Point plane_intersection_point = old_position + t * ray.Vector();
//        //
//        //              if ((t > 0.00001) && ((plane_intersection_point - circle->Center()).Length() <= circle->Radius()))
//        //              {
//        //                R3Intersection plane_intersection;
//        //                plane_intersection.hit = true;
//        //                plane_intersection.node = scene->Root();
//        //                plane_intersection.position = plane_intersection_point;
//        //                plane_intersection.normal = plane.Normal();
//        //                if (ray.Vector().Dot(plane_intersection.normal) > 0)
//        //                  plane_intersection.normal = -plane_intersection.normal;
//        //                plane_intersection.normal.Normalize();
//        //                plane_intersection.distance = (plane_intersection.position - old_position).Length();
//        //                plane_intersection.t = ray.T(plane_intersection_point);
//        //                plane_intersection.AssertValid();
//        //
//        //                sink_intersection = plane_intersection;
//        //              }
//        //            }
//        //          }
//
//        //          if (sink_intersection.IsHit()) // implementation assures that this is false if no intesection calcualted above (see constructor)
//        //          {
//        //            if ((!closest_intersection.IsHit()) ||
//        //                (closest_intersection.IsHit() && sink_intersection.distance < closest_intersection.distance))
//        //            {
//        //              closest_intersection = sink_intersection;
//        //            }
//        //          }
//        //        }
//
//
//        // if no collision
//        if (!closest_intersection.IsHit() || (closest_intersection.IsHit() && closest_intersection.distance > delta_distance))
//        {
//          particle->position = new_position;
//          particle->velocity = new_velocity;
//          break;
//        }
//
//        // if collision
//        else
//        {
//
//          // collision with sink --> end the particle
//          if (closest_intersection.node == scene->Root())
//          {
//            particle->lifetime = 0; // end the particle after this for loop
//            particle->position = closest_intersection.position;
//            particle->velocity = R3zero_vector;
//            remaining_time = 0;
//            break;
//          }
//
//          // not just particle->velocity if non-Euler integration type
//          R3Vector old_velocity = (new_position - old_position) / remaining_time;
//          assert(old_velocity.Length() > 0);
//
//          // get info about collision
//          double collision_time = closest_intersection.distance / old_velocity.Length(); // distance = rate * time
//          remaining_time -= collision_time;
//          assert(remaining_time >= 0);
//
//          // get info about bounce
//          R3Vector N = closest_intersection.normal;
//          assert(abs(N.Length() - 1) < 0.001);
//
//          // reflect over normal
//          double V_old_length = old_velocity.Length();
//          R3Vector V_old = -old_velocity;
//          V_old.Normalize();
//          R3Vector R_old = 2 * N * (N.Dot(V_old)) - V_old;
//          R_old *= V_old_length;
//
//          double V_new_length = new_velocity.Length();
//          R3Vector V_new = -new_velocity;
//          V_new.Normalize();
//          R3Vector R_new = 2 * N * (N.Dot(V_new) / V_new.Length()) - V_new;
//          R_new *= V_new_length;
//
//          // adjust velocity for elasticity
//          R3Vector projection_old = R_old;
//          projection_old.Project(N);
//          R3Vector projection_new = R_new;
//          projection_new.Project(N);
//
//          old_velocity = R_old + ((particle->elasticity - 1) * projection_old);
//          new_velocity = R_new + ((particle->elasticity - 1) * projection_new);
//
//          // update positions after bounce
//          old_position = closest_intersection.position;
//          new_position = old_position + remaining_time * old_velocity; // correct regardless of integration type
//        }
//      }
//    }




















//========================== Intersection Computations =========================== //

// construct small epsilon s.t. intersection t must be > epsilon to count (talked about in precept)
static const double EPSILON = 0.00001;



R3Intersection ComputeIntersectionWithAircrafts(R3Scene* scene, R3Ray ray, R3Particle *bullet)
{
  assert(scene != NULL);

  R3Intersection closest_intersection;

  for (int i = 0; i < scene->NAircrafts(); i++)
  {
    R3Aircraft *aircraft = scene->Aircraft(i);

    // don't count intersections with the plane that fired the bullet
    if (aircraft == bullet->aircraft_fired_from)
      continue;

    R3Ray new_ray = ray;
    new_ray.InverseTransform(aircraft->T);

    // speed intersection check up by just intersecting with a box
    //    R3Intersection aircraft_intersection = IntersectRayWithMesh(new_ray, aircraft->mesh, NULL);
    R3Intersection aircraft_intersection = IntersectRayWithBox(new_ray, &aircraft->mesh->bbox, NULL);
    if (aircraft_intersection.IsHit())
      aircraft_intersection.aircraft = aircraft;
    aircraft_intersection.AssertValid();

    if (aircraft_intersection.IsHit() && aircraft_intersection.t >= EPSILON)
    {
      aircraft_intersection.Transform(aircraft->T, ray);
      aircraft_intersection.AssertValid();

      if (aircraft_intersection.distance < closest_intersection.distance)
        closest_intersection = aircraft_intersection;
    }
  }

  return closest_intersection;
}




R3Intersection ComputeIntersection(R3Scene *scene, R3Node *node, R3Ray ray)
{
  assert(node != NULL);
  assert(scene != NULL);

  R3Intersection closest_intersection;

  // Caching last intersection for acceleration (1 point)
  static R3Node *last_intersection_node = NULL;
  if (node == scene->root && last_intersection_node != NULL) // only do if on bottom (1st) level of recursion
  {
    // get all transformations
    vector<R3Matrix> vector_transformations(0);
    R3Node *current = last_intersection_node;
    vector_transformations.push_back(current->transformation);
    while (current != scene->root)
    {
      current = current->parent;
      vector_transformations.push_back(current->transformation);
    }

    // do all transformations to the ray in opposite order (from root down)
    int num_transforms = vector_transformations.size();
    vector<R3Ray> transformed_ray(num_transforms + 1);
    transformed_ray[0] = ray;
    for (int i = 0; i < num_transforms; i++)
    {
      transformed_ray[i + 1] = transformed_ray[i];
      transformed_ray[i + 1].InverseTransform(vector_transformations[num_transforms - i - 1]);
    }

    // find the intersection
    R3Intersection accelerated_intersection;

    // if root node or otherwise no shape
    if (last_intersection_node->shape != NULL)
    {
      switch (last_intersection_node->shape->type)
      {
      case R3_SPHERE_SHAPE:
        accelerated_intersection = IntersectRayWithSphere(transformed_ray[num_transforms], (R3Sphere*) last_intersection_node->shape->sphere, last_intersection_node);
        break;
      case R3_BOX_SHAPE:
        accelerated_intersection = IntersectRayWithBox(transformed_ray[num_transforms], (R3Box*) last_intersection_node->shape->box, last_intersection_node);
        break;
      case R3_CYLINDER_SHAPE:
        accelerated_intersection = IntersectRayWithCylinder(transformed_ray[num_transforms], (R3Cylinder*) last_intersection_node->shape->cylinder, last_intersection_node);
        break;
      case R3_CONE_SHAPE:
        accelerated_intersection = IntersectRayWithCone(transformed_ray[num_transforms], (R3Cone*) last_intersection_node->shape->cone, last_intersection_node);
        break;
      case R3_MESH_SHAPE:
        accelerated_intersection = IntersectRayWithMesh(transformed_ray[num_transforms], (R3Mesh*) last_intersection_node->shape->mesh, last_intersection_node);
        break;
      case R3_SEGMENT_SHAPE:
        accelerated_intersection = IntersectRayWithSegment(transformed_ray[num_transforms], (R3Segment*) last_intersection_node->shape->segment, last_intersection_node);
        break;
      default:
        exit(1);
      }
    }

    // undo all transformations if intersection is hit
    if (accelerated_intersection.IsHit() && accelerated_intersection.t >= EPSILON) {
      closest_intersection = accelerated_intersection;
      closest_intersection.AssertValid();

      for (int i = 0; i < num_transforms - 1; i++)
      {
        closest_intersection.Transform(vector_transformations[i ], transformed_ray[num_transforms - i - 1]);
        closest_intersection.AssertValid();
      }
    }

  }


  // Transform ray by inverse of node's transformation
  R3Ray new_ray = ray;
  new_ray.InverseTransform(node->transformation);

  // Check for intersection with shape
  R3Intersection shape_intersection;

  // if root node or otherwise no shape
  if (node->shape != NULL)
  {
    switch (node->shape->type)
    {
    case R3_SPHERE_SHAPE:
      shape_intersection = IntersectRayWithSphere(new_ray, (R3Sphere*) node->shape->sphere, node);
      break;
    case R3_BOX_SHAPE:
      shape_intersection = IntersectRayWithBox(new_ray, (R3Box*) node->shape->box, node);
      break;
    case R3_CYLINDER_SHAPE:
      shape_intersection = IntersectRayWithCylinder(new_ray, (R3Cylinder*) node->shape->cylinder, node);
      break;
    case R3_CONE_SHAPE:
      shape_intersection = IntersectRayWithCone(new_ray, (R3Cone*) node->shape->cone, node);
      break;
    case R3_MESH_SHAPE:
      shape_intersection = IntersectRayWithMesh(new_ray, (R3Mesh*) node->shape->mesh, node);
      break;
    case R3_SEGMENT_SHAPE:
      shape_intersection = IntersectRayWithSegment(new_ray, (R3Segment*) node->shape->segment, node);
      break;
    default:
      exit(1);
    }
  }


  if (shape_intersection.IsHit() && shape_intersection.t >= EPSILON) {
    closest_intersection = shape_intersection;
  }


  // Check for intersection with children nodes
  int num_children = node->children.size();

  // Accelerate ray-scene intersection by visiting children nodes in front-to-back order

  // pairs of t values (for intersection with bbox) and the corresponding nodes
  std::vector<std::pair<double, R3Node*> > children_to_check(0);
  for (int i = 0; i < num_children; i++)
  {
    R3Node* current_child = (node->children)[i];

    // BOUNDING BOX ACCELERATION!!
    // check for intersection with child bounding box first
    R3Intersection bbox_intersection = IntersectRayWithBox(new_ray, &(current_child->bbox), current_child);

    // only bother checking children if hit bbox AND if closest intersection with bbox
    // is better than closest intersection otherwise seen so far
    if (bbox_intersection.IsHit() && (!closest_intersection.IsHit() || bbox_intersection.t < closest_intersection.t))
    {
      children_to_check.push_back( std::pair<double, R3Node*> (bbox_intersection.t, current_child));
    }
  }

  // note that this works because std::pair overloads the < operator to sort by the first element of the pair
  // source: http://stackoverflow.com/questions/4610232/sorting-a-stdvectorstdpairstdstring-bool-by-the-string
  std::sort(children_to_check.begin(), children_to_check.end());


  int num_children_to_check = children_to_check.size();
  for (int i = 0; i < num_children_to_check; i++)
  {
    // if already found a better t from a closer child, then we can stop
    if (closest_intersection.IsHit() && closest_intersection.t <= children_to_check[i].first)
      break;

    R3Node* current_child = children_to_check[i].second;
    R3Intersection child_intersection = ComputeIntersection(scene, current_child, new_ray);
    child_intersection.AssertValid();
    if (child_intersection.IsHit() && child_intersection.t >= EPSILON && child_intersection.distance < closest_intersection.distance)
      closest_intersection = child_intersection;
  }


  // Transform intersection by node's transformation
  if (closest_intersection.IsHit())
  {
    closest_intersection.AssertValid();
    closest_intersection.Transform(node->transformation, ray);
  }

  closest_intersection.AssertValid();

  // If at the bottom (1st) level of recursion, cache the node that was intersected for checking next time
  if (node == scene->root)
  {
    last_intersection_node = closest_intersection.node;
  }

  // Return closest intersection in tree rooted at this node
  return closest_intersection;
}



R3Intersection IntersectRayWithSphere(R3Ray ray, R3Sphere *sphere, R3Node *node)
{
  R3Intersection intersection;
  intersection.SetMiss();
  intersection.AssertValid();

  // follows Ray-Sphere Intersection method II in the slides for March 24
  R3Point P_0 = ray.Start();
  R3Vector V = ray.Vector();
  R3Point O = sphere->Center();
  double r = sphere->Radius();
  double r_squared = r * r;

  R3Vector L = O - P_0;
  double t_ca = L.Dot(V);
  if (t_ca < 0) { // reject (no intersection)
    intersection.SetMiss();
    intersection.AssertValid();
    return intersection;
  }

  double d_squared = L.Dot(L) - t_ca * t_ca;
  if (d_squared > r_squared) { // reject (no intersection)
    intersection.SetMiss();
    intersection.AssertValid();
    return intersection;
  }

  double t_hc = sqrt(r_squared - d_squared);
  double t_1 = t_ca - t_hc;
  double t_2 = t_ca + t_hc;

  // update members of intersection
  if (t_1 >= EPSILON)
    intersection.t = t_1;
  else if (t_2 >= EPSILON)
    intersection.t = t_2;
  else
  {
    intersection.SetMiss();
    intersection.AssertValid();
    return intersection;
  }

  intersection.hit = true;
  intersection.node = node;
  intersection.position = P_0 + intersection.t * V;

  // cases for if ray starts inside or outside sphere
  intersection.normal = O - intersection.position;
  if (V.Dot(intersection.normal) > 0)
    intersection.normal = -intersection.normal;
  intersection.normal.Normalize();

  intersection.distance = intersection.t * V.Length();
  intersection.AssertValid();
  return intersection;
}

// helper function for meshes and boxes
inline bool IsOnCorrectSide(R3Point& T_1, R3Point& T_2, R3Point& P, R3Point& P_0)
{
  R3Vector V_1 = T_1 - P;
  R3Vector V_2 = T_2 - P;

  R3Vector N = V_2;
  N.Cross(V_1);
  N.Normalize();

  return (P - P_0).Dot(N) >= 0;
}


R3Intersection IntersectRayWithMesh(R3Ray ray, R3Mesh *mesh, R3Node *node)
{
  // follows Ray-Sphere Intersection method II in the slides for March 24
  R3Intersection intersection;
  R3Point P_0 = ray.Start();
  R3Vector V = ray.Vector();

  int num_faces = mesh->faces.size();

  for (int f = 0; f < num_faces; f++)
  {
    // Ray-Triangle Intersection II
    vector<R3MeshVertex *> vertices = mesh->faces[f]->vertices;

    // allowed to assume that the meshes are all triangle meshes
    // the only reason I commented this out is so that the Makefile will still run on all images
    // See the comments I put in the writeup.html about stilllife.scn, etc.
    //    assert(vertices.size() == 3);

    R3Point T_1 = vertices[0]->position;
    R3Point T_2 = vertices[1]->position;
    R3Point T_3 = vertices[2]->position;
    R3Plane plane = mesh->faces[f]->plane;


    // check the vertices are in counterclockwise order, from point of view of ray
    // see piazza post https://piazza.com/class/hqsinyn8h2i6h2?cid=203
    if (plane.Normal().Dot(V) > 0)
    {
      // switch the order of vertices
      T_3 = vertices[1]->position;
      T_2 = vertices[2]->position;
      plane = R3Plane(T_1, T_2, T_3);
    }

    double t = -(R3Vector(P_0.X(), P_0.Y(), P_0.Z()).Dot(plane.Normal()) + plane.D()) / (V.Dot(plane.Normal()));
    R3Point P = P_0 + t * V;

    if (IsOnCorrectSide(T_1, T_2, P, P_0) &&
        IsOnCorrectSide(T_2, T_3, P, P_0) &&
        IsOnCorrectSide(T_3, T_1, P, P_0))
    {
      // calculate distance from point to plane
      double dist = t * V.Length();

      // if closest intersection seen so far
      if (t >= EPSILON && dist < intersection.distance)
      {
        // calculate intersection members
        intersection.distance = dist;
        intersection.t = t;
        intersection.hit = true;
        intersection.node = node;
        intersection.position = P;
        intersection.normal = plane.Normal();
        if (V.Dot(intersection.normal) > 0)
          intersection.normal = -intersection.normal;
        intersection.normal.Normalize();
        intersection.AssertValid();
      }
    }
  }

  intersection.AssertValid();
  return intersection;
}

R3Intersection IntersectRayWithBox(R3Ray ray, R3Box *box, R3Node *node)
{
  R3Intersection intersection;
  R3Point P_0 = ray.Start();
  R3Vector V = ray.Vector();

  R3Point box_faces[6][4];

  R3Point a(box->XMax(), box->YMin(), box->ZMin());
  R3Point b(box->XMax(), box->YMax(), box->ZMin());
  R3Point c(box->XMin(), box->YMax(), box->ZMin());
  R3Point d(box->XMin(), box->YMin(), box->ZMin());
  R3Point e(box->XMax(), box->YMin(), box->ZMax());
  R3Point f(box->XMax(), box->YMax(), box->ZMax());
  R3Point g(box->XMin(), box->YMax(), box->ZMax());
  R3Point h(box->XMin(), box->YMin(), box->ZMax());

  box_faces[0][0] = a;
  box_faces[0][1] = b;
  box_faces[0][2] = c;
  box_faces[0][3] = d;

  box_faces[1][0] = a;
  box_faces[1][1] = b;
  box_faces[1][2] = f;
  box_faces[1][3] = e;

  box_faces[2][0] = b;
  box_faces[2][1] = c;
  box_faces[2][2] = g;
  box_faces[2][3] = f;

  box_faces[3][0] = c;
  box_faces[3][1] = d;
  box_faces[3][2] = h;
  box_faces[3][3] = g;

  box_faces[4][0] = a;
  box_faces[4][1] = e;
  box_faces[4][2] = h;
  box_faces[4][3] = d;

  box_faces[5][0] = f;
  box_faces[5][1] = g;
  box_faces[5][2] = h;
  box_faces[5][3] = e;

  for (int f = 0; f < 6; f++)
  {
    // Ray-Triangle Intersection II
    R3Point T_1 = box_faces[f][0];
    R3Point T_2 = box_faces[f][1];
    R3Point T_3 = box_faces[f][2];
    R3Point T_4 = box_faces[f][3];
    // construct plane and assert T4 is also on the same plane
    R3Plane plane = R3Plane(T_1, T_2, T_3);
    assert(abs(plane.Normal().Dot(R3Vector(T_4.X(), T_4.Y(), T_4.Z())) + plane.D()) < 0.01);

    // check the vertices are in counterclockwise order, from point of view of ray
    // see piazza post https://piazza.com/class/hqsinyn8h2i6h2?cid=203
    if (plane.Normal().Dot(V) > 0)
    {
      // switch the order of vertices
      T_2 = box_faces[f][3];
      T_4 = box_faces[f][1];
      R3Plane plane = R3Plane(T_1, T_2, T_3);
      assert(abs(plane.Normal().Dot(R3Vector(T_4.X(), T_4.Y(), T_4.Z())) + plane.D()) < 0.01);
    }

    double t = -(R3Vector(P_0.X(), P_0.Y(), P_0.Z()).Dot(plane.Normal()) + plane.D()) / (V.Dot(plane.Normal()));
    R3Point P = P_0 + t * V;

    // check P is in the face
    if (IsOnCorrectSide(T_1, T_2, P, P_0) &&
        IsOnCorrectSide(T_2, T_3, P, P_0) &&
        IsOnCorrectSide(T_3, T_4, P, P_0) &&
        IsOnCorrectSide(T_4, T_1, P, P_0))
    {
      // calculate distance from point to plane
      double dist = t * V.Length();

      // if closest intersection seen so far
      if (dist >= EPSILON && dist < intersection.distance)
      {
        // calculate intersection members
        intersection.distance = dist;
        intersection.t = t;
        intersection.hit = true;
        intersection.node = node;
        intersection.position = P;
        intersection.normal = plane.Normal();
        if (V.Dot(intersection.normal) > 0)
          intersection.normal = -intersection.normal;
        intersection.AssertValid();
      }
    }
  }

  return intersection;
}


// Note: I used this source as an aid:
// http://mrl.nyu.edu/~dzorin/rendering/lectures/lecture3/lecture3-6pp.pdf
R3Intersection IntersectRayWithCylinder(R3Ray ray, R3Cylinder *cylinder, R3Node *node)
{
  R3Intersection intersection;
  R3Point P_0 = ray.Start();
  R3Vector V = ray.Vector();
  R3Point center = cylinder->Center();
  double radius = cylinder->Radius();
  double height = cylinder->Height();

  // first check both caps
  double top_cap_y = center.Y() + (height/2.0);
  double bottom_cap_y = center.Y() - (height/2.0);
  R3Point top_cap_center = cylinder->Axis().End();
  R3Point bottom_cap_center = cylinder->Axis().Start();

  if (V.Y() == 0) { // "degenerate" case of ray towards caps
    if (P_0.Y() == top_cap_y) {
      R3Sphere fake_sphere(R3Point(center.X(), top_cap_y, center.Z()), radius);
      intersection = IntersectRayWithSphere(ray, &fake_sphere, node);
    } else if (P_0.Y() == bottom_cap_y) {
      R3Sphere fake_sphere(R3Point(center.X(), bottom_cap_y, center.Z()), radius);
      intersection = IntersectRayWithSphere(ray, &fake_sphere, node);
    }
  }
  else { // "non-degenerate" case of ray towards caps
    // calculate points of intersection for both caps
    double t_top    = (top_cap_y - P_0.Y()) / V.Y();
    double t_bottom = (bottom_cap_y - P_0.Y()) / V.Y();
    R3Point P_top = P_0 + t_top * V;
    R3Point P_bottom = P_0 + t_bottom * V;

    // check actually intersected
    double dist1 = (top_cap_center - P_top).Length();
    double dist2 = (bottom_cap_center - P_bottom).Length();
    double top_dist = (P_top - P_0).Length() / V.Length();
    double bottom_dist = (P_bottom - P_0).Length() / V.Length();
    bool top_intersect    = (dist1 * dist1 <= radius * radius) && (top_dist >= EPSILON);
    bool bottom_intersect = (dist2 * dist2 <= radius * radius) && (bottom_dist >= EPSILON);

    // calculate intersection members
    if (top_intersect && t_top >= EPSILON)
    {
      // calculate intersection members
      intersection.hit = true;
      intersection.t = t_top;
      intersection.position = P_top;
      intersection.distance = top_dist;
      intersection.node = node;
      intersection.normal = (P_0.Y() > top_cap_y) ? R3posy_vector : R3negy_vector;
      if (V.Dot(intersection.normal) > 0)
        intersection.normal = -intersection.normal;
      intersection.AssertValid();
    }

    if (bottom_intersect && t_bottom >= EPSILON && bottom_dist < intersection.distance)
    {
      // calculate intersection members
      intersection.hit = true;
      intersection.t = t_bottom;
      intersection.position = P_bottom;
      intersection.distance = bottom_dist;
      intersection.node = node;
      intersection.normal = (P_0.Y() > bottom_cap_y) ? R3posy_vector : R3negy_vector;
      if (V.Dot(intersection.normal) > 0)
        intersection.normal = -intersection.normal;
      intersection.AssertValid();
    }
  }

  // check intersection with the middle of the cylinder (not the caps)
  R3Vector V_a = cylinder->Axis().End() - cylinder->Axis().Start();
  V_a.Normalize();
  R3Vector delta_P = P_0 - cylinder->Axis().Start();
  double A = (V - V.Dot(V_a) * V_a ).Length();
  A *= A; // A^2
  double B = 2 * (V - V.Dot(V_a)*V_a).Dot(delta_P - delta_P.Dot(V_a) * V_a);
  double C = (delta_P - delta_P.Dot(V_a) * V_a).Length();
  C *= C;
  C -= (radius * radius);

  double radical = sqrt(B * B - 4 * A * C);
  double t_1 = (-B - radical) / (2 * A);
  double t_2 = (-B + radical) / (2 * A);

  // check if actually intersections
  R3Point P_1 = P_0 + t_1 * V;
  R3Point P_2 = P_0 + t_2 * V;
  bool intersection_1 = (t_1 >= EPSILON) && (P_1.Y() <= top_cap_y) && (P_1.Y() >= bottom_cap_y);
  bool intersection_2 = (t_2 >= EPSILON) && (P_2.Y() <= top_cap_y) && (P_2.Y() >= top_cap_y);

  double t = -1;
  if (intersection_1)
    t = t_1;
  else if (intersection_2)
    t = t_2;
  else
  {
    intersection.AssertValid();
    return intersection;
  }
  double distance = t * V.Length();
  if (t >= EPSILON && distance < intersection.distance)
  {
    // calculate intersection members
    intersection.hit = true;
    intersection.t = t;
    intersection.position = P_0 + t * V;
    intersection.distance = distance;
    intersection.node = node;
    intersection.normal = intersection.position - R3Point(center.X(), intersection.position.Y(), center.Z());
    if (V.Dot(intersection.normal) > 0)
      intersection.normal = -intersection.normal;
    intersection.AssertValid();
  }

  intersection.AssertValid();
  return intersection;
}


// Note: I used this source as an aid:
// http://mrl.nyu.edu/~dzorin/rendering/lectures/lecture3/lecture3-6pp.pdf
R3Intersection IntersectRayWithCone(R3Ray ray, R3Cone *cone, R3Node *node)
{
  R3Intersection intersection;
  R3Point P_0 = ray.Start();
  R3Vector V = ray.Vector();
  R3Point center = cone->Center();
  double radius = cone->Radius();
  double height = cone->Height();

  // check intersection with cap
  // first check both caps
  double cap_y = center.Y() - (height / 2.0);
  R3Point cap_center = cone->Axis().Start();

  if (V.Y() == 0) { // "degenerate" case of ray towards caps
    if (P_0.Y() == cap_y) {
      R3Sphere fake_sphere(cap_center, radius);
      intersection = IntersectRayWithSphere(ray, &fake_sphere, node);
    }
  }
  else { // "non-degenerate" case of ray towards caps
    // calculate points of intersection for both caps
    double t    = (cap_y - P_0.Y()) / V.Y();
    R3Point P = P_0 + t * V;

    // check actually intersected
    double radial_dist = (cap_center - P).Length();
    double phys_dist = (P - P_0).Length() / V.Length();
    bool intersect = (radial_dist * radial_dist <= radius * radius) && (phys_dist >= EPSILON);

    // calculate intersection members
    if (intersect && t >= EPSILON)
    {
      // calculate intersection members
      intersection.hit = true;
      intersection.t = t;
      intersection.position = P;
      intersection.distance = phys_dist;
      intersection.node = node;
      intersection.normal = (P_0.Y() > cap_y) ? R3posy_vector : R3negy_vector;
      if (V.Dot(intersection.normal) > 0)
        intersection.normal = -intersection.normal;
      intersection.AssertValid();
    }
  }


  // check intersection with the non-cap part of the cylinder
  double theta = atan(radius / height);
  double cos_theta = cos(theta);
  double sin_theta = sin(theta);

  R3Point P_a = center + height / 2.0 * R3posy_vector;
  R3Vector delta_p = P_0 - P_a;
  R3Vector V_a = cone->Axis().End() - cone->Axis().Start();
  V_a.Normalize();
  R3Vector V_proj = V - V.Dot(V_a) * V_a;
  double V_proj_length = V_proj.Length();
  R3Vector delta_p_proj = delta_p - delta_p.Dot(V_a) * V_a;
  double delta_p_proj_length = delta_p_proj.Length();

  double A = cos_theta * cos_theta * V_proj_length * V_proj_length
      - sin_theta * sin_theta * V.Dot(V_a) * V.Dot(V_a);
  double B = 2 * cos_theta * cos_theta * V_proj.Dot(delta_p_proj)
      - 2 * sin_theta * sin_theta * V.Dot(V_a) * delta_p.Dot(V_a);
  double C = cos_theta * cos_theta * delta_p_proj_length * delta_p_proj_length
      - sin_theta * sin_theta * delta_p.Dot(V_a) * delta_p.Dot(V_a);


  double radical = sqrt(B * B - 4 * A * C);
  double t_1 = (-B - radical) / (2 * A);
  double t_2 = (-B + radical) / (2 * A);

  // check if actually intersections
  R3Point P_1 = P_0 + t_1 * V;
  R3Point P_2 = P_0 + t_2 * V;
  double y_max = center.Y() + height / 2.0;
  double y_min = center.Y() - height / 2.0;
  bool intersection_1 = (t_1 >= EPSILON) && (P_1.Y() <= y_max) && (P_1.Y() >= y_min);
  bool intersection_2 = (t_2 >= EPSILON) && (P_2.Y() <= y_max) && (P_2.Y() >= y_min);

  double t = -1;
  if (intersection_1)
    t = t_1;
  else if (intersection_2)
    t = t_2;
  else
  {
    intersection.AssertValid();
    return intersection;
  }
  double distance = t * V.Length();
  if (t >= EPSILON && distance < intersection.distance)
  {
    // calculate intersection members
    intersection.hit = true;
    intersection.t = t;
    intersection.position = P_0 + t * V;
    intersection.distance = distance;
    intersection.node = node;

    // calculate normal
    double d = (P_a - intersection.position).Length();
    double h = d / cos_theta; // length down from P_a to get to center axis along normal from intersection
    R3Point Q = P_a - h * R3posy_vector;
    intersection.normal = intersection.position - Q;
    intersection.normal.Normalize();
    if (V.Dot(intersection.normal) > 0)
      intersection.normal = -intersection.normal;
    intersection.AssertValid();
  }

  intersection.AssertValid();
  return intersection;
}


R3Intersection IntersectRayWithSegment(R3Ray ray, R3Segment *segment, R3Node *node)
{
  R3Intersection intersection;
  return intersection;
}

