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

// force constants
static const R3Vector GRAVITY_VECTOR(0.0, 0.0, -2.8); // 9.8 m/s readjusted to modeling system


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


  // TRAILS!
  // remember the positions of the particles for the last K time steps
  const int K_TRAIL_SIZE = 10; // hardcoded, but can be changed easily

  for (int i = 0; i < scene->NParticles(); i++)
  {
    // update trails
    R3Particle *particle = scene->Particle(i);
    deque<R3Point> &trail = particle->trail;
    trail.push_front(particle->position);
    if (trail.size() > K_TRAIL_SIZE)
      trail.pop_back();

    // draw trails
    R3Rgb material_color(1, 0.6, 0, 1);
    R3Rgb white(1.0, 1.0, 1.0, 1.0);

    for (int j = 0; j < trail.size() - 1; j++)
    {
      double c = (double) j / (double) trail.size();
      R3Rgb interpolated_color = material_color * (1 - c) + c * white;
      interpolated_color.Clamp();

      glBegin(GL_LINES);
      glColor3d(interpolated_color.Red(), interpolated_color.Green(), interpolated_color.Blue());
      glVertex3d(trail[j].X(), trail[j].Y(), trail[j].Z());
      glVertex3d(trail[j+1].X(), trail[j+1].Y(), trail[j+1].Z());
      glEnd();
    }
  }
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
      delete scene->particles[i];
      scene->particles.erase(scene->particles.begin() + i);
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


// modified by Jason so bullets intersect with airplanes, but not mesh (on May 10)
// modified by Kyle so bullets are not computing intersections (before May 10)
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


    else if (particle->is_bullet)
    {
      R3Point new_position = particle->position + particle->velocity * delta_time;

      // calculate intersections with other aircrafts
      R3Ray ray(particle->position, new_position);
      R3Intersection closest_intersection = ComputeIntersectionWithAircrafts(scene, ray, particle);

      // make sure intersection is actually an intersection
      if (closest_intersection.IsHit())
      {
        if (scene->Aircraft(0)->freeze_time >= 0)
        {
          // don't care about collision if frozen / animation time
          if (closest_intersection.aircraft == scene->Aircraft(0))
            closest_intersection.SetMiss();
        }

        else
        {
          double t = ray.T(new_position);
          double t_collision = ray.T(closest_intersection.position);
          if (t_collision < 0 || t_collision > t)
            closest_intersection.SetMiss();
        }
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

//        cout << "HIT" << endl; // TODO: KYLE, MAKE HEADSUP DISPLAY FOR HIT
        closest_intersection.aircraft->HitAircraft(scene);
      }
    }


    // only executed for shrapnel from aircraft explosions: don't check for intersections with scene here
    else
    {
      // note: we can do this without a buffer, and still be a valid integration technique
      // that updates the system all at once because we are not calculating inter-particle
      // forces, such as gravitational attraction, etc.
      particle->position += particle->velocity * delta_time;
      particle->velocity += GRAVITY_VECTOR * delta_time;
    }
  }


  EndParticles(scene, delta_time);
}





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
      case R3_BOX_SHAPE:
        accelerated_intersection = IntersectRayWithBox(transformed_ray[num_transforms], (R3Box*) last_intersection_node->shape->box, last_intersection_node);
        break;
      case R3_MESH_SHAPE:
        accelerated_intersection = IntersectRayWithMesh(transformed_ray[num_transforms], (R3Mesh*) last_intersection_node->shape->mesh, last_intersection_node);
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
    case R3_BOX_SHAPE:
      shape_intersection = IntersectRayWithBox(new_ray, (R3Box*) node->shape->box, node);
      break;
    case R3_MESH_SHAPE:
      shape_intersection = IntersectRayWithMesh(new_ray, (R3Mesh*) node->shape->mesh, node);
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
