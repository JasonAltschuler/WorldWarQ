
#include "R3Aircraft.h"
#include "particleview.h"
#include "particle.h"
#include "R3Scene.h"
#include <math.h>
#include <signal.h>
#include <stdlib.h>
#include <unistd.h>


////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////

#define PI 3.14159

// TODO: make THETA and SEC_TO_MAX_THRUST a command-line input?
// TODO: have different thetas for rolling left/right or up/down
//static const double THETA = 0.00174532925; // .1 degree in radians
//static const double THETA = 0.872664625; // 50 degree in radians
static const double THETA = 0.4363323125; // 25degrees in radians

static const double SECONDS_TO_MAX_THRUST = 2.0; // TODO: play around with this

static const double BULLET_VELOCITY = 100.0;

static const double AIRCRAFT_EXHAUST_RATE_MAX = 100.0;



// TODO: play with these?
const double AI_RADIUS_SHOOTING_RANGE = 0.130899694;                 // 7.5 degrees in radians
const double AI_RADIUS_MOVING_RANGE = AI_RADIUS_SHOOTING_RANGE / 50.0;  // the smaller this is, the better the AI is/ TODO: change with hard mode?
const double AI_DISTANCE_HI_LO_THRUST = 75.0; // probably somewhere between 75 and 100 is best

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



////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////


R3Aircraft::R3Aircraft(void) :
  velocity(R3zero_vector),
  T(R3identity_matrix),
  mesh(NULL),
  material(NULL),
  mass(-1),
  drag(-1),
  thrust_magnitude(-1),
  max_thrust(-1),
  firing_rate(-1),
  hitpoints(-1),
  respawn_velocity(R3zero_vector),
  respawn_T(R3identity_matrix),
  respawn_thrust_magnitude(-1),
  respawn_hitpoints(-1)

{
  sources.resize(2);
}



void R3Aircraft::
AssertValid(void)
{
  assert(mesh != NULL);
  assert(mass >= 0);
  assert(drag >= 0);
  assert(thrust_magnitude >= 0);
  assert(max_thrust >= 0);
//  assert(hitpoints > 0); // TODO

  if (hard_mode == 0) // invariant only holds on easy mode (see writeup for more details)
    assert(velocity.X() >= 0);
}


////////////////////////////////////////////////////////////
// Aircraft actions
////////////////////////////////////////////////////////////

void R3Aircraft::
FireBullet(R3Scene *scene)
{
//  BULLET_VELOCITY

  double pi = 3.14159265;
  double angle_cutoff = .01;

  R3Vector bullet_origin_modeling (3, 0, 0);
  R3Vector bullet_origin_world = Modeling_To_World(bullet_origin_modeling);

  // TODO add bullet spread
  R3Vector N(1, 0, 0);
  R3Vector A = N;
  A.Cross(R3Vector(.2,.1,0));
  A.Normalize();
  double t1 = RandomNumber()*2*pi;
  double t2 = RandomNumber()*sin(angle_cutoff);
  R3Vector V = A;
  V.Rotate(N, t1);
  R3Vector cross = V;
  cross.Cross(N);
  V.Rotate(cross, acos(t2));
  V.Normalize();

//  R3Vector bullet_velocity_modeling (BULLET_VELOCITY + velocity.X(), 0, 0);
  R3Vector bullet_velocity_modeling = V*(BULLET_VELOCITY + velocity.X());

  R3Vector bullet_velocity_world = bullet_velocity_modeling;
  bullet_velocity_world.Transform(T);

  double bullet_mass = 1;
  double bullet_fixed = false;
  double bullet_elasticity = 0;
  double bullet_drag = 0;
  double bullet_lifetime = 10.0;
  R3Material *bullet_material = material;
  vector<R3ParticleSpring *> bullet_springs(0);
  bool bullet_is_bullet = true; // derp
  R3Aircraft *aircraft_fired_from = this;

  R3Particle * new_bullet = new R3Particle(bullet_origin_world.Point(), bullet_velocity_world,
      bullet_mass, bullet_fixed, bullet_drag, bullet_elasticity, bullet_lifetime,
      bullet_material, bullet_springs, bullet_is_bullet, aircraft_fired_from);

  scene->particles.push_back(new_bullet);
}


void R3Aircraft::
ThrustForward(double delta_time)
{
  double delta_thrust = max_thrust * delta_time / SECONDS_TO_MAX_THRUST;
  thrust_magnitude = min(thrust_magnitude + delta_thrust, max_thrust); // always <= max_thrust
  AssertValid();
}

void R3Aircraft::
BrakeBackward(double delta_time)
{
  double delta_thrust = max_thrust * delta_time / SECONDS_TO_MAX_THRUST;
  thrust_magnitude = max(thrust_magnitude - delta_thrust, 0.0); // always non-negative
  AssertValid();
}


void R3Aircraft::
PitchUp(double delta_time)
{
  const double COS_THETA = cos(THETA * delta_time);
  const double SIN_THETA = sin(THETA * delta_time);

  R3Matrix mat(COS_THETA, 0, -SIN_THETA, 0,
               0, 1, 0, 0,
               SIN_THETA, 0, COS_THETA, 0,
               0, 0, 0, 1);
  T *= mat;
  if (hard_mode == 1) // makes aircraft velocity more realistic. see writeup for more details
    velocity.Transform(mat.Transpose());
  AssertValid();
}

void R3Aircraft::
PitchDown(double delta_time)
{
  const double COS_THETA = cos(THETA * delta_time);
  const double SIN_THETA = sin(THETA * delta_time);

  R3Matrix mat(COS_THETA, 0, SIN_THETA, 0,
               0, 1, 0, 0,
               -SIN_THETA, 0, COS_THETA, 0,
               0, 0, 0, 1);
  T *= mat;
  if (hard_mode == 1) // makes aircraft velocity more realistic. see writeup for more details
    velocity.Transform(mat.Transpose());
  AssertValid();
}

void R3Aircraft::
RollLeft(double delta_time)
{
  const double COS_THETA = cos(2 * THETA * delta_time);
  const double SIN_THETA = sin(2 * THETA * delta_time);

  R3Matrix mat(1, 0, 0, 0,
               0, COS_THETA, SIN_THETA, 0,
               0, -SIN_THETA, COS_THETA, 0,
               0, 0, 0, 1);
  T *= mat;
  if (hard_mode == 1) // makes aircraft velocity more realastic. see writeup for more details
    velocity.Transform(mat.Transpose());
  AssertValid();
}



void R3Aircraft::
RollRight(double delta_time)
{
  const double COS_THETA = cos(2 * THETA * delta_time);
  const double SIN_THETA = sin(2 * THETA * delta_time);

  R3Matrix mat(1, 0, 0, 0,
               0, COS_THETA, -SIN_THETA, 0,
               0, SIN_THETA, COS_THETA, 0,
               0, 0, 0, 1);
  T *= mat;
  if (hard_mode == 1) // makes aircraft velocity more realistic. see writeup for more details
    velocity.Transform(mat.Transpose());
  AssertValid();
}


/////////////////////////////////////////////////////////////////////////////////////////
// Actions for if aircraft is hit/destroyed (bullets hit or aircraft crashes into mesh)
//////////////////////////////////////////////////////////////////////////////////////////

void R3Aircraft::
HitAircraft(R3Scene *scene)
{
  hitpoints--;

  // plane dies if hitpoints <= 0
  if (hitpoints <= 0)
  {
    // if user-controlled airplane that blows up, you lose!
    if (this == scene->Aircraft(0))
    {
      cout << "GAME OVER. YOU LOSE. " << endl << endl << endl; // TODO
      this->hitpoints = respawn_hitpoints;
    }

    // else, it is an AI plane
    else
    {
      cout << "PLANE DESTROYED" << endl;
      cout << "attaway big boy" << endl;
      bool should_explode = true;
      bool should_respawn = true;
      this->Destroy(scene, should_explode, should_respawn);
    }
  }
}

void R3Aircraft::
Destroy(R3Scene *scene, bool should_explode, bool should_respawn)
{
  if (should_explode)
    this->Explode(scene);

  if (should_respawn)
    this->Respawn();
}




void R3Aircraft::
Explode(R3Scene *scene)
{
  // set up materials only once
  static bool is_material_intialized = false;
  static R3Material orange_shrapnel, red_shrapnel, black_shrapnel;
  if (!is_material_intialized)
  {
    is_material_intialized = true;

    // set up orange shrapnel
    orange_shrapnel.ka.Reset(0.2,0.2,0.2,1);
    orange_shrapnel.kd.Reset(1, 0.5, 0,1);
    orange_shrapnel.ks.Reset(0, 0, 1,1);
    orange_shrapnel.kt.Reset(0,0,0,1);
    orange_shrapnel.emission.Reset(0,0,0,1);
    orange_shrapnel.shininess = 1;
    orange_shrapnel.indexofrefraction = 1;
    orange_shrapnel.texture = NULL;
    orange_shrapnel.texture_index = -1;

    // set up red shrapnel
    red_shrapnel.ka.Reset(0.2,0.2,0.2,1);
    red_shrapnel.kd.Reset(1, 0,0,1);
    red_shrapnel.ks.Reset(0, 0, 1,1);
    red_shrapnel.kt.Reset(0,0,0,1);
    red_shrapnel.emission.Reset(0,0,0,1);
    red_shrapnel.shininess = 1;
    red_shrapnel.indexofrefraction = 1;
    red_shrapnel.texture = NULL;
    red_shrapnel.texture_index = -1;

    // set up black shrapnel
    black_shrapnel.ka.Reset(0.2,0.2,0.2,1);
    black_shrapnel.kd.Reset(0, 0, 0,1);
    black_shrapnel.ks.Reset(0, 0, 1,1);
    black_shrapnel.kt.Reset(0,0,0,1);
    black_shrapnel.emission.Reset(0,0,0,1);
    black_shrapnel.shininess = 1;
    black_shrapnel.indexofrefraction = 1;
    black_shrapnel.texture = NULL;
    black_shrapnel.texture_index = -1;
  }



  // TODO: play with these
  const int num_particles_to_generate = 1000;
  const double radius = 1;
  const double fast_velocity = 30;
  const double particle_lifetime = 0.25;

  R3ParticleSource * new_source = new R3ParticleSource();
  R3Point center = Modeling_To_World(R3Vector(0, 0, 0)).Point();
  R3Sphere *sphere = new R3Sphere(center, radius);
  R3Shape *shape = new R3Shape();
  shape->sphere = sphere;
  shape->type = R3_SPHERE_SHAPE;
  new_source->shape = shape;
  new_source->rate = 0;
  new_source->velocity = fast_velocity;
  new_source->angle_cutoff = 0;
  new_source->mass = 0.01;
  new_source->fixed = false;
  new_source->drag = 0;
  new_source->elasticity = 0;
  new_source->lifetime = particle_lifetime;
  new_source->material = &orange_shrapnel; // TODO: change to make different colors (red, black, white, orange?)




// TODO: delete this and DrawSource(...) method in particleview.cpp && particleview.h
  DrawSource(new_source);


  int one_third = (int) num_particles_to_generate / 3.0;
  int two_third = one_third * 2;

  for (int i = 0; i < num_particles_to_generate; i++)
  {
    if (i > one_third && i < two_third)
      new_source->material = &red_shrapnel;
    if (i > two_third)
      new_source->material = &black_shrapnel;

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
    double t2 = RandomNumber() * sin(new_source->angle_cutoff);
    direction_emanate.Rotate(N, t1);
    R3Vector cross = direction_emanate;
    cross.Cross(N);
    direction_emanate.Rotate(cross, acos(t2));
    direction_emanate.Normalize();

    // generate a particle
    double rand1 = RandomNumber();
    double rand2 = RandomNumber();
    new_source->velocity = fast_velocity * rand1 * rand1 * rand1 * rand1 * rand1;
    new_source->lifetime = particle_lifetime * rand2 * rand2 * rand2;
    R3Particle * new_particle = new R3Particle(point_emanate, direction_emanate, new_source);
    scene->particles.push_back(new_particle);
  }
  //  }

  delete new_source;
  delete sphere;
  delete shape;
}

void R3Aircraft::
Respawn(void)
{
  velocity = respawn_velocity;
  T = respawn_T;
  thrust_magnitude = respawn_thrust_magnitude;
  hitpoints = respawn_hitpoints;
  AssertValid();
}


//////////////////////////////////////////////////
// Intelligent aircraft movement
//////////////////////////////////////////////////

void R3Aircraft::
AI_decision(R3Scene *scene, R3Aircraft *enemy, double delta_time)
{
  // modeling coordinates -> world coordinates for both planes
  R3Point aircraft_position = this->Modeling_To_World(R3Vector(0, 0, 0)).Point();
  R3Point enemy_position = enemy->Modeling_To_World(R3Vector(0, 0, 0)).Point();

  // convert cartesian coordinates (to_enemy) to spherical coordinates: (x,y,z) -> (dist, theta, phi)
  R3Vector vector_to_enemy_xyz = enemy_position - aircraft_position;
  double dist_to_enemy = vector_to_enemy_xyz.Length();

  // convert cartesian coordinates (to_enemy) to spherical coordinates: rotated(x,y,z) -> (dist, theta, phi)
  R3Vector vector_to_enemy_rotated = vector_to_enemy_xyz;
  vector_to_enemy_rotated.Transform(this->T.Transpose());
  double phi_rotated = atan2(sqrt(vector_to_enemy_rotated.X() * vector_to_enemy_rotated.X() + vector_to_enemy_rotated.Y() * vector_to_enemy_rotated.Y()), vector_to_enemy_rotated.Z());
  double theta_rotated = atan2(vector_to_enemy_rotated.Y(), vector_to_enemy_rotated.X());


  //  cout << theta_rotated << "\t" << phi_rotated << endl;
  //  cout << (int) dist_to_enemy << endl;

  // shoot if aimed properly
  if (abs(theta_rotated) < AI_RADIUS_SHOOTING_RANGE && abs(phi_rotated - PI/2.0) < AI_RADIUS_SHOOTING_RANGE)
    this->FireBullet(scene); // TODO: AI is actually way too good... maybe make it only shoot with probability 1/2 or something?


  // ROLL LEFT OR RIGHT TO AIM
  if (abs(theta_rotated) > AI_RADIUS_MOVING_RANGE)
  {
    // boundary case: if enemy is directly behind AI
    if (abs(theta_rotated - PI) < AI_RADIUS_MOVING_RANGE)
      PitchUp(delta_time);

    // else, not directly behind: so just make small adjustments to aim
    else if (theta_rotated < 0)
      RollRight(delta_time);
    else if (theta_rotated > 0)
      RollLeft(delta_time);
  }


  // TILT UP OR DOWN TO AIM
  // Note: "else if" because planes can't simultaneously roll and tilt at the same time; this also
  // makes turning left / right more smooth
  else if (abs(phi_rotated - PI/2.0) > AI_RADIUS_MOVING_RANGE)
  {

    if (phi_rotated < PI/2.0)
      PitchUp(delta_time);
    else if (phi_rotated > PI/2.0)
      PitchDown(delta_time);
  }


  // SPEED UP OR SLOW DOWN BASED ON HOW FAR AWAY THE ENEMY IS
  if (abs(dist_to_enemy - AI_DISTANCE_HI_LO_THRUST) > 1)
  {
    if (abs(dist_to_enemy) > AI_DISTANCE_HI_LO_THRUST)
      ThrustForward(delta_time);
    else
      BrakeBackward(delta_time);
  }




//  // TODO: delete visualization later!
//  // Draw x, y, z for each aircraft
//  // Draw meshes under transformation
//  glDisable(GL_LIGHTING);
//  glLineWidth(3);
//  glBegin(GL_LINES);
//
//  R3Aircraft *aircraft = this;
//  R3Vector origin = aircraft->Modeling_To_World(R3Vector(0, 0, 0));
//  R3Vector x_vec = aircraft->Modeling_To_World(R3Vector(1, 0, 0));
//  R3Vector y_vec = aircraft->Modeling_To_World(R3Vector(0, 1, 0));
//  R3Vector z_vec = aircraft->Modeling_To_World(R3Vector(0, 0, 1));
//
//  R3Vector enemy_vec_modeling2 = dist_to_enemy * R3Vector(sin(phi_rotated) * cos(theta_rotated), sin(phi_rotated) * sin(theta_rotated), cos(phi_rotated));
//  enemy_vec_modeling2.Transform(this->T);
//  R3Vector enemy_vec2 = aircraft->Modeling_To_World(enemy_vec_modeling2);
//
//  // draw x in RED
//  glColor3d(1, 0, 0);
//  glVertex3f(origin.X(), origin.Y(), origin.Z());
//  glVertex3f(x_vec.X(), x_vec.Y(), x_vec.Z());
//  // draw y in GREEN
//  glColor3d(0, 1, 0);
//  glVertex3f(origin.X(), origin.Y(), origin.Z());
//  glVertex3f(y_vec.X(), y_vec.Y(), y_vec.Z());
//  // draw z in BLUE
//  glColor3d(0, 0, 1);
//  glVertex3f(origin.X(), origin.Y(), origin.Z());
//  glVertex3f(z_vec.X(), z_vec.Y(), z_vec.Z());
//
////  // draw enemy_vec1 in PINK
////  glColor3d(1, 0, 1);
////  glVertex3f(origin.X(), origin.Y(), origin.Z());
////  glVertex3f(enemy_vec1.X(), enemy_vec1.Y(), enemy_vec1.Z());
//
//  // draw enemy_vec2 in YELLOW
//   glColor3d(1, 1, 0);
//   glVertex3f(origin.X(), origin.Y(), origin.Z());
//   glVertex3f(enemy_vec2.X(), enemy_vec2.Y(), enemy_vec2.Z());
//   glEnd();
}

/////////////////////////////////////////////////////////////////////////////////////////
// Updating (moving) and rendering aircrafts
//////////////////////////////////////////////////////////////////////////////////////////

void UpdateAircrafts(R3Scene *scene, double current_time, double delta_time, int integration_type)
{
  for (int i = 0; i < scene->NAircrafts(); i++)
  {
    R3Aircraft *aircraft = scene->Aircraft(i);

    // PLAYER-CONTROLLED AIRCRAFT
    if (i == 0)
    {
      if (pitch_up)
        aircraft->PitchUp(delta_time);
      if (pitch_down)
        aircraft->PitchDown(delta_time);
      if (roll_left)
        aircraft->RollLeft(delta_time);
      if (roll_right)
        aircraft->RollRight(delta_time);
      if (thrust_forward)
        aircraft->ThrustForward(delta_time);
      if (brake_backward)
        aircraft->BrakeBackward(delta_time);
      if (firing_bullets)
      {
        double mult_rate = aircraft->firing_rate * delta_time;
        if (mult_rate >= 1)
          aircraft->FireBullet(scene);
        else if (RandomNumber() < mult_rate)
          aircraft->FireBullet(scene);
      }
    }

    // AI-CONTROLLED AIRCRAFT
    else
    {
      // TODO: if make teams for dogfights, actually find the closest_enemy (easy).
      R3Aircraft *closest_enemy = scene->Aircraft(0);

      aircraft->AI_decision(scene, closest_enemy, delta_time);
    }

    // UPDATE POSITION with velocity (simple Euler integration)
    R3Vector change_position_modeling = aircraft->velocity * delta_time;

    // check no collision
    R3Vector prev_position (aircraft->T[0][3], aircraft->T[1][3], aircraft->T[2][3]);
    R3Vector change_position_world = change_position_modeling;
    change_position_world.Transform(aircraft->T);
    R3Ray ray(prev_position.Point(), change_position_world);

    R3Intersection closest_intersection = ComputeIntersection(scene, scene->Root(), ray);
    if (closest_intersection.IsHit())
    {
//        cout << delta_time << endl;
        if (closest_intersection.distance < change_position_world.Length())
            cout << "COLLISION!!! " << endl; // TODO: do more
    }

    aircraft->T.Translate(change_position_modeling);

    R3Vector source_1_position_modeling(AIRCRAFT_SOURCE_BACK_NEG, AIRCRAFT_SOURCE_SIDE_POS, AIRCRAFT_SOURCE_DOWN_NEG);
    R3Vector source_2_position_modeling(AIRCRAFT_SOURCE_BACK_NEG, -AIRCRAFT_SOURCE_SIDE_POS, AIRCRAFT_SOURCE_DOWN_NEG);
    R3Vector source_1_position_world = aircraft->Modeling_To_World(source_1_position_modeling);
    R3Vector source_2_position_world = aircraft->Modeling_To_World(source_2_position_modeling);
    R3Vector normal_modeling(-1, 0, 0);
    R3Vector normal_world = normal_modeling;
    normal_world.Transform(aircraft->T);

    aircraft->sources[0]->shape->circle->Reposition(source_1_position_world.Point());
    aircraft->sources[0]->shape->circle->Align(normal_world);

    aircraft->sources[1]->shape->circle->Reposition(source_2_position_world.Point());
    aircraft->sources[1]->shape->circle->Align(normal_world);

    aircraft->sources[0]->rate = fmax(5, AIRCRAFT_EXHAUST_RATE_MAX * aircraft->thrust_magnitude / aircraft->max_thrust);
    aircraft->sources[1]->rate = fmax(5, AIRCRAFT_EXHAUST_RATE_MAX * aircraft->thrust_magnitude / aircraft->max_thrust);


    // UPDATE VELOCITY with acceleration (simple Euler integration)
    R3Vector net_force = R3Vector(aircraft->thrust_magnitude, 0, 0) - aircraft->drag * aircraft->velocity;
    R3Vector acceleration = net_force / aircraft->mass;
    aircraft->velocity += acceleration * delta_time;

//    // TODO: delete later
//    cout << "velocity: " << aircraft->velocity.X() << endl;
//    if ((acceleration.X() / delta_time) < 0.01)
//      cout << "TERMINAL!" << endl;


    aircraft->AssertValid();
  }
}





void RenderAircrafts(R3Scene *scene, double current_time, double delta_time)
{
  // Draw meshes under transformation
  glEnable(GL_LIGHTING);
  for (int i = 0; i < scene->NAircrafts(); i++) {
    R3Aircraft *aircraft = scene->Aircraft(i);

    glPushMatrix();
    LoadMatrix(&aircraft->T);

    // Load material
    if (aircraft->material) LoadMaterial(aircraft->material);

    // Draw shape
    if (aircraft->mesh) aircraft->mesh->Draw();
    else { fprintf(stderr, "problem drawing mesh!"); exit(1); }

    glPopMatrix();
  }



  // TODO: delete later (only for visualization)
  // Draw x, y, z for each aircraft
  // Draw meshes under transformation
//  glDisable(GL_LIGHTING);
//  glLineWidth(3);
//  glBegin(GL_LINES);
//
//  R3Aircraft *player_aircraft = scene->Aircraft(0);
//  R3Vector origin = player_aircraft->Modeling_To_World(R3Vector(0, 0, 0));
//  R3Vector x_vec = player_aircraft->Modeling_To_World(R3Vector(1, 0, 0));
//  R3Vector y_vec = player_aircraft->Modeling_To_World(R3Vector(0, 1, 0));
//  R3Vector z_vec = player_aircraft->Modeling_To_World(R3Vector(0, 0, 1));
//
//  // draw x in RED
//  glColor3d(1, 0, 0);
//  glVertex3f(origin.X(), origin.Y(), origin.Z());
//  glVertex3f(x_vec.X(), x_vec.Y(), x_vec.Z());
//
//  // draw y in GREEN
//  glColor3d(0, 1, 0);
//  glVertex3f(origin.X(), origin.Y(), origin.Z());
//  glVertex3f(y_vec.X(), y_vec.Y(), y_vec.Z());
//
//  // draw z in BLUE
//  glColor3d(0, 0, 1);
//  glVertex3f(origin.X(), origin.Y(), origin.Z());
//  glVertex3f(z_vec.X(), z_vec.Y(), z_vec.Z());
//
//
//  glEnd();


//  WORKING! Draw every particle
//  glDisable(GL_LIGHTING);
//  glPointSize(5);
//  glBegin(GL_POINTS);
//  for (int i = 0; i < scene->NAircrafts(); i++) {
//    R3Aircraft *aircraft = scene->Aircraft(i);
//    glColor3d(1, 1, 1);
//    const R3Point& position = aircraft->T * R3Point(0, 0, 0);
//    glVertex3d(position[0], position[1], position[2]);
//  }
}
