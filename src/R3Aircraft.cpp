
#include "R3Aircraft.h"
#include "particleview.h"
#include "particle.h"
#include "R3Scene.h"
#include <math.h>


////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////

// TODO: make THETA and SEC_TO_MAX_THRUST a command-line input?
// TODO: have different thetas for rolling left/right or up/down
//static const double THETA = 0.00174532925; // .1 degree in radians
static const double THETA = 0.872664625; // 1 degree in radians

static const double SECONDS_TO_MAX_THRUST = 2.0; // TODO: play around with this

static const double BULLET_VELOCITY = 100.0;

static const double AIRCRAFT_EXHAUST_RATE_MAX = 100.0;

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
  hitpoints(-1)
{
  sources.resize(2);
}


void FireBullet(R3Scene *scene, R3Aircraft *aircraft, int aircraft_id)
{
//  BULLET_VELOCITY

  double pi = 3.14159265;
  double angle_cutoff = .01;

  R3Vector bullet_origin_modeling (10, 0, 0);
  R3Vector bullet_origin_world = aircraft->Modeling_To_World(bullet_origin_modeling);

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

//  R3Vector bullet_velocity_modeling (BULLET_VELOCITY + aircraft->velocity.X(), 0, 0);
  R3Vector bullet_velocity_modeling = V*(BULLET_VELOCITY + aircraft->velocity.X());

  R3Vector bullet_velocity_world = bullet_velocity_modeling;
  bullet_velocity_world.Transform(aircraft->T);

  double bullet_mass = 1;
  double bullet_fixed = false;
  double bullet_elasticity = 0;
  double bullet_drag = 0;
  double bullet_lifetime = 10.0;
  R3Material *bullet_material = aircraft->material;
  vector<R3ParticleSpring *> bullet_springs(0);
  bool bullet_is_bullet = true; // derp
  R3Aircraft *aircraft_fired_from = aircraft;

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
  if (hard_mode == 1) // makes aircraft velocity more realastic. see writeup for more details
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
  if (hard_mode == 1) // makes aircraft velocity more realastic. see writeup for more details
    velocity.Transform(mat.Transpose());
  AssertValid();
}

void R3Aircraft::
RollLeft(double delta_time)
{
  const double COS_THETA = cos(THETA * delta_time);
  const double SIN_THETA = sin(THETA * delta_time);

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
  const double COS_THETA = cos(THETA * delta_time);
  const double SIN_THETA = sin(THETA * delta_time);

  R3Matrix mat(1, 0, 0, 0,
               0, COS_THETA, -SIN_THETA, 0,
               0, SIN_THETA, COS_THETA, 0,
               0, 0, 0, 1);
  T *= mat;
  if (hard_mode == 1) // makes aircraft velocity more realistic. see writeup for more details
    velocity.Transform(mat.Transpose());
  AssertValid();
}

void HitAircraft(R3Scene *scene, R3Aircraft *aircraft)
{
  aircraft->hitpoints--;

  // plane dies if hitpoints <= 0
  if (aircraft->hitpoints <= 0)
  {
    // if user-controlled airplane that blows up, you lose!
    if (aircraft == scene->Aircraft(0))
    {
      cout << "GAME OVER. YOU LOSE. " << endl; // TODO
    }

    // else, it is an AI plane
    else
    {
      cout << "PLANE DEAD" << endl;
      aircraft->Respawn();
    }
  }
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


void UpdateAircrafts(R3Scene *scene, double current_time, double delta_time, int integration_type)
{
  // TODO: actually take into account drag, gravity, etc.
  // TODO: add collisions, as in UpdateParticles() in particle.cpp (already implemented there) -- if collision, just blow up!!

  for (int i = 0; i < scene->NAircrafts(); i++)
  {
    R3Aircraft *aircraft = scene->Aircraft(i);

    // PLAYER CONTROLS for the first aircraft
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
          FireBullet(scene, aircraft, 0);
        else if (RandomNumber() < mult_rate)
          FireBullet(scene, aircraft, 0);
      }
    }

    // UPDATE POSITION with velocity (simple Euler integration)
    R3Vector change_position_modeling = aircraft->velocity * delta_time;

    // check no collision
    R3Vector prev_position (aircraft->T[0][3], aircraft->T[1][3], aircraft->T[2][3]);
    R3Vector change_position_world = change_position_modeling;
    change_position_world.Transform(aircraft->T);
    R3Ray ray(prev_position.Point(), change_position_world);

    // KYLE: I discovered the problem with intersections. Current mesh intersection code can only handle triangle faces.
    // TODO: It should be pretty easy to make it handle rectangle faces as well. Or we could make Daway's code generate triangle meshes.

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


    // UPDATE VELOCITY with acceleration (simple Euler integration) // TODO: combine into 1 line
    R3Vector net_force(0, 0, 0);

    // account for thrust
    R3Vector thrust(aircraft->thrust_magnitude, 0, 0);
    net_force += thrust;

    // account for drag
    R3Vector drag = -aircraft->velocity * aircraft->drag;
    net_force += drag;

    R3Vector acceleration = net_force / aircraft->mass;
    aircraft->velocity += acceleration * delta_time;

//    // TODO: delete later
//    cout << "velocity: " << aircraft->velocity.X() << endl;
//    if ((acceleration.X() / delta_time) < 0.01)
//      cout << "TERMINAL!" << endl;


    // quick assert to make sure we didn't make any no-no's
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


//    // Trails (implemented for one point)
//
//    // remember the positions of the particles for the last K time steps
//    const int K_TRAIL_SIZE = 50; // hardcoded, but can be changed easily
//    static deque<vector<R3Point> > deque(0);
//
//    // store latest positions
//    vector<R3Point> most_recent_positions(scene->NParticles());
//    for (int i = 0; i < scene->NParticles(); i++)
//      most_recent_positions[i] = scene->Particle(i)->position;
//    deque.push_front(most_recent_positions);
//
//    // remove positions if queue overflowing
//    if (deque.size() > K_TRAIL_SIZE)
//      deque.pop_back();
//
//    // draw trails
//    glBegin(GL_LINES);
//    for (int j = 0; j < deque.size() - 1; j++) {
//      // interpolate between black and (0.0, 0.0, 1.0) blue for color of line,
//      // with the end of the trail closer to black because it is fading out
//
//      R3Rgb white(1.0, 1.0, 1.0, 1.0);
//      R3Rgb black(0.0, 0.0, 0.0, 1.0);
//
//      double c = (double) j / (double) K_TRAIL_SIZE;
//      R3Rgb interpolated_color = white * (1 - c) + c * black;
//
//      vector<R3Point> vec_after = deque[j];
//      vector<R3Point> vec_before = deque[j + 1];
//      int num_particles = min(vec_after.size(), vec_before.size());
//
//      glColor3d(interpolated_color.Red(), interpolated_color.Green(), interpolated_color.Blue());
//      glLineWidth(2);
//      for (int i = 0; i < num_particles; i++)
//      {
//        glVertex3d(vec_after[i].X(), vec_after[i].Y(), vec_after[i].Z());
//        glVertex3d(vec_before[i].X(), vec_before[i].Y(), vec_before[i].Z());
//      }
//    }
//
//    glEnd();
}
