
#include "R3Aircraft.h"
#include "particleview.h"
#include "R3Scene.h"
#include <math.h>


// TODO: make this a command-line input?
// TODO: have different thetas for rolling left/right or up/down
//static const double THETA = 0.00174532925; // .1 degree in radians
static const double THETA = 0.0174532925; // .1 degree in radians


static const double cos_theta = cos(THETA);
static const double sin_theta = sin(THETA);

// TODO: actually pitch and roll. Note, right now implementation just moves left right up and down
void R3Aircraft::
PitchUp(void)
{
  R3Matrix mat(cos_theta, 0, -sin_theta, 0,
               0, 1, 0, 0,
               sin_theta, 0, cos_theta, 0,
               0, 0, 0, 1);
  T *= mat;
  AssertValid();
}

void R3Aircraft::
PitchDown(void)
{
  R3Matrix mat(cos_theta, 0, sin_theta, 0,
               0, 1, 0, 0,
               -sin_theta, 0, cos_theta, 0,
               0, 0, 0, 1);
  T *= mat;
  AssertValid();
}

void R3Aircraft::
RollLeft(void)
{
  R3Matrix mat(1, 0, 0, 0,
               0, cos_theta, sin_theta, 0,
               0, -sin_theta, cos_theta, 0,
               0, 0, 0, 1);
  T *= mat;
  AssertValid();
}

void R3Aircraft::
RollRight(void)
{
  R3Matrix mat(1, 0, 0, 0,
               0, cos_theta, -sin_theta, 0,
               0, sin_theta, cos_theta, 0,
               0, 0, 0, 1);
  T *= mat;
  AssertValid();
}

void R3Aircraft::
AssertValid(void)
{
  assert(mesh != NULL);
  assert(velocity.X() >= 0);
  assert(mass >= 0);
  assert(drag >= 0);
  assert(thrust_magnitude >= 0);
//  assert(right.IsNormalized());
//  assert(up.IsNormalized());
//  assert(forward.IsNormalized());
//
//  // assert right cross up == -forward --> right x up + forward = 0;
//  R3Vector right_copy;
//  right_copy.Cross(up);
//  assert(abs(right_copy.X() + forward.X()) < 0.0001);
//  assert(abs(right_copy.Y() + forward.Y()) < 0.0001);
//  assert(abs(right_copy.Z() + forward.Z()) < 0.0001);
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
        aircraft->PitchUp();
      if (pitch_down)
        aircraft->PitchDown();
      if (roll_left)
        aircraft->RollLeft();
      if (roll_right)
        aircraft->RollRight();
    }

    // UPDATE POSITION with velocity (simple Euler integration)
    R3Vector change_position = aircraft->velocity * delta_time;
    aircraft->T.Translate(change_position);

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

    // TODO: delete later
    cout << "velocity: " << aircraft->velocity.X() << endl;
    if ((acceleration.X() / delta_time) < 0.01)
      cout << "TERMINAL!" << endl;


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
  glDisable(GL_LIGHTING);
  glLineWidth(3);
  glBegin(GL_LINES);

  R3Aircraft *player_aircraft = scene->Aircraft(0);

  R3Vector origin(0, 0, 0);
  R3Vector x_vec(1, 0, 0);
  R3Vector y_vec(0, 1, 0);
  R3Vector z_vec(0, 0, 01);
  origin.Transform(player_aircraft->T);
  x_vec.Transform(player_aircraft->T);
  y_vec.Transform(player_aircraft->T);
  z_vec.Transform(player_aircraft->T);

  origin = R3Vector(origin.X() + player_aircraft->T[0][3],
                    origin.Y() + player_aircraft->T[1][3],
                    origin.Z() + player_aircraft->T[2][3]);

  x_vec = R3Vector(x_vec.X() + player_aircraft->T[0][3],
                   x_vec.Y() + player_aircraft->T[1][3],
                   x_vec.Z() + player_aircraft->T[2][3]);

  y_vec = R3Vector(y_vec.X() + player_aircraft->T[0][3],
                   y_vec.Y() + player_aircraft->T[1][3],
                   y_vec.Z() + player_aircraft->T[2][3]);

  z_vec = R3Vector(z_vec.X() + player_aircraft->T[0][3],
                   z_vec.Y() + player_aircraft->T[1][3],
                   z_vec.Z() + player_aircraft->T[2][3]);


  // draw x in RED
  glColor3d(1, 0, 0);
  glVertex3f(origin.X(), origin.Y(), origin.Z());
  glVertex3f(x_vec.X(), x_vec.Y(), x_vec.Z());

  // draw y in GREEN
  glColor3d(0, 1, 0);
  glVertex3f(origin.X(), origin.Y(), origin.Z());
  glVertex3f(y_vec.X(), y_vec.Y(), y_vec.Z());

  // draw z in BLUE
  glColor3d(0, 0, 1);
  glVertex3f(origin.X(), origin.Y(), origin.Z());
  glVertex3f(z_vec.X(), z_vec.Y(), z_vec.Z());



  glEnd();


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
