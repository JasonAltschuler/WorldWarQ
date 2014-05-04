
#include "R3Aircraft.h"
#include "particleview.h"
#include "R3Scene.h"
#include <math.h>


// TODO: make this a command-line input?
// TODO: have different thetas for rolling left/right or up/down
//static const double THETA = 0.0174532925; // 1 degree in radians
static const double THETA = 0.000174532925; // 1 degree in radians

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
  // TODO: actually take into account velocity
  // TODO: actually take into account drag, gravity, etc.
  // TODO: add collisions, as in UpdateParticles() in particle.cpp (already implemented there) -- if collision, just blow up!!

  for (int i = 0; i < scene->NAircrafts(); i++)
  {
    R3Aircraft *aircraft = scene->Aircraft(i);
    // The first aircraft is by convention the player-controlled aircraft
    if (i == 0)
    {
      if (pitch_up)
      {
//        pitch_up = 0;
        aircraft->PitchUp();
      }

      if (pitch_down)
      {
//        pitch_down = 0;
        aircraft->PitchDown();
      }

      if (roll_left)
      {
//        roll_left = 0;
        aircraft->RollLeft();
      }

      if (roll_right)
      {
//        roll_right = 0;
        aircraft->RollRight();
      }
    }


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
  glPushMatrix();
  LoadMatrix(&player_aircraft->T);

  // draw x in RED
  glColor3d(1, 0, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(2, 0, 0);

  // draw y in GREEN
  glColor3d(0, 1, 0);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 2, 0);

  // draw z in BLUE
  glColor3d(0, 0, 1);
  glVertex3f(0, 0, 0);
  glVertex3f(0, 0, 2);

  glPopMatrix();




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
