#ifndef R3AIRCRAFT_H_
#define R3AIRCRAFT_H_

#include "R3/R3.h"
#include <assert.h>

struct R3Material;
class R3ParticleSource;

// TODO: add destructor
struct R3Aircraft {
  R3Aircraft(void);

  R3Vector velocity;
  R3Matrix T;
  R3Mesh *mesh;
  R3Material *material;

  double mass;
  double drag;
  double thrust_magnitude;
  double max_thrust;
  vector<R3ParticleSource *> sources;
  double firing_rate;
  int hitpoints; // alive if positive (so should always be positive)

  // remember original fields for when aircraft is destroyed and respawns
  R3Vector respawn_velocity;
  R3Matrix respawn_T;
  double respawn_thrust_magnitude;
  int respawn_hitpoints; // equivalently: max_hitpoints


  // move the plane
  void PitchUp(double delta_time);
  void PitchDown(double delta_time);
  void RollLeft(double delta_time);
  void RollRight(double delta_time);
  void ThrustForward(double delta_time);
  void BrakeBackward(double delta_time);

  // get X, Y, and Z vectors (in modeling coordinates) when transformed to world coordinates
  R3Vector Modeling_To_World(R3Vector vector_modeling_coordinates);

  // respawn if aircraft is destroyed
  void Respawn(void);

  void AssertValid(void);
};


inline R3Vector R3Aircraft::
Modeling_To_World(R3Vector vector_modeling_coordinates)
{
  R3Vector vector_world_coordinates(vector_modeling_coordinates);
  vector_world_coordinates.Transform(T);
  vector_world_coordinates[0] += T[0][3];
  vector_world_coordinates[1] += T[1][3];
  vector_world_coordinates[2] += T[2][3];
  return vector_world_coordinates;
}


#endif /* R3AIRCRAFT_H_ */
