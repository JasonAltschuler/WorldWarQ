#ifndef R3AIRCRAFT_H_
#define R3AIRCRAFT_H_

#include "R3/R3.h"
#include <assert.h>

struct R3Material;

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

  // TODO: do we need these?
//  double mass;
//  bool fixed;
//  double drag;
//  double elasticity;
//  double lifetime;


  // move the plane
  void PitchUp(double delta_time);
  void PitchDown(double delta_time);
  void RollLeft(double delta_time);
  void RollRight(double delta_time);
  void ThrustForward(double delta_time);
  void BrakeBackward(double delta_time);

  // get X, Y, and Z vectors (in modeling coordinates) when transformed to world coordinates
  R3Vector Modeling_To_World(R3Vector vector_modeling_coordinates);

  void AssertValid(void);
};

inline R3Aircraft::
R3Aircraft(void) :
  velocity(R3zero_vector),
  T(R3identity_matrix),
  mesh(NULL),
  material(NULL),
  mass(-1),
  drag(-1),
  thrust_magnitude(-1),
  max_thrust(-1)
{
}

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
