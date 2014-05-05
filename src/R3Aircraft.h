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


  // methods
  void PitchUp(double delta_time);
  void PitchDown(double delta_time);
  void RollLeft(double delta_time);
  void RollRight(double delta_time);
  void ThrustForward(double delta_time);
  void BrakeBackward(double delta_time);

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



#endif /* R3AIRCRAFT_H_ */
