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


  // TODO: do we need these?
//  double mass;
//  bool fixed;
//  double drag;
//  double elasticity;
//  double lifetime;


  // methods
  void PitchUp(void);
  void PitchDown(void);
  void RollLeft(void);
  void RollRight(void);

  void AssertValid(void);
};

inline R3Aircraft::
R3Aircraft(void) :
  velocity(R3zero_vector),
  T(R3identity_matrix),
  mesh(NULL),
  material(NULL)
{
}



#endif /* R3AIRCRAFT_H_ */
