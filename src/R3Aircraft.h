#ifndef R3AIRCRAFT_H_
#define R3AIRCRAFT_H_

#include "R3/R3.h"
#include <assert.h>

struct R3Scene;
struct R3Material;
class R3ParticleSource;

// Set up background music
void BackgroundMusic_Init(void);

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
  double freeze_time; // if <= 0 --> not frozen
  double time_since_last_fired;
  bool is_hit;
  // remember original fields for when aircraft is destroyed and respawns
  R3Vector respawn_velocity;
  R3Matrix respawn_T;
  double respawn_thrust_magnitude;
  int respawn_hitpoints; // equivalently: max_hitpoints


  // ------------------------- Methods for aircraft actions ------------------------- //

  // intelligent aircraft movement
  void AI_decision(R3Scene* scene, R3Aircraft *enemy, double delta_time);

  // basic plane movements
  void FireBullet(R3Scene *scene);
  void PitchUp(double delta_time);
  void PitchDown(double delta_time);
  void RollLeft(double delta_time);
  void RollRight(double delta_time);
  void ThrustForward(double delta_time);
  void BrakeBackward(double delta_time);

  // actions for if aircraft is destroyed (bullets hit or crash into mesh)
  void HitAircraft(R3Scene *scene);
  void Destroy(R3Scene *scene, bool should_explode, bool should_respawn);
  void Explode(R3Scene *scene, bool is_nonbullet_collision);
  void Respawn(void);


  // ---------------------------- Helper methods ---------------------------- //

  // get X, Y, and Z vectors (in modeling coordinates) when transformed to world coordinates
  R3Vector Modeling_To_World(R3Vector vector_modeling_coordinates);

  // checks this aircraft
  void AssertValid(void);
};

// Update and render aircrafts
void UpdateAircrafts(R3Scene *scene, double current_time, double delta_time, int integration_type);
void RenderAircrafts(R3Scene *scene, double current_time, double delta_time);

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
