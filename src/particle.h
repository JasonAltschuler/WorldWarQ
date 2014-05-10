// Include file for the particle system

#ifndef PARTICLE_INCLUDED
#define PARTICLE_INCLUDED

// Particle system integration types

enum {
  EULER_INTEGRATION,
  MIDPOINT_INTEGRATION,
  ADAPTIVE_STEP_SIZE_INTEGRATION,
  RK4_INTEGRATION
};



// Particle system functions
void UpdateParticles(R3Scene *scene, double current_time, double delta_time, int integration_type);
void UpdateAircrafts(R3Scene *scene, double current_time, double delta_time, int integration_type);
void FireBullet(R3Scene *scene, R3Aircraft *aircraft, int aircraft_id);
void GenerateParticles(R3Scene *scene, double current_time, double delta_time);
void RenderParticles(R3Scene *scene, double current_time, double delta_time);
void RenderAircrafts(R3Scene *scene, double current_time, double delta_time);

void HitAircraft(R3Scene *scene, R3Aircraft *aircraft);



// Intersection computations
R3Intersection ComputeIntersection(R3Scene* scene, R3Node *node, R3Ray ray);
R3Intersection ComputeIntersectionWithAircrafts(R3Scene* scene, R3Ray ray, R3Particle *bullet);
R3Intersection IntersectRayWithSphere(R3Ray ray, R3Sphere *sphere, R3Node *node);
R3Intersection IntersectRayWithBox(R3Ray ray, R3Box *box, R3Node *node);
R3Intersection IntersectRayWithCylinder(R3Ray ray, R3Cylinder *cylinder, R3Node *node);
R3Intersection IntersectRayWithCone(R3Ray ray, R3Cone *cone, R3Node *node);
R3Intersection IntersectRayWithMesh(R3Ray ray, R3Mesh *mesh, R3Node *node);
R3Intersection IntersectRayWithSegment(R3Ray ray, R3Segment *segment, R3Node *node);


#endif /* PARTICLE_INCLUDED */
