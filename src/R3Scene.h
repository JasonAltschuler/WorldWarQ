// Include file for the R3 scene stuff

#ifndef R3SCENE_INCLUDED
#define R3SCENE_INCLUDED

#define R3Rgb R2Pixel

#include "R3Aircraft.h"
#include <iostream>
#include <vector>
#include <deque>


// Constant definitions

typedef enum {
  R3_BOX_SHAPE,
  R3_SPHERE_SHAPE,
  R3_CYLINDER_SHAPE,
  R3_CONE_SHAPE,
  R3_MESH_SHAPE,
  R3_SEGMENT_SHAPE,
  R3_CIRCLE_SHAPE,
  R3_NUM_SHAPE_TYPES
} R3ShapeType;

typedef enum {
  R3_DIRECTIONAL_LIGHT,
  R3_POINT_LIGHT,
  R3_SPOT_LIGHT,
  R3_AREA_LIGHT,
  R3_NUM_LIGHT_TYPES
} R3LightType;



// Scene element definitions

struct R3Shape {
  R3ShapeType type;
  R3Box *box;
  R3Sphere *sphere;
  R3Cylinder *cylinder;
  R3Cone *cone;
  R3Mesh *mesh;
  R3Segment *segment;
  R3Circle *circle;
};  

struct R3Material {
  R3Rgb ka;
  R3Rgb kd;
  R3Rgb ks;
  R3Rgb kt;
  R3Rgb emission;
  double shininess;
  double indexofrefraction;
  R2Image *texture;
  int texture_index;
  int id;
};

struct R3Light {
  R3LightType type;
  R3Point position;
  R3Vector direction;
  double radius;
  R3Rgb color;
  double constant_attenuation;
  double linear_attenuation;
  double quadratic_attenuation;
  double angle_attenuation;
  double angle_cutoff;
};

struct R3Camera {
  R3Point eye;
  R3Vector towards;
  R3Vector right;
  R3Vector up;
  double xfov, yfov;
  double neardist, fardist;
};

struct R3Node {
  struct R3Node *parent;
  vector<struct R3Node *> children;
  R3Shape *shape;
  R3Matrix transformation;
  R3Material *material;
  R3Box bbox;
};


class R3ParticleSource {
public:
  R3Shape *shape;
  double rate;
  double velocity;
  double angle_cutoff;
  double mass;
  bool fixed;
  double drag;
  double elasticity;
  double lifetime;
  R3Material *material;
};

// Particle system definitions

struct R3Particle {
  R3Particle(void);
  R3Particle(R3Point position, R3Vector direction_emanate, R3ParticleSource * source);
  R3Particle(R3Point position, R3Vector velocity, double mass, bool fixed, double drag, double
      elasticity, double lifetime, R3Material *material, vector<struct R3ParticleSpring *> springs,
      bool is_bullet, R3Aircraft *aircraft_fired_from);

  R3Point position;
  R3Vector velocity;
  double mass;
  bool fixed;
  double drag;
  double elasticity;
  double lifetime;
  R3Material *material;
  vector<struct R3ParticleSpring *> springs;
  bool is_bullet;
  R3Aircraft *aircraft_fired_from; // NULL if not a bullet. otherwise a pointer to the aircraft that fired the bullet
  deque<R3Point> trail;
};

inline R3Particle::R3Particle(void) :
    position(R3zero_point),
    velocity(R3zero_vector),
    mass(-1),
    fixed(false),
    drag(-1),
    elasticity(-1),
    lifetime(-1),
    material(NULL),
    springs(vector<struct R3ParticleSpring *>(0)),
    is_bullet(false),
    aircraft_fired_from(NULL),
    trail(0)
{
}

inline R3Particle::R3Particle(R3Point position, R3Vector direction_emanate, R3ParticleSource *source)
{
  this->position = position;
  direction_emanate.Normalize();
  this->velocity = direction_emanate * (source->velocity);
  this->mass = source->mass;
  this->fixed = source->fixed;
  this->drag = source->drag;
  this->elasticity = source->elasticity;
  this->lifetime = source->lifetime;
  this->material = source->material;
  this->springs.clear();
  this->is_bullet = false;
  this->aircraft_fired_from = NULL;
  this->trail.resize(0);
}

inline R3Particle::R3Particle(R3Point position, R3Vector velocity, double mass, bool fixed, double drag,
    double elasticity, double lifetime, R3Material *material, vector<struct R3ParticleSpring *> springs,
    bool is_bullet, R3Aircraft *aircraft_fired_from) :
    position(position),
    velocity(velocity),
    mass(mass),
    fixed(fixed),
    drag(drag),
    elasticity(elasticity),
    lifetime(lifetime),
    material(material),
    springs(springs),
    is_bullet(is_bullet),
    aircraft_fired_from(aircraft_fired_from),
    trail(0)
{
}


struct R3ParticleSink {
  R3Shape *shape;
  double intensity;
  double constant_attenuation;
  double linear_attenuation;
  double quadratic_attenuation;
};

struct R3ParticleSpring {
  R3Particle *particles[2];
  double rest_length;
  double ks;
  double kd;

  R3Particle* OtherParticle(const R3Particle *);
};

inline R3Particle *R3ParticleSpring::
OtherParticle(const R3Particle * particle1)
{
  if (particle1 == particles[0])
    return particles[1];
   return particles[0];
}

// Scene graph definition

struct R3Scene {
 public:
  // Constructor functions
  R3Scene(void);

  // Access functions
  R3Node *Root(void) const;
  int NLights(void) const;
  R3Light *Light(int k) const;
  R3Camera& Camera(void);
  R3Box& BBox(void);

  // Particle stuff
  int NParticles(void) const;
  R3Particle *Particle(int k) const;
  int NParticleSources(void) const;
  R3ParticleSource *ParticleSource(int k) const;
  int NParticleSinks(void) const;
  R3ParticleSink *ParticleSink(int k) const;
  int NParticleSprings(void) const;
  R3ParticleSpring *ParticleSpring(int k) const;
  int NAircrafts(void) const;
  R3Aircraft *Aircraft(int k) const;

  // I/O functions
  int Read(const char *filename, R3Node *root = NULL);

 public:
  R3Node *root;
  vector<R3Aircraft *> aircrafts;
  vector<R3Particle *> particles;
  vector<R3ParticleSource *> particle_sources;
  vector<R3ParticleSink *> particle_sinks;
  vector<R3ParticleSpring *> particle_springs;
  vector<R3Light *> lights;
  R3Vector gravity;
  R3Camera camera;
  R3Box bbox;
  R3Rgb background;
  R3Rgb ambient;
};



// Inline functions 

inline R3Node *R3Scene::
Root(void) const
{
  // Return root node
  return root;
}



inline int R3Scene::
NLights(void) const
{
  // Return number of lights
  return lights.size();
}



inline R3Light *R3Scene::
Light(int k) const
{
  // Return kth light
  return lights[k];
}



inline R3Camera& R3Scene::
Camera(void) 
{
  // Return camera
  return camera;
}



inline R3Box& R3Scene::
BBox(void) 
{
  // Return bounding box 
  return bbox;
}



inline int R3Scene::
NParticles(void) const
{
  // Return number of particles
  return particles.size();
}



inline R3Particle *R3Scene::
Particle(int k) const
{
  // Return kth particle 
  return particles[k];
}



inline int R3Scene::
NParticleSources(void) const
{
  // Return number of particle sources
  return particle_sources.size();
}



inline R3ParticleSource *R3Scene::
ParticleSource(int k) const
{
  // Return kth particle source
  return particle_sources[k];
}



inline int R3Scene::
NParticleSinks(void) const
{
  // Return number of particle sinks
  return particle_sinks.size();
}



inline R3ParticleSink *R3Scene::
ParticleSink(int k) const
{
  // Return kth particle sink
  return particle_sinks[k];
}



inline int R3Scene::
NParticleSprings(void) const
{
  // Return number of particle springs
  return particle_springs.size();
}



inline R3ParticleSpring *R3Scene::
ParticleSpring(int k) const
{
  // Return kth particle spring
  return particle_springs[k];
}

inline int R3Scene::
NAircrafts(void) const
{
  // Return number of particle springs
  return aircrafts.size();
}



inline R3Aircraft *R3Scene::
Aircraft(int k) const
{
  // Return kth particle spring
  return aircrafts[k];
}





// ----- the following intersection code is copied from raytrace.h in my assignment 3! ----- //


#define DOUBLE_MAX 1.7976931348623158e+308 // max value

struct R3Intersection {
 public:
  // Constructor functions
  R3Intersection(void);

  // member variables
  bool hit;
  R3Node *node;
  R3Aircraft *aircraft;
  R3Point position;
  R3Vector normal;
  double t;
  double distance;

  bool IsHit(void) const;
  void SetMiss();
  void Transform(R3Matrix, R3Ray);
  void AssertValid(void); // for debugging
};


inline R3Intersection::
R3Intersection(void)
  : hit(false),
    node(NULL),
    aircraft(NULL),
    position(R3zero_point),
    normal(R3zero_vector),
    t(-1),
    distance(DOUBLE_MAX)
{
}

// for debugging
inline void R3Intersection::
AssertValid(void)
{
  if (hit) {
    assert(t > 0);
    assert(distance > 0);
//    assert(node != NULL || aircraft != NULL);
    assert(abs(normal.Length() - 1) < 0.01);
  }
  else {
    assert(t <= 0);
    assert(distance = DOUBLE_MAX);
    assert(node == NULL);
    assert(aircraft == NULL);
  }
}

inline void R3Intersection::
SetMiss(void)
{
  hit = false;
  t = -1;
  distance = DOUBLE_MAX;
  node = NULL;
  aircraft = NULL;
}

inline bool R3Intersection::
IsHit(void) const
{
  return hit;
}

inline void R3Intersection::
Transform(R3Matrix transformation, R3Ray oldRay)
{
  // transform intersection point
  position.Transform(transformation);

  // transform intersection normal
  normal.Transform(transformation.Inverse().Transpose());
  normal.Normalize();

  // recompute parametric value on t on the ray
  t = oldRay.T(position);

  // recompute distance
  distance = (oldRay.Start() - position).Length();
}


#endif /* R3SCENE_INCLUDED */
