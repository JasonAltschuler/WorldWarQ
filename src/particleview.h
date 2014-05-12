
#ifndef PARTICLEVIEW_H_
#define PARTICLEVIEW_H_

#include "R3/R3.h"
#include "R3Scene.h"

// extern variables set in particleview.cpp by listener and used to update the
// player-controlled aircraft in R3Aircraft.cpp
extern int pitch_up;
extern int pitch_down;
extern int roll_left;
extern int roll_right;
extern int thrust_forward;
extern int brake_backward;
extern int firing_bullets;

// mode for hard or not
extern int hard_mode;

// keep track of number deaths / kills for heads-up display
extern int num_deaths;
extern int num_kills;

// source properties for aircraft engines
const double AIRCRAFT_SOURCE_RADIUS = 0.003;
const double AIRCRAFT_SOURCE_BACK_NEG = -0.3;
const double AIRCRAFT_SOURCE_SIDE_POS = 0.2;
const double AIRCRAFT_SOURCE_DOWN_NEG = -0.1;

// textures
void LoadMatrix(R3Matrix *matrix);
void LoadMaterial(R3Material *material);
void LoadLights(R3Scene *scene);
void drawSkybox(double D);
void initSkyBox();

#endif /* PARTICLEVIEW_H_ */
