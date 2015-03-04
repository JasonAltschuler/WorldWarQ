
#include "R3Aircraft.h"
#include "WorldWarQ.h"
#include "particle.h"
#include "R3Scene.h"
#include <math.h>
#include <signal.h>
#include <stdlib.h>
//#include <unistd.h>



// NOTE: USING IRRKLANG SOUND LIBRARY
// sounds (irrklang only works well on Macs)
#ifdef __APPLE__
#include <../irrKlang32/include/irrKlang.h>
static irrklang::ISoundEngine *engine = irrklang::createIrrKlangDevice();
#endif


////////////////////////////////////////////////////////////
// Constants
////////////////////////////////////////////////////////////

#define PI 3.14159

// 25 degrees in radians
static const double THETA = 0.4363323125;
//static const double THETA = 0.363323125;

// "0 to 60 in 3.5"
static const double SECONDS_TO_MAX_THRUST = 2.0;

// we assume bullet has no acceleration on it (although other non-bullet particles have forces)
//static const double BULLET_VELOCITY = 100.0;
static const double BULLET_VELOCITY = 500.0;
static const double BULLET_SPREAD = .005;


// rate for particles coming out of aircraft engine for exhaust
static const double AIRCRAFT_EXHAUST_RATE_MAX = 100.0;

// how long to stay put for player death (to show animation of explosion, etc.)
static const double FREEZE_TIME = 3.0;

// 7.5 degrees in radians
static const double AI_RADIUS_SHOOTING_RANGE = 0.130899694;

// the smaller this is, the better the AI is
static const double AI_RADIUS_MOVING_RANGE = AI_RADIUS_SHOOTING_RANGE / 50.0;

// probably around 250 is best
static const double AI_DISTANCE_HI_LO_THRUST = 200.0;


////////////////////////////////////////////////////////////
// TRACK NUMBER OF KILLS / DEATHS FOR HEADS-UP DISPLAY
////////////////////////////////////////////////////////////

int num_deaths = 0;
int num_kills = 0;

////////////////////////////////////////////////////////////
// Random Number Generator
////////////////////////////////////////////////////////////

static double
RandomNumber(void)
{
#if defined(_WIN32)
    int r1 = rand();
    double r2 = ((double) rand()) / ((double) (RAND_MAX + 1));
    return (r1 + r2) / ((double) (RAND_MAX + 1));
#else
    return drand48();
#endif
}



////////////////////////////////////////////////////////////
// INITIALIZE BACKGROUND MUSIC
////////////////////////////////////////////////////////////


void BackgroundMusic_Init(void)
{
#ifdef __APPLE__
    // background music for the game -- only set up once!
    static bool background_sounds_init = false;

    // initialize thrust sounds if not done so already (only done once!)
    if (!background_sounds_init)
    {
        cout << "Got here" << endl;
        background_sounds_init = true;
        engine->play2D("../music/music.ogg", true);
    }
    cout << "playing music" << endl;
#endif
}



////////////////////////////////////////////////////////////
// Constructor
////////////////////////////////////////////////////////////


R3Aircraft::R3Aircraft(void) :
          velocity(R3zero_vector),
          T(R3identity_matrix),
          mesh(NULL),
          material(NULL),
          mass(-1),
          drag(-1),
//          team(0),
          thrust_magnitude(-1),
          max_thrust(-1),
          sources(),
          firing_rate(-1),
          hitpoints(-1),
          freeze_time(-1),
          time_since_last_fired(0.0),
          respawn_velocity(R3zero_vector),
          respawn_T(R3identity_matrix),
          respawn_thrust_magnitude(-1),
          respawn_hitpoints(-1),
          is_hit(false)
{
//    cout << "here " << endl;
    sources.resize(2);
}



void R3Aircraft::
AssertValid(void)
{
    assert(mesh != NULL);
    assert(mass >= 0);
    assert(drag >= 0);
    assert(thrust_magnitude >= 0);
    assert(max_thrust >= 0);
}



////////////////////////////////////////////////////////////
// Aircraft actions
////////////////////////////////////////////////////////////

void R3Aircraft::
FireBullet(R3Scene *scene)
{
    time_since_last_fired = 0.0;

    double pi = 3.14159265;
    double angle_cutoff = BULLET_SPREAD;

    R3Vector bullet_origin_modeling (3, 0, 0);
    R3Vector bullet_origin_world = Modeling_To_World(bullet_origin_modeling);

    R3Vector N(1, 0, 0);
    R3Vector A = N;
    A.Cross(R3Vector(.2,.1,0));
    A.Normalize();
    double t1 = RandomNumber()*2*pi;
    double t2 = RandomNumber()*sin(angle_cutoff);
    R3Vector V = A;
    V.Rotate(N, t1);
    R3Vector cross = V;
    cross.Cross(N);
    V.Rotate(cross, acos(t2));
    V.Normalize();

    //  R3Vector bullet_velocity_modeling (BULLET_VELOCITY + velocity.X(), 0, 0);
    R3Vector bullet_velocity_modeling = V*(BULLET_VELOCITY + velocity.X());

    R3Vector bullet_velocity_world = bullet_velocity_modeling;
    bullet_velocity_world.Transform(T);

    double bullet_mass = 1;
    double bullet_fixed = false;
    double bullet_elasticity = 0;
    double bullet_drag = 0;
    double bullet_lifetime = 10.0;
    R3Material *bullet_material = material;
    vector<R3ParticleSpring *> bullet_springs(0);
    bool bullet_is_bullet = true; // derp
    R3Aircraft *aircraft_fired_from = this;

    R3Particle * new_bullet = new R3Particle(bullet_origin_world.Point(), bullet_velocity_world,
            bullet_mass, bullet_fixed, bullet_drag, bullet_elasticity, bullet_lifetime,
            bullet_material, bullet_springs, bullet_is_bullet, aircraft_fired_from);

    scene->particles.push_back(new_bullet);


    // make sound for firing bullet
#ifdef __APPLE__
    // 3D sound!!
    R3Aircraft *player_aircraft = scene->Aircraft(0);
    R3Vector pos_player_aircraft(0, 0, 0);
    pos_player_aircraft = player_aircraft->Modeling_To_World(pos_player_aircraft);

    R3Vector dir_player_aircraft(1, 0, 0);
    dir_player_aircraft.Transform(player_aircraft->T);

    //  cout << "here" << endl;

    engine->setListenerPosition(irrklang::vec3df(pos_player_aircraft.X(), pos_player_aircraft.Y(), pos_player_aircraft.Z()),
            irrklang::vec3df(dir_player_aircraft.X(), dir_player_aircraft.Y(), dir_player_aircraft.Z()));

    irrklang::ISound* bullet_sound = engine->play3D("../music/shot.wav", irrklang::vec3df(bullet_origin_world.X(),
            bullet_origin_world.Y(), bullet_origin_world.Z()), false, false, true);

    if (bullet_sound)
        bullet_sound->setMinDistance(25.0f);

    bullet_sound->setVolume(0.5);
#endif

}


void R3Aircraft::
ThrustForward(double delta_time)
{
    double delta_thrust = max_thrust * delta_time / SECONDS_TO_MAX_THRUST;
    thrust_magnitude = min(thrust_magnitude + delta_thrust, max_thrust); // always <= max_thrust
    AssertValid();
}

void R3Aircraft::
BrakeBackward(double delta_time)
{
    double delta_thrust = max_thrust * delta_time / SECONDS_TO_MAX_THRUST;
    thrust_magnitude = max(thrust_magnitude - delta_thrust, max_thrust / 10.0); // can't go too slow (how lift works bro)
    AssertValid();
}

void R3Aircraft::
HardModeUpdate(R3Matrix mat) {
    R3Vector transformed = velocity;
    transformed.Transform(mat.Transpose());
    transformed /= 2;

    R3Vector forward = velocity;
    forward /= 2;

    R3Vector newVelocity = transformed + forward;
    velocity.SetX(newVelocity.X());
    velocity.SetY(newVelocity.Y());
    velocity.SetZ(newVelocity.Z());
    // optionally split velocity in two directions, one in transformed, the other in original
}


void R3Aircraft::
PitchUp(double delta_time)
{
    // use mass to slow turn rate
    const double COS_THETA = cos(THETA/mass * delta_time);
    const double SIN_THETA = sin(THETA/mass * delta_time);

    R3Matrix mat(COS_THETA, 0, -SIN_THETA, 0,
            0, 1, 0, 0,
            SIN_THETA, 0, COS_THETA, 0,
            0, 0, 0, 1);
    T *= mat;
    if (hard_mode == 1) { // makes aircraft velocity more realistic. see writeup for more details
        HardModeUpdate(mat);
    }
    AssertValid();
}

void R3Aircraft::
PitchDown(double delta_time)
{
    const double COS_THETA = cos(THETA/mass * delta_time);
    const double SIN_THETA = sin(THETA/mass * delta_time);

    R3Matrix mat(COS_THETA, 0, SIN_THETA, 0,
            0, 1, 0, 0,
            -SIN_THETA, 0, COS_THETA, 0,
            0, 0, 0, 1);
    T *= mat;
    if (hard_mode == 1) // makes aircraft velocity more realistic. see writeup for more details
        HardModeUpdate(mat);
    AssertValid();
}

void R3Aircraft::
YawLeft(double delta_time)
{
    const double COS_THETA = cos(2 * THETA/mass/10 * delta_time);
    const double SIN_THETA = sin(2 * THETA/mass/10 * delta_time);

    R3Matrix mat(COS_THETA, -SIN_THETA, 0, 0,
            SIN_THETA,  COS_THETA, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);
    T *= mat;
    if (hard_mode == 1) // makes aircraft velocity more realastic. see writeup for more details
        HardModeUpdate(mat);
    AssertValid();
}

void R3Aircraft::
YawRight(double delta_time)
{
    const double COS_THETA = cos(2 * THETA/mass/10 * delta_time);
    const double SIN_THETA = sin(2 * THETA/mass/10 * delta_time);

    R3Matrix mat(COS_THETA, SIN_THETA, 0, 0,
            -SIN_THETA,  COS_THETA, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1);
    T *= mat;
    if (hard_mode == 1) // makes aircraft velocity more realastic. see writeup for more details
        HardModeUpdate(mat);
    AssertValid();
}


void R3Aircraft::
RollLeft(double delta_time)
{
    const double COS_THETA = cos(2 * THETA/mass * delta_time);
    const double SIN_THETA = sin(2 * THETA/mass * delta_time);

    R3Matrix mat(1, 0, 0, 0,
            0, COS_THETA, SIN_THETA, 0,
            0, -SIN_THETA, COS_THETA, 0,
            0, 0, 0, 1);
    T *= mat;
    if (hard_mode == 1) // makes aircraft velocity more realastic. see writeup for more details
        HardModeUpdate(mat);
    AssertValid();
}


void R3Aircraft::
RollRight(double delta_time)
{
    const double COS_THETA = cos(2 * THETA/mass * delta_time);
    const double SIN_THETA = sin(2 * THETA/mass * delta_time);

    R3Matrix mat(1, 0, 0, 0,
            0, COS_THETA, -SIN_THETA, 0,
            0, SIN_THETA, COS_THETA, 0,
            0, 0, 0, 1);
    T *= mat;
    if (hard_mode == 1) // makes aircraft velocity more realistic. see writeup for more details
        HardModeUpdate(mat);
    AssertValid();
}


/////////////////////////////////////////////////////////////////////////////////////////
// Actions for if aircraft is hit/destroyed (bullets hit or aircraft crashes into mesh)
//////////////////////////////////////////////////////////////////////////////////////////

void R3Aircraft::
HitAircraft(R3Scene *scene)
{
    hitpoints--;
    is_hit = true;

    // plane dies if hitpoints <= 0
    if (hitpoints <= 0)
    {
        // if user-controlled airplane that blows up, you lose!
        if (this == scene->Aircraft(0))
        {
            num_deaths++;
            this->hitpoints = respawn_hitpoints;
            this->freeze_time = FREEZE_TIME;
            bool should_explode = true;
            bool should_respawn = false;
            this->Destroy(scene, should_explode, should_respawn);
        }

        // else, it is an AI plane
        else
        {
            num_kills++;
            bool should_explode = true;
            bool should_respawn = true;
            this->Destroy(scene, should_explode, should_respawn);
        }
    }
}

void R3Aircraft::
Destroy(R3Scene *scene, bool should_explode, bool should_respawn)
{
    if (should_explode)
        this->Explode(scene, false);

    if (should_respawn)
        this->Respawn();
}




void R3Aircraft::
Explode(R3Scene *scene, bool is_collision_scene)
{
    // set up materials only once
    static bool is_material_initialized = false;
    static R3Material orange_shrapnel, red_shrapnel, black_shrapnel;
    if (!is_material_initialized)
    {
        is_material_initialized = true;

        // set up orange shrapnel
        orange_shrapnel.ka.Reset(0.2,0.2,0.2,1);
        orange_shrapnel.kd.Reset(1, 0.5, 0,1);
        orange_shrapnel.ks.Reset(0, 0, 1,1);
        orange_shrapnel.kt.Reset(0,0,0,1);
        orange_shrapnel.emission.Reset(0,0,0,1);
        orange_shrapnel.shininess = 1;
        orange_shrapnel.indexofrefraction = 1;
        orange_shrapnel.texture = NULL;
        orange_shrapnel.texture_index = -1;

        // set up red shrapnel
        red_shrapnel.ka.Reset(0.2,0.2,0.2,1);
        red_shrapnel.kd.Reset(1, 0,0,1);
        red_shrapnel.ks.Reset(0, 0, 1,1);
        red_shrapnel.kt.Reset(0,0,0,1);
        red_shrapnel.emission.Reset(0,0,0,1);
        red_shrapnel.shininess = 1;
        red_shrapnel.indexofrefraction = 1;
        red_shrapnel.texture = NULL;
        red_shrapnel.texture_index = -1;

        // set up black shrapnel
        black_shrapnel.ka.Reset(0.2,0.2,0.2,1);
        black_shrapnel.kd.Reset(0, 0, 0,1);
        black_shrapnel.ks.Reset(0, 0, 1,1);
        black_shrapnel.kt.Reset(0,0,0,1);
        black_shrapnel.emission.Reset(0,0,0,1);
        black_shrapnel.shininess = 1;
        black_shrapnel.indexofrefraction = 1;
        black_shrapnel.texture = NULL;
        black_shrapnel.texture_index = -1;
    }


    const int num_particles_to_generate = 500;
    int particles_to_generate = num_particles_to_generate * (is_collision_scene ? 5 : 1);
    const double radius = 1;
    const double fast_velocity = 15;
    const double particle_lifetime = 1;


    R3ParticleSource * new_source = new R3ParticleSource();
    R3Point center = Modeling_To_World(R3Vector(0, 0, 0)).Point();
    R3Sphere *sphere = new R3Sphere(center, radius);
    R3Shape *shape = new R3Shape();
    shape->sphere = sphere;
    shape->type = R3_SPHERE_SHAPE;
    new_source->shape = shape;
    new_source->rate = 0;
    new_source->velocity = fast_velocity;
    new_source->angle_cutoff = 0;
    new_source->mass = 0.01;
    new_source->fixed = false;
    new_source->drag = 0;
    new_source->elasticity = 0;
    new_source->lifetime = particle_lifetime;
    new_source->material = &orange_shrapnel;


    int one_third = (int) particles_to_generate / 3.0;
    int two_third = one_third * 2;

    for (int i = 0; i < particles_to_generate; i++)
    {
        if (i > one_third && i < two_third)
            new_source->material = &red_shrapnel;
        if (i > two_third)
            new_source->material = &black_shrapnel;

        // calculate point of emanation
        double z = RandomNumber() * 2 * radius - radius;
        double phi = RandomNumber() * 2 * PI;
        double d = sqrt(radius * radius - z * z);
        double px = center.X() + d * cos(phi);
        double py = center.Y() + d * sin(phi);
        double pz = center.Z() + z;
        R3Point point_emanate(px, py, pz);

        // calculate direction of emanation
        R3Vector N = (point_emanate - center); // surface normal
        N.Normalize();
        R3Vector direction_emanate = (N.X() == 0 && N.Y() == 0) ?
                R3Vector(1.0, 0.0, 0.0) : R3Vector(N.Y(), -N.X(), 0.0); // vector on tangent plane (handle edge cases)
        double t1 = RandomNumber() * 2 * PI;
        double t2 = RandomNumber() * sin(new_source->angle_cutoff);
        direction_emanate.Rotate(N, t1);
        R3Vector cross = direction_emanate;
        cross.Cross(N);
        direction_emanate.Rotate(cross, acos(t2));
        direction_emanate.Normalize();

        // generate a particle
        double rand1 = RandomNumber();
        double rand2 = RandomNumber();
        new_source->velocity = fast_velocity * rand1 * rand1 * rand1 * rand1 * rand1;
        new_source->lifetime = particle_lifetime * rand2;
        R3Particle * new_particle = new R3Particle(point_emanate, direction_emanate, new_source);
        scene->particles.push_back(new_particle);
    }
    //  }

    delete new_source;
    delete sphere;
    delete shape;


    // sound for explosion
#ifdef __APPLE__
    // 3D sound!!
    R3Aircraft *player_aircraft = scene->Aircraft(0);
    R3Vector pos_player_aircraft(0, 0, 0);
    pos_player_aircraft = player_aircraft->Modeling_To_World(pos_player_aircraft);

    R3Vector dir_player_aircraft(1, 0, 0);
    dir_player_aircraft.Transform(player_aircraft->T);

    engine->setListenerPosition(irrklang::vec3df(pos_player_aircraft.X(), pos_player_aircraft.Y(), pos_player_aircraft.Z()),
            irrklang::vec3df(dir_player_aircraft.X(), dir_player_aircraft.Y(), dir_player_aircraft.Z()));

    irrklang::ISound* music = engine->play3D("../music/explosion.wav", irrklang::vec3df(center.X(),
            center.Y(), center.Z()), false, false, true);

    if (music)
        music->setMinDistance(25.0f);

#endif
}


void R3Aircraft::
Respawn(void)
{
    velocity = respawn_velocity;
    T = respawn_T;
    thrust_magnitude = respawn_thrust_magnitude;
    hitpoints = respawn_hitpoints;
    AssertValid();
}


//////////////////////////////////////////////////
// Intelligent aircraft movement
//////////////////////////////////////////////////

void R3Aircraft::
AI_decision(R3Scene *scene, R3Aircraft *enemy, double delta_time)
{
    // modeling coordinates -> world coordinates for both planes
    R3Point aircraft_position = this->Modeling_To_World(R3Vector(0, 0, 0)).Point();
    R3Point enemy_position = enemy->Modeling_To_World(R3Vector(0, 0, 0)).Point();

    // convert cartesian coordinates (to_enemy) to spherical coordinates: (x,y,z) -> (dist, theta, phi)
    R3Vector vector_to_enemy_xyz = enemy_position - aircraft_position;
    double dist_to_enemy = vector_to_enemy_xyz.Length();

    // convert cartesian coordinates (to_enemy) to spherical coordinates: rotated(x,y,z) -> (dist, theta, phi)
    R3Vector vector_to_enemy_rotated = vector_to_enemy_xyz;
    vector_to_enemy_rotated.Transform(this->T.Transpose());
    double phi_rotated = atan2(sqrt(vector_to_enemy_rotated.X() *
            vector_to_enemy_rotated.X() + vector_to_enemy_rotated.Y() * vector_to_enemy_rotated.Y()), vector_to_enemy_rotated.Z());
    double theta_rotated = atan2(vector_to_enemy_rotated.Y(), vector_to_enemy_rotated.X());


    //  cout << phi_rotated << "\t" << theta_rotated << endl;

    // shoot if aimed properly
    if (abs(theta_rotated) < AI_RADIUS_SHOOTING_RANGE && abs(phi_rotated - PI/2.0) < AI_RADIUS_SHOOTING_RANGE)
    {
        // make sure doesn't fire too rapidly
        time_since_last_fired += delta_time;
        if (time_since_last_fired > (1.0 / firing_rate))
        {
            // don't fire when player-controlled aircraft is frozen / animation time
            if (scene->Aircraft(0)->freeze_time < 0)
                FireBullet(scene);
        }
    }

    // ROLL LEFT OR RIGHT TO AIM
    if (abs(theta_rotated) > AI_RADIUS_MOVING_RANGE)
    {
        // boundary case: if enemy is directly behind AI
        if (abs(theta_rotated - PI) < 1.0 || abs(theta_rotated + PI) < 1.0)
            PitchUp(delta_time);

        // else, not directly behind: so just make small adjustments to aim
        else if (theta_rotated < 0)
            RollRight(delta_time);
        else if (theta_rotated > 0)
            RollLeft(delta_time);
    }


    // TILT UP OR DOWN TO AIM
    // Note: "else if" because planes can't simultaneously roll and tilt at the same time; this also
    // makes turning left / right more smooth
    if (abs(phi_rotated - PI/2.0) > AI_RADIUS_MOVING_RANGE)
    {
        if (phi_rotated < PI/2.0)
            PitchUp(delta_time);
        else if (phi_rotated > PI/2.0)
            PitchDown(delta_time);
    }


    // SPEED UP OR SLOW DOWN BASED ON HOW FAR AWAY THE ENEMY IS
    if (abs(dist_to_enemy - AI_DISTANCE_HI_LO_THRUST) > AI_DISTANCE_HI_LO_THRUST)
    {
        if (abs(dist_to_enemy) < AI_DISTANCE_HI_LO_THRUST || abs(theta_rotated - PI) < AI_RADIUS_MOVING_RANGE)
            BrakeBackward(delta_time);
        else
            ThrustForward(delta_time);
    }


    //  // For visualization of modeling coordinates with respect to the aircraft, in the real world
    //  // Draw x, y, z for each aircraft
    //  // Draw meshes under transformation
    //  glDisable(GL_LIGHTING);
    //  glLineWidth(3);
    //  glBegin(GL_LINES);
    //
    //  R3Aircraft *aircraft = this;
    //  R3Vector origin = aircraft->Modeling_To_World(R3Vector(0, 0, 0));
    //  R3Vector x_vec = aircraft->Modeling_To_World(R3Vector(1, 0, 0));
    //  R3Vector y_vec = aircraft->Modeling_To_World(R3Vector(0, 1, 0));
    //  R3Vector z_vec = aircraft->Modeling_To_World(R3Vector(0, 0, 1));
    //
    //  R3Vector enemy_vec_modeling2 = dist_to_enemy * R3Vector(sin(phi_rotated) * cos(theta_rotated), sin(phi_rotated) * sin(theta_rotated), cos(phi_rotated));
    //  enemy_vec_modeling2.Transform(this->T);
    //  R3Vector enemy_vec2 = aircraft->Modeling_To_World(enemy_vec_modeling2);
    //
    //  // draw x in RED
    //  glColor3d(1, 0, 0);
    //  glVertex3f(origin.X(), origin.Y(), origin.Z());
    //  glVertex3f(x_vec.X(), x_vec.Y(), x_vec.Z());
    //  // draw y in GREEN
    //  glColor3d(0, 1, 0);
    //  glVertex3f(origin.X(), origin.Y(), origin.Z());
    //  glVertex3f(y_vec.X(), y_vec.Y(), y_vec.Z());
    //  // draw z in BLUE
    //  glColor3d(0, 0, 1);
    //  glVertex3f(origin.X(), origin.Y(), origin.Z());
    //  glVertex3f(z_vec.X(), z_vec.Y(), z_vec.Z());
    //
    ////  // draw enemy_vec1 in PINK
    ////  glColor3d(1, 0, 1);
    ////  glVertex3f(origin.X(), origin.Y(), origin.Z());
    ////  glVertex3f(enemy_vec1.X(), enemy_vec1.Y(), enemy_vec1.Z());
    //
    //  // draw enemy_vec2 in YELLOW
    //   glColor3d(1, 1, 0);
    //   glVertex3f(origin.X(), origin.Y(), origin.Z());
    //   glVertex3f(enemy_vec2.X(), enemy_vec2.Y(), enemy_vec2.Z());
    //   glEnd();
}

/////////////////////////////////////////////////////////////////////////////////////////
// Updating (moving) and rendering aircrafts
//////////////////////////////////////////////////////////////////////////////////////////

void UpdateAircrafts(R3Scene *scene, double current_time, double delta_time, int integration_type)
{
    for (int i = 0; i < scene->NAircrafts(); i++)
    {
        R3Aircraft *aircraft = scene->Aircraft(i);

        // PLAYER-CONTROLLED AIRCRAFT
        if (i == 0)
        {
            // if you suck and your aircraft blows up
            if (aircraft->freeze_time >= 0)
            {
                aircraft->freeze_time -= delta_time;
                if (aircraft->freeze_time < 0)
                    aircraft->Respawn();
                continue;
            }

            // listen to player keyboard controls
            if (pitch_up)
                aircraft->PitchUp(delta_time);
            if (pitch_down)
                aircraft->PitchDown(delta_time);
            if (roll_left)
                aircraft->RollLeft(delta_time);
            if (roll_right)
                aircraft->RollRight(delta_time);
            if (yaw_left)
                aircraft->YawLeft(delta_time);
            if (yaw_right)
                aircraft->YawRight(delta_time);
            if (thrust_forward)
                aircraft->ThrustForward(delta_time);
            if (brake_backward)
                aircraft->BrakeBackward(delta_time);
            if (firing_bullets)
            {
                // make firing rate more constant --> sounds more like machine gun
                aircraft->time_since_last_fired += delta_time;
                if (aircraft->time_since_last_fired > (1.0 / aircraft->firing_rate))
                    aircraft->FireBullet(scene);
            }
        }

        // AI-CONTROLLED AIRCRAFT
        else
        {
            // Note: could easily generalize to making teams for dogfights: simply find the closest_enemy (easy).

//            if (aircraft == scene->Aircraft(1)) aircraft->team = 1;
//            else if (aircraft == scene->Aircraft(2)) aircraft->team = 2;

//            double closest_dist = 1000000;
//            R3Aircraft *closest_enemy = aircraft;
            R3Aircraft *closest_enemy = scene->Aircraft(0);

//            for (int i = 0; i < scene->NAircrafts(); i++) {
//                R3Aircraft *enemy_aircraft = scene->Aircraft(i);
//                if (aircraft == enemy_aircraft || aircraft->team == enemy_aircraft->team) continue;
//                double dist = (enemy_aircraft->Modeling_To_World(R3Vector(0,0,0)) - aircraft->Modeling_To_World(R3Vector(0,0,0))).Length();
//                if (dist < closest_dist) {
//                    closest_enemy = enemy_aircraft;
//                    closest_dist = dist;
//                }
//            }

//            if (closest_enemy != aircraft)
                aircraft->AI_decision(scene, closest_enemy, delta_time);
        }

        // UPDATE POSITION with velocity (simple Euler integration)
        R3Vector change_position_modeling = aircraft->velocity * delta_time;

        // check no collision
        R3Vector prev_position (aircraft->T[0][3], aircraft->T[1][3], aircraft->T[2][3]);
        R3Vector change_position_world = change_position_modeling;
        change_position_world.Transform(aircraft->T);
        R3Ray ray(prev_position.Point(), change_position_world);
        R3Particle *fake_bullet = new R3Particle();
        fake_bullet->aircraft_fired_from = aircraft;

        R3Intersection scene_intersection;
        R3Intersection airplane_intersection;
        if (scene->Aircraft(0)->freeze_time > 0)
        {
            scene_intersection.SetMiss();
            airplane_intersection.SetMiss();
        }
        else
        {
            scene_intersection = ComputeIntersection(scene, scene->Root(), ray);
            airplane_intersection = ComputeIntersectionWithAircrafts(scene, ray, fake_bullet);
        }
        scene_intersection.AssertValid();
        airplane_intersection.AssertValid();

        delete fake_bullet;

        if (scene_intersection.IsHit() && scene_intersection.distance < change_position_world.Length())
        {
            // freeze aircraft if player-controlled aircraft blows up --> see the explosion before respawn
            bool is_collision_scene = false;
            aircraft->Explode(scene, is_collision_scene);

            // if player-controlled aircraft
            if (aircraft == scene->Aircraft(0))
            {
                aircraft->freeze_time = FREEZE_TIME;
                num_deaths++;
                is_collision_scene = true;
            }

            // if AI-controlled aircraft
            else
            {
                is_collision_scene = false;
                aircraft->Respawn();
            }
            return;
        }

        if (airplane_intersection.IsHit() && airplane_intersection.distance < change_position_world.Length())
        {
            R3Aircraft *other_aircraft = airplane_intersection.aircraft;
            assert(other_aircraft != aircraft);

            // if player-controlled aircraft
            if (aircraft == scene->Aircraft(0) || other_aircraft == scene->Aircraft(0))
            {
                aircraft->freeze_time = FREEZE_TIME;
                num_deaths++;
                num_kills++;
            }

            // explode both aircrafts
            other_aircraft->Explode(scene, true);
            aircraft->Explode(scene, true);

            // make the AI controlled aircraft respawn
            if (aircraft == scene->Aircraft(0))
                other_aircraft->Respawn();
            else
                aircraft->Respawn();

            return;
        }


        aircraft->T.Translate(change_position_modeling);

        R3Vector source_1_position_modeling(AIRCRAFT_SOURCE_BACK_NEG, AIRCRAFT_SOURCE_SIDE_POS, AIRCRAFT_SOURCE_DOWN_NEG);
        R3Vector source_2_position_modeling(AIRCRAFT_SOURCE_BACK_NEG, -AIRCRAFT_SOURCE_SIDE_POS, AIRCRAFT_SOURCE_DOWN_NEG);
        R3Vector source_1_position_world = aircraft->Modeling_To_World(source_1_position_modeling);
        R3Vector source_2_position_world = aircraft->Modeling_To_World(source_2_position_modeling);
        R3Vector normal_modeling(-1, 0, 0);
        R3Vector normal_world = normal_modeling;
        normal_world.Transform(aircraft->T);

        aircraft->sources[0]->shape->circle->Reposition(source_1_position_world.Point());
        aircraft->sources[0]->shape->circle->Align(normal_world);

        aircraft->sources[1]->shape->circle->Reposition(source_2_position_world.Point());
        aircraft->sources[1]->shape->circle->Align(normal_world);

        aircraft->sources[0]->rate = AIRCRAFT_EXHAUST_RATE_MAX * aircraft->thrust_magnitude / aircraft->max_thrust;
		if (aircraft->sources[0]->rate < 5) aircraft->sources[0]->rate = 5;
       
		aircraft->sources[1]->rate = aircraft->sources[0]->rate;


        R3Vector v_normalized = aircraft->velocity;
        v_normalized.Normalize();
        double drag_scale = (v_normalized.Dot(R3Vector(1, 0, 0)));
//        cout << drag_scale << endl;

		if (drag_scale < 0) drag_scale = 0;
       // drag_scale = std::max(drag_scale, 0);

        // UPDATE VELOCITY with acceleration (simple Euler integration)
        R3Vector net_force = R3Vector(aircraft->thrust_magnitude, 0, 0) - aircraft->drag * aircraft->velocity;

        // account for lift if on hard mode
        if (hard_mode)
        {
            // Source: https://www.grc.nasa.gov/www/k-12/WindTunnel/Activities/lift_formula.html
            // An aircraft's lift capabilities can be measured from the following formula:
            // L = (1/2) d v^2 s CL
            // L = Lift, which must equal the airplane's weight in pounds
            // d = density of the air. This will change due to altitude. These values can be found in a I.C.A.O. Standard Atmosphere Table.
            // v = velocity of an aircraft expressed in feet per second
            // s = the wing area of an aircraft in square feet
            // CL = Coefficient of lift , which is determined by the type of airfoil and angle of attack.

            // note that for us, we have LIFT_CONSTANT ==defined== d * s * CL

            const double LIFT_CONSTANT = 3;

            double velocity_magnitude = aircraft->velocity.Length();
            R3Vector velocity_world = aircraft->velocity;
            velocity_world.Transform(aircraft->T);
            velocity_world.Normalize();

            R3Vector vec_forward(1, 0, 0);
            vec_forward.Transform(aircraft->T);


            double v = (aircraft->velocity.Dot(R3Vector(1, 0, 0)));

            //      cout << v << endl;

            double lift_magnitude = v * LIFT_CONSTANT / 2.0;
            R3Vector lift_force(0, 0, lift_magnitude);
            //      lift_force.Transform(aircraft->T);
            net_force += lift_force;
        }

        const double MAX_Z_ACCELERATION = .1;

        R3Vector acceleration = net_force / aircraft->mass;

        R3Vector GRAVITY_VECTOR(0,0,-5);

        //    cout << acceleration.Z() << endl;

        // account for gravity if on hard mode
        if (hard_mode)
        {
            acceleration += GRAVITY_VECTOR;
            acceleration.SetZ(std::min(MAX_Z_ACCELERATION, acceleration.Z()));
        }

        aircraft->velocity += acceleration * delta_time;
        aircraft->AssertValid();
    }


#ifdef __APPLE__
    // 3D THRUST SOUNDS FOR AIRPLANES -- set up once then update based on positions
    static bool thrust_sounds_init = false;
    static vector<irrklang::ISound*> thrust_sounds;


    // get position and direction of listener (player-controlled aircraft)
    R3Aircraft *player_aircraft = scene->Aircraft(0);
    R3Vector pos_player_aircraft(0, 0, 0);
    pos_player_aircraft = player_aircraft->Modeling_To_World(pos_player_aircraft);
    R3Vector dir_player_aircraft(1, 0, 0);
    dir_player_aircraft.Transform(player_aircraft->T);

    // update position and direction of listener (player-controlled aircraft)
    engine->setListenerPosition(irrklang::vec3df(pos_player_aircraft.X(), pos_player_aircraft.Y(), pos_player_aircraft.Z()),
            irrklang::vec3df(dir_player_aircraft.X(), dir_player_aircraft.Y(), dir_player_aircraft.Z()));

    // initialize thrust sounds if not done so already (only done once!)
    if (!thrust_sounds_init)
    {
        thrust_sounds_init = true;
        thrust_sounds.resize(scene->NAircrafts());

        for (int i = 0; i < scene->NAircrafts(); i++)
        {
            // make 3D thrust sound for each aircraft
            R3Aircraft *other_aircraft = scene->Aircraft(i);
            R3Vector pos_other_aircraft(0, 0, 0);
            pos_other_aircraft = other_aircraft->Modeling_To_World(pos_other_aircraft);

            bool should_loop = true;
            irrklang::ISound* thrust_sound = engine->play3D("../music/thrust.wav", irrklang::vec3df(pos_other_aircraft.X(),
                    pos_other_aircraft.Y(), pos_other_aircraft.Z()), should_loop, false, true);

            if (!thrust_sound)
            {
                fprintf(stderr, "Problem creating thrust sound with irrklang");
                exit(1);
            }

            // set up sound parameters
            thrust_sound->setMinDistance(25.0f);
            thrust_sound->setVolume(other_aircraft->thrust_magnitude / other_aircraft->max_thrust);
            thrust_sounds[i] = thrust_sound;
        }
    }

    // if background_sounds already initialized --> update based on new positions of aircrafts
    else
    {
        for (int i = 0; i < scene->NAircrafts(); i++)
        {
            R3Aircraft *other_aircraft = scene->Aircraft(i);
            R3Vector pos_other_aircraft(0, 0, 0);
            pos_other_aircraft = other_aircraft->Modeling_To_World(pos_other_aircraft);

            // change volume based on thrust level
            thrust_sounds[i]->setVolume(other_aircraft->thrust_magnitude / other_aircraft->max_thrust);


            // update posiiton of sound source (3D sound!)
            thrust_sounds[i]->setPosition(irrklang::vec3df(pos_other_aircraft.X(), pos_other_aircraft.Y(), pos_other_aircraft.Z()));
        }
    }
#endif

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
        R3Rgb kd_backup(aircraft->material->kd);
        if (aircraft->material) {
            // if hit, flash red
            if (aircraft->is_hit) {
                //           cout << "print" << endl;
                aircraft->material->ks.SetRed(1);
                aircraft->material->ks.SetBlue(0);
                aircraft->material->ks.SetGreen(0);
            }
            LoadMaterial(aircraft->material);
        }

        // Draw shape
        // don't draw if user is destroyed
        if (i != 0 || aircraft->freeze_time <= 0)
        {
            if (aircraft->mesh) aircraft->mesh->Draw();
            else { fprintf(stderr, "problem drawing mesh!"); exit(1); }
        }

        // Restore material
        if (aircraft->is_hit) {
            aircraft->is_hit = false;
            aircraft->material->ks.SetRed(kd_backup.Red());
            aircraft->material->ks.SetBlue(kd_backup.Blue());
            aircraft->material->ks.SetGreen(kd_backup.Green());
        }

        glPopMatrix();
    }



    //  // For visualization of modeling coordinates with respect to the aircraft, in terms of world coordinates
    //  // Draw x, y, z for each aircraft
    //  // Draw meshes under transformation
    //  glDisable(GL_LIGHTING);
    //  glLineWidth(3);
    //  glBegin(GL_LINES);
    //
    //  R3Aircraft *player_aircraft = scene->Aircraft(0);
    //  R3Vector origin = player_aircraft->Modeling_To_World(R3Vector(0, 0, 0));
    //  R3Vector x_vec = player_aircraft->Modeling_To_World(R3Vector(1, 0, 0));
    //  R3Vector y_vec = player_aircraft->Modeling_To_World(R3Vector(0, 1, 0));
    //  R3Vector z_vec = player_aircraft->Modeling_To_World(R3Vector(0, 0, 1));
    //
    //  // draw x in RED
    //  glColor3d(1, 0, 0);
    //  glVertex3f(origin.X(), origin.Y(), origin.Z());
    //  glVertex3f(x_vec.X(), x_vec.Y(), x_vec.Z());
    //
    //  // draw y in GREEN
    //  glColor3d(0, 1, 0);
    //  glVertex3f(origin.X(), origin.Y(), origin.Z());
    //  glVertex3f(y_vec.X(), y_vec.Y(), y_vec.Z());
    //
    //  // draw z in BLUE
    //  glColor3d(0, 0, 1);
    //  glVertex3f(origin.X(), origin.Y(), origin.Z());
    //  glVertex3f(z_vec.X(), z_vec.Y(), z_vec.Z());
    //
    //
    //  glEnd();


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
}
