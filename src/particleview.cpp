// Source file for the scene file viewer



////////////////////////////////////////////////////////////
// INCLUDE FILES
////////////////////////////////////////////////////////////

#include "particleview.h"
#include "R3Aircraft.h"
#include "R3/R3.h"
#include "R3Scene.h"
#include "particle.h"
#include "fglut/fglut.h"
//#include <GL/freeglut.h>


// Define skybox constants
#define SKY_FRONT 0
#define SKY_RIGHT 1
#define SKY_LEFT 2
#define SKY_BACK 3
#define SKY_UP 4
#define SKY_DOWN 5

////////////////////////////////////////////////////////////
// GLOBAL CONSTANTS
////////////////////////////////////////////////////////////

// XXX: Note when I created the videos, I commented out line 20 and uncommented line 21
static const double VIDEO_FRAME_DELAY = 1./25.; // 25 FPS
//static const double VIDEO_FRAME_DELAY = 1./100.; // 100 FPS
static const double METERS_PER_UNIT = 7;
static const float DEG2RAD = 3.14159/180;

static int skybox[6];

// rock, grass, snow, and dirt
static R3Material **texture_materials = new R3Material*[5];


////////////////////////////////////////////////////////////
// GLOBAL VARIABLES
////////////////////////////////////////////////////////////

// Program arguments

static char *input_scene_name = NULL;
static char *output_image_name = NULL;
static const char *video_prefix = "./video-frames/";
static int integration_type = EULER_INTEGRATION;



// Display variables

static R3Scene *scene = NULL;
static R3Camera camera;

// Added by Kyle. Keeps track of current position of all camera views for SkyBox purposes.
static R3Point current_camera_position(0,0,0);

static int show_faces = 1;
static int show_edges = 0;
static int show_bboxes = 0;
static int show_lights = 0;
static int show_camera = 0;
static int show_particles = 1;
static int show_particle_springs = 1;
static int show_particle_sources_and_sinks = 1;
static int save_image = 0;
static int save_video = 0;
static int num_frames_to_record = -1; 
static int quit = 0;
static int camera_view = 2; // defaulted to free view

// controls for the aircraft
int pitch_up = 0;
int pitch_down = 0;
int roll_left = 0;
int roll_right = 0;
int thrust_forward = 0;
int brake_backward = 0;
int firing_bullets = 0;

int hard_mode = 0; // default to easy

// GLUT variables 

static int GLUTwindow = 0;
static int GLUTwindow_height = 1000;
static int GLUTwindow_width = 1000;
static int GLUTmouse[2] = { 0, 0 };
static int GLUTbutton[3] = { 0, 0, 0 };
static int GLUTmodifiers = 0;



// GLUT command list

enum {
    DISPLAY_FACE_TOGGLE_COMMAND,
    DISPLAY_EDGE_TOGGLE_COMMAND,
    DISPLAY_BBOXES_TOGGLE_COMMAND,
    DISPLAY_LIGHTS_TOGGLE_COMMAND,
    DISPLAY_CAMERA_TOGGLE_COMMAND,
    DISPLAY_PARTICLES_TOGGLE_COMMAND,
    DISPLAY_PARTICLE_SPRINGS_TOGGLE_COMMAND,
    DISPLAY_PARTICLE_SOURCES_AND_SINKS_TOGGLE_COMMAND,
    SAVE_IMAGE_COMMAND,
    SAVE_VIDEO_COMMAND,
    QUIT_COMMAND,
};



// TEXTURE VARIABLES


typedef enum
{
    ROCK,
    GRASS,
    SNOW,
    DIRT,
    OCEAN,
    NUM_TEXTURES
} R3Texture;


////////////////////////////////////////////////////////////
// TIMER CODE
////////////////////////////////////////////////////////////

#ifdef _WIN32
#  include <windows.h>
#else
#  include <sys/time.h>
#endif

static double GetTime(void)
{
#ifdef _WIN32
    // Return number of seconds since start of execution
    static int first = 1;
    static LARGE_INTEGER timefreq;
    static LARGE_INTEGER start_timevalue;

    // Check if this is the first time
    if (first) {
        // Initialize first time
        QueryPerformanceFrequency(&timefreq);
        QueryPerformanceCounter(&start_timevalue);
        first = 0;
        return 0;
    }
    else {
        // Return time since start
        LARGE_INTEGER current_timevalue;
        QueryPerformanceCounter(&current_timevalue);
        return ((double) current_timevalue.QuadPart -
                (double) start_timevalue.QuadPart) /
                (double) timefreq.QuadPart;
    }
#else
    // Return number of seconds since start of execution
    static int first = 1;
    static struct timeval start_timevalue;

    // Check if this is the first time
    if (first) {
        // Initialize first time
        gettimeofday(&start_timevalue, NULL);
        first = 0;
        return 0;
    }
    else {
        // Return time since start
        struct timeval current_timevalue;
        gettimeofday(&current_timevalue, NULL);
        int secs = current_timevalue.tv_sec - start_timevalue.tv_sec;
        int usecs = current_timevalue.tv_usec - start_timevalue.tv_usec;
        return (double) (secs + 1.0E-6F * usecs);
    }
#endif
}



////////////////////////////////////////////////////////////
// SCENE DRAWING CODE
////////////////////////////////////////////////////////////

void DrawShape(R3Shape *shape)
{
    // Check shape type
    if (shape->type == R3_BOX_SHAPE) shape->box->Draw();
    else if (shape->type == R3_SPHERE_SHAPE) shape->sphere->Draw();
    else if (shape->type == R3_CYLINDER_SHAPE) shape->cylinder->Draw();
    else if (shape->type == R3_CONE_SHAPE) shape->cone->Draw();
    else if (shape->type == R3_MESH_SHAPE) shape->mesh->Draw();
    else if (shape->type == R3_SEGMENT_SHAPE) shape->segment->Draw();
    else if (shape->type == R3_CIRCLE_SHAPE) shape->circle->Draw();
    else fprintf(stderr, "Unrecognized shape type: %d\n", shape->type);
}



void LoadMatrix(R3Matrix *matrix)
{
    // Multiply matrix by top of stack
    // Take transpose of matrix because OpenGL represents vectors with
    // column-vectors and R3 represents them with row-vectors
    R3Matrix m = matrix->Transpose();
    glMultMatrixd((double *) &m);
}



void LoadMaterial(R3Material *material)
{
    GLfloat c[4];

    // Check if same as current
    static R3Material *current_material = NULL;
    if (material == current_material) return;
    current_material = material;

    // Compute "opacity"
    double opacity = 1 - material->kt.Luminance();

    // Load ambient
    //    c[0] = material->ka[0];
    //    c[1] = material->ka[1];
    //    c[2] = material->ka[2];
    //    c[3] = opacity;
    //    glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, c);
    //
    //    // Load diffuse
    //    c[0] = material->kd[0];
    //    c[1] = material->kd[1];
    //    c[2] = material->kd[2];
    //    c[3] = opacity;
    //    glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, c);
    //
    //    // Load specular
    //    c[0] = material->ks[0];
    //    c[1] = material->ks[1];
    //    c[2] = material->ks[2];
    //    c[3] = opacity;
    //    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, c);
    //
    //    // Load emission
    //    c[0] = material->emission.Red();
    //    c[1] = material->emission.Green();
    //    c[2] = material->emission.Blue();
    //    c[3] = opacity;
    //    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, c);
    //
    //    // Load shininess
    //    c[0] = material->shininess;
    //    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, c[0]);

    // Load texture
    if (material->texture) {
        if (material->texture_index <= 0) {
            // Create texture in OpenGL
            GLuint texture_index;
            glGenTextures(1, &texture_index);
            material->texture_index = (int) texture_index;
            glBindTexture(GL_TEXTURE_2D, material->texture_index);
            R2Image *image = material->texture;
            int npixels = image->NPixels();
            R2Pixel *pixels = image->Pixels();
            GLfloat *buffer = new GLfloat [ 4 * npixels ];
            R2Pixel *pixelsp = pixels;
            GLfloat *bufferp = buffer;
            for (int j = 0; j < npixels; j++) {
                *(bufferp++) = pixelsp->Red();
                *(bufferp++) = pixelsp->Green();
                *(bufferp++) = pixelsp->Blue();
                *(bufferp++) = pixelsp->Alpha();
                pixelsp++;
            }
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_S,GL_REPEAT);
            glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_WRAP_T,GL_REPEAT);
            glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_MODULATE);
            glEnable(GL_BLEND);
            glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
            glTexImage2D(GL_TEXTURE_2D, 0, 4, image->Width(), image->Height(), 0, GL_RGBA, GL_FLOAT, buffer);
            delete [] buffer;
        }

        // Select texture
        glBindTexture(GL_TEXTURE_2D, material->texture_index);
        glEnable(GL_TEXTURE_2D);
    }
    else {
        glDisable(GL_TEXTURE_2D);
    }

    //    // Enable blending for transparent surfaces
    //    if (opacity < 1) {
    //        glDepthMask(false);
    //        glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    //        glEnable(GL_BLEND);
    //    }
    //    else {
    //        glDisable(GL_BLEND);
    //        glBlendFunc(GL_ONE, GL_ZERO);
    //        glDepthMask(true);
    //    }
}



void LoadCamera(R3Camera *camera)
{
    // Set projection transformation
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(2*180.0*camera->yfov/M_PI, (GLdouble) GLUTwindow_width /(GLdouble) GLUTwindow_height, 0.01, 10000);

    // free view (default view if no toggle)
    if (camera_view == 1)
    {
        R3Vector t = -(camera->towards);
        R3Vector& u = camera->up;
        R3Vector& r = camera->right;
        GLdouble camera_matrix[16] = { r[0], u[0], t[0], 0, r[1], u[1], t[1], 0, r[2], u[2], t[2], 0, 0, 0, 0, 1 };
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMultMatrixd(camera_matrix);
        glTranslated(-(camera->eye[0]), -(camera->eye[1]), -(camera->eye[2]));

        R3Aircraft * player_aircraft = scene->aircrafts[0];
        R3Vector displacement_vec(-3, 0, 0.4);
        displacement_vec.Transform(player_aircraft->T);
        R3Vector camera_position = player_aircraft->Modeling_To_World(R3Vector(0, 0, 0)); // centroid of aircraft
        camera_position += displacement_vec;

        R3Vector towards (1, 0, 0); // note -1 for real! below
        towards.Transform(player_aircraft->T);
        towards += camera_position;

        R3Vector up (0, 0, 1);
        up.Transform(player_aircraft->T);
        up += camera_position;

        R3Vector right (0, -1, 0);
        right.Transform(player_aircraft->T);
        right += camera_position;

        /// ----

        glPointSize(5);
        glBegin(GL_POINTS);
        glColor3d(1, 0, 0);
        glVertex3d(camera_position[0], camera_position[1], camera_position[2]);
        glEnd();

        current_camera_position.SetX(camera_position[0]);
        current_camera_position.SetY(camera_position[1]);
        current_camera_position.SetZ(camera_position[2]);

        //    glLineWidth(3);
        //    glBegin(GL_LINES);

        //    R3Vector towards = player_aircraft->Modeling_To_World(R3Vector(1, 0, 0)  + displacement_vec);
        //    R3Vector up = player_aircraft->Modeling_To_World(R3Vector(0, 0, 1) + displacement_vec);
        //    R3Vector right = player_aircraft->Modeling_To_World(R3Vector(0, -1, 0) + displacement_vec); // -y
        // draw x in RED
        //    glColor3d(1, 0, 0);
        //    glVertex3f(camera_position[0], camera_position[1], camera_position[2]);
        //    glVertex3f(towards.X(), towards.Y(), towards.Z());
        //
        //    // draw y in GREEN
        //    glColor3d(0, 1, 0);
        //    glVertex3f(camera_position[0], camera_position[1], camera_position[2]);
        //    glVertex3f(up.X(), up.Y(), up.Z());
        //
        //    // draw z in BLUE
        //    glColor3d(0, 0, 1);
        //    glVertex3f(camera_position[0], camera_position[1], camera_position[2]);
        //    glVertex3f(right.X(), right.Y(), right.Z());

        //    glEnd();
    }

    // 3rd person pov (above the plane looking forward)
    else if (camera_view == 2)
    {
        R3Aircraft * player_aircraft = scene->aircrafts[0];
        R3Vector displacement_vec(-3, 0, 0.4);
        displacement_vec.Transform(player_aircraft->T);
        R3Vector camera_position = player_aircraft->Modeling_To_World(R3Vector(0, 0, 0)); // centroid of aircraft
        camera_position += displacement_vec;


        R3Vector t (-1, 0, 0);
        t.Transform(player_aircraft->T);
        R3Vector u (0, 0, 1);
        u.Transform(player_aircraft->T);
        R3Vector r (0, -1, 0);
        r.Transform(player_aircraft->T);

        assert(t.IsNormalized());
        assert(u.IsNormalized());
        assert(r.IsNormalized());

        //    t.Normalize();
        //    u.Normalize();
        //    r.Normalize();

        GLdouble camera_matrix[16] = { r[0], u[0], t[0], 0, r[1], u[1], t[1], 0, r[2], u[2], t[2], 0, 0, 0, 0, 1 };
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMultMatrixd(camera_matrix);
        glTranslated(-(camera_position[0]), -(camera_position[1]), -(camera_position[2]));

        current_camera_position.SetX(camera_position[0]);
        current_camera_position.SetY(camera_position[1]);
        current_camera_position.SetZ(camera_position[2]);
    }

    else if (camera_view == 3)
    {
        R3Aircraft * player_aircraft = scene->aircrafts[0];
        R3Vector displacement_vec(2.0, 0.0, 0.5);
        displacement_vec.Transform(player_aircraft->T);
        R3Vector camera_position = player_aircraft->Modeling_To_World(R3Vector(0, 0, 0)); // centroid of aircraft
        camera_position += displacement_vec;


        // Kyle added a bit of height so you can see when you shoot bullets
        R3Vector t (-1, 0, 0);
        t.Transform(player_aircraft->T);
        R3Vector u (0, 0, 1);
        u.Transform(player_aircraft->T);
        R3Vector r (0, -1, 0);
        r.Transform(player_aircraft->T);

        assert(t.IsNormalized());
        assert(u.IsNormalized());
        assert(r.IsNormalized());

        GLdouble camera_matrix[16] = { r[0], u[0], t[0], 0, r[1], u[1], t[1], 0, r[2], u[2], t[2], 0, 0, 0, 0, 1 };
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMultMatrixd(camera_matrix);
        glTranslated(-(camera_position[0]), -(camera_position[1]), -(camera_position[2]));

        current_camera_position.SetX(camera_position[0]);
        current_camera_position.SetY(camera_position[1]);
        current_camera_position.SetZ(camera_position[2]);
    }

    // third person, static angle
    else if (camera_view == 4)
    {
        R3Aircraft * player_aircraft = scene->aircrafts[0];
        double backwards = -1*player_aircraft->velocity.Length()/4 + -5;

        R3Vector camera_position = player_aircraft->Modeling_To_World(R3Vector(0, 0, 0)); // centroid of aircraft

        R3Vector x_direction = R3Vector(1, 0, 0);
        x_direction.Transform(player_aircraft->T);

        R3Plane xy_plane(R3Vector(0, 0, 1), 0);
        x_direction.Project(xy_plane);
        x_direction.Normalize();

        camera_position += backwards * x_direction;
        camera_position += R3Vector(0, 0, 1);

        R3Vector t(-1, 0, 0);
        t.Transform(player_aircraft->T);
        t.Normalize();
        R3Vector u (0, 0, 1);
        R3Vector r = -t;
        r.Cross(u);
        r.Normalize();

        assert(!t.IsZero());
        assert(!u.IsZero());
        assert(!r.IsZero());

        assert(t.IsNormalized());
        assert(u.IsNormalized());
        assert(r.IsNormalized());

        GLdouble camera_matrix[16] = { r[0], u[0], t[0], 0, r[1], u[1], t[1], 0, r[2], u[2], t[2], 0, 0, 0, 0, 1 };
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMultMatrixd(camera_matrix);
        glTranslated(-(camera_position[0]), -(camera_position[1]), -(camera_position[2]));

        current_camera_position.SetX(camera_position[0]);
        current_camera_position.SetY(camera_position[1]);
        current_camera_position.SetZ(camera_position[2]);
    }

    // 3rd person view (looking backwards)
    else if (camera_view == 5)
    {
        R3Aircraft * player_aircraft = scene->aircrafts[0];
        R3Vector displacement_vec(5, 0, 0.4);
        displacement_vec.Transform(player_aircraft->T);
        R3Vector camera_position = player_aircraft->Modeling_To_World(R3Vector(0, 0, 0)); // centroid of aircraft
        camera_position += displacement_vec;


        R3Vector t (1, 0, 0);
        t.Transform(player_aircraft->T);
        R3Vector u (0, 0, 1);
        u.Transform(player_aircraft->T);
        R3Vector r (0, -1, 0);
        r.Transform(player_aircraft->T);

        assert(t.IsNormalized());
        assert(u.IsNormalized());
        assert(r.IsNormalized());

        //    t.Normalize();
        //    u.Normalize();
        //    r.Normalize();

        GLdouble camera_matrix[16] = { r[0], u[0], t[0], 0, r[1], u[1], t[1], 0, r[2], u[2], t[2], 0, 0, 0, 0, 1 };
        glMatrixMode(GL_MODELVIEW);
        glLoadIdentity();
        glMultMatrixd(camera_matrix);
        glTranslated(-(camera_position[0]), -(camera_position[1]), -(camera_position[2]));

        current_camera_position.SetX(camera_position[0]);
        current_camera_position.SetY(camera_position[1]);
        current_camera_position.SetZ(camera_position[2]);
    }





    else {
        fprintf(stderr, "Invalid view");
        exit(1);
    }
}



void LoadLights(R3Scene *scene)
{
    GLfloat buffer[4];

    // Load ambient light
    static GLfloat ambient[4];
    ambient[0] = scene->ambient[0];
    ambient[1] = scene->ambient[1];
    ambient[2] = scene->ambient[2];
    ambient[3] = 1;
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, ambient);

    // Load scene lights
    for (int i = 0; i < (int) scene->lights.size(); i++) {
        R3Light *light = scene->lights[i];
        int index = GL_LIGHT0 + i;

        // Temporarily disable light
        glDisable(index);

        // Load color
        buffer[0] = light->color[0];
        buffer[1] = light->color[1];
        buffer[2] = light->color[2];
        buffer[3] = 1.0;
        glLightfv(index, GL_DIFFUSE, buffer);
        glLightfv(index, GL_SPECULAR, buffer);

        // Load attenuation with distance
        buffer[0] = light->constant_attenuation;
        buffer[1] = light->linear_attenuation;
        buffer[2] = light->quadratic_attenuation;
        glLightf(index, GL_CONSTANT_ATTENUATION, buffer[0]);
        glLightf(index, GL_LINEAR_ATTENUATION, buffer[1]);
        glLightf(index, GL_QUADRATIC_ATTENUATION, buffer[2]);

        // Load spot light behavior
        buffer[0] = 180.0 * light->angle_cutoff / M_PI;
        buffer[1] = light->angle_attenuation;
        glLightf(index, GL_SPOT_CUTOFF, buffer[0]);
        glLightf(index, GL_SPOT_EXPONENT, buffer[1]);

        // Load positions/directions
        if (light->type == R3_DIRECTIONAL_LIGHT) {
            // Load direction
            buffer[0] = -(light->direction.X());
            buffer[1] = -(light->direction.Y());
            buffer[2] = -(light->direction.Z());
            buffer[3] = 0.0;
            glLightfv(index, GL_POSITION, buffer);
        }
        else if (light->type == R3_POINT_LIGHT) {
            // Load position
            buffer[0] = light->position.X();
            buffer[1] = light->position.Y();
            buffer[2] = light->position.Z();
            buffer[3] = 1.0;
            glLightfv(index, GL_POSITION, buffer);
        }
        else if (light->type == R3_SPOT_LIGHT) {
            // Load position
            buffer[0] = light->position.X();
            buffer[1] = light->position.Y();
            buffer[2] = light->position.Z();
            buffer[3] = 1.0;
            glLightfv(index, GL_POSITION, buffer);

            // Load direction
            buffer[0] = light->direction.X();
            buffer[1] = light->direction.Y();
            buffer[2] = light->direction.Z();
            buffer[3] = 1.0;
            glLightfv(index, GL_SPOT_DIRECTION, buffer);
        }
        else if (light->type == R3_AREA_LIGHT) {
            // Load position
            buffer[0] = light->position.X();
            buffer[1] = light->position.Y();
            buffer[2] = light->position.Z();
            buffer[3] = 1.0;
            glLightfv(index, GL_POSITION, buffer);

            // Load direction
            buffer[0] = light->direction.X();
            buffer[1] = light->direction.Y();
            buffer[2] = light->direction.Z();
            buffer[3] = 1.0;
            glLightfv(index, GL_SPOT_DIRECTION, buffer);
        }
        else {
            fprintf(stderr, "Unrecognized light type: %d\n", light->type);
            return;
        }

        // Enable light
        glEnable(index);
    }
}


void DrawGround(R3Scene *scene)
{
    const double CLIFF_DROPOFF_THRESHOLD = 150;
    const double SNOW_HEIGHT_THRESHOLD = 200;
    const double DIRT_HEIGHT_THRESHOLD = 100;

    // under assumption that scene only has one node: the ground mesh
    assert(scene->root->children.size() > 0);
    R3Node *ground_node = scene->root->children[0];
    assert(ground_node != NULL);
    assert(ground_node->shape->type == R3_MESH_SHAPE);
    R3Mesh *mesh = ground_node->shape->mesh;

    // Push transformation onto stack
    glPushMatrix();
    LoadMatrix(&ground_node->transformation);


    // TODO don't do this every frame... stupid.
    double max_height = -100000;
    double min_height = 100000;
    for (int f = 0; f < mesh->NFaces(); f++)
    {
        R3MeshFace *face = mesh->faces[f];
        vector<R3MeshVertex *> face_vertices = face->vertices;
        for (int v1 = 0; v1 < face_vertices.size(); v1++) {
            double height = face_vertices[v1]->position.Z();
            if (height > max_height) max_height = height;
            if (height < min_height) min_height = height;
        }
    }

    //    cout << "min: " << min_height << endl;
    //    cout << "max: " << max_height << endl;

    for (int f = 0; f < mesh->NFaces(); f++)
    {
        // Draw each face separately, so that we can texture map on each face
        R3MeshFace *face = mesh->faces[f];
        vector<R3MeshVertex *> face_vertices = face->vertices;
        assert(face_vertices.size() == 3); // generated map should be a triangular mesh

        // map textures based on max_dist_between_vertices and average_height for a face
        double max_dist_between_vertices = 0;
        double average_height = 0;
        for (int v1 = 0; v1 < face_vertices.size(); v1++) {
            average_height += face_vertices[v1]->position.Z();
            for (int v2 = v1 + 1; v2 < face_vertices.size(); v2++) {
                double dist = (face_vertices[v1]->position - face_vertices[v2]->position).Length();
                if (dist > max_dist_between_vertices)
                    max_dist_between_vertices = dist;
            }
        }
        average_height /= 3.0;

        // 0 -> 1 is minheight to maxheight
        // interpolate how far we are between them

        float dirt_weight = (average_height - min_height) / (max_height-min_height);
        //        dirt_weight -= .6;
        //        if (dirt_weight < 0) dirt_weight = 0;
        //        cout << dirt_weight << endl;
        dirt_weight = 1-dirt_weight;

        //        dirt_weight = 1;

        //        glEnable(GL_BLEND);
        //        glBlendFunc (GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
        //        glDisable(GL_LIGHTING);

        if (face->vertices.size() == 3)
        {
            LoadMaterial(texture_materials[DIRT]);
            //            glColor4f(1.0f, 1.0f, 1.0f, dirt_weight);
            glColor4f(dirt_weight, dirt_weight, dirt_weight, 1);

            glBegin(GL_POLYGON);

            // vertex 1
            R3MeshVertex *vertex = face->vertices[0];
            const R3Point& p1 = vertex->position;
            glTexCoord2f(0,0);
            glVertex3d(p1[0], p1[1], p1[2]);

            // vertex 2
            vertex = face->vertices[1];
            const R3Point& p2 = vertex->position;
            glTexCoord2f(0,1);
            glVertex3d(p2[0], p2[1], p2[2]);// vertex 1

            // vertex 3
            vertex = face->vertices[2];
            const R3Point& p3 = vertex->position;
            glTexCoord2f(1,1);
            glVertex3d(p3[0], p3[1], p3[2]);
            glEnd();
        }

        //        if (face->vertices.size() == 3)
        //        {
        //            cout << "running here" << endl;
        //            LoadMaterial(texture_materials[DIRT]);
        //            glColor4f(1.0f, 1.0f, 1.0f, 1.0f-dirt_weight);
        //
        //            glBegin(GL_POLYGON);
        //
        //            // vertex 1
        //            R3MeshVertex *vertex = face->vertices[0];
        //            const R3Point& p1 = vertex->position;
        //            glTexCoord2f(0,0);
        //            glVertex3d(p1[0], p1[1], p1[2]);
        //
        //            // vertex 2
        //            vertex = face->vertices[1];
        //            const R3Point& p2 = vertex->position;
        //            glTexCoord2f(0,1);
        //            glVertex3d(p2[0], p2[1], p2[2]);// vertex 1
        //
        //            // vertex 3
        //            vertex = face->vertices[2];
        //            const R3Point& p3 = vertex->position;
        //            glTexCoord2f(1,1);
        //            glVertex3d(p3[0], p3[1], p3[2]);
        //        }

        //        glDisable(GL_BLEND);
        glEnable(GL_LIGHTING);
        glEnd();
    }

    // restore previous transformation
    glPopMatrix();
}


void DrawNode(R3Scene *scene, R3Node *node)
{
    // Push transformation onto stack
    glPushMatrix();
    LoadMatrix(&node->transformation);

    // Load material
    if (node->material) LoadMaterial(node->material);

    // Draw shape
    if (node->shape) DrawShape(node->shape);

    // Draw children nodes
    for (int i = 0; i < (int) node->children.size(); i++)
        DrawNode(scene, node->children[i]);

    // Restore previous transformation
    glPopMatrix();

    // Show bounding box
    if (show_bboxes) {
        GLboolean lighting = glIsEnabled(GL_LIGHTING);
        glDisable(GL_LIGHTING);
        node->bbox.Outline();
        if (lighting) glEnable(GL_LIGHTING);
    }
}



void DrawLights(R3Scene *scene)
{
    // Check if should draw lights
    if (!show_lights) return;

    // Setup
    GLboolean lighting = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);

    // Draw all lights
    double radius = scene->bbox.DiagonalRadius();
    for (int i = 0; i < scene->NLights(); i++) {
        R3Light *light = scene->Light(i);
        glColor3d(light->color[0], light->color[1], light->color[2]);
        if (light->type == R3_DIRECTIONAL_LIGHT) {
            // Draw direction vector
            glLineWidth(5);
            glBegin(GL_LINES);
            R3Point centroid = scene->bbox.Centroid();
            R3Vector vector = radius * light->direction;
            glVertex3d(centroid[0] - vector[0], centroid[1] - vector[1], centroid[2] - vector[2]);
            glVertex3d(centroid[0] - 1.25*vector[0], centroid[1] - 1.25*vector[1], centroid[2] - 1.25*vector[2]);
            glEnd();
            glLineWidth(1);
        }
        else if (light->type == R3_POINT_LIGHT) {
            // Draw sphere at point light position
            R3Sphere(light->position, 0.1 * radius).Draw();
        }
        else if (light->type == R3_SPOT_LIGHT) {
            // Draw sphere at point light position and line indicating direction
            R3Sphere(light->position, 0.1 * radius).Draw();

            // Draw direction vector
            glLineWidth(5);
            glBegin(GL_LINES);
            R3Vector vector = radius * light->direction;
            glVertex3d(light->position[0], light->position[1], light->position[2]);
            glVertex3d(light->position[0] + 0.25*vector[0], light->position[1] + 0.25*vector[1], light->position[2] + 0.25*vector[2]);
            glEnd();
            glLineWidth(1);
        }
        else if (light->type == R3_AREA_LIGHT) {
            // Draw circular area
            R3Vector v1, v2;
            double r = light->radius;
            R3Point p = light->position;
            int dim = light->direction.MinDimension();
            if (dim == 0) { v1 = light->direction % R3posx_vector; v1.Normalize(); v2 = light->direction % v1; }
            else if (dim == 1) { v1 = light->direction % R3posy_vector; v1.Normalize(); v2 = light->direction % v1; }
            else { v1 = light->direction % R3posz_vector; v1.Normalize(); v2 = light->direction % v1; }
            glBegin(GL_POLYGON);
            glVertex3d(p[0] +  1.00*r*v1[0] +  0.00*r*v2[0], p[1] +  1.00*r*v1[1] +  0.00*r*v2[1], p[2] +  1.00*r*v1[2] +  0.00*r*v2[2]);
            glVertex3d(p[0] +  0.71*r*v1[0] +  0.71*r*v2[0], p[1] +  0.71*r*v1[1] +  0.71*r*v2[1], p[2] +  0.71*r*v1[2] +  0.71*r*v2[2]);
            glVertex3d(p[0] +  0.00*r*v1[0] +  1.00*r*v2[0], p[1] +  0.00*r*v1[1] +  1.00*r*v2[1], p[2] +  0.00*r*v1[2] +  1.00*r*v2[2]);
            glVertex3d(p[0] + -0.71*r*v1[0] +  0.71*r*v2[0], p[1] + -0.71*r*v1[1] +  0.71*r*v2[1], p[2] + -0.71*r*v1[2] +  0.71*r*v2[2]);
            glVertex3d(p[0] + -1.00*r*v1[0] +  0.00*r*v2[0], p[1] + -1.00*r*v1[1] +  0.00*r*v2[1], p[2] + -1.00*r*v1[2] +  0.00*r*v2[2]);
            glVertex3d(p[0] + -0.71*r*v1[0] + -0.71*r*v2[0], p[1] + -0.71*r*v1[1] + -0.71*r*v2[1], p[2] + -0.71*r*v1[2] + -0.71*r*v2[2]);
            glVertex3d(p[0] +  0.00*r*v1[0] + -1.00*r*v2[0], p[1] +  0.00*r*v1[1] + -1.00*r*v2[1], p[2] +  0.00*r*v1[2] + -1.00*r*v2[2]);
            glVertex3d(p[0] +  0.71*r*v1[0] + -0.71*r*v2[0], p[1] +  0.71*r*v1[1] + -0.71*r*v2[1], p[2] +  0.71*r*v1[2] + -0.71*r*v2[2]);
            glEnd();
        }
        else {
            fprintf(stderr, "Unrecognized light type: %d\n", light->type);
            return;
        }
    }

    // Clean up
    if (lighting) glEnable(GL_LIGHTING);
}



void DrawCamera(R3Scene *scene)
{
    // Check if should draw lights
    if (!show_camera) return;

    // Setup
    GLboolean lighting = glIsEnabled(GL_LIGHTING);
    glDisable(GL_LIGHTING);
    glColor3d(1.0, 1.0, 1.0);
    glLineWidth(5);

    // Draw view frustum
    R3Camera& c = scene->camera;
    double radius = scene->bbox.DiagonalRadius();
    R3Point org = c.eye + c.towards * radius;
    R3Vector dx = c.right * radius * tan(c.xfov);
    R3Vector dy = c.up * radius * tan(c.yfov);
    R3Point ur = org + dx + dy;
    R3Point lr = org + dx - dy;
    R3Point ul = org - dx + dy;
    R3Point ll = org - dx - dy;
    glBegin(GL_LINE_LOOP);
    glVertex3d(ur[0], ur[1], ur[2]);
    glVertex3d(ul[0], ul[1], ul[2]);
    glVertex3d(ll[0], ll[1], ll[2]);
    glVertex3d(lr[0], lr[1], lr[2]);
    glVertex3d(ur[0], ur[1], ur[2]);
    glVertex3d(c.eye[0], c.eye[1], c.eye[2]);
    glVertex3d(lr[0], lr[1], lr[2]);
    glVertex3d(ll[0], ll[1], ll[2]);
    glVertex3d(c.eye[0], c.eye[1], c.eye[2]);
    glVertex3d(ul[0], ul[1], ul[2]);
    glEnd();

    // Clean up
    glLineWidth(1);
    if (lighting) glEnable(GL_LIGHTING);
}



void DrawScene(R3Scene *scene) 
{
    // Draw nodes recursively
    //  DrawNode(scene, scene->root);

    DrawGround(scene);



    //  TODO: delete later? also, note quite working...
    // Added by Jason: May 10. Shows bounding boxes of airplanes
    //  GLboolean lighting = glIsEnabled(GL_LIGHTING);
    //  glDisable(GL_LIGHTING);
    //  for (int i = 0; i < scene->NAircrafts(); i++)
    //  {
    //    R3Aircraft *aircraft = scene->Aircraft(i);
    //
    //    // Push transformation onto stack
    //    glPushMatrix();
    //    LoadMatrix(&aircraft->T);
    //
    //    // Load material
    //    if (aircraft->material) LoadMaterial(aircraft->material);
    //
    //    // Draw shape
    //    aircraft->mesh->bbox.Outline();
    //    if (aircraft->mesh) aircraft->mesh->Draw();
    //    else { fprintf(stderr, "problem drawing bounding aircraft bounding boxes!"); exit(1); }
    //  }
    //
    //  if (lighting) glEnable(GL_LIGHTING);

}


void DrawParticlesAndAircrafts(R3Scene *scene)
{
    // Get current time (in seconds) since start of execution
    double current_time = GetTime();
    static double previous_time = 0;


    static double time_lost_taking_videos = 0; // for switching back and forth
    // between recording and not
    // recording smoothly

    // program just started up?
    if (previous_time == 0) previous_time = current_time;

    // time passed since starting
    double delta_time = current_time - previous_time;

    // XXX: Note that when I created the videos, I commented out lines 563-570 and uncommented line 571

    if (save_video) { // in video mode, the time that passes only depends on the frame rate ...
        delta_time = VIDEO_FRAME_DELAY;
        // ... but we need to keep track how much time we gained and lost so that we can arbitrarily switch back and forth ...
        time_lost_taking_videos += (current_time - previous_time) - VIDEO_FRAME_DELAY;
    } else { // real time simulation
        delta_time = current_time - previous_time;
    }

    //  double delta_time = 1 / 200.0;


    // Update particles
    UpdateParticles(scene, current_time - time_lost_taking_videos, delta_time, integration_type);
    UpdateAircrafts(scene, current_time - time_lost_taking_videos, delta_time, integration_type);

    //  // Generate new particles
    GenerateParticles(scene, current_time - time_lost_taking_videos, delta_time);

    // Render particles
    if (show_particles) RenderParticles(scene, current_time - time_lost_taking_videos, delta_time);
    RenderAircrafts(scene, current_time - time_lost_taking_videos, delta_time);


    // Remember previous time
    previous_time = current_time;
}

//
//void DrawParticleSources(R3Scene *scene)
//{
//  // Check if should draw particle sources
//  if (!show_particle_sources_and_sinks) return;
//
//  // Setup
//  GLboolean lighting = glIsEnabled(GL_LIGHTING);
//  glEnable(GL_LIGHTING);
//
//  // Define source material
//  static R3Material source_material;
//  if (source_material.id != 33) {
//    source_material.ka.Reset(0.2,0.2,0.2,1);
//    source_material.kd.Reset(0,1,0,1);
//    source_material.ks.Reset(0,1,0,1);
//    source_material.kt.Reset(0,0,0,1);
//    source_material.emission.Reset(0,0,0,1);
//    source_material.shininess = 1;
//    source_material.indexofrefraction = 1;
//    source_material.texture = NULL;
//    source_material.texture_index = -1;
//    source_material.id = 33;
//  }
//
//  // Draw all particle sources
//  glEnable(GL_LIGHTING);
//  LoadMaterial(&source_material);
//  for (int i = 0; i < scene->NParticleSources(); i++) {
//    R3ParticleSource *source = scene->ParticleSource(i);
//    DrawShape(source->shape);
//  }
//
//  // TODO: delete later
//  for (int i = 0; i < scene->NAircrafts(); i++)
//  {
//    vector<R3ParticleSource *> sources = scene->Aircraft(i)->sources;
//    glEnable(GL_LIGHTING);
//    LoadMaterial(&source_material);
//    for (int j = 0; j < sources.size(); j++) {
//      R3ParticleSource *source = sources[j];
//      DrawShape(source->shape);
//    }
//  }
//
//  // Clean up
//  if (!lighting) glDisable(GL_LIGHTING);
//}
//
//
//
//void DrawParticleSinks(R3Scene *scene)
//{
//  // Check if should draw particle sinks
//  if (!show_particle_sources_and_sinks) return;
//
//  // Setup
//  GLboolean lighting = glIsEnabled(GL_LIGHTING);
//  glEnable(GL_LIGHTING);
//
//  // Define sink material
//  static R3Material sink_material;
//  if (sink_material.id != 33) {
//    sink_material.ka.Reset(0.2,0.2,0.2,1);
//    sink_material.kd.Reset(1,0,0,1);
//    sink_material.ks.Reset(1,0,0,1);
//    sink_material.kt.Reset(0,0,0,1);
//    sink_material.emission.Reset(0,0,0,1);
//    sink_material.shininess = 1;
//    sink_material.indexofrefraction = 1;
//    sink_material.texture = NULL;
//    sink_material.texture_index = -1;
//    sink_material.id = 33;
//  }
//
//  // Draw all particle sinks
//  glEnable(GL_LIGHTING);
//  LoadMaterial(&sink_material);
//  for (int i = 0; i < scene->NParticleSinks(); i++) {
//    R3ParticleSink *sink = scene->ParticleSink(i);
//    DrawShape(sink->shape);
//  }
//
//  // Clean up
//  if (!lighting) glDisable(GL_LIGHTING);
//}
//
//
//
//void DrawParticleSprings(R3Scene *scene)
//{
//  // Check if should draw particle springs
//  if (!show_particle_springs) return;
//
//  // Setup
//  GLboolean lighting = glIsEnabled(GL_LIGHTING);
//  glDisable(GL_LIGHTING);
//
//  // Draw all particle sources
//  glColor3d(0.5, 0.5, 0.5);
//  glBegin(GL_LINES);
//  for (unsigned int i = 0; i < scene->particle_springs.size(); i++) {
//    R3ParticleSpring *spring = scene->particle_springs[i];
//    const R3Point& p0 = spring->particles[0]->position;
//    const R3Point& p1 = spring->particles[1]->position;
//    glVertex3d(p0[0], p0[1], p0[2]);
//    glVertex3d(p1[0], p1[1], p1[2]);
//  }
//  glEnd();
//
//  // Clean up
//  if (lighting) glEnable(GL_LIGHTING);
//}



////////////////////////////////////////////////////////////
// GLUT USER INTERFACE CODE
////////////////////////////////////////////////////////////

void GLUTMainLoop(void)
{
    // Run main loop -- never returns
    glutMainLoop();
}



void GLUTDrawText(const R3Point& p, const char *s)
{
    // Draw text string s and position p
    glRasterPos3d(p[0], p[1], p[2]);
    while (*s) glutBitmapCharacter(GLUT_BITMAP_9_BY_15, *(s++));
    //  while (*s) glutBitmapCharacter(GLUT_BITMAP_HELVETICA_12, *(s++));
}



void GLUTSaveImage(const char *filename)
{ 
    // Create image
    R2Image image(GLUTwindow_width, GLUTwindow_height);

    // Read screen into buffer
    GLfloat *pixels = new GLfloat [ 3 * GLUTwindow_width * GLUTwindow_height ];
    glReadPixels(0, 0, GLUTwindow_width, GLUTwindow_height, GL_RGB, GL_FLOAT, pixels);

    // Load pixels from frame buffer
    GLfloat *pixelsp = pixels;
    for (int j = 0; j < GLUTwindow_height; j++) {
        for (int i = 0; i < GLUTwindow_width; i++) {
            double r = (double) *(pixelsp++);
            double g = (double) *(pixelsp++);
            double b = (double) *(pixelsp++);
            R2Pixel pixel(r, g, b, 1);
            image.SetPixel(i, j, pixel);
        }
    }

    // Write image to file
    image.Write(filename);

    // Delete buffer
    delete [] pixels;
}



void GLUTStop(void)
{
    // Destroy window
    glutDestroyWindow(GLUTwindow);

    // Delete scene
    delete scene;

    // Exit
    exit(0);
}



void GLUTIdle(void)
{
    // Set current window
    if ( glutGetWindow() != GLUTwindow )
        glutSetWindow(GLUTwindow);

    // Redraw
    glutPostRedisplay();
}



void GLUTResize(int w, int h)
{
    // Resize window
    glViewport(0, 0, w, h);

    // Resize camera vertical field of view to match aspect ratio of viewport
    camera.yfov = atan(tan(camera.xfov) * (double) h/ (double) w);

    // Remember window size
    GLUTwindow_width = w;
    GLUTwindow_height = h;

    // Redraw
    glutPostRedisplay();
}

void GLUTRedraw(void)
{
    static bool is_texture_materials_initialized = false;
    if (!is_texture_materials_initialized)
    {
        is_texture_materials_initialized = true;

        // ROCK
        R3Material* rock = new R3Material();
        rock->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
        rock->kd = R3Rgb(0.2, 0.2, 0.2, 0.0);
        rock->ks = R3Rgb(0.2, 0.2, 0.2, 0.0);
        rock->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
        rock->emission = R3Rgb(0, 0, 0, 0);
        rock->shininess = 10;
        rock->indexofrefraction = 1;
        rock->id = 100; // sufficiently large s.t. won't be same as any of the scene file materials

        // Read texture image
        rock->texture = new R2Image();
        rock->texture->Read("../textures/rock.jpg");
        texture_materials[ROCK] = rock;

        // GRASS
        R3Material* grass = new R3Material();
        grass->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
        grass->kd = R3Rgb(0.2, 0.2, 0.2, 0.0);
        grass->ks = R3Rgb(0.2, 0.2, 0.2, 0.0);
        grass->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
        grass->emission = R3Rgb(0, 0, 0, 0);
        grass->shininess = 10;
        grass->indexofrefraction = 1;
        grass->id = 101; // sufficiently large s.t. won't be same as any of the scene file materials

        // Read texture image
        grass->texture = new R2Image();
        grass->texture->Read("../textures/grass.jpg");
        texture_materials[GRASS] = grass;

        // SNOW
        R3Material* snow = new R3Material();
        snow->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
        snow->kd = R3Rgb(0.2, 0.2, 0.2, 0.0);
        snow->ks = R3Rgb(0.2, 0.2, 0.2, 0.0);
        snow->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
        snow->emission = R3Rgb(0, 0, 0, 0);
        snow->shininess = 10;
        snow->indexofrefraction = 1;
        snow->id = 102; // sufficiently large s.t. won't be same as any of the scene file materials

        // Read texture image
        snow->texture = new R2Image();
        snow->texture->Read("../textures/snow.jpg");
        texture_materials[SNOW] = snow;

        // DIRT
        R3Material* dirt = new R3Material();
        dirt->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
        dirt->kd = R3Rgb(0.2, 0.2, 0.2, 0.0);
        dirt->ks = R3Rgb(0.2, 0.2, 0.2, 0.0);
        dirt->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
        dirt->emission = R3Rgb(0, 0, 0, 0);
        dirt->shininess = 10;
        dirt->indexofrefraction = 1;
        dirt->id = 103;

        // Read texture image
        dirt->texture = new R2Image();
        dirt->texture->Read("../textures/dirt.jpg");
        texture_materials[DIRT] = dirt; // sufficiently large s.t. won't be same as any of the scene file materials


        // DIRT
        R3Material* ocean = new R3Material();
        ocean->ka = R3Rgb(0.0, 0.0, 0.0, 0.0);
        ocean->kd = R3Rgb(0.2, 0.2, 0.2, 0.0);
        ocean->ks = R3Rgb(0.2, 0.2, 0.2, 0.0);
        ocean->kt = R3Rgb(0.0, 0.0, 0.0, 0.0);
        ocean->emission = R3Rgb(0, 0, 0, 0);
        ocean->shininess = 10;
        ocean->indexofrefraction = 1;
        ocean->id = 103;

        // Read texture image
        ocean->texture = new R2Image();
        ocean->texture->Read("../textures/ocean.jpg");
        texture_materials[OCEAN] = ocean; // sufficiently large s.t. won't be same as any of the scene file materials
    }

    // Initialize OpenGL drawing modes
    glEnable(GL_LIGHTING);
    //    glDisable(GL_BLEND);
    //    glBlendFunc(GL_ONE, GL_ZERO);

    // enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);


    glDepthMask(true);

    // Clear window
    R3Rgb background = scene->background;
    glClearColor(background[0], background[1], background[2], background[3]);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    // Load camera
    LoadCamera(&camera);

    // Load scene lights
    LoadLights(scene);

    // Draw scene camera
    DrawCamera(scene);

    // Draw scene lights
    DrawLights(scene);

    // Draw particles
    DrawParticlesAndAircrafts(scene);

    //  // Draw particle sources
    //  DrawParticleSources(scene);
    //
    //  // Draw particle sinks
    //  DrawParticleSinks(scene);
    //
    //  // Draw particle springs
    //  DrawParticleSprings(scene);

    drawSkybox(100);

    // Draw scene surfaces
    if (show_faces) {
        glEnable(GL_LIGHTING);
        DrawScene(scene);
    }

    // Draw scene edges
    if (show_edges) {
        glDisable(GL_LIGHTING);
        glColor3d(1 - background[0], 1 - background[1], 1 - background[2]);
        glPolygonMode(GL_FRONT_AND_BACK, GL_LINE);
        DrawScene(scene);
        glPolygonMode(GL_FRONT_AND_BACK, GL_FILL);
    }

    // draw skybox

    // draw 2d HUD (heads up display)
    // added by Kyle

    glDisable(GL_LIGHTING);
    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    glOrtho(0.0, GLUTwindow_width, GLUTwindow_height, 0.0, -1.0, 10.0);
    glMatrixMode(GL_MODELVIEW);
    //  glPushMatrix();    //    ----Not sure if I need this
    glLoadIdentity();
    glDisable(GL_CULL_FACE);
    glClear(GL_DEPTH_BUFFER_BIT);
    glBindTexture(GL_TEXTURE_2D, 0);


    // draw crosshair
    // code from https://www.opengl.org/discussion_boards/showthread.php/167955-drawing-a-smooth-circle
    if (camera_view == 2 || camera_view == 3) {
        glBegin(GL_LINE_LOOP);
        glColor3f(1, 1, 1);
        int circle_x = GLUTwindow_width/2;
        int circle_y = GLUTwindow_height/2 + 7;

        double radius = 10;
        //      if (camera_view == 3)
        //          radius = 20;

        for (int i = 0; i < 360; i++)
        {
            float degInRad = i*DEG2RAD;
            glVertex2f(cos(degInRad)*radius + circle_x,sin(degInRad)*radius + circle_y);
        }
        glEnd();
        glPointSize(2);
        glColor3f(1, 1, 1);
        glBegin(GL_POINTS);
        glVertex2f(circle_x, circle_y);
        glEnd();
    }

    //    glEnable(GL_BLEND);
    int map_width = 200;
    int map_height = map_width;

    // minimap coordinates:
    int left = GLUTwindow_width-map_width;
    int right = GLUTwindow_width;
    int top = 0;
    int bottom = map_height;

    // draw point at center of radar
    //     glEnable(GL_POINT_SMOOTH);
    //     glHint(GL_POINT_SMOOTH_HINT, GL_NICEST);

    glColor4f(0, 0, 1, 1);
    glPointSize(5);
    glBegin(GL_POLYGON);
    glVertex2f(left + map_width/2, top + map_height/2);
    glVertex2f(left + map_width/2 - 5, top + map_height/2 + 5);
    glVertex2f(left + map_width/2 + 5, top + map_height/2 + 5);
    glEnd();

    // calculate relative positions of enemies:
    // first, translate everything to plane's coordinate system
    // then, get rid of the most unimportant dimension
    // then scale
    // then draw

    // max units away a plane can be to be visible on minimap
    double max_dist = 5;

    glColor4f(1, 0, 0, 1);
    for (int i = 1; i < scene->NAircrafts(); i++)
    {
        R3Aircraft *player_aircraft = scene->Aircraft(0);
        R3Aircraft *AI_aircraft = scene->Aircraft(i);

        // get position of player and AI in world
        R3Vector AI_center_world(0, 0, 0);
        AI_center_world = AI_aircraft->Modeling_To_World(AI_center_world);

        R3Vector player_center_world(0, 0, 0);
        player_center_world = player_aircraft->Modeling_To_World(player_center_world);

        // subtract vectors to put AI into player's modeling coordinates
        R3Vector to_enemy_modeling = AI_center_world - player_center_world;


        // rotated AI based on player's rotation
        to_enemy_modeling.Transform(player_aircraft->T.Transpose()); // TODO: transpose?

        // eliminate z vector in this rotated coordinate system
        to_enemy_modeling.SetZ(0);

        // rescale for minimap size
        to_enemy_modeling /= max_dist;


        // figure out which way AI is pointing with respect to player aircraft's direction
        R3Vector AI_direction_modeling = AI_aircraft->velocity;
        AI_direction_modeling.Transform(AI_aircraft->T);
        AI_direction_modeling.Transform(player_aircraft->T.Transpose());

        AI_direction_modeling.SetZ(0);
        AI_direction_modeling.Normalize();

        R3Vector AI_direction_perp_modeling = AI_direction_modeling;
        AI_direction_perp_modeling.Cross(R3Vector(0, 0, 1));

        //       glBegin(GL_POINTS);
        glBegin(GL_POLYGON);
        glVertex2f(left + map_width/2 - to_enemy_modeling.Y(), top + map_height/2 - to_enemy_modeling.X());

        R3Vector back_center = -5 * AI_direction_modeling + to_enemy_modeling;
        R3Point back_left = (back_center - 5 * AI_direction_perp_modeling).Point();
        R3Point back_right = (back_center + 5 * AI_direction_perp_modeling).Point();

        //       glVertex2f(left + map_width/2 - back_center.Y(), top + map_height/2 - back_center.X());

        glVertex2f(left + map_width/2 - back_left.Y(), top + map_height/2 - back_left.X());
        glVertex2f(left + map_width/2 - back_right.Y(), top + map_height/2 - back_right.X());
        glEnd();
    }

    // draw Minimap background radar
    glColor4f(.5, .5, .5, .5);
    glBegin(GL_QUADS);
    glVertex2f(right, top);
    glVertex2f(right, bottom);
    glVertex2f(left, bottom);
    glVertex2f(left, top);
    glEnd();



    // draw thrust string
    glColor3f(1, 1, 1);
    int percentage_thrust = scene->Aircraft(0)->thrust_magnitude/scene->Aircraft(0)->max_thrust*100;
    char buffer[50];
    sprintf(buffer, "Thrust: %d%%", percentage_thrust);
    GLUTDrawText(R3Point(7, 15, 0), buffer);

    // draw velocity string
    glColor3f(1, 1, 1);
    double velocity = scene->Aircraft(0)->velocity.Length() * METERS_PER_UNIT;
    sprintf(buffer, "Velocity: %.2f m/s", velocity);
    GLUTDrawText(R3Point(7, 30, 0), buffer);

    // draw altitude string
    glColor3f(1, 1, 1);
    R3Vector altitude_vec = scene->Aircraft(0)->Modeling_To_World(R3Vector(0, 0, 0));
    double altitude = altitude_vec.Z();
    sprintf(buffer, "Altitude: %.2f m", altitude);
    GLUTDrawText(R3Point(7, 45, 0), buffer);

    // draw kills string
    glColor3f(1, 1, 1);
    sprintf(buffer, "Kills: %d", num_kills);
    GLUTDrawText(R3Point(7, GLUTwindow_height-40, 0), buffer);

    // draw deaths string
    glColor3f(1, 1, 1);
    sprintf(buffer, "Deaths: %d", num_deaths);
    GLUTDrawText(R3Point(7, GLUTwindow_height-15, 0), buffer);





    // Making sure we can render 3d again
    glMatrixMode(GL_PROJECTION);
    //  glPopMatrix();
    glMatrixMode(GL_MODELVIEW);
    glEnable(GL_LIGHTING);


    // Save image
    if (save_image) {
        char image_name[256];
        static int image_number = 1;
        for (;;) {
            sprintf(image_name, "image%d.jpg", image_number++);
            FILE *fp = fopen(image_name, "r");
            if (!fp) break;
            else fclose(fp);
        }
        GLUTSaveImage(image_name);
        printf("Saved %s\n", image_name);
        save_image = 0;
    }

    // Save video
    if (save_video) {
        char frame_name[512];
        static int next_frame = 0;
        static int num_frames_recorded = 0;
        for (;;) {
            sprintf(frame_name, "%sframe%04d.jpg", video_prefix, next_frame++);
            FILE *fp = fopen(frame_name, "r");
            if (!fp) break;
            else fclose(fp);
        }
        GLUTSaveImage(frame_name);
        if (next_frame % 100 == 1) {
            printf("Saved %s\n", frame_name);
        }
        if (num_frames_to_record == ++num_frames_recorded) {
            save_video = 0;
            printf("Recorded %d frames, stopping as instructed.\n", num_frames_recorded);
            quit = 1;
        }
    }

    // Quit here so that can save image before exit
    if (quit) {
        if (output_image_name) GLUTSaveImage(output_image_name);
        GLUTStop();
    }

    glutSwapBuffers();
}    

void drawSkybox(double D)
{
    float white[]={1,1,1,1};
    glColor3fv(white);
    glEnable(GL_TEXTURE_2D);
    glDisable(GL_LIGHTING);
    //      cout << "drawing" << endl;


    D = 5000;

    //    glBegin(GL_QUADS);
    //    glTexCoord2f(0,0); glVertex3f(-D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    //    glTexCoord2f(1,0); glVertex3f(+D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    //    glTexCoord2f(1,1); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    //    glTexCoord2f(0,1); glVertex3f(-D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    //    glEnd();

    /* Sides */
    glBindTexture(GL_TEXTURE_2D,skybox[SKY_DOWN]);
    glBegin(GL_QUADS);
    glTexCoord2f(0,0); glVertex3f(-D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,0); glVertex3f(+D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,1); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(0,1); glVertex3f(-D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    glEnd();
    glBindTexture(GL_TEXTURE_2D,skybox[SKY_FRONT]);
    glBegin(GL_QUADS);

    //    glTexCoord2f(0,1); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,0); glVertex3f(+D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,1); glVertex3f(+D+current_camera_position.X(),-D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(0,1); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(0,0); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    glEnd();
    glBindTexture(GL_TEXTURE_2D,skybox[SKY_UP]);
    glBegin(GL_QUADS);
    glTexCoord2f(0,1); glVertex3f(+D+current_camera_position.X(),-D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(0,0); glVertex3f(-D+current_camera_position.X(),-D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(1,0); glVertex3f(-D+current_camera_position.X(),+D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(1,1); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),+D+current_camera_position.Z());
    glEnd();
    glBindTexture(GL_TEXTURE_2D,skybox[SKY_BACK]);
    glBegin(GL_QUADS);
    glTexCoord2f(0,1); glVertex3f(-D+current_camera_position.X(),-D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(0,0); glVertex3f(-D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,0); glVertex3f(-D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,1); glVertex3f(-D+current_camera_position.X(),+D+current_camera_position.Y(),+D+current_camera_position.Z());
    glEnd();

    /* Left and Right */
    glBindTexture(GL_TEXTURE_2D,skybox[SKY_RIGHT]);
    glBegin(GL_QUADS);
    glTexCoord2f(0,0); glVertex3f(-D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,0); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,1); glVertex3f(+D+current_camera_position.X(),+D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(0,1); glVertex3f(-D+current_camera_position.X(),+D+current_camera_position.Y(),+D+current_camera_position.Z());
    glEnd();

    glBindTexture(GL_TEXTURE_2D,skybox[SKY_LEFT]);
    glBegin(GL_QUADS);
    glTexCoord2f(0,0); glVertex3f(+D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,0); glVertex3f(-D+current_camera_position.X(),-D+current_camera_position.Y(),-D+current_camera_position.Z());
    glTexCoord2f(1,1); glVertex3f(-D+current_camera_position.X(),-D+current_camera_position.Y(),+D+current_camera_position.Z());
    glTexCoord2f(0,1); glVertex3f(+D+current_camera_position.X(),-D+current_camera_position.Y(),+D+current_camera_position.Z());
    glEnd();

    glDisable(GL_TEXTURE_2D);
    glEnable(GL_LIGHTING);
}



void GLUTMotion(int x, int y)
{
    // Invert y coordinate
    y = GLUTwindow_height - y;

    // Compute mouse movement
    int dx = x - GLUTmouse[0];
    int dy = y - GLUTmouse[1];

    // Process mouse motion event
    if ((dx != 0) || (dy != 0)) {
        R3Point scene_center = scene->bbox.Centroid();
        if ((GLUTbutton[0] && (GLUTmodifiers & GLUT_ACTIVE_SHIFT)) || GLUTbutton[1]) {
            // Scale world
            double factor = (double) dx / (double) GLUTwindow_width;
            factor += (double) dy / (double) GLUTwindow_height;
            factor = exp(2.0 * factor);
            factor = (factor - 1.0) / factor;
            R3Vector translation = (scene_center - camera.eye) * factor;
            camera.eye += translation;
            glutPostRedisplay();
        }
        else if (GLUTbutton[0] && (GLUTmodifiers & GLUT_ACTIVE_CTRL)) {
            // Translate world
            double length = R3Distance(scene_center, camera.eye) * tan(camera.yfov);
            double vx = length * (double) dx / (double) GLUTwindow_width;
            double vy = length * (double) dy / (double) GLUTwindow_height;
            R3Vector translation = -((camera.right * vx) + (camera.up * vy));
            camera.eye += translation;
            glutPostRedisplay();
        }
        else if (GLUTbutton[0]) {
            // Rotate world
            double vx = (double) dx / (double) GLUTwindow_width;
            double vy = (double) dy / (double) GLUTwindow_height;
            double theta = 4.0 * (fabs(vx) + fabs(vy));
            R3Vector vector = (camera.right * vx) + (camera.up * vy);
            R3Vector rotation_axis = camera.towards % vector;
            rotation_axis.Normalize();
            camera.eye.Rotate(R3Line(scene_center, rotation_axis), theta);
            camera.towards.Rotate(rotation_axis, theta);
            camera.up.Rotate(rotation_axis, theta);
            camera.right = camera.towards % camera.up;
            camera.up = camera.right % camera.towards;
            camera.towards.Normalize();
            camera.up.Normalize();
            camera.right.Normalize();
            glutPostRedisplay();
        }
    }

    // Remember mouse position
    GLUTmouse[0] = x;
    GLUTmouse[1] = y;
}



void GLUTMouse(int button, int state, int x, int y)
{
    // Invert y coordinate
    y = GLUTwindow_height - y;

    // Process mouse button event
    if (state == GLUT_DOWN) {
        if (button == GLUT_LEFT_BUTTON) {
        }
        else if (button == GLUT_MIDDLE_BUTTON) {
        }
        else if (button == GLUT_RIGHT_BUTTON) {
        }
    }

    // Remember button state
    int b = (button == GLUT_LEFT_BUTTON) ? 0 : ((button == GLUT_MIDDLE_BUTTON) ? 1 : 2);
    GLUTbutton[b] = (state == GLUT_DOWN) ? 1 : 0;

    // Remember modifiers
    GLUTmodifiers = glutGetModifiers();

    // Remember mouse position
    GLUTmouse[0] = x;
    GLUTmouse[1] = y;

    // Redraw
    glutPostRedisplay();
}



void GLUTSpecial(int key, int x, int y)
{
    // Invert y coordinate
    y = GLUTwindow_height - y;

    // Process keyboard button event
    switch (key) {
    case GLUT_KEY_F1:
        save_image = 1;
        break;
    case GLUT_KEY_F2:
        save_video = save_video ^ 1;
        break;
    }

    // Remember mouse position
    GLUTmouse[0] = x;
    GLUTmouse[1] = y;

    // Remember modifiers
    GLUTmodifiers = glutGetModifiers();

    // Redraw
    glutPostRedisplay();
}


// Added by Kyle on 5/4/14: this is how OpenGL handles keyboard "up", when a user releases a key.
// Video games use this to have smooth controls when a user is holding a key.
void GLUTKeyboardUp(unsigned char key, int x, int y)
{
    switch (key) {
    case 'W':
    case 'w':
        pitch_down = 0;
        break;

    case 'S':
    case 's':
        pitch_up = 0;
        break;

    case 'A':
    case 'a':
        roll_left = 0;
        break;

    case 'D':
    case 'd':
        roll_right = 0;
        break;

    case ' ':
        firing_bullets = 0;
        break;

    case 'J':
    case 'j':
        thrust_forward = 0;
        break;

    case 'K':
    case 'k':
        brake_backward = 0;
        break;
    }
}


// Modified by Kyle on 5/4/14: I set W to be pitch_down and S to be pitch_up, because this is how flight simulators are typically controlled.
void GLUTKeyboard(unsigned char key, int x, int y)
{
    // Invert y coordinate
    y = GLUTwindow_height - y;

    // Process keyboard button event
    switch (key) {
    case 'B':
    case 'b':
        show_bboxes = !show_bboxes;
        break;

    case 'C':
    case 'c':
        show_camera = !show_camera;
        break;

    case 'E':
    case 'e':
        show_edges = !show_edges;
        break;

    case 'F':
    case 'f':
        show_faces = !show_faces;
        break;

    case 'L':
    case 'l':
        show_lights = !show_lights;
        break;

    case 'P':
    case 'p':
        show_particles = !show_particles;
        break;

    case 'R':
    case 'r':
        show_particle_springs = !show_particle_springs;
        break;

    case 'O':
    case 'o':
        show_particle_sources_and_sinks = !show_particle_sources_and_sinks;
        break;

        // TODO: Add documentation
    case 'W':
    case 'w':
        pitch_down = 1;
        break;

    case 'S':
    case 's':
        pitch_up = 1;
        break;

    case 'A':
    case 'a':
        roll_left = 1;
        break;

    case 'D':
    case 'd':
        roll_right = 1;
        break;

    case 'J':
    case 'j':
        thrust_forward = 1;
        break;

    case 'K':
    case 'k':
        brake_backward = 1;
        break;

    case ' ':
        firing_bullets = 1;
        break;

    case 'H':
    case 'h':
        hard_mode = !hard_mode;
        break;

    case '1':
        camera_view = 1;
        break;

    case '2':
        camera_view = 2;
        break;

    case '3':
        camera_view = 3;
        break;

    case '4':
        camera_view = 4;
        break;

    case '5':
        camera_view = 5;
        break;


        //  case 'Q':
        //  case 'q':
    case 27: // ESCAPE
        quit = 1;
        break;

        //  case ' ': {
        //    printf("camera %g %g %g  %g %g %g  %g %g %g  %g  %g %g \n",
        //           camera.eye[0], camera.eye[1], camera.eye[2],
        //           camera.towards[0], camera.towards[1], camera.towards[2],
        //           camera.up[0], camera.up[1], camera.up[2],
        //           camera.xfov, camera.neardist, camera.fardist);
        //    break; }
    }

    // Remember mouse position
    GLUTmouse[0] = x;
    GLUTmouse[1] = y;

    // Remember modifiers
    GLUTmodifiers = glutGetModifiers();

    // Redraw
    glutPostRedisplay();
}



void GLUTCommand(int cmd)
{
    // Execute command
    switch (cmd) {
    case DISPLAY_PARTICLES_TOGGLE_COMMAND: show_particles = !show_particles; break;
    case DISPLAY_PARTICLE_SPRINGS_TOGGLE_COMMAND: show_particle_springs = !show_particle_springs; break;
    case DISPLAY_PARTICLE_SOURCES_AND_SINKS_TOGGLE_COMMAND: show_particle_sources_and_sinks = !show_particle_sources_and_sinks; break;
    case DISPLAY_FACE_TOGGLE_COMMAND: show_faces = !show_faces; break;
    case DISPLAY_EDGE_TOGGLE_COMMAND: show_edges = !show_edges; break;
    case DISPLAY_BBOXES_TOGGLE_COMMAND: show_bboxes = !show_bboxes; break;
    case DISPLAY_LIGHTS_TOGGLE_COMMAND: show_lights = !show_lights; break;
    case DISPLAY_CAMERA_TOGGLE_COMMAND: show_camera = !show_camera; break;
    case SAVE_IMAGE_COMMAND: save_image = 1; break;
    case SAVE_VIDEO_COMMAND: save_video = save_video ^ 1; break;
    case QUIT_COMMAND: quit = 1; break;
    }

    // Mark window for redraw
    glutPostRedisplay();
}



void GLUTCreateMenu(void)
{
    // Display sub-menu
    int display_menu = glutCreateMenu(GLUTCommand);
    glutAddMenuEntry("Particles (P)", DISPLAY_PARTICLES_TOGGLE_COMMAND);
    glutAddMenuEntry("Particle springs (R)", DISPLAY_PARTICLE_SPRINGS_TOGGLE_COMMAND);
    glutAddMenuEntry("Particle sources and sinks (S)", DISPLAY_PARTICLE_SOURCES_AND_SINKS_TOGGLE_COMMAND);
    glutAddMenuEntry("Faces (F)", DISPLAY_FACE_TOGGLE_COMMAND);
    glutAddMenuEntry("Edges (E)", DISPLAY_EDGE_TOGGLE_COMMAND);
    glutAddMenuEntry("Bounding boxes (B)", DISPLAY_BBOXES_TOGGLE_COMMAND);
    glutAddMenuEntry("Lights (L)", DISPLAY_LIGHTS_TOGGLE_COMMAND);
    glutAddMenuEntry("Camera (C)", DISPLAY_CAMERA_TOGGLE_COMMAND);

    // Main menu
    glutCreateMenu(GLUTCommand);
    glutAddSubMenu("Display", display_menu);
    glutAddMenuEntry("Save Image (F1)", SAVE_IMAGE_COMMAND);
    glutAddMenuEntry("Capture Video (F2)", SAVE_VIDEO_COMMAND);
    glutAddMenuEntry("Quit", QUIT_COMMAND);

    // Attach main menu to right mouse button
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}



void GLUTInit(int *argc, char **argv)
{
    // Open window
    glutInit(argc, argv);
    glutInitWindowPosition(0, 0);
    glutInitWindowSize(GLUTwindow_width, GLUTwindow_height);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH); // | GLUT_STENCIL
    GLUTwindow = glutCreateWindow("Sky battle");


    // try full screen
    //    glutFullScreen();


    // Initialize GLUT callback functions
    glutIdleFunc(GLUTIdle);
    glutReshapeFunc(GLUTResize);
    glutDisplayFunc(GLUTRedraw);
    glutKeyboardFunc(GLUTKeyboard);
    glutKeyboardUpFunc(GLUTKeyboardUp);
    glutSpecialFunc(GLUTSpecial);
    glutMouseFunc(GLUTMouse);
    glutMotionFunc(GLUTMotion);

    // Initialize graphics modes
    glEnable(GL_NORMALIZE);
    glEnable(GL_LIGHTING);
    glEnable(GL_DEPTH_TEST);
    glLightModeli(GL_LIGHT_MODEL_LOCAL_VIEWER, GL_TRUE);

    // Create menus
    GLUTCreateMenu();
}


// Kyle: Below code is for adding skybox
void fatal(const char* format , ...)
{
    va_list args;
    va_start(args,format);
    vfprintf(stderr,format,args);
    va_end(args);
    exit(1);
}

void errCheck(char* where)
{
    int err = glGetError();
    if (err) fprintf(stderr,"ERROR: %s [%s]\n",gluErrorString(err),where);
}

static void reverse(void* x,const int n)
{
    int k;
    char* ch = (char*)x;
    for (k=0;k<n/2;k++)
    {
        char tmp = ch[k];
        ch[k] = ch[n-1-k];
        ch[n-1-k] = tmp;
    }
}

unsigned int loadTexBMP(char* file)
{
    unsigned int   texture;    /* Texture name */
    FILE*          f;          /* File pointer */
    unsigned short magic;      /* Image magic */
    unsigned int   dx,dy,size; /* Image dimensions */
    unsigned short nbp,bpp;    /* Planes and bits per pixel */
    unsigned char* image;      /* Image data */
    unsigned int   k;          /* Counter */

    /*  Open file */
    f = fopen(file,"rb");
    if (!f) fatal("Cannot open file %s\n",file);
    /*  Check image magic */
    if (fread(&magic,2,1,f)!=1) fatal("Cannot read magic from %s\n",file);
    if (magic!=0x4D42 && magic!=0x424D) fatal("Image magic not BMP in %s\n",file);
    /*  Seek to and read header */
    if (fseek(f,16,SEEK_CUR) || fread(&dx ,4,1,f)!=1 || fread(&dy ,4,1,f)!=1 ||
            fread(&nbp,2,1,f)!=1 || fread(&bpp,2,1,f)!=1 || fread(&k,4,1,f)!=1)
        fatal("Cannot read header from %s\n",file);
    /*  Reverse bytes on big endian hardware (detected by backwards magic) */
    if (magic==0x424D) {
        reverse(&dx,4);
        reverse(&dy,4);
        reverse(&nbp,2);
        reverse(&bpp,2);
        reverse(&k,4);
    }
    /*  Check image parameters */
    if (dx<1 || dx>65536) fatal("%s image width out of range: %d\n",file,dx);
    if (dy<1 || dy>65536) fatal("%s image height out of range: %d\n",file,dy);
    if (nbp!=1)  fatal("%s bit planes is not 1: %d\n",file,nbp);
    if (bpp!=24) fatal("%s bits per pixel is not 24: %d\n",file,bpp);
    if (k!=0)    fatal("%s compressed files not supported\n",file);
#ifndef GL_VERSION_2_0
    /*  OpenGL 2.0 lifts the restriction that texture size must be a power of two */
    for (k=1;k<dx;k*=2);
    if (k!=dx) fatal("%s image width not a power of two: %d\n",file,dx);
    for (k=1;k<dy;k*=2);
    if (k!=dy) fatal("%s image height not a power of two: %d\n",file,dy);
#endif

    /*  Allocate image memory */
    size = 3*dx*dy;
    image = (unsigned char*) malloc(size);
    if (!image) fatal("Cannot allocate %d bytes of memory for image %s\n",size,file);
    /*  Seek to and read image */
    if (fseek(f,20,SEEK_CUR) || fread(image,size,1,f)!=1)
        fatal("Error reading data from image %s\n",file);
    fclose(f);
    /*  Reverse colors (BGR -> RGB) */
    for (k=0;k<size;k+=3) {
        unsigned char temp = image[k];
        image[k]   = image[k+2];
        image[k+2] = temp;
    }

    /*  Sanity check */
    errCheck("loadTexBMP");
    /*  Generate 2D texture */
    glGenTextures(1,&texture);
    glBindTexture(GL_TEXTURE_2D,texture);
    /*  Copy image */
    glTexImage2D(GL_TEXTURE_2D,0,3,dx,dy,0,GL_RGB,GL_UNSIGNED_BYTE,image);
    if (glGetError()) fatal("Error in glTexImage2D %s %dx%d\n",file,dx,dy);
    /*  Scale linearly when image size doesn't match */
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MAG_FILTER,GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D,GL_TEXTURE_MIN_FILTER,GL_LINEAR);

    /*  Free image memory */
    free(image);
    /*  Return texture name */
    //  cout << "returning texture" << endl;
    return texture;
}

void initSkyBox(int skybox_type)
{
    //    cout << "initializeing" << endl;

    // skyboxes taken from these sites:
    // http://www.redsorceress.com/skybox.html

    if (skybox_type == 1)
    {
        skybox[SKY_FRONT] = loadTexBMP("bmp/txStormydays_front.bmp");
        skybox[SKY_RIGHT] = loadTexBMP("bmp/txStormydays_right.bmp");
        skybox[SKY_LEFT] = loadTexBMP("bmp/txStormydays_left.bmp");
        skybox[SKY_BACK] = loadTexBMP("bmp/txStormydays_back.bmp");
        skybox[SKY_UP] = loadTexBMP("bmp/txStormydays_up.bmp");
        skybox[SKY_DOWN] = loadTexBMP("bmp/txStormydays_down.bmp");
    } else if (skybox_type == 2)
    {
        skybox[SKY_FRONT] = loadTexBMP("bmp/front.bmp");
        skybox[SKY_RIGHT] = loadTexBMP("bmp/right.bmp");
        skybox[SKY_LEFT] = loadTexBMP("bmp/left.bmp");
        skybox[SKY_BACK] = loadTexBMP("bmp/back.bmp");
        skybox[SKY_UP] = loadTexBMP("bmp/up.bmp");
        skybox[SKY_DOWN] = loadTexBMP("bmp/down.bmp");
    } else if (skybox_type == 0)
    {
        skybox[SKY_FRONT] = loadTexBMP("bmp/desert_back.bmp");
        skybox[SKY_RIGHT] = loadTexBMP("bmp/desert_right.bmp");
        skybox[SKY_LEFT] = loadTexBMP("bmp/desert_left.bmp");
        skybox[SKY_BACK] = loadTexBMP("bmp/desert_front.bmp");
        skybox[SKY_UP] = loadTexBMP("bmp/desert_top.bmp");
        skybox[SKY_DOWN] = loadTexBMP("bmp/down.bmp");
    }
}



////////////////////////////////////////////////////////////
// SCENE READING
////////////////////////////////////////////////////////////


R3Scene *
ReadScene(const char *filename)
{
    // Allocate scene
    R3Scene *scene = new R3Scene();
    if (!scene) {
        fprintf(stderr, "Unable to allocate scene\n");
        return NULL;
    }

    // Read file
    if (!scene->Read(filename)) {
        fprintf(stderr, "Unable to read scene from %s\n", filename);
        return NULL;
    }

    // Remember initial camera
    camera = scene->camera;

    // Return scene
    return scene;
}



////////////////////////////////////////////////////////////
// PROGRAM ARGUMENT PARSING
////////////////////////////////////////////////////////////

int 
ParseArgs(int argc, char **argv)
{
    // Innocent until proven guilty
    int print_usage = 0;

    // Parse arguments
    argc--; argv++;
    while (argc > 0) {
        if ((*argv)[0] == '-') {
            if (!strcmp(*argv, "-help")) { print_usage = 1; }
            else if (!strcmp(*argv, "-exit_immediately")) { quit = 1; }
            else if (!strcmp(*argv, "-output_image")) { argc--; argv++; output_image_name = *argv; }
            else if (!strcmp(*argv, "-video_prefix")) { argc--; argv++; video_prefix = *argv; }
            else if (!strcmp(*argv, "-euler")) integration_type = EULER_INTEGRATION;
            else if (!strcmp(*argv, "-midpoint")) integration_type = MIDPOINT_INTEGRATION;
            else if (!strcmp(*argv, "-rk4")) integration_type = RK4_INTEGRATION;
            else if (!strcmp(*argv, "-adaptive_step_size")) integration_type = ADAPTIVE_STEP_SIZE_INTEGRATION;
            else if (!strcmp(*argv, "-recordandquit")) {
                argc--; argv++; num_frames_to_record = atoi(*argv);
                GLUTwindow_width = 256;
                GLUTwindow_height = 256;
                save_video = 1;
            }
            else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
            argv++; argc--;
        }
        else {
            if (!input_scene_name) input_scene_name = *argv;
            else { fprintf(stderr, "Invalid program argument: %s", *argv); exit(1); }
            argv++; argc--;
        }
    }

    // Check input_scene_name
    if (!input_scene_name || print_usage) {
        printf("Usage: particleview <input.scn> [-exit_immediately] [-output_image OUTPUT.JPG]  [-video_prefix VIDEO_DIR/PREFIX_] [-euler] [-midpoint] [-rk4] [-adaptive_step_size]  [-recordandquit NUM_FRAMES] [-v]\n");
        return 0;
    }

    // Return OK status
    return 1;
}



////////////////////////////////////////////////////////////
// MAIN
////////////////////////////////////////////////////////////

int 
main(int argc, char **argv)
{
    // Parse program arguments
    if (!ParseArgs(argc, argv)) exit(1);

    // Initialize GLUT
    GLUTInit(&argc, argv);

    // Read scene
    scene = ReadScene(input_scene_name);
    if (!scene) exit(-1);

    // Initialize skybox
    initSkyBox(scene->skybox_type);

    // Run GLUT interface
    GLUTMainLoop();

    // Return success
    return 0;
}



