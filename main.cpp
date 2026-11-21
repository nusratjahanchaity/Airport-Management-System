// ============================================================================
//  Airport Management System  (CSE-4742 Computer Graphics Lab)
//  STEP 1 : Airport Environment                          (18.11.2026)  done
//  STEP 2 : Aircraft takeoff, landing and taxiing animation (21.11.2026)
//
//  Scene   : terminal, 2 runways, taxiways, apron + 5 parking bays, control
//            tower, passenger waiting area, baggage handling zone, roads,
//            parking area, service zone, static vehicles
//  Step 2  : aircraft land on runway 1, taxi to a free bay and park; parked
//            aircraft are pushed back, taxi to runway 2 and take off.
//  Graphics: OpenGL primitives, lighting, depth test, double buffering,
//            gluLookAt + perspective camera, timer callback, right-click menu,
//            bitmap-text HUD   (C++ / OpenGL / GLU / freeGLUT)
//
//  Code::Blocks (MinGW) : link libraries in this order
//        freeglut  opengl32  glu32      (Linker settings -> Link libraries)
//
//  CONTROLS
//     Left mouse drag ........ rotate camera      Mouse wheel / + - .. zoom
//     Arrow keys ............. rotate camera      PageUp / PageDown ... zoom
//     1 / 2 / 3 / 4 .......... Isometric / Top / Terminal / Runway view
//     F ...................... follow an aircraft (press again = next / off)
//     A ...................... add an arriving flight
//     D ...................... send a parked aircraft out (pushback + take-off)
//     P ...................... pause / resume       M ... auto traffic on/off
//     [  ] ................... slower / faster simulation
//     N ...................... Day / Night          Right click ... menu
//     ESC .................... exit
//  (L, R, S, T are kept free for the next steps of the proposal.)
// ============================================================================
#include <GL/freeglut.h>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>

// ------------------------------------------------------------------ globals
static GLUquadric* Q = 0;
static GLuint      sceneList = 0;
static int         winW = 1100, winH = 700;

static float camYaw = 35.f, camPitch = 32.f, camDist = 230.f;
static float camTX = 0.f, camTY = 0.f, camTZ = 8.f;
static bool  night = false;
static bool  dragging = false;
static int   lastX = 0, lastY = 0;

static float radarAngle = 0.f;
static bool  beaconOn = true;
static float fps = 0.f;

struct Lamp { float x, z, h; };
static std::vector<Lamp> lamps;

// ------------------------------------------------------------ small helpers
static void col(float r, float g, float b) { glColor3f(r, g, b); }

static float rnd(int i)                       // deterministic pseudo random 0..1
{
    float v = sinf(i * 12.9898f) * 43758.5453f;
    return v - floorf(v);
}

// flat horizontal rectangle on the ground plane (x0<x1, z0<z1)
static void rect(float x0, float z0, float x1, float z1, float y)
{
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);
    glVertex3f(x0, y, z1); glVertex3f(x1, y, z1);
    glVertex3f(x1, y, z0); glVertex3f(x0, y, z0);
    glEnd();
}

// box centred on (cx,cz), base at y0
static void box(float cx, float y0, float cz, float w, float h, float d)
{
    float x0 = cx - w / 2, x1 = cx + w / 2, z0 = cz - d / 2, z1 = cz + d / 2, y1 = y0 + h;
    glBegin(GL_QUADS);
    glNormal3f(0, 1, 0);  glVertex3f(x0, y1, z1); glVertex3f(x1, y1, z1); glVertex3f(x1, y1, z0); glVertex3f(x0, y1, z0);
    glNormal3f(0, 0, 1);  glVertex3f(x0, y0, z1); glVertex3f(x1, y0, z1); glVertex3f(x1, y1, z1); glVertex3f(x0, y1, z1);
    glNormal3f(0, 0, -1); glVertex3f(x1, y0, z0); glVertex3f(x0, y0, z0); glVertex3f(x0, y1, z0); glVertex3f(x1, y1, z0);
    glNormal3f(1, 0, 0);  glVertex3f(x1, y0, z1); glVertex3f(x1, y0, z0); glVertex3f(x1, y1, z0); glVertex3f(x1, y1, z1);
    glNormal3f(-1, 0, 0); glVertex3f(x0, y0, z0); glVertex3f(x0, y0, z1); glVertex3f(x0, y1, z1); glVertex3f(x0, y1, z0);
    glEnd();
}

// vertical (truncated) cylinder / cone
static void cylT(float x, float y0, float z, float r1, float r2, float h, int sl = 20)
{
    glPushMatrix();
    glTranslatef(x, y0, z);
    glRotatef(-90, 1, 0, 0);
    gluCylinder(Q, r1, r2, h, sl, 1);
    if (r2 > 0.f) { glTranslatef(0, 0, h); gluDisk(Q, 0, r2, sl, 1); }
    glPopMatrix();
}
static void cyl(float x, float y0, float z, float r, float h, int sl = 20) { cylT(x, y0, z, r, r, h, sl); }
static void cone(float x, float y0, float z, float r, float h, int sl = 12) { cylT(x, y0, z, r, 0.f, h, sl); }

// horizontal cylinder along +X starting at x0
static void hcyl(float x0, float y, float z, float r1, float r2, float len, int sl = 16)
{
    glPushMatrix();
    glTranslatef(x0, y, z);
    glRotatef(90, 0, 1, 0);
    gluCylinder(Q, r1, r2, len, sl, 1);
    glPopMatrix();
}

static void sph(float x, float y, float z, float r, int sl = 14)
{
    glPushMatrix(); glTranslatef(x, y, z); glutSolidSphere(r, sl, sl); glPopMatrix();
}

// wheel with axis along Z
static void wheel(float x, float y, float z, float r, float w)
{
    glPushMatrix();
    glTranslatef(x, y, z - w / 2);
    gluCylinder(Q, r, r, w, 12, 1);
    gluDisk(Q, 0, r, 12, 1);
    glTranslatef(0, 0, w);
    gluDisk(Q, 0, r, 12, 1);
    glPopMatrix();
}

static void dashX(float x0, float x1, float z, float w, float dash, float gap, float y)
{
    for (float x = x0; x < x1; x += dash + gap)
        rect(x, z - w / 2, (x + dash < x1 ? x + dash : x1), z + w / 2, y);
}

// stroke-font text (lighting off).  flat = lies on the ground
static void signText(const char* s, float x, float y, float z, float sc, float rotY,
                     float r, float g, float b, bool flat)
{
    glDisable(GL_LIGHTING);
    glColor3f(r, g, b);
    glLineWidth(2.5f);
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(rotY, 0, 1, 0);
    if (flat) glRotatef(-90, 1, 0, 0);
    glScalef(sc, sc, sc);
    float len = glutStrokeLength(GLUT_STROKE_ROMAN, (const unsigned char*)s);
    glTranslatef(-len / 2, 0, 0);
    glutStrokeString(GLUT_STROKE_ROMAN, (const unsigned char*)s);
    glPopMatrix();
    glEnable(GL_LIGHTING);
}

// ------------------------------------------------------------------ ground
static void drawGround()
{
    col(0.30f, 0.56f, 0.26f);
    rect(-120, -75, 120, 75, 0.0f);
}

// ----------------------------------------------------------------- runways
static void runway(float cz, const char* leftNo, const char* rightNo)
{
    col(0.17f, 0.17f, 0.19f);
    rect(-90, cz - 6, 90, cz + 6, 0.03f);

    col(0.95f, 0.95f, 0.95f);                                   // edge lines
    rect(-88, cz - 5.6f, 88, cz - 5.35f, 0.08f);
    rect(-88, cz + 5.35f, 88, cz + 5.6f, 0.08f);
    dashX(-84, 82, cz, 0.5f, 8, 6, 0.08f);                      // centre line

    for (int k = 0; k < 8; k++) {                               // threshold bars
        float z = cz - 4.6f + k * 1.3f;
        rect(-88, z, -80, z + 0.6f, 0.08f);
        rect(80, z, 88, z + 0.6f, 0.08f);
    }
    for (int e = -1; e <= 1; e += 2) {                          // aiming points
        float a = e < 0 ? -60.f : 48.f;
        rect(a, cz - 3.8f, a + 12, cz - 2.6f, 0.08f);
        rect(a, cz + 2.6f, a + 12, cz + 3.8f, 0.08f);
    }
    signText(leftNo,  -74, 0.12f, cz, 0.04f, -90, 1, 1, 1, true);
    signText(rightNo,  74, 0.12f, cz, 0.04f,  90, 1, 1, 1, true);
}

static void taxiX(float x0, float x1, float cz, float w)
{
    col(0.27f, 0.27f, 0.30f); rect(x0, cz - w / 2, x1, cz + w / 2, 0.03f);
    col(0.95f, 0.80f, 0.10f); rect(x0, cz - 0.12f, x1, cz + 0.12f, 0.08f);
}
static void taxiZ(float cx, float z0, float z1, float w)
{
    col(0.27f, 0.27f, 0.30f); rect(cx - w / 2, z0, cx + w / 2, z1, 0.03f);
    col(0.95f, 0.80f, 0.10f); rect(cx - 0.12f, z0, cx + 0.12f, z1, 0.08f);
}

static void drawRunwaysAndTaxiways()
{
    runway(-55, "09L", "27R");
    runway(-35, "09R", "27L");

    taxiX(-75, 75, -15, 6);                                     // parallel taxiway
    const float cx[4] = { -70, -35, 35, 70 };
    for (int i = 0; i < 4; i++) {
        taxiZ(cx[i], -29, -18, 5);                              // to runway 2
        col(0.95f, 0.80f, 0.10f);
        rect(cx[i] - 2.5f, -27.0f, cx[i] + 2.5f, -26.6f, 0.08f);   // hold line
    }
    taxiZ(-70, -49, -41, 5);                                    // between runways
    taxiZ(70, -49, -41, 5);
}

// ------------------------------------------------------------ apron + bays
static const float bayX[5] = { -28, -14, 0, 14, 28 };

static void drawApron()
{
    col(0.58f, 0.58f, 0.60f);
    rect(-45, -12, 45, 14.8f, 0.03f);

    col(0.95f, 0.80f, 0.10f);                                   // stand lines
    for (int i = -3; i <= 3; i++) {
        float x = i * 14.f;
        if (i == 0) continue;
        rect(x - 0.1f, -11, x + 0.1f, 13, 0.08f);
    }
    for (int i = 0; i < 5; i++) {
        rect(bayX[i] - 0.15f, -11, bayX[i] + 0.15f, 13, 0.08f);       // lead-in line
        rect(bayX[i] - 1.5f, 10.0f, bayX[i] + 1.5f, 10.3f, 0.08f);    // stop bar
    }
}

// ---------------------------------------------------------------- terminal
static void drawTerminal()
{
    col(0.82f, 0.84f, 0.88f);  box(0, 0, 22, 60, 10, 14);           // main body
    col(0.30f, 0.33f, 0.40f);  box(0, 10, 22, 62, 0.7f, 17);        // roof slab
    col(0.60f, 0.62f, 0.66f);                                       // roof units
    box(-18, 10.7f, 22, 5, 1.2f, 4); box(0, 10.7f, 22, 5, 1.2f, 4); box(18, 10.7f, 22, 5, 1.2f, 4);

    col(0.34f, 0.56f, 0.76f);                                       // glass front/back
    box(0, 1.0f, 29.1f, 56, 5.0f, 0.2f);
    box(0, 1.0f, 14.9f, 56, 5.0f, 0.2f);
    col(0.95f, 0.95f, 0.95f);
    for (float x = -28; x <= 28.1f; x += 4) {
        box(x, 1.0f, 29.2f, 0.25f, 5.0f, 0.15f);
        box(x, 1.0f, 14.8f, 0.25f, 5.0f, 0.15f);
    }
    col(0.13f, 0.22f, 0.48f);  box(0, 6.0f, 29.05f, 60, 4.0f, 0.1f); // sign band
    signText("AIRPORT", 0, 7.1f, 29.15f, 0.022f, 0, 1, 1, 1, false);

    col(0.28f, 0.28f, 0.33f);  box(0, 5.9f, 31.5f, 56, 0.35f, 5);    // canopy
    col(0.95f, 0.95f, 0.95f);
    for (float x = -26; x <= 26.1f; x += 13) cyl(x, 0, 33.8f, 0.3f, 5.9f, 10);

    // jet bridges
    for (int i = 0; i < 5; i++) {
        col(0.78f, 0.80f, 0.84f);
        box(bayX[i], 1.1f, 12.9f, 2.6f, 2.2f, 4.2f);
        cyl(bayX[i], 0, 14.5f, 1.8f, 3.0f, 14);
        col(0.45f, 0.45f, 0.50f);
        cyl(bayX[i], 0, 10.9f, 0.25f, 1.2f, 8);
    }
}

// ----------------------------------------------------------- control tower
static void drawTower()
{
    const float tx = 52, tz = 2;
    col(0.70f, 0.72f, 0.76f);  box(tx, 0, tz, 7, 3, 7);
    col(0.92f, 0.92f, 0.94f);  cyl(tx, 3, tz, 1.6f, 16, 24);             // shaft
    col(0.85f, 0.15f, 0.15f);  cyl(tx, 8, tz, 1.65f, 1.4f, 24); cyl(tx, 13, tz, 1.65f, 1.4f, 24);
    col(0.25f, 0.28f, 0.34f);  cylT(tx, 19, tz, 3.0f, 4.7f, 2.0f, 24);   // cabin base
    col(0.38f, 0.64f, 0.86f);  cyl(tx, 21, tz, 4.6f, 2.6f, 24);          // glass cabin
    col(0.90f, 0.90f, 0.92f);  cyl(tx, 23.6f, tz, 5.2f, 0.5f, 24);       // roof
    col(0.40f, 0.40f, 0.44f);  cyl(tx, 24.1f, tz, 0.15f, 3.4f, 8);       // mast
}

// ----------------------------------------------------- baggage handling zone
static void drawBaggageZone()
{
    col(0.70f, 0.72f, 0.76f);  box(41, 0, 22, 18, 6, 14);
    col(0.40f, 0.40f, 0.45f);  box(41, 6, 22, 19, 0.5f, 15);
    col(0.20f, 0.22f, 0.26f);  box(41, 0, 14.9f, 5, 3.5f, 0.2f);          // door
    col(0.10f, 0.15f, 0.32f);  box(41, 4.2f, 29.05f, 16, 1.8f, 0.1f);     // sign board
    signText("BAGGAGE HANDLING", 41, 4.7f, 29.12f, 0.0105f, 0, 1, 1, 1, false);
    col(0.34f, 0.56f, 0.76f);  box(41, 1.5f, 29.05f, 12, 2.0f, 0.1f);

    col(0.14f, 0.14f, 0.16f);  box(41, 0.45f, 12, 1.6f, 0.35f, 6.4f);     // conveyor
    col(0.55f, 0.55f, 0.58f);
    box(40.1f, 0.45f, 12, 0.1f, 0.6f, 6.4f); box(41.9f, 0.45f, 12, 0.1f, 0.6f, 6.4f);
    for (int i = 0; i < 3; i++) {                                         // bags
        col(0.8f - 0.2f * i, 0.2f + 0.25f * i, 0.2f + 0.2f * (i % 2));
        box(41, 0.8f, 10.3f + 1.7f * i, 0.9f, 0.6f, 0.8f);
    }
}

// ------------------------------------------------------ passenger waiting area
static void bench(float x, float z)
{
    col(0.50f, 0.33f, 0.20f);
    box(x, 0.45f, z, 5, 0.2f, 1.1f);
    box(x, 0.65f, z - 0.5f, 5, 0.7f, 0.15f);
    col(0.2f, 0.2f, 0.22f);
    box(x - 2.1f, 0, z, 0.2f, 0.45f, 0.9f); box(x + 2.1f, 0, z, 0.2f, 0.45f, 0.9f);
}

static void drawWaitingArea()
{
    col(0.76f, 0.76f, 0.79f);  rect(-52, 14, -32, 30, 0.05f);
    col(0.95f, 0.95f, 0.95f);
    cyl(-51, 0, 15, 0.3f, 4.6f, 10); cyl(-51, 0, 29, 0.3f, 4.6f, 10);
    cyl(-33, 0, 15, 0.3f, 4.6f, 10); cyl(-33, 0, 29, 0.3f, 4.6f, 10);
    col(0.22f, 0.42f, 0.68f);  box(-42, 4.5f, 22, 21, 1.4f, 16.5f);      // roof
    signText("WAITING AREA", -42, 4.95f, 30.3f, 0.0085f, 0, 1, 1, 1, false);
    for (int r = 0; r < 3; r++) { bench(-46, 18.f + 4 * r); bench(-38, 18.f + 4 * r); }
}

// -------------------------------------------------------- roads + parking area
static void drawRoadsAndParking()
{
    col(0.70f, 0.70f, 0.72f);  rect(-50, 29, 50, 31, 0.05f);             // pavement

    col(0.22f, 0.22f, 0.24f);
    rect(-120, 58, 120, 66, 0.03f);                                      // main road
    rect(-50, 31, 50, 37, 0.03f);                                        // drop-off lane
    rect(43.5f, 37, 48.5f, 58, 0.03f);                                   // connectors
    rect(-48.5f, 37, -43.5f, 58, 0.03f);
    rect(40, 46, 43.5f, 50, 0.03f); rect(-43.5f, 46, -40, 50, 0.03f);    // lot entrances
    rect(48.5f, 40, 58, 46, 0.03f);                                      // to service zone
    rect(45, 8, 58, 14, 0.03f);                                          // apron -> service

    col(0.95f, 0.95f, 0.95f);
    rect(-120, 58.3f, 120, 58.5f, 0.08f); rect(-120, 65.5f, 120, 65.7f, 0.08f);
    dashX(-118, 118, 62, 0.3f, 6, 5, 0.08f);
    dashX(-48, 48, 34, 0.25f, 4, 3, 0.08f);
    for (int k = 0; k < 6; k++) rect(8.f + k * 1.0f, 31.4f, 8.6f + k * 1.0f, 36.6f, 0.08f);  // zebra

    // parking area
    col(0.31f, 0.31f, 0.34f);  rect(-40, 40, 40, 56, 0.03f);
    col(0.95f, 0.95f, 0.95f);
    for (int i = 0; i <= 20; i++) {
        float x = -40.f + i * 4.f;
        rect(x - 0.08f, 40.3f, x + 0.08f, 46, 0.08f);
        rect(x - 0.08f, 50, x + 0.08f, 55.7f, 0.08f);
    }
    signText("PARKING", 0, 0.12f, 48.6f, 0.022f, 0, 1, 1, 1, true);
}

// ------------------------------------------------------------------ vehicles
static void drawCar(float x, float z, float yaw, float r, float g, float b)
{
    glPushMatrix(); glTranslatef(x, 0, z); glRotatef(yaw, 0, 1, 0);
    col(r, g, b);                  box(0, 0.35f, 0, 3.8f, 0.8f, 1.7f);
    col(0.75f, 0.85f, 0.92f);      box(-0.2f, 1.15f, 0, 2.0f, 0.7f, 1.5f);
    col(0.05f, 0.05f, 0.05f);
    wheel(1.2f, 0.35f, 0.85f, 0.35f, 0.3f);  wheel(-1.2f, 0.35f, 0.85f, 0.35f, 0.3f);
    wheel(1.2f, 0.35f, -0.85f, 0.35f, 0.3f); wheel(-1.2f, 0.35f, -0.85f, 0.35f, 0.3f);
    glPopMatrix();
}

static void drawBus(float x, float z, float yaw)
{
    glPushMatrix(); glTranslatef(x, 0, z); glRotatef(yaw, 0, 1, 0);
    col(0.95f, 0.75f, 0.10f);  box(0, 0.5f, 0, 8, 2.6f, 2.6f);
    col(0.92f, 0.92f, 0.92f);  box(0, 3.1f, 0, 7.6f, 0.15f, 2.3f);
    col(0.18f, 0.28f, 0.38f);
    box(0, 1.6f, 1.31f, 6.6f, 0.9f, 0.05f); box(0, 1.6f, -1.31f, 6.6f, 0.9f, 0.05f);
    box(4.01f, 1.5f, 0, 0.05f, 1.1f, 2.0f);
    col(0.05f, 0.05f, 0.05f);
    wheel(2.5f, 0.5f, 1.3f, 0.5f, 0.4f);  wheel(-2.5f, 0.5f, 1.3f, 0.5f, 0.4f);
    wheel(2.5f, 0.5f, -1.3f, 0.5f, 0.4f); wheel(-2.5f, 0.5f, -1.3f, 0.5f, 0.4f);
    glPopMatrix();
}

static void drawFuelTruck(float x, float z, float yaw)
{
    glPushMatrix(); glTranslatef(x, 0, z); glRotatef(yaw, 0, 1, 0);
    col(0.20f, 0.20f, 0.22f);  box(0, 0.5f, 0, 9, 0.4f, 2.0f);
    col(0.92f, 0.92f, 0.92f);  box(3.2f, 0.9f, 0, 2.2f, 2.0f, 2.3f);
    col(0.18f, 0.28f, 0.38f);  box(4.31f, 1.9f, 0, 0.05f, 0.8f, 1.8f);
    col(0.85f, 0.10f, 0.10f);  hcyl(-4.2f, 2.0f, 0, 1.15f, 1.15f, 6.4f, 18);
    col(0.05f, 0.05f, 0.05f);
    wheel(3.2f, 0.5f, 1.1f, 0.5f, 0.4f);  wheel(3.2f, 0.5f, -1.1f, 0.5f, 0.4f);
    wheel(-2.5f, 0.5f, 1.1f, 0.5f, 0.4f); wheel(-2.5f, 0.5f, -1.1f, 0.5f, 0.4f);
    wheel(-3.8f, 0.5f, 1.1f, 0.5f, 0.4f); wheel(-3.8f, 0.5f, -1.1f, 0.5f, 0.4f);
    glPopMatrix();
}

static void drawBaggageCart(float x, float z, float yaw)
{
    glPushMatrix(); glTranslatef(x, 0, z); glRotatef(yaw, 0, 1, 0);
    col(0.20f, 0.45f, 0.80f);  box(2.4f, 0.3f, 0, 1.8f, 1.1f, 1.2f);
    col(0.90f, 0.90f, 0.90f);  box(2.6f, 1.4f, 0, 0.9f, 0.1f, 1.1f);
    for (int i = 0; i < 2; i++) {
        float cx = 0.3f - 2.7f * i;
        col(0.50f, 0.50f, 0.55f);  box(cx, 0.35f, 0, 2.4f, 0.2f, 1.6f);
        col(0.75f - 0.3f * i, 0.30f + 0.2f * i, 0.25f);
        box(cx - 0.4f, 0.55f, 0.1f, 0.9f, 0.7f, 0.8f); box(cx + 0.5f, 0.55f, -0.2f, 0.8f, 0.5f, 0.7f);
        col(0.05f, 0.05f, 0.05f);
        wheel(cx + 0.8f, 0.25f, 0.8f, 0.25f, 0.2f); wheel(cx + 0.8f, 0.25f, -0.8f, 0.25f, 0.2f);
    }
    col(0.05f, 0.05f, 0.05f);
    wheel(2.4f, 0.3f, 0.65f, 0.3f, 0.25f); wheel(2.4f, 0.3f, -0.65f, 0.3f, 0.25f);
    glPopMatrix();
}

// ------------------------------------------------------------------ aircraft
// (x,y,z) = main-gear contact point on the ground, nose points to +X (yaw 0).
// pitch is applied about the main gear, bank about the fuselage axis.
static void drawAircraft(float x, float y, float z, float yaw, float pitch, float bank,
                         float s, float tr, float tg, float tb, bool gearDown)
{
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(yaw, 0, 1, 0);
    glTranslatef(-1.0f * s, 0, 0);
    glRotatef(pitch, 0, 0, 1);
    glRotatef(bank, 1, 0, 0);
    glTranslatef(1.0f * s, 0, 0);
    glScalef(s, s, s);

    col(0.95f, 0.95f, 0.97f);
    hcyl(-9, 2.6f, 0, 1.5f, 1.5f, 18, 20);                    // fuselage
    hcyl(-16, 2.6f, 0, 0.5f, 1.5f, 7, 20);                    // tail cone
    glPushMatrix(); glTranslatef(9, 2.6f, 0); glScalef(1.3f, 1, 1); glutSolidSphere(1.5, 20, 16); glPopMatrix();

    col(0.15f, 0.20f, 0.28f);                                 // windows
    for (int i = 0; i < 10; i++) {
        float wx = -6.f + i * 1.3f;
        box(wx, 2.8f, 1.47f, 0.6f, 0.4f, 0.1f); box(wx, 2.8f, -1.47f, 0.6f, 0.4f, 0.1f);
    }
    col(0.82f, 0.84f, 0.88f);                                 // swept wings
    glPushMatrix(); glTranslatef(-1.5f, 1.9f, 6); glRotatef(-25, 0, 1, 0); box(0, 0, 0, 4.5f, 0.3f, 12); glPopMatrix();
    glPushMatrix(); glTranslatef(-1.5f, 1.9f, -6); glRotatef(25, 0, 1, 0);  box(0, 0, 0, 4.5f, 0.3f, 12); glPopMatrix();
    box(-14.5f, 3.0f, 0, 2.2f, 0.2f, 7);                      // stabilisers

    col(tr, tg, tb);                                          // livery colour
    box(-13, 3.1f, 0, 3.2f, 3.6f, 0.3f);                      // tail fin
    hcyl(-0.5f, 1.2f, 4.8f, 0.8f, 0.8f, 3, 14);               // engines
    hcyl(-0.5f, 1.2f, -4.8f, 0.8f, 0.8f, 3, 14);

    if (gearDown) {                                           // landing gear
        col(0.20f, 0.20f, 0.22f);
        box(6.5f, 0.3f, 0, 0.3f, 1.7f, 0.3f);
        box(-1, 0.3f, 1.5f, 0.3f, 1.8f, 0.3f); box(-1, 0.3f, -1.5f, 0.3f, 1.8f, 0.3f);
        col(0.05f, 0.05f, 0.05f);
        wheel(6.5f, 0.35f, 0, 0.35f, 0.5f);
        wheel(-1, 0.4f, 1.5f, 0.4f, 0.6f); wheel(-1, 0.4f, -1.5f, 0.4f, 0.6f);
    }
    glPopMatrix();
}

static void drawParkedThings()
{
    // cars in parking area
    const float pal[6][3] = { {0.8f,0.1f,0.1f}, {0.1f,0.3f,0.8f}, {0.9f,0.9f,0.9f},
                              {0.1f,0.1f,0.1f}, {0.9f,0.7f,0.1f}, {0.2f,0.6f,0.3f} };
    for (int row = 0; row < 2; row++)
        for (int i = 0; i < 20; i++) {
            int h = (int)(rnd(i * 7 + row * 13 + 3) * 10);
            if (h > 5) continue;
            float x = -38.f + i * 4.f;
            float z = row == 0 ? 43.2f : 52.8f;
            const float* c = pal[h % 6];
            drawCar(x, z, 90, c[0], c[1], c[2]);
        }

    // service zone vehicles
    drawBus(68, 18, 0);        drawBus(80, 18, 0);
    drawFuelTruck(92, 20, 0);
    drawBaggageCart(68, 27, 0); drawBaggageCart(78, 27, 0); drawBaggageCart(88, 27, 0);
    // vehicles on the apron service road
    drawBus(50, 11, 180);
}

// -------------------------------------------------------------- service zone
static void drawServiceZone()
{
    col(0.36f, 0.36f, 0.38f);  rect(58, 8, 100, 50, 0.03f);
    col(0.95f, 0.80f, 0.10f);                                   // border lines
    rect(58, 8, 100, 8.3f, 0.08f); rect(58, 49.7f, 100, 50, 0.08f); rect(99.7f, 8, 100, 50, 0.08f);
    signText("SERVICE ZONE", 79, 0.12f, 12, 0.025f, 0, 1, 1, 1, true);

    col(0.62f, 0.64f, 0.68f);  box(86, 0, 41, 24, 7, 14);       // garage / hangar
    col(0.35f, 0.38f, 0.45f);  box(86, 7, 41, 25, 0.6f, 15);
    col(0.20f, 0.22f, 0.26f);
    box(78.5f, 0, 33.95f, 6, 5, 0.2f); box(86, 0, 33.95f, 6, 5, 0.2f); box(93.5f, 0, 33.95f, 6, 5, 0.2f);

    col(0.80f, 0.80f, 0.82f);                                   // fuel tanks
    cyl(64, 0, 38, 3.0f, 6, 24); cyl(64, 0, 45, 3.0f, 6, 24);
    col(0.85f, 0.15f, 0.15f);
    cyl(64, 3, 38, 3.05f, 0.8f, 24); cyl(64, 3, 45, 3.05f, 0.8f, 24);
}

// -------------------------------------------------------------------- trees
static void tree(float x, float z, float s)
{
    col(0.40f, 0.26f, 0.13f);  cyl(x, 0, z, 0.35f * s, 2.0f * s, 8);
    col(0.12f, 0.45f, 0.16f);
    cone(x, 1.5f * s, z, 1.7f * s, 3.5f * s, 10);
    cone(x, 3.2f * s, z, 1.3f * s, 2.8f * s, 10);
}

static void drawTrees()
{
    int id = 0;
    for (float x = -112; x <= 112; x += 12) {
        tree(x + rnd(id) * 3, -68.5f + rnd(id + 1) * 2, 0.9f + rnd(id + 2) * 0.5f); id += 3;
        tree(x + rnd(id) * 3,  71.5f + rnd(id + 1) * 2, 0.9f + rnd(id + 2) * 0.5f); id += 3;
    }
    for (float gz = -10; gz <= 54; gz += 9) {
        for (float gx = -112; gx <= -58; gx += 9) {
            tree(gx + rnd(id) * 4, gz + rnd(id + 1) * 4, 0.9f + rnd(id + 2) * 0.5f); id += 3;
        }
        for (float gx = 104; gx <= 116; gx += 9) {
            tree(gx + rnd(id) * 3, gz + rnd(id + 1) * 4, 0.9f + rnd(id + 2) * 0.5f); id += 3;
        }
    }
}

// -------------------------------------------------------------- lamp posts
static void buildLamps()
{
    for (float x = -110; x <= 110; x += 20) { Lamp l = { x, 57.2f, 6 };  lamps.push_back(l); }
    for (float x = -40;  x <= 40;  x += 16) { Lamp l = { x, 38.4f, 5 };  lamps.push_back(l); }
    const float mx[2] = { -44, 44 };
    for (int i = 0; i < 2; i++) {
        Lamp a = { mx[i], -11.0f, 12 }; lamps.push_back(a);
        Lamp b = { mx[i], 13.0f, 12 };  lamps.push_back(b);
    }
    for (float x = 62; x <= 98; x += 12) { Lamp l = { x, 9.5f, 7 };  lamps.push_back(l); }
}

static void drawLampPoles()
{
    col(0.30f, 0.30f, 0.33f);
    for (size_t i = 0; i < lamps.size(); i++) cyl(lamps[i].x, 0, lamps[i].z, 0.18f, lamps[i].h, 8);
}

// ------------------------------------------------- complete static scene list
static void drawStaticScene()
{
    drawGround();
    drawRunwaysAndTaxiways();
    drawApron();
    drawTerminal();
    drawTower();
    drawBaggageZone();
    drawWaitingArea();
    drawRoadsAndParking();
    drawServiceZone();
    drawParkedThings();
    drawLampPoles();
    drawTrees();
}

// ============================================================================
//  STEP 2 : AIRCRAFT SIMULATION  -  landing, taxiing, pushback, takeoff
//
//  Motion  = translation + rotation (yaw / pitch) + simple physics
//            (constant acceleration and deceleration, glide-slope, flare).
//  Taxiing = waypoint steering (A* graph routing comes in Step 5).
//  Runways = a simple "claim" (r1Owner / r2Owner) stops an aircraft from
//            entering a busy runway (priority-queue scheduling: Step 3).
//
//  Runway 1 (z = -55) : arrivals land west -> east, leave at the x = 70 link
//  Runway 2 (z = -35) : departures line up at the x = -70 link and take off,
//                       arrivals cross it at x = 70 on the way to the apron.
// ============================================================================
enum State  { S_APPROACH, S_ROLLOUT, S_TAXI_IN, S_PARKED, S_PUSHBACK, S_TAXI_OUT, S_TAKEOFF, S_CLIMB };
enum WpType { WP_NORMAL, WP_HOLD_R2, WP_REL_R1, WP_REL_R2, WP_TAKEOFF, WP_PARK };

struct Wp { float x, z; int type; };

struct Aircraft {
    int   id;
    State st;
    float x, y, z, yaw, pitch, bank, v;
    float cr, cg, cb;                 // livery colour
    int   bay;
    std::vector<Wp> path;
    size_t wp;
    float timer, pbT;
    bool  force, dead;
};

static const float RAD       = 3.14159265f / 180.f;
static const float AC_SCALE  = 0.6f;
static const float RWY1_Z    = -55.f;
static const float RWY2_Z    = -35.f;
static const float STAND_Z   = 4.f;
static const float TAXI_V    = 8.f;
static const float APP_V     = 55.f;      // approach speed
static const float APP_X0    = -240.f;    // where an arrival appears
static const float APP_H     = 24.f;      // height at that point
static const float TD_X      = -62.f;     // touch-down point
static const float LAND_DECEL = 11.9f;
static const float ROT_V     = 60.f;      // rotation speed
static const float TO_ACC    = 16.f;      // take-off acceleration
static const float PB_V      = 3.f;       // pushback speed

static const float LIVERY[5][3] = { {0.80f,0.10f,0.10f}, {0.10f,0.30f,0.75f}, {0.10f,0.60f,0.25f},
                                    {0.90f,0.55f,0.05f}, {0.55f,0.15f,0.65f} };

static std::vector<Aircraft> fleet;
static int   nextId = 1, r1Owner = -1, r2Owner = -1;
static int   bayOwner[5] = { -1, -1, -1, -1, -1 };
static int   pendingArrivals = 0, landedCount = 0, departedCount = 0;
static float arrivalTimer = 3.f;
static bool  paused = false, autoMode = true;
static float simSpeed = 1.f;
static int   followId = -1;
static int   rndCtr = 100;

static float rr(float a, float b) { return a + (b - a) * rnd(rndCtr++); }

static float normAng(float a)
{
    while (a > 180.f) a -= 360.f;
    while (a < -180.f) a += 360.f;
    return a;
}

static bool isGround(const Aircraft& a)
{
    return a.st == S_ROLLOUT || a.st == S_TAXI_IN || a.st == S_PARKED ||
           a.st == S_PUSHBACK || a.st == S_TAXI_OUT || a.st == S_TAKEOFF;
}

static void addWp(std::vector<Wp>& p, float x, float z, int t = WP_NORMAL)
{
    Wp w; w.x = x; w.z = z; w.type = t; p.push_back(w);
}

static int freeBay()
{
    const int order[5] = { 2, 0, 4, 3, 1 };
    for (int i = 0; i < 5; i++) if (bayOwner[order[i]] == -1) return order[i];
    return -1;
}

static void initFleet()
{
    fleet.clear();
    const int bays[3] = { 0, 2, 4 };
    for (int i = 0; i < 3; i++) {
        Aircraft a;
        a.id = nextId++; a.st = S_PARKED;
        a.bay = bays[i];
        a.x = bayX[a.bay]; a.y = 0; a.z = STAND_Z;
        a.yaw = -90; a.pitch = 0; a.bank = 0; a.v = 0;
        a.cr = LIVERY[i][0]; a.cg = LIVERY[i][1]; a.cb = LIVERY[i][2];
        a.wp = 0; a.timer = 9.f + i * 9.f; a.pbT = 0; a.force = false; a.dead = false;
        bayOwner[a.bay] = a.id;
        fleet.push_back(a);
    }
}

static void spawnArrival(int bay)
{
    Aircraft a;
    a.id = nextId++; a.st = S_APPROACH; a.bay = bay;
    a.x = APP_X0; a.y = APP_H; a.z = RWY1_Z;
    a.yaw = 0; a.pitch = -8; a.bank = 0; a.v = APP_V;
    int c = a.id % 5;
    a.cr = LIVERY[c][0]; a.cg = LIVERY[c][1]; a.cb = LIVERY[c][2];
    a.wp = 0; a.timer = 0; a.pbT = 0; a.force = false; a.dead = false;
    bayOwner[bay] = a.id;
    r1Owner = a.id;
    fleet.push_back(a);
}

static void buildArrivalPath(Aircraft& a)
{
    float bx = bayX[a.bay];
    a.path.clear(); a.wp = 0;
    addWp(a.path, 64, -54);
    addWp(a.path, 70, -47.5f, WP_HOLD_R2);      // hold short of runway 2
    addWp(a.path, 70, -38,    WP_REL_R1);       // tail has left runway 1
    addWp(a.path, 70, -30);
    addWp(a.path, 70, -18,    WP_REL_R2);       // clear of runway 2
    addWp(a.path, 62, -15);
    addWp(a.path, bx + 12, -15);
    addWp(a.path, bx, -5);
    addWp(a.path, bx, STAND_Z, WP_PARK);
}

static void buildDeparturePath(Aircraft& a)
{
    a.path.clear(); a.wp = 0;
    addWp(a.path, a.x - 9, -15);
    addWp(a.path, -60, -15);
    addWp(a.path, -70, -19.5f);
    addWp(a.path, -70, -22.2f, WP_HOLD_R2);     // hold short of runway 2
    addWp(a.path, -70, -31);
    addWp(a.path, -64, RWY2_Z);
    addWp(a.path, -58, RWY2_Z, WP_TAKEOFF);     // line-up -> take-off roll
}

static bool areaClear(float cx, float cz, float r, int selfId)
{
    for (size_t i = 0; i < fleet.size(); i++) {
        const Aircraft& b = fleet[i];
        if (b.id == selfId || !isGround(b) || b.st == S_PARKED) continue;
        float dx = b.x - cx, dz = b.z - cz;
        if (sqrtf(dx * dx + dz * dz) < r) return false;
    }
    return true;
}

// simple spacing rule used while taxiing (full collision handling: Step 5)
static bool groundBlocked(const Aircraft& a)
{
    float h = a.yaw * RAD, hx = cosf(h), hz = -sinf(h);
    for (size_t i = 0; i < fleet.size(); i++) {
        const Aircraft& b = fleet[i];
        if (b.id == a.id || !isGround(b) || b.st == S_PARKED) continue;
        float dx = b.x - a.x, dz = b.z - a.z;
        float d = sqrtf(dx * dx + dz * dz);
        if (b.st == S_PUSHBACK) {                    // keep clear of the tail swing
            float ex = b.x + 9.f - a.x, ez = -12.f - a.z;
            if (d < 28.f || sqrtf(ex * ex + ez * ez) < 24.f) return true;
        }
        if (d < 28.f) {
            float dot = (dx * hx + dz * hz) / (d + 1e-4f);
            if (dot > 0.6f && (b.v < 2.5f || b.id < a.id)) return true;
        }
    }
    return false;
}

static void taxiStep(Aircraft& a, float dt)
{
    if (a.wp >= a.path.size()) { a.v = 0; return; }
    const Wp w = a.path[a.wp];
    float dx = w.x - a.x, dz = w.z - a.z;
    float d = sqrtf(dx * dx + dz * dz);
    float cap = (w.type == WP_PARK) ? 0.6f : (w.type == WP_HOLD_R2 ? 1.2f : 3.5f);

    if (d < cap) {                                   // waypoint reached
        if (w.type == WP_HOLD_R2) {
            if (r2Owner == -1 || r2Owner == a.id) r2Owner = a.id;
            else { a.v = 0; return; }                // wait for the runway
        }
        if (w.type == WP_REL_R1 && r1Owner == a.id) r1Owner = -1;
        if (w.type == WP_REL_R2 && r2Owner == a.id) r2Owner = -1;
        a.wp++;
        if (w.type == WP_TAKEOFF) { a.st = S_TAKEOFF; return; }
        if (w.type == WP_PARK) {
            a.x = w.x; a.z = w.z; a.yaw = -90; a.v = 0;
            a.st = S_PARKED; a.timer = rr(18.f, 30.f);
        }
        return;
    }

    float vt = TAXI_V;
    if (w.type == WP_HOLD_R2 || w.type == WP_PARK) vt = fminf(vt, fmaxf(1.0f, d * 0.7f));
    float want = atan2f(-dz, dx) / RAD;
    float diff = normAng(want - a.yaw);
    if (fabsf(diff) > 50.f) vt = fminf(vt, 3.f);
    if (groundBlocked(a)) vt = 0.f;

    if (a.v < vt) a.v = fminf(vt, a.v + 6.f * dt);
    else          a.v = fmaxf(vt, a.v - 14.f * dt);

    float step = 65.f * dt * fminf(1.f, 0.25f + a.v / 3.f);
    a.yaw = normAng(a.yaw + (fabsf(diff) <= step ? diff : (diff > 0 ? step : -step)));
    a.x += cosf(a.yaw * RAD) * a.v * dt;
    a.z -= sinf(a.yaw * RAD) * a.v * dt;
}

static void updateAircraft(Aircraft& a, float dt)
{
    const float L = TD_X - APP_X0;
    switch (a.st) {

    case S_APPROACH: {                               // glide slope + flare
        a.v = APP_V;
        a.x += a.v * dt;
        float p = (a.x - APP_X0) / L;
        if (p >= 1.f) { a.x = TD_X; a.y = 0; a.pitch = 5.f; a.st = S_ROLLOUT; landedCount++; break; }
        a.y = APP_H * powf(1.f - p, 1.3f);
        float slope = -APP_H * 1.3f * powf(1.f - p, 0.3f) / L;
        a.pitch = atanf(slope) / RAD;
        if (p > 0.85f) a.pitch += 5.f * (p - 0.85f) / 0.15f;
        break; }

    case S_ROLLOUT:                                  // braking on the runway
        a.v -= LAND_DECEL * dt;
        a.x += a.v * dt;
        a.z = RWY1_Z;
        if (a.pitch > 0.f) a.pitch = fmaxf(0.f, a.pitch - 12.f * dt);
        if (a.v <= TAXI_V + 0.5f) { a.st = S_TAXI_IN; buildArrivalPath(a); }
        break;

    case S_TAXI_IN:
    case S_TAXI_OUT:
        taxiStep(a, dt);
        break;

    case S_PARKED:
        a.timer -= dt;
        if (a.timer <= 0.f && (autoMode || a.force)) {
            if (areaClear(a.x + 4, -10, 26.f, a.id) && areaClear(a.x + 9, -12, 24.f, a.id)) { a.st = S_PUSHBACK; a.pbT = 0; a.force = false; }
            else a.timer = 2.f;
        }
        break;

    case S_PUSHBACK: {                               // tug pushes the tail out
        a.pbT += dt;
        a.v = PB_V;
        float h = a.yaw * RAD;
        a.x -= cosf(h) * PB_V * dt;
        a.z += sinf(h) * PB_V * dt;
        if (a.pbT > 2.5f) {
            float d = normAng(180.f - a.yaw), step = 20.f * dt;
            a.yaw = normAng(a.yaw + (fabsf(d) <= step ? d : (d > 0 ? step : -step)));
            if (fabsf(normAng(180.f - a.yaw)) < 0.01f) {
                a.yaw = 180.f; a.v = 0;
                bayOwner[a.bay] = -1;
                a.st = S_TAXI_OUT; buildDeparturePath(a);
            }
        }
        break; }

    case S_TAKEOFF: {                                // accelerate, rotate
        float d = normAng(0.f - a.yaw), step = 45.f * dt;
        a.yaw = normAng(a.yaw + (fabsf(d) <= step ? d : (d > 0 ? step : -step)));
        a.z += (RWY2_Z - a.z) * fminf(1.f, 2.f * dt);
        a.v += TO_ACC * dt;
        a.x += a.v * dt;
        if (a.v >= ROT_V) {
            a.pitch = fminf(8.f, a.pitch + 10.f * dt);
            if (a.pitch >= 8.f) { a.st = S_CLIMB; a.y = 0.f; }
        }
        break; }

    case S_CLIMB:                                    // lift-off and climb out
        a.v = fminf(95.f, a.v + 10.f * dt);
        a.pitch = (a.y < 2.f) ? 8.f : fminf(13.f, a.pitch + 3.f * dt);
        a.x += cosf(a.pitch * RAD) * a.v * dt;
        a.y += sinf(a.pitch * RAD) * a.v * dt;
        if (a.y > 4.f && r2Owner == a.id) r2Owner = -1;
        if (a.x > 260.f) { a.dead = true; departedCount++; }
        break;
    }
}

static void stepSim(float dt)
{
    if (autoMode) {
        arrivalTimer -= dt;
        if (arrivalTimer <= 0.f) { if (pendingArrivals < 6) pendingArrivals++; arrivalTimer = rr(22.f, 32.f); }
    }
    if (pendingArrivals > 0 && r1Owner == -1) {
        int b = freeBay();
        if (b >= 0) { spawnArrival(b); pendingArrivals--; }
    }
    for (size_t i = 0; i < fleet.size(); i++) updateAircraft(fleet[i], dt);
    for (size_t i = 0; i < fleet.size();) {
        if (fleet[i].dead) fleet.erase(fleet.begin() + i); else ++i;
    }
}

static void updateSim(float total)
{
    int n = (int)ceilf(total / 0.025f);
    if (n < 1) n = 1;
    float h = total / n;
    for (int i = 0; i < n; i++) stepSim(h);
}

// ---- user actions
static void addFlight()    { if (pendingArrivals < 6) pendingArrivals++; }
static void forceDepart()
{
    for (size_t i = 0; i < fleet.size(); i++)
        if (fleet[i].st == S_PARKED && !fleet[i].force) { fleet[i].force = true; fleet[i].timer = 0.f; return; }
}
static void cycleFollow()
{
    size_t idx = fleet.size();
    for (size_t i = 0; i < fleet.size(); i++) if (fleet[i].id == followId) idx = i;
    size_t next = (followId < 0) ? 0 : idx + 1;
    if (fleet.empty() || next >= fleet.size()) { followId = -1; camTX = 0; camTY = 0; camTZ = 8; }
    else { followId = fleet[next].id; if (camDist > 150.f) camDist = 110.f; }
}
static void updateFollowCamera()
{
    if (followId < 0) return;
    for (size_t i = 0; i < fleet.size(); i++)
        if (fleet[i].id == followId) {
            camTX += (fleet[i].x - camTX) * 0.08f;
            camTY += (fleet[i].y - camTY) * 0.08f;
            camTZ += (fleet[i].z - camTZ) * 0.08f;
            return;
        }
    followId = -1; camTX = 0; camTY = 0; camTZ = 8;
}

static const char* stateName(const Aircraft& a)
{
    if ((a.st == S_TAXI_IN || a.st == S_TAXI_OUT) && a.v < 0.2f) return "Holding / waiting";
    switch (a.st) {
    case S_APPROACH: return "Final approach";
    case S_ROLLOUT:  return "Landing roll";
    case S_TAXI_IN:  return "Taxi to gate";
    case S_PARKED:   return "Parked";
    case S_PUSHBACK: return "Pushback";
    case S_TAXI_OUT: return "Taxi to runway";
    case S_TAKEOFF:  return "Take-off roll";
    case S_CLIMB:    return "Climb out";
    }
    return "";
}

// ---- drawing
static void drawFleet()
{
    for (size_t i = 0; i < fleet.size(); i++) {
        const Aircraft& a = fleet[i];
        drawAircraft(a.x, a.y, a.z, a.yaw, a.pitch, a.bank, AC_SCALE, a.cr, a.cg, a.cb, a.y < 3.f);
    }
    // soft shadows of flying aircraft
    glDisable(GL_LIGHTING);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    for (size_t i = 0; i < fleet.size(); i++) {
        const Aircraft& a = fleet[i];
        if (a.y < 0.3f) continue;
        float al = 0.35f * fmaxf(0.15f, 1.f - a.y / 80.f);
        glColor4f(0, 0, 0, al);
        glPushMatrix();
        glTranslatef(a.x + a.y * 0.45f, 0.12f, a.z + a.y * 0.25f);
        glRotatef(a.yaw, 0, 1, 0);
        glBegin(GL_QUADS);
        glVertex3f(-9, 0, -0.9f); glVertex3f(6, 0, -0.9f); glVertex3f(6, 0, 0.9f); glVertex3f(-9, 0, 0.9f);
        glVertex3f(-4, 0, -6.5f); glVertex3f(0, 0, -6.5f); glVertex3f(0, 0, 6.5f); glVertex3f(-4, 0, 6.5f);
        glEnd();
        glPopMatrix();
    }
    glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}

// --------------------------------------------- animated + emissive (every frame)
static void drawDynamic()
{
    const float tx = 52, tz = 2;
    glPushMatrix();                                              // rotating radar
    glTranslatef(tx, 26.8f, tz);
    glRotatef(radarAngle, 0, 1, 0);
    col(0.30f, 0.30f, 0.34f);  box(0, 0, 0, 0.3f, 0.7f, 0.3f);
    col(0.75f, 0.75f, 0.78f);  box(0, 0.7f, 0, 4.0f, 0.35f, 0.6f);
    box(1.9f, 0.7f, 0, 0.2f, 1.0f, 0.6f);
    glPopMatrix();

    drawFleet();                                                 // Step 2 : aircraft
}

static void drawGlow()
{
    glDisable(GL_LIGHTING);

    // tower beacon (blinking)
    if (beaconOn) { glColor3f(1.0f, 0.1f, 0.1f); sph(52, 28.2f, 2, night ? 0.9f : 0.6f, 12); }

    float s = night ? 0.40f : 0.22f;                             // runway edge lights
    glColor3f(1.0f, 1.0f, 0.85f);
    for (int r = 0; r < 2; r++) {
        float cz = r == 0 ? -55.f : -35.f;
        for (float x = -88; x <= 88.1f; x += 11) { sph(x, 0.25f, cz - 6.4f, s, 6); sph(x, 0.25f, cz + 6.4f, s, 6); }
        for (int k = 0; k < 7; k++) {
            float z = cz - 5.4f + k * 1.8f;
            glColor3f(0.1f, 1.0f, 0.2f); sph(-90.6f, 0.25f, z, s, 6);   // green threshold
            glColor3f(1.0f, 0.1f, 0.1f); sph(90.6f, 0.25f, z, s, 6);    // red end
        }
        glColor3f(1.0f, 1.0f, 0.85f);
    }
    glColor3f(0.2f, 0.4f, 1.0f);                                 // taxiway lights
    for (float x = -72; x <= 72.1f; x += 8) sph(x, 0.2f, -18.4f, s, 6);

    glColor3f(1.0f, 0.95f, 0.65f);                               // lamp heads
    for (size_t i = 0; i < lamps.size(); i++)
        sph(lamps[i].x, lamps[i].h + 0.3f, lamps[i].z, night ? 0.7f : 0.4f, 8);

    if (night) {                                                 // lit windows
        glColor3f(1.0f, 0.88f, 0.50f);
        box(0, 1.0f, 29.12f, 55, 4.8f, 0.05f);
        box(0, 1.0f, 14.82f, 55, 4.8f, 0.05f);
        box(41, 1.5f, 29.1f, 11, 1.8f, 0.05f);
        cyl(52, 21, 2, 4.62f, 2.6f, 24);
    }
    glEnable(GL_LIGHTING);
}

// ---------------------------------------------------------------------- HUD
static void hudText(float x, float y, const char* s)
{
    glRasterPos2f(x, y);
    glutBitmapString(GLUT_BITMAP_HELVETICA_12, (const unsigned char*)s);
}

static void hudPanel(float x0, float y0, float x1, float y1)
{
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0, 0, 0, 0.55f);
    glBegin(GL_QUADS);
    glVertex2f(x0, y0); glVertex2f(x1, y0); glVertex2f(x1, y1); glVertex2f(x0, y1);
    glEnd();
    glDisable(GL_BLEND);
}

static void drawHUD()
{
    glDisable(GL_LIGHTING); glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0, winW, 0, winH);
    glMatrixMode(GL_MODELVIEW);  glPushMatrix(); glLoadIdentity();

    char buf[96];

    // ---- left panel : title + controls + status
    hudPanel(10, winH - 134, 440, winH - 10);
    glColor3f(1, 1, 1);
    hudText(20, winH - 28, "AIRPORT MANAGEMENT SYSTEM  -  Step 2: Aircraft Animation");
    glColor3f(0.85f, 0.9f, 1.0f);
    hudText(20, winH - 48, "Mouse drag / Arrows: rotate     Wheel / + - : zoom");
    hudText(20, winH - 64, "1 Iso  2 Top  3 Terminal  4 Runway      F : follow aircraft");
    hudText(20, winH - 80, "A : add arriving flight      D : depart a parked aircraft");
    hudText(20, winH - 96, "P : pause    M : auto traffic    [ ] : sim speed    N : day/night");
    snprintf(buf, sizeof(buf), "FPS %.0f | %s | Sim: %s x%.2g | Auto: %s", fps, night ? "Night" : "Day",
             paused ? "PAUSED" : "RUNNING", simSpeed, autoMode ? "ON" : "OFF");
    glColor3f(1.0f, 0.9f, 0.4f);
    hudText(20, winH - 118, buf);

    // ---- right panel : flight status
    int n = (int)fleet.size();
    float ph = 16.f * (n + 5) + 14.f;
    hudPanel(winW - 340, winH - 10 - ph, winW - 10, winH - 10);
    float y = winH - 28.f;
    glColor3f(1, 1, 1);
    hudText(winW - 330, y, "FLIGHT STATUS"); y -= 18;
    glColor3f(0.85f, 0.9f, 1.0f);
    snprintf(buf, sizeof(buf), "Active: %d    Waiting to land: %d", n, pendingArrivals);
    hudText(winW - 330, y, buf); y -= 16;
    snprintf(buf, sizeof(buf), "Runway 1 (landing): %s", r1Owner >= 0 ? "BUSY" : "FREE");
    hudText(winW - 330, y, buf); y -= 16;
    snprintf(buf, sizeof(buf), "Runway 2 (take-off / crossing): %s", r2Owner >= 0 ? "BUSY" : "FREE");
    hudText(winW - 330, y, buf); y -= 16;
    snprintf(buf, sizeof(buf), "Landed: %d    Departed: %d", landedCount, departedCount);
    hudText(winW - 330, y, buf); y -= 18;
    for (int i = 0; i < n; i++) {
        const Aircraft& a = fleet[i];
        if (a.id == followId) glColor3f(1.0f, 0.9f, 0.4f); else glColor3f(1, 1, 1);
        if (a.st == S_PARKED) snprintf(buf, sizeof(buf), "BG%03d  Parked (bay %d)", 100 + a.id, a.bay + 1);
        else                  snprintf(buf, sizeof(buf), "BG%03d  %s", 100 + a.id, stateName(a));
        hudText(winW - 330, y, buf); y -= 16;
    }

    glMatrixMode(GL_PROJECTION); glPopMatrix();
    glMatrixMode(GL_MODELVIEW);  glPopMatrix();
    glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING);
}

// ------------------------------------------------------------ camera + light
static void setView(int v)
{
    followId = -1;
    switch (v) {
    case 1: camYaw = 35;  camPitch = 32; camDist = 230; camTX = 0; camTZ = 8;   break; // isometric
    case 2: camYaw = 0;   camPitch = 88; camDist = 270; camTX = 0; camTZ = 0;   break; // top view
    case 3: camYaw = -22; camPitch = 17; camDist = 125; camTX = 0; camTZ = 12;  break; // terminal
    case 4: camYaw = 78;  camPitch = 11; camDist = 160; camTX = 0; camTZ = -35; break; // runway
    }
}

static void setupLighting()
{
    float amb[4], dif[4], fil[4];
    if (!night) {
        float a[4] = { 0.40f, 0.40f, 0.44f, 1 }, d[4] = { 0.85f, 0.82f, 0.75f, 1 }, f[4] = { 0.18f, 0.20f, 0.28f, 1 };
        for (int i = 0; i < 4; i++) { amb[i] = a[i]; dif[i] = d[i]; fil[i] = f[i]; }
    } else {
        float a[4] = { 0.14f, 0.15f, 0.24f, 1 }, d[4] = { 0.22f, 0.26f, 0.42f, 1 }, f[4] = { 0.05f, 0.05f, 0.08f, 1 };
        for (int i = 0; i < 4; i++) { amb[i] = a[i]; dif[i] = d[i]; fil[i] = f[i]; }
    }
    float p0[4] = { 0.45f, 0.85f, 0.35f, 0.0f };                 // sun / moon
    float p1[4] = { -0.50f, 0.30f, -0.60f, 0.0f };               // soft fill
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);  glLightfv(GL_LIGHT0, GL_POSITION, p0);
    glLightfv(GL_LIGHT1, GL_DIFFUSE, fil);  glLightfv(GL_LIGHT1, GL_POSITION, p1);
}

// ----------------------------------------------------------------- callbacks
static void display()
{
    if (night) glClearColor(0.03f, 0.05f, 0.12f, 1); else glClearColor(0.55f, 0.76f, 0.95f, 1);
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    updateFollowCamera();
    const float d2r = 3.14159265f / 180.f;
    float ex = camTX + camDist * cosf(camPitch * d2r) * sinf(camYaw * d2r);
    float ey = camTY + camDist * sinf(camPitch * d2r);
    float ez = camTZ + camDist * cosf(camPitch * d2r) * cosf(camYaw * d2r);
    gluLookAt(ex, ey, ez, camTX, camTY, camTZ, 0, 1, 0);

    setupLighting();
    glCallList(sceneList);
    drawDynamic();
    drawGlow();
    drawHUD();

    static int frames = 0, lastT = 0;
    frames++;
    int t = glutGet(GLUT_ELAPSED_TIME);
    if (t - lastT >= 1000) { fps = frames * 1000.f / (t - lastT); frames = 0; lastT = t; }

    glutSwapBuffers();
}

static void reshape(int w, int h)
{
    if (h == 0) h = 1;
    winW = w; winH = h;
    glViewport(0, 0, w, h);
    glMatrixMode(GL_PROJECTION); glLoadIdentity();
    gluPerspective(48.0, (double)w / h, 1.0, 700.0);
    glMatrixMode(GL_MODELVIEW);
}

static void timer(int)                       // simulation clock (16 ms step)
{
    static int lastT = 0;
    int t = glutGet(GLUT_ELAPSED_TIME);
    float dt = (t - lastT) / 1000.f;
    lastT = t;
    if (dt > 0.05f) dt = 0.05f;

    radarAngle = fmodf(t * 0.12f, 360.f);
    beaconOn = ((t / 500) % 2) == 0;
    if (!paused) updateSim(dt * simSpeed);       // advance aircraft simulation
    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
}

static void clampCam()
{
    if (camPitch < 8.f) camPitch = 8.f;
    if (camPitch > 88.f) camPitch = 88.f;
    if (camDist < 50.f) camDist = 50.f;
    if (camDist > 400.f) camDist = 400.f;
}

static void keyboard(unsigned char k, int, int)
{
    switch (k) {
    case 27: exit(0); break;
    case '1': case '2': case '3': case '4': setView(k - '0'); break;
    case 'n': case 'N': night = !night; break;
    case 'a': case 'A': addFlight(); break;
    case 'd': case 'D': forceDepart(); break;
    case 'p': case 'P': paused = !paused; break;
    case 'm': case 'M': autoMode = !autoMode; break;
    case 'f': case 'F': cycleFollow(); break;
    case '[': simSpeed = fmaxf(0.25f, simSpeed * 0.5f); break;
    case ']': simSpeed = fminf(4.f, simSpeed * 2.f); break;
    case '+': case '=': camDist -= 10; break;
    case '-': case '_': camDist += 10; break;
    }
    clampCam();
}

static void special(int k, int, int)
{
    switch (k) {
    case GLUT_KEY_LEFT:      camYaw -= 4; break;
    case GLUT_KEY_RIGHT:     camYaw += 4; break;
    case GLUT_KEY_UP:        camPitch += 3; break;
    case GLUT_KEY_DOWN:      camPitch -= 3; break;
    case GLUT_KEY_PAGE_UP:   camDist -= 10; break;
    case GLUT_KEY_PAGE_DOWN: camDist += 10; break;
    }
    clampCam();
}

static void mouse(int button, int state, int x, int y)
{
    if (button == GLUT_LEFT_BUTTON) { dragging = (state == GLUT_DOWN); lastX = x; lastY = y; }
}

static void motion(int x, int y)
{
    if (!dragging) return;
    camYaw   -= (x - lastX) * 0.4f;
    camPitch += (y - lastY) * 0.4f;
    lastX = x; lastY = y;
    clampCam();
}

static void wheelFn(int, int dir, int, int)
{
    camDist -= dir * 12.f;
    clampCam();
}

static void menuFn(int id)
{
    switch (id) {
    case 1: case 2: case 3: case 4: setView(id); break;
    case 5: night = !night; break;
    case 6: exit(0); break;
    }
}

static void init()
{
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    Q = gluNewQuadric();
    gluQuadricNormals(Q, GLU_SMOOTH);

    buildLamps();
    initFleet();
    sceneList = glGenLists(1);                       // static geometry -> display list
    glNewList(sceneList, GL_COMPILE);
    drawStaticScene();
    glEndList();

    glutCreateMenu(menuFn);
    glutAddMenuEntry("Isometric view", 1);
    glutAddMenuEntry("Top view", 2);
    glutAddMenuEntry("Terminal view", 3);
    glutAddMenuEntry("Runway view", 4);
    glutAddMenuEntry("Day / Night", 5);
    glutAddMenuEntry("Exit", 6);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

int main(int argc, char** argv)
{
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(winW, winH);
    glutInitWindowPosition(80, 40);
    glutCreateWindow("Airport Management System - Step 2: Aircraft Animation");

    init();
    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(special);
    glutMouseFunc(mouse);
    glutMotionFunc(motion);
    glutMouseWheelFunc(wheelFn);
    glutTimerFunc(16, timer, 0);
    glutMainLoop();
    return 0;
}
