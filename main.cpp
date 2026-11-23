// ============================================================================
//  Airport Management System  (CSE-4742 Computer Graphics Lab)
//  STEP 1 : Airport Environment                          (18.11.2026)  done
//  STEP 2 : Aircraft takeoff, landing and taxiing animation (21.11.2026) done
//
//  STEP 3 : Priority arrival queue, runway and gate assignment, flight board,
//           baggage / luggage handling system, clear airport signboards
//
//  Scene   : terminal, 2 runways, taxiways, apron + 5 parking bays, control
//            tower, passenger waiting area, baggage hall with claim carousel,
//            baggage conveyors, roads, parking area, service zone, vehicles
//  Step 2  : aircraft land on runway 1, taxi to a free bay and park; parked
//            aircraft are pushed back, taxi to runway 2 and take off.
//  Step 3  : arrivals wait in a priority queue (emergency first), get a runway
//            and a bay, and the board shows queue / runway / bay status.
//  Baggage : 3 baggage tugs, each towing a cart, run on a one-way service road
//            in front of the stands.
//              Arrival  : belt loader -> tug cart -> baggage hall inbound belt
//                         -> claim carousel -> passengers collect bags
//              Departure: check-in feed belt -> carousel -> outbound belt
//                         -> tug at the hall rank -> belt loader -> aircraft
//            An aircraft is pushed back only when its bags are loaded and the
//            belt loader has been withdrawn.
//  Graphics: OpenGL primitives, lighting, depth test, double buffering,
//            gluLookAt + perspective camera, timer callback, right-click menu,
//            bitmap-text HUD, bold stroke-font signboards
//            (C++ / OpenGL / GLU / freeGLUT)
//
//  Code::Blocks (MinGW) : link libraries in this order
//        freeglut  opengl32  glu32      (Linker settings -> Link libraries)
//
//  CONTROLS
//     Left mouse drag ........ rotate camera      Mouse wheel / + - .. zoom
//     Arrow keys ............. rotate camera      PageUp / PageDown ... zoom
//     1 / 2 / 3 / 4 / 5 ...... Isometric / Top / Terminal / Runway / Baggage
//     F ...................... follow an aircraft (press again = next / off)
//     A ...................... add an arriving flight
//     E ...................... add an emergency arrival (goes to queue front)
//     R / L .................. assign runway + bay / clear for landing
//     D ...................... send a parked aircraft out (pushback + take-off)
//     P ...................... pause / resume       S ... resume
//     M ...................... automatic / manual dispatch
//     T ...................... flight + baggage board on / off
//     [  ] ................... slower / faster simulation
//     N ...................... Day / Night          Right click ... menu
//     ESC .................... exit
// ============================================================================
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <vector>
#include <queue>
#include <algorithm>
#include <cctype>
#include <cstdarg>
#include <initializer_list>

static const float bayX[5] = { -28, -14, 0, 14, 28 };

// --------------------------------------------- shared baggage-system layout
// Used by both the static scene (drawn once into a display list) and the
// simulation.  Service road: west-bound lane just in front of the aircraft
// noses, east-bound lane under the jet bridges, turning loops at both ends.
struct P2 { float x, z; };
static const float PI_F     = 3.14159265f;
static const float LANE_W   = 10.95f;   // west-bound service lane
static const float LANE_E   = 12.35f;   // east-bound service lane
static const float STAND_Z  = 8.2f;     // tug + cart wait here beside the nose
static const float LOADER_Z = 6.45f;    // forward cargo door / belt loader line
static const float DOCK_X   = 44.f;     // tug stops here to unload at the hall
static const float CAR_CX = 41, CAR_CZ = 24.8f, CAR_L = 7, CAR_R = 1.4f;   // carousel
static const float CAR_P  = 2 * CAR_L + 2 * PI_F * CAR_R;               // its length
static const P2 IN_BELT[2]   = { {41, 13.45f}, {41, 22.8f} };                   // dock -> carousel
static const P2 OUT_BELT[4]  = { {46.4f, 24.8f}, {51.5f, 24.8f}, {51.5f, 14}, {61, 14} }; // carousel -> tug rank
static const P2 FEED_BELT[2] = { {32.4f, 24.8f}, {35.6f, 24.8f} };                    // check-in -> carousel

static float polyLen(const P2* p, int n)
{
    float L = 0;
    for (int i = 1; i < n; i++) L += std::hypot(p[i].x - p[i - 1].x, p[i].z - p[i - 1].z);
    return L;
}
// point at distance s along a polyline, with its heading in degrees
static void polyAt(const P2* p, int n, float s, float& x, float& z, float& yaw)
{
    for (int i = 1; i < n; i++) {
        float dx = p[i].x - p[i - 1].x, dz = p[i].z - p[i - 1].z, L = std::hypot(dx, dz);
        if (s <= L || i == n - 1) {
            float t = L > 0 ? std::min(1.f, std::max(0.f, s / L)) : 0.f;
            x = p[i - 1].x + dx * t; z = p[i - 1].z + dz * t;
            yaw = std::atan2(-dz, dx) * 180 / PI_F;
            return;
        }
        s -= L;
    }
    x = p[0].x; z = p[0].z; yaw = 0;
}
// point on the oval carousel: s = distance along the belt, off = outward offset
static void carouselAt(float s, float off, float& x, float& z, float& yaw)
{
    s = std::fmod(s, CAR_P); if (s < 0) s += CAR_P;
    const float W = CAR_CX - CAR_L / 2, E = CAR_CX + CAR_L / 2, arc = PI_F * CAR_R, r = CAR_R + off;
    float t;
    if (s < CAR_L) { x = W + s; z = CAR_CZ - r; yaw = 0; return; }     // south side
    s -= CAR_L;
    if (s < arc) { t = -PI_F / 2 + s / CAR_R; x = E + r * std::cos(t); z = CAR_CZ + r * std::sin(t); }
    else {
        s -= arc;
        if (s < CAR_L) { x = E - s; z = CAR_CZ + r; yaw = 180; return; } // north side
        s -= CAR_L; t = PI_F / 2 + s / CAR_R;
        x = W + r * std::cos(t); z = CAR_CZ + r * std::sin(t);
    }
    yaw = std::atan2(-std::cos(t), -std::sin(t)) * 180 / PI_F;
}
static const float CAR_IN_S   = 41 - (CAR_CX - CAR_L / 2);               // inbound belt joins here
static const float CAR_OUT_S  = CAR_L + PI_F * CAR_R / 2;                 // outbound belt leaves here
static const float CAR_FEED_S = 2 * CAR_L + PI_F * CAR_R * 1.5f;          // check-in feed joins here

#ifndef AIRPORT_HEADLESS
#ifndef NOMINMAX
#define NOMINMAX
#endif
#include <GL/freeglut.h>
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

// ------------------------------------------------------------- sign writing
// The stroke font is only a set of 1-pixel lines, which is why the old signs
// were hard to read.  Each string is now traced 9 times with a small offset
// (a real, world-space stroke width that scales with the sign), with smooth
// lines, so the letters are bold and stay readable from far away.
static void textBegin(float r, float g, float b)
{
    glDisable(GL_LIGHTING);                 // text is self-lit: full brightness
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glEnable(GL_LINE_SMOOTH); glHint(GL_LINE_SMOOTH_HINT, GL_NICEST);
    glLineWidth(1.6f);
    glColor3f(r, g, b);
}
static void textEnd()
{
    glLineWidth(1.f);
    glDisable(GL_LINE_SMOOTH); glDisable(GL_BLEND);
    glEnable(GL_LIGHTING);
}
static void strokeBold(const char* s, float weight)            // weight in font units
{
    static const float o[9][2] = { {0,0},{1,0},{-1,0},{0,1},{0,-1},
                                   {.7f,.7f},{-.7f,.7f},{.7f,-.7f},{-.7f,-.7f} };
    for (int k = 0; k < 9; k++) {
        glPushMatrix();
        glTranslatef(o[k][0] * weight, o[k][1] * weight, 0);
        glutStrokeString(GLUT_STROKE_ROMAN, (const unsigned char*)s);
        glPopMatrix();
    }
}

// text painted on the ground (flat) or standing (not flat), centred on x
static void signText(const char* s, float x, float y, float z, float sc, float rotY,
                     float r, float g, float b, bool flat)
{
    textBegin(r, g, b);
    glPushMatrix();
    glTranslatef(x, y, z);
    glRotatef(rotY, 0, 1, 0);
    if (flat) glRotatef(-90, 1, 0, 0);
    glScalef(sc, sc, sc);
    float len = (float)glutStrokeLength(GLUT_STROKE_ROMAN, (const unsigned char*)s);
    glTranslatef(-len / 2, 0, 0);
    strokeBold(s, 6.f);
    glPopMatrix();
    textEnd();
}

// Framed sign board mounted on a wall.  (cx,cz) = point on the wall face,
// cy = centre height, rotY = direction the board faces (0:+Z 180:-Z 90:+X
// -90:-X), w x h = panel size.  The text is auto-sized to fit the panel and
// centred both ways, so nothing overflows the board any more.
static void signBoard(const char* s, float cx, float cy, float cz, float rotY, float w, float h,
                      float br, float bg, float bb, float tr, float tg, float tb)
{
    const float d = .14f;
    glPushMatrix();
    glTranslatef(cx, cy, cz);
    glRotatef(rotY, 0, 1, 0);
    col(.16f, .17f, .20f); box(0, -h / 2 - .08f, d / 2, w + .16f, h + .16f, d);   // frame
    col(br, bg, bb);       box(0, -h / 2, d + .01f, w, h, .02f);                    // panel
    float len = (float)glutStrokeLength(GLUT_STROKE_ROMAN, (const unsigned char*)s);
    float sc = std::min(h * .58f / 100.f, w * .86f / len);
    textBegin(tr, tg, tb);
    glTranslatef(0, -50 * sc, d + .07f);      // capitals are ~100 units tall
    glScalef(sc, sc, sc);
    glTranslatef(-len / 2, 0, 0);
    strokeBold(s, 5.5f);
    textEnd();
    glPopMatrix();
}
// free-standing sign on two posts
static void postSign(const char* s, float cx, float cy, float cz, float rotY, float w, float h,
                     float br, float bg, float bb, float tr, float tg, float tb)
{
    glPushMatrix();
    glTranslatef(cx, 0, cz); glRotatef(rotY, 0, 1, 0);
    col(.35f, .36f, .40f);
    box(-w * .36f, 0, -.06f, .16f, cy, .16f); box(w * .36f, 0, -.06f, .16f, cy, .16f);
    glPopMatrix();
    signBoard(s, cx, cy, cz, rotY, w, h, br, bg, bb, tr, tg, tb);
}
// sign colour schemes
#define NAVY_WHITE   .07f, .16f, .42f,  1, 1, 1
#define BLACK_YELLOW .06f, .06f, .07f,  1, .84f, .12f
#define YELLOW_BLACK .98f, .80f, .10f,  .03f, .03f, .03f
#define GREEN_WHITE  .04f, .40f, .22f,  1, 1, 1
#define BLUE_WHITE   .05f, .30f, .70f,  1, 1, 1
#define RED_WHITE    .72f, .08f, .08f,  1, 1, 1

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
    signText(leftNo,  -74, 0.12f, cz, 0.05f, -90, 1, 1, 1, true);
    signText(rightNo,  74, 0.12f, cz, 0.05f,  90, 1, 1, 1, true);
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
static void drawApron()
{
    col(0.58f, 0.58f, 0.60f);
    rect(-45, -12, 45, 14.8f, 0.03f);

    // baggage service road (one way: west-bound in front of the noses,
    // east-bound under the jet bridges, turning loop at the west end)
    col(0.45f, 0.45f, 0.48f);
    rect(-44.2f, 10.25f, 45, 13.05f, 0.05f);
    rect(-44.2f, 9.3f, -38, 14.0f, 0.05f);
    col(0.95f, 0.95f, 0.95f);
    rect(-38, 10.22f, 45, 10.32f, 0.08f);
    rect(-38, 12.98f, 45, 13.08f, 0.08f);
    dashX(-38, 66, (LANE_W + LANE_E) / 2, 0.14f, 2.0f, 1.6f, 0.08f);

    col(0.95f, 0.80f, 0.10f);                                   // stand lines
    for (int i = -3; i <= 3; i++) {
        float x = i * 14.f;
        if (i == 0) continue;
        rect(x - 0.1f, -11, x + 0.1f, 13, 0.08f);
    }
    for (int i = 0; i < 5; i++) {
        rect(bayX[i] - 0.15f, -11, bayX[i] + 0.15f, 13, 0.08f);       // lead-in line
        rect(bayX[i] - 1.5f, 10.0f, bayX[i] + 1.5f, 10.3f, 0.08f);    // stop bar
        char n[2] = { char('1' + i), 0 };                             // stand number
        signText(n, bayX[i] + 4.0f, 0.1f, -9.0f, 0.03f, 180, 0.95f, 0.80f, 0.10f, true);
    }
}

// ---------------------------------------------------------------- terminal
static void drawTrolley(float x, float z, float yaw, bool loaded, int seed)
{
    glPushMatrix(); glTranslatef(x, 0, z); glRotatef(yaw, 0, 1, 0);
    col(0.72f, 0.74f, 0.78f);
    box(0, 0.22f, 0, 1.1f, 0.06f, 0.62f);                                   // deck
    box(-0.52f, 0.22f, 0.27f, 0.05f, 0.85f, 0.05f); box(-0.52f, 0.22f, -0.27f, 0.05f, 0.85f, 0.05f);
    col(0.85f, 0.15f, 0.15f); box(-0.52f, 1.05f, 0, 0.07f, 0.07f, 0.62f);  // handle
    col(0.05f, 0.05f, 0.05f);
    box(0.4f, 0, 0.25f, 0.14f, 0.22f, 0.07f); box(0.4f, 0, -0.25f, 0.14f, 0.22f, 0.07f);
    box(-0.4f, 0, 0.25f, 0.14f, 0.22f, 0.07f); box(-0.4f, 0, -0.25f, 0.14f, 0.22f, 0.07f);
    if (loaded) {
        col(0.25f + 0.6f * rnd(seed), 0.2f + 0.4f * rnd(seed + 1), 0.2f + 0.5f * rnd(seed + 2));
        box(0.05f, 0.28f, 0, 0.85f, 0.38f, 0.52f);
        col(0.15f + 0.6f * rnd(seed + 3), 0.15f + 0.5f * rnd(seed + 4), 0.2f + 0.6f * rnd(seed + 5));
        box(0.05f, 0.66f, 0, 0.62f, 0.3f, 0.42f);
    }
    glPopMatrix();
}

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

    // landside name board, above the canopy
    signBoard("SHAH AMANAT INTERNATIONAL AIRPORT", 0, 8.1f, 29.0f, 0, 54, 2.6f, NAVY_WHITE);

    col(0.28f, 0.28f, 0.33f);  box(0, 5.9f, 31.5f, 56, 0.35f, 5);    // canopy
    col(0.95f, 0.95f, 0.95f);
    for (float x = -26; x <= 26.1f; x += 13) cyl(x, 0, 33.8f, 0.3f, 5.9f, 10);

    // departures / arrivals boards hanging under the canopy edge
    col(0.30f, 0.30f, 0.34f);
    for (int s = -1; s <= 1; s += 2)
        for (int k = -1; k <= 1; k += 2) box(s * 19.5f + k * 3.6f, 5.8f, 33.82f, 0.08f, 0.12f, 0.08f);
    signBoard("DEPARTURES", -19.5f, 5.25f, 33.75f, 0, 8.5f, 1.05f, BLACK_YELLOW);
    signBoard("ARRIVALS",    19.5f, 5.25f, 33.75f, 0, 8.5f, 1.05f, BLACK_YELLOW);

    // passenger luggage trolleys on the kerb
    for (int i = 0; i < 5; i++) drawTrolley(-24.5f + i * 0.42f, 30.2f, 180, false, 0);
    for (int i = 0; i < 3; i++) drawTrolley(15.5f + i * 3.2f, 30.1f, 170 + 15 * i, true, 11 * i + 3);

    // jet bridges: raised tunnels so the baggage tugs can drive underneath
    for (int i = 0; i < 5; i++) {
        col(0.78f, 0.80f, 0.84f);
        box(bayX[i], 1.95f, 12.15f, 2.4f, 2.0f, 3.3f);              // tunnel
        cyl(bayX[i], 0, 14.8f, 1.5f, 4.0f, 14);                     // rotunda
        col(0.55f, 0.57f, 0.62f);
        box(bayX[i], 1.95f, 10.75f, 2.8f, 2.2f, 0.5f);              // cab
        col(0.20f, 0.28f, 0.38f);                                   // tunnel windows
        box(bayX[i] - 1.21f, 2.9f, 12.15f, 0.02f, 0.5f, 2.8f);
        box(bayX[i] + 1.21f, 2.9f, 12.15f, 0.02f, 0.5f, 2.8f);

        char g[8]; std::snprintf(g, sizeof(g), "GATE %d", i + 1);  // airside gate sign
        signBoard(g, bayX[i], 7.35f, 15.0f, 180, 6.4f, 1.5f, YELLOW_BLACK);
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
    signBoard("CONTROL TOWER", tx, 1.75f, tz + 3.5f, 0,   6.2f, 1.2f, BLACK_YELLOW);
    signBoard("ATC TOWER",     tx + 3.5f, 1.75f, tz, 90,  6.2f, 1.2f, BLACK_YELLOW);
    signBoard("ATC TOWER",     tx - 3.5f, 1.75f, tz, -90, 6.2f, 1.2f, BLACK_YELLOW);
}

// ----------------------------------------------------- baggage handling hall
// conveyor frame + belt along a polyline (belt surface at y = 0.78)
static void beltFrame(const P2* p, int n)
{
    for (int i = 1; i < n; i++) {
        float dx = p[i].x - p[i - 1].x, dz = p[i].z - p[i - 1].z, L = std::hypot(dx, dz);
        glPushMatrix();
        glTranslatef((p[i].x + p[i - 1].x) / 2, 0, (p[i].z + p[i - 1].z) / 2);
        glRotatef(std::atan2(-dz, dx) * 180 / PI_F, 0, 1, 0);
        col(0.55f, 0.56f, 0.60f); box(0, 0.50f, 0, L + 0.4f, 0.22f, 0.95f);   // side frame
        col(0.12f, 0.12f, 0.14f); box(0, 0.72f, 0, L + 0.4f, 0.06f, 0.75f);   // rubber belt
        col(0.40f, 0.40f, 0.44f);
        for (float u = -L / 2; u <= L / 2 + 0.01f; u += 2.f) {                // legs
            box(u, 0, 0.38f, 0.12f, 0.5f, 0.12f); box(u, 0, -0.38f, 0.12f, 0.5f, 0.12f);
        }
        glPopMatrix();
    }
}

// ------------------------------------------- baggage screening + sorting yard
// Real-airport style baggage flow, drawn once into the static scene:
//   departures : check-in -> X-ray screening tunnel -> carousel -> outbound belt
//                -> barcode scanner gantry -> sorter with one chute per gate
//                -> gate cages (make-up area) -> belt end / baggage tug
//   arrivals   : tug -> inbound belt -> arrival scanner -> flap curtain -> claim carousel
static const float SORT_PAL[8][3] = {{.75f,.12f,.12f},{.12f,.22f,.55f},{.10f,.10f,.11f},{.15f,.50f,.22f},
                                     {.92f,.52f,.08f},{.48f,.20f,.58f},{.60f,.60f,.63f},{.08f,.52f,.56f}};
static const float CHUTE_Z[5]  = { 22.6f, 20.9f, 19.2f, 17.5f, 15.8f };   // chute i serves gate i+1
static const float CHUTE_X0 = 52.4f, CHUTE_X1 = 55.0f, CAGE_X = 56.2f, PLATE_X = 57.55f;
static const float SCAN_Z = 23.6f;       // outbound barcode scanner gantry (belt x = 51.5)
static const float IN_SCAN_Z = 17.5f;    // arrival scanner gantry (belt x = 41)
static const float TUNNEL_X = 34.0f;     // X-ray tunnel on the check-in feed belt (z = 24.8)

static void sortBag(float x, float y, float z, float yaw, int c)
{
    glPushMatrix(); glTranslatef(x, y, z); glRotatef(yaw, 0, 1, 0);
    col(SORT_PAL[c][0], SORT_PAL[c][1], SORT_PAL[c][2]); box(0, 0, 0, .5f, .32f, .4f);
    col(.08f, .08f, .08f); box(0, .32f, 0, .18f, .05f, .06f);
    glPopMatrix();
}

// arch over a belt that runs along Z
static void scanGantry(float cx, float cz)
{
    col(.30f, .32f, .38f);
    box(cx - .95f, 0, cz, .2f, 2.0f, .3f); box(cx + .95f, 0, cz, .2f, 2.0f, .3f);
    box(cx, 1.85f, cz, 2.1f, .35f, .45f);
    col(.95f, .80f, .10f);                                    // hazard band
    for (int k = -4; k <= 4; k += 2) box(cx + k * .2f, 1.66f, cz + .226f, .2f, .12f, .02f);
    col(.05f, .05f, .06f);                                    // camera heads
    box(cx - .62f, 1.55f, cz, .22f, .22f, .32f); box(cx + .62f, 1.55f, cz, .22f, .22f, .32f);
    box(cx, 1.55f, cz, .22f, .22f, .32f);
}

static void flapCurtainZ(float cx, float cz, float w)         // rubber flaps across a Z-running belt
{
    col(.06f, .06f, .07f);
    for (float x = cx - w / 2 + .1f; x < cx + w / 2; x += .2f) box(x, .8f, cz, .15f, .7f, .04f);
}
static void flapCurtainX(float cx, float cz, float w)         // rubber flaps across an X-running belt
{
    col(.06f, .06f, .07f);
    for (float z = cz - w / 2 + .1f; z < cz + w / 2; z += .2f) box(cx, .8f, z, .04f, .7f, .15f);
}

static void drawSortingYard()
{
    // ---- check-in feed: X-ray screening tunnel with operator desk
    col(.22f, .30f, .45f);
    box(TUNNEL_X, .78f, 24.8f - .62f, 2.0f, .95f, .12f); box(TUNNEL_X, .78f, 24.8f + .62f, 2.0f, .95f, .12f);
    col(.30f, .38f, .55f); box(TUNNEL_X, 1.73f, 24.8f, 2.2f, .30f, 1.5f);
    col(.85f, .86f, .90f); box(TUNNEL_X, 2.03f, 24.8f, 1.2f, .10f, .8f);
    flapCurtainX(TUNNEL_X - 1.0f, 24.8f, 1.1f); flapCurtainX(TUNNEL_X + 1.0f, 24.8f, 1.1f);
    col(.40f, .40f, .44f);
    box(TUNNEL_X, 0, 26.1f, 1.8f, .75f, .6f);                  // operator desk
    col(.06f, .08f, .12f); box(TUNNEL_X - .4f, .75f, 26.1f, .55f, .38f, .06f);
    col(.25f, .85f, .95f); box(TUNNEL_X - .4f, .82f, 26.06f, .45f, .26f, .01f);   // x-ray image
    col(.20f, .20f, .24f); box(TUNNEL_X + .5f, 0, 26.75f, .5f, .45f, .5f);        // stool
    col(.35f, .36f, .40f);
    box(TUNNEL_X - 1.8f, 0, 27.2f, .1f, 2.8f, .1f); box(TUNNEL_X + 1.8f, 0, 27.2f, .1f, 2.8f, .1f);
    signBoard("BAGGAGE SCREENING", TUNNEL_X, 2.9f, 27.2f, 0, 4.0f, .8f, RED_WHITE);
    signBoard("FROM CHECK-IN", 32.35f, 2.9f, 24.8f, 90, 3.4f, .8f, GREEN_WHITE);

    // ---- carousel: hazard edge and flap curtains where belts join
    glBegin(GL_QUADS); glNormal3f(0, 1, 0);
    const int N = 84;
    for (int i = 0; i < N; i++) {
        float s0 = CAR_P * i / N, s1 = CAR_P * (i + 1) / N, yw, ax, az, bx, bz, cx, cz, dx, dz;
        carouselAt(s0, .50f, ax, az, yw); carouselAt(s1, .50f, bx, bz, yw);
        carouselAt(s1, .34f, cx, cz, yw); carouselAt(s0, .34f, dx, dz, yw);
        if ((i / 2) % 2) col(.95f, .80f, .10f); else col(.05f, .05f, .06f);
        glVertex3f(ax, .775f, az); glVertex3f(bx, .775f, bz); glVertex3f(cx, .775f, cz); glVertex3f(dx, .775f, dz);
    }
    glEnd();
    flapCurtainZ(41, 23.05f, 1.4f);                            // inbound belt -> carousel
    flapCurtainX(46.1f, 24.8f, 1.2f);                          // carousel -> outbound belt
    col(.95f, .80f, .10f);                                     // walkway lines along the inbound belt
    rect(39.8f, 15.2f, 39.95f, 22.2f, .075f); rect(42.05f, 15.2f, 42.2f, 22.2f, .075f);

    // ---- arrival scanner on the inbound belt
    scanGantry(41, IN_SCAN_Z);
    signBoard("ARRIVAL BAG SCAN", 41, 2.7f, IN_SCAN_Z + .24f, 0, 2.6f, .6f, NAVY_WHITE);

    // ---- claim hall: luggage trolleys
    drawTrolley(35.6f, 27.4f, 0, false, 0);  drawTrolley(36.9f, 27.5f, 8, false, 0);
    drawTrolley(46.3f, 27.5f, 190, true, 21); drawTrolley(47.8f, 27.4f, 175, false, 0);

    // ---- sorting yard east of the hall
    col(.50f, .51f, .55f); rect(51.9f, 14.9f, 58, 25.4f, .045f);
    col(.95f, .80f, .10f);
    rect(51.9f, 14.9f, 58, 15.05f, .06f); rect(51.9f, 25.25f, 58, 25.4f, .06f);
    scanGantry(51.5f, SCAN_Z);                                  // barcode / RFID tag reader
    col(.35f, .36f, .40f);
    box(51.5f, 2.03f, SCAN_Z - .55f, .1f, .3f, .1f); box(51.5f, 2.03f, SCAN_Z + .55f, .1f, .3f, .1f);

    for (int i = 0; i < 5; i++) {
        const float z = CHUTE_Z[i], mid = (CHUTE_X0 + CHUTE_X1) / 2, len = CHUTE_X1 - CHUTE_X0;
        // diverter housing on the belt side
        col(.95f, .80f, .10f); box(51.0f, 0, z + .5f, .22f, 1.0f, .3f);
        col(.20f, .20f, .22f); box(51.0f, .6f, z + .5f, .24f, .08f, .32f);
        // roller chute
        col(.82f, .83f, .87f);
        box(mid, .50f, z - .6f, len, .24f, .06f); box(mid, .50f, z + .6f, len, .24f, .06f);
        col(.18f, .18f, .20f); box(mid, .46f, z, len, .06f, 1.1f);
        col(.62f, .64f, .68f);
        for (float x = CHUTE_X0 + .15f; x < CHUTE_X1; x += .3f) box(x, .50f, z, .12f, .12f, 1.05f);
        col(.40f, .40f, .44f);
        box(CHUTE_X0 + .1f, 0, z - .55f, .08f, .5f, .08f); box(CHUTE_X0 + .1f, 0, z + .55f, .08f, .5f, .08f);
        box(CHUTE_X1 - .1f, 0, z - .55f, .08f, .5f, .08f); box(CHUTE_X1 - .1f, 0, z + .55f, .08f, .5f, .08f);
        // gate cage (make-up trolley) with sorted bags
        col(.25f, .27f, .32f); box(CAGE_X, .16f, z, 1.5f, .1f, 1.0f);
        col(.05f, .05f, .06f);
        for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2) box(CAGE_X + a * .6f, 0, z + b * .4f, .12f, .16f, .12f);
        static const int fill[5] = { 5, 8, 3, 10, 6 };
        for (int k = 0; k < fill[i]; k++) {
            float bx = CAGE_X + ((k % 6) % 3 - 1) * .5f, bz = z + (((k % 6) / 3) ? .24f : -.24f);
            sortBag(bx, .22f + (k / 6) * .34f, bz, 0, (k + i * 3) % 8);
        }
        col(.72f, .74f, .78f);
        for (int a = -1; a <= 1; a += 2) for (int b = -1; b <= 1; b += 2) box(CAGE_X + a * .72f, .2f, z + b * .46f, .05f, 1.2f, .05f);
        box(CAGE_X, 1.38f, z - .46f, 1.5f, .05f, .05f); box(CAGE_X, 1.38f, z + .46f, 1.5f, .05f, .05f);
        box(CAGE_X - .72f, 1.38f, z, .05f, .05f, .92f); box(CAGE_X + .72f, 1.38f, z, .05f, .05f, .92f);
        box(CAGE_X, .80f, z + .46f, 1.5f, .04f, .04f);
        // gate plate on a post
        col(.35f, .36f, .40f); box(PLATE_X - .1f, 0, z, .1f, 2.5f, .1f);
        char g[12]; std::snprintf(g, sizeof(g), "GATE %d", i + 1);
        signBoard(g, PLATE_X, 2.9f, z, 90, 1.45f, .7f, YELLOW_BLACK);
    }

    // canopy over the sorting yard (columns, beams, header sign)
    col(.72f, .74f, .78f);
    box(52.2f, 0, 14.95f, .25f, 4.6f, .25f); box(52.2f, 0, 25.2f, .25f, 4.6f, .25f);
    box(57.7f, 0, 14.95f, .25f, 4.6f, .25f); box(57.7f, 0, 25.2f, .25f, 4.6f, .25f);
    col(.36f, .38f, .44f);
    for (int k = 0; k < 5; k++) box(54.95f, 4.55f, 14.95f + k * 2.56f, 5.9f, .15f, .2f);
    box(52.2f, 4.55f, 20.07f, .2f, .15f, 10.4f); box(57.7f, 4.55f, 20.07f, .2f, .15f, 10.4f);
    col(.70f, .72f, .76f); box(54.95f, 3.85f, 25.3f, 6.0f, .9f, .15f);
    signBoard("BAGGAGE SORTING", 54.95f, 4.3f, 25.38f, 0, 5.6f, .75f, NAVY_WHITE);
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE);
    glColor4f(.65f, .82f, .95f, .22f); rect(52.1f, 14.95f, 57.8f, 25.2f, 4.7f);
    glDepthMask(GL_TRUE); glDisable(GL_BLEND);
}

static void drawBaggageZone()
{
    const float x0 = 32, x1 = 50, z0 = 15, z1 = 29, mx = (x0 + x1) / 2, mz = (z0 + z1) / 2;
    col(0.60f, 0.61f, 0.64f);  rect(x0, z0, x1, z1, 0.06f);                 // hall floor
    col(0.62f, 0.62f, 0.64f);  rect(50.8f, 14.0f, 58, 25.4f, 0.032f);       // under outbound belt

    // open-sided hall: corner columns, roof, fascia band, low landside wall
    col(0.72f, 0.74f, 0.78f);
    for (int i = 0; i < 2; i++) for (int k = 0; k < 2; k++)
        box(i ? x1 - 0.35f : x0 + 0.35f, 0, k ? z1 - 0.35f : z0 + 0.35f, 0.7f, 6, 0.7f);
    box(x0 + 0.15f, 0, mz, 0.3f, 4.3f, z1 - z0);                            // wall to terminal
    box(mx, 0, z1 - 0.15f, x1 - x0, 0.9f, 0.3f);                            // landside wall
    col(0.36f, 0.38f, 0.44f);                                               // roof around a skylight
    box(mx, 6, 17.5f, 19, 0.5f, 6);     box(mx, 6, 28.75f, 19, 0.5f, 1.5f);
    box(33, 6, 24.25f, 3, 0.5f, 7.5f);  box(49, 6, 24.25f, 3, 0.5f, 7.5f);
    col(0.70f, 0.72f, 0.76f);
    box(mx, 4.6f, z0 + 0.15f, x1 - x0, 1.4f, 0.3f);
    box(mx, 4.6f, z1 - 0.15f, x1 - x0, 1.4f, 0.3f);
    box(x1 - 0.15f, 4.6f, mz, 0.3f, 1.4f, z1 - z0);
    col(0.55f, 0.57f, 0.62f);                                               // skylight glazing bars
    for (float gx = 37.75f; gx < 47; gx += 3.25f) box(gx, 6.1f, 24.25f, 0.12f, 0.3f, 7.5f);

    // conveyors: inbound (dock -> carousel), outbound (carousel -> tug rank),
    // check-in feed (terminal -> carousel)
    beltFrame(IN_BELT, 2); beltFrame(OUT_BELT, 4); beltFrame(FEED_BELT, 2);

    // oval claim / sorting carousel
    const int N = 84;
    glBegin(GL_QUADS);
    for (int i = 0; i < N; i++) {
        float s0 = CAR_P * i / N, s1 = CAR_P * (i + 1) / N, yw;
        float ax, az, bx, bz, cx, cz, dx, dz;
        carouselAt(s0, 0.5f, ax, az, yw);  carouselAt(s1, 0.5f, bx, bz, yw);
        carouselAt(s1, -0.5f, cx, cz, yw); carouselAt(s0, -0.5f, dx, dz, yw);
        col(0.30f, 0.31f, 0.35f); glNormal3f(0, 1, 0);                      // plates
        glVertex3f(ax, 0.76f, az); glVertex3f(bx, 0.76f, bz); glVertex3f(cx, 0.76f, cz); glVertex3f(dx, 0.76f, dz);
        float nx = ax - dx, nz = az - dz, nl = std::hypot(nx, nz);
        col(0.78f, 0.79f, 0.83f); glNormal3f(nx / nl, 0, nz / nl);          // steel skirt
        glVertex3f(ax, 0.06f, az); glVertex3f(bx, 0.06f, bz); glVertex3f(bx, 0.76f, bz); glVertex3f(ax, 0.76f, az);
    }
    glEnd();
    col(0.66f, 0.68f, 0.72f);                                               // centre island
    box(CAR_CX, 0, CAR_CZ, CAR_L, 0.95f, 1.8f);
    cyl(CAR_CX - CAR_L / 2, 0, CAR_CZ, 0.9f, 0.95f, 16);
    cyl(CAR_CX + CAR_L / 2, 0, CAR_CZ, 0.9f, 0.95f, 16);

    // signs: landside, apron side, east side and a double-sided claim sign
    signBoard("BAGGAGE HANDLING", mx, 5.3f, z1, 0, 16.4f, 1.15f, NAVY_WHITE);
    signBoard("BAGGAGE HALL",     mx, 5.3f, z0, 180, 12, 1.15f, NAVY_WHITE);
    signBoard("BAGGAGE",          x1, 5.3f, mz, 90, 10, 1.15f, NAVY_WHITE);
    col(0.30f, 0.30f, 0.33f);
    box(CAR_CX - 2.6f, 3.95f, CAR_CZ, 0.06f, 2.05f, 0.06f);
    box(CAR_CX + 2.6f, 3.95f, CAR_CZ, 0.06f, 2.05f, 0.06f);
    signBoard("BAGGAGE CLAIM 1", CAR_CX, 3.55f, CAR_CZ + 0.02f, 0,   6.6f, 0.85f, BLACK_YELLOW);
    signBoard("BAGGAGE CLAIM 1", CAR_CX, 3.55f, CAR_CZ - 0.02f, 180, 6.6f, 0.85f, BLACK_YELLOW);

    drawSortingYard();

    // glass skylight: see-through roof over the carousel
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA); glDepthMask(GL_FALSE);
    glColor4f(0.65f, 0.82f, 0.95f, 0.30f); rect(34.5f, 20.5f, 47.5f, 28, 6.4f);
    glDepthMask(GL_TRUE); glDisable(GL_BLEND);
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
    signBoard("PASSENGER WAITING AREA", -42, 5.2f, 30.25f, 0, 19.5f, 1.1f, GREEN_WHITE);
    for (int r = 0; r < 3; r++) { bench(-46, 18.f + 4 * r); bench(-38, 18.f + 4 * r); }
    drawTrolley(-34.3f, 25.2f, 90, true, 41); drawTrolley(-49.6f, 19.5f, -90, true, 57);
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
    signText("PARKING", 0, 0.12f, 49.2f, 0.024f, 0, 1, 1, 1, true);
    postSign("CAR PARK", 0, 3.4f, 57.0f, 0, 8.5f, 1.5f, BLUE_WHITE);
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

static void drawStaticScene()
{
    drawGround(); drawRunwaysAndTaxiways(); drawApron(); drawTerminal();
    drawTower(); drawBaggageZone(); drawWaitingArea(); drawRoadsAndParking();

    // service zone
    col(.36f,.36f,.38f); rect(58,8,100,50,.03f);
    col(.95f,.80f,.10f);
    rect(58,8,100,8.3f,.08f); rect(58,49.7f,100,50,.08f); rect(99.7f,8,100,50,.08f);
    signText("SERVICE ZONE",86,.12f,13.5f,.02f,0,1,1,1,true);
    col(.62f,.64f,.68f); box(86,0,41,24,7,14);
    col(.35f,.38f,.45f); box(86,7,41,25,.6f,15);
    col(.20f,.22f,.26f);
    for(int i=-1;i<=1;++i) box(86+i*7.5f,0,33.95f,6,5,.2f);
    signBoard("GROUND SERVICES",86,6.0f,34.0f,180,13,1.2f,NAVY_WHITE);
    signBoard("GROUND SERVICES",86,5.4f,48.0f,0,15,1.5f,NAVY_WHITE);
    for(int i=0;i<2;++i) {
        col(.8f,.8f,.82f); cyl(64,0,38+7*i,3,6,24);
        col(.85f,.15f,.15f); cyl(64,3,38+7*i,3.05f,.8f,24);
    }
    postSign("FUEL FARM",64,2.4f,49.4f,0,6,1.1f,RED_WHITE);
    drawBus(68,23,0); drawBus(80,23,0); drawBus(92,30,180);
    drawFuelTruck(92,20,0);
    for(int i=0;i<3;++i) drawBaggageCart(68+10*i,27,0);           // spare carts

    const float colors[][3]={{.8f,.1f,.1f},{.1f,.3f,.8f},{.9f,.9f,.9f},
                              {.1f,.1f,.1f},{.9f,.7f,.1f},{.2f,.6f,.3f}};
    for(int row=0;row<2;++row) for(int i=0;i<20;++i) {
        int c=int(rnd(i*7+row*13+3)*10);
        if(c<6) drawCar(-38+4*i,row?52.8f:43.2f,90,colors[c][0],colors[c][1],colors[c][2]);
    }
    for(size_t i=0;i<lamps.size();++i) {
        col(.3f,.3f,.33f); cyl(lamps[i].x,0,lamps[i].z,.18f,lamps[i].h,8);
    }
    int seed=0;
    for(int z=-68;z<=72;z+=7) for(int x=-112;x<=112;x+=12) {
        if(z!=-68 && z!=72 && (z<-10 || z>54 || (x>-58 && x<104))) continue;
        float tx=x+3*rnd(++seed), tz=z+2*rnd(++seed), s=.9f+.5f*rnd(++seed);
        col(.4f,.26f,.13f); cyl(tx,0,tz,.35f*s,2*s,8);
        col(.12f,.45f,.16f); cone(tx,1.5f*s,tz,1.7f*s,3.5f*s,10);
        cone(tx,3.2f*s,tz,1.3f*s,2.8f*s,10);
    }
}
#endif

// ============================================================================
// Simulation core. All durations below are measured in simulation seconds.
static const float RAD = PI_F/180.f, RW1=-55, RW2=-35, STAND=4;
enum State { APPROACH, ROLLOUT, TAXI_IN, PARKED, PUSHBACK, TAXI_OUT, TAKEOFF, CLIMB };
struct Request {
    int id;
    bool emergency;
    double entered;
};
struct LowerPriority {
    bool operator()(const Request& a,const Request& b) const {
        if(a.emergency!=b.emergency) return !a.emergency;
        if(a.entered!=b.entered) return a.entered>b.entered;
        return a.id>b.id; // stable order even when paused at the same timestamp
    }
};
typedef std::priority_queue<Request,std::vector<Request>,LowerPriority> ArrivalQueue;
enum Action { PASS, CLAIM_R2, CLEAR_R1, CLEAR_R2, AT_GATE, LINE_UP };
struct Waypoint {float x,z; Action action;};
enum BagPhase { BAG_INFLIGHT, BAG_UNLOAD_WAIT, BAG_UNLOADING, BAG_LOAD_WAIT, BAG_LOADING, BAG_READY };
struct Aircraft {
    int id,bay;
    bool emergency=false, force=false, finished=false;
    State state=PARKED;
    float x=0,y=0,z=STAND,yaw=-90,pitch=0,speed=0,elapsed=0;
    double readyAt=0;
    std::vector<Waypoint> path;
    size_t target=0;
    BagPhase bagPhase=BAG_READY;
    int bagsIn=0, bagsOut=0, bagsOutTarget=0;   // inbound in hold, outbound loaded / required
};
static ArrivalQueue waiting;
static Request assigned={-1,false,0};
static std::vector<Aircraft> fleet;
static int nextId=101, bayOwner[5]={-1,-1,-1,-1,-1}, assignedBay=-1;
static int r1Owner=-1,r2Owner=-1,surfaceOwner=-1;
static int landed=0,departed=0,cleared=0;
static double simTime=0,totalWait=0,maxWait=0,nextAuto=3;
static bool autoMode=true, paused=false;
#ifndef AIRPORT_HEADLESS
static bool showBoard=true;
static float simSpeed=1;
#endif
static char notice[128]="Automatic traffic enabled.";

// ---------------------------------------------------------- baggage state
static const int   NTUG = 3, CART_CAP = 20;
static const float TUG_HALF = 1.2f, CART_HALF = 1.5f, CART_GAP = 0.3f;
static const float rankX[NTUG] = { 64, 57, 50 };          // tug rank, front first
static const float XFER_T = .42f, DROP_T = .30f, OUT_T = .35f, FEED_T = 2.2f;
static const float IN_SPEED = 2.4f, OUT_SPEED = 3.0f, FEED_SPEED = 1.6f, CAR_SPEED = 1.8f;
enum TugState { T_RANK, T_PICKUP, T_TO_STAND, T_AT_STAND, T_TO_DOCK, T_DROP };
struct Tug {
    int id=0; TugState state=T_RANK;
    float x=0, z=LANE_E, yaw=0, speed=0;            // tug
    float cx=0, cz=LANE_E, cyaw=0;                  // towed cart
    int bags=0, cart[CART_CAP]={0};                 // bags on the cart + their colours
    int flight=-1, bay=-1, need=0, requested=0, trip=0;
    bool load=false;
    float timer=0, xfer=0, blocked=0;
    std::vector<P2> path; size_t target=0;
};
struct BagJob { int flight; bool load, emergency; double t; };
struct BeltBag { float s; int color, tug; };
struct CarouselBag { float s; int color; bool inbound; float life; };
static Tug tugs[NTUG];
static std::vector<int> tugRank;                     // idle tugs at the hall, front first
static std::vector<BagJob> bagJobs;
static std::vector<BeltBag> inBags, outBags, feedBags;
static std::vector<CarouselBag> carBags;
static float loaderExt[5] = {0,0,0,0,0}, feedTimer = 0;
static int bagsUnloaded=0, bagsLoaded=0, bagsClaimed=0, bagSeed=7;

static float hash01(int i) { double v=std::sin(i*12.9898+.73)*43758.5453; return float(v-std::floor(v)); }
static int bagColor(int seed) { return int(hash01(seed)*8)%8; }
static int inboundColor(const Tug& t) { return bagColor(t.id*7919+t.trip*131+t.bags); }
static Aircraft* findAircraft(int id) {
    for(Aircraft& a:fleet) if(a.id==id) return &a;
    return 0;
}
static void queueBagJob(const Aircraft& a,bool load) {
    BagJob j={a.id,load,a.emergency,simTime}; bagJobs.push_back(j);
}

static void report(const char* format, ...) {
    va_list args; va_start(args,format);
    vsnprintf(notice,sizeof(notice),format,args); va_end(args);
}
static int freeBay() {
    for(int b=0;b<5;++b) if(bayOwner[b]<0) return b;
    return -1;
}
static void addFlight(bool emergency) {
    Request r={nextId++,emergency,simTime};
    waiting.push(r);
    report("BG%d queued (%s).",r.id,emergency?"emergency":"routine");
}
static bool assignRunway() {
    if(assigned.id>=0) {report("BG%d already assigned. Press L to clear.",assigned.id); return false;}
    if(waiting.empty()) {report("Arrival queue empty."); return false;}
    if(r1Owner>=0) {report("Runway 1 occupied by BG%d.",r1Owner); return false;}
    int b=freeBay();
    if(b<0) {report("All bays reserved. A departure will free a bay."); return false;}
    assigned=waiting.top();
    waiting.pop();
    assignedBay=b; bayOwner[b]=assigned.id; r1Owner=assigned.id;
    report("BG%d assigned runway 1 and bay %d. L clears landing.",assigned.id,b+1);
    return true;
}
static void clearLanding() {
    if(assigned.id<0) {report("Assign a flight with R first."); return;}
    Aircraft a; a.id=assigned.id; a.bay=assignedBay; a.emergency=assigned.emergency;
    a.state=APPROACH; a.x=-240; a.y=24; a.z=RW1; a.yaw=0; a.pitch=-8; a.speed=55;
    a.bagsIn=12+(a.id*7)%9; a.bagsOutTarget=10+(a.id*5)%11; a.bagPhase=BAG_INFLIGHT;
    fleet.push_back(a);
    double delay=simTime-assigned.entered;
    ++cleared; totalWait+=delay; maxWait=std::max(maxWait,delay);
    report("BG%d cleared to land. Wait %.1f s.",a.id,delay);
    assigned.id=-1; assignedBay=-1;
}
static void initFleet() {
    for(int i=0;i<3;++i) {
        Aircraft a; a.id=nextId++; a.bay=i*2; a.x=bayX[a.bay]; a.readyAt=9+i*9;
        a.bagsOutTarget=10+(a.id*5)%11;
        if(i==0) {a.bagsOut=a.bagsOutTarget; a.bagPhase=BAG_READY;}   // already loaded
        else a.bagPhase=BAG_LOAD_WAIT;
        bayOwner[a.bay]=a.id; fleet.push_back(a);
        if(i) queueBagJob(a,true);
    }
    for(int t=0;t<NTUG;++t) {
        Tug& g=tugs[t]; g.id=t; g.x=rankX[t]; g.z=LANE_E; g.yaw=0;
        g.cx=g.x-(TUG_HALF+CART_GAP+CART_HALF); g.cz=LANE_E; g.cyaw=0;
        tugRank.push_back(t);
    }
}
// D = pushback request.  An aircraft whose bags are loaded leaves at once;
// otherwise it is marked and pushes back as soon as baggage handling is done.
static void forceDepart() {
    static const BagPhase order[5]={BAG_READY,BAG_LOADING,BAG_LOAD_WAIT,BAG_UNLOADING,BAG_UNLOAD_WAIT};
    for(int k=0;k<5;++k) for(Aircraft& a:fleet) {
        if(a.state!=PARKED || a.force || a.bagPhase!=order[k]) continue;
        a.force=true;
        if(k==0) report("BG%d requested pushback.",a.id);
        else report("BG%d will push back when baggage handling is complete.",a.id);
        return;
    }
    report("No parked aircraft available for departure.");
}
static float angleDiff(float angle) {
    while(angle>180) angle-=360;
    while(angle<-180) angle+=360;
    return angle;
}
static void steer(Aircraft& a,float heading,float dt) {
    float d=angleDiff(heading-a.yaw), rate=85*dt;
    a.yaw=angleDiff(a.yaw+std::max(-rate,std::min(rate,d)));
}
static void taxi(Aircraft& a,float dt) {
    const Waypoint w=a.path[a.target];
    float dx=w.x-a.x,dz=w.z-a.z,dist=std::hypot(dx,dz);
    if(dist>.001f) {
        float heading=std::atan2(-dz,dx)/RAD;
        steer(a,heading,dt);
        float limit=std::fabs(angleDiff(heading-a.yaw))>22?2.5f:8.f;
        a.speed=std::min(limit,a.speed+6*dt);
        float travel=std::min(dist,a.speed*dt);
        a.x+=dx*(travel/dist); a.z+=dz*(travel/dist);
        if(travel<dist) return;
    }
    a.x=w.x; a.z=w.z;
    switch(w.action) {
    case CLAIM_R2:
        if(r2Owner>=0 && r2Owner!=a.id) {a.speed=0;return;}
        r2Owner=a.id; break;
    case CLEAR_R1: if(r1Owner==a.id) r1Owner=-1; break;
    case CLEAR_R2: if(r2Owner==a.id) r2Owner=-1; break;
    case AT_GATE:
        a.state=PARKED; a.yaw=-90; a.speed=0; surfaceOwner=-1;
        a.readyAt=simTime+18+a.id%13;
        a.bagPhase=BAG_UNLOAD_WAIT; queueBagJob(a,false);     // call a baggage tug
        break;
    case LINE_UP:
        a.state=TAKEOFF; a.yaw=0; surfaceOwner=-1; break;
    case PASS: break;
    }
    ++a.target;
}
static void moveAircraft(Aircraft& a,float dt) {
    switch(a.state) {
    case APPROACH: {
        a.x=std::min(-62.f,a.x+55*dt);
        float left=(-62-a.x)/178;
        a.y=24*std::pow(left,1.3f);
        a.pitch=-10*std::pow(left,.3f)+5*std::max(0.f,1-left/.15f);
        if(a.x>=-62) {a.state=ROLLOUT; ++landed;}
        break;
    }
    case ROLLOUT:
        a.speed=std::max(6.f,a.speed-11.9f*dt);
        a.x=std::min(62.f,a.x+a.speed*dt); a.pitch=std::max(0.f,a.pitch-12*dt);
        if(a.x>=62) {
            a.speed=0;
            if(surfaceOwner>=0) break;
            surfaceOwner=a.id; a.state=TAXI_IN; a.target=0;
            float bx=bayX[a.bay];
            a.path={{65,RW1,PASS},{70,-50,CLAIM_R2},{70,-38,CLEAR_R1},
                    {70,-18,CLEAR_R2},{66,-15,PASS},{bx+7,-15,PASS},
                    {bx,-8,PASS},{bx,STAND,AT_GATE}};
        }
        break;
    case TAXI_IN: case TAXI_OUT: taxi(a,dt); break;
    case PARKED:
        // Pushback only with baggage loaded and the belt loader withdrawn.
        if(surfaceOwner<0 && a.bagPhase==BAG_READY && loaderExt[a.bay]<.02f &&
           (a.force || (autoMode && simTime>=a.readyAt))) {
            // Give an arrival already at the exit the next turn onto the shared taxi lane.
            for(const Aircraft& b:fleet) if(b.state==ROLLOUT && b.x>=62) return;
            surfaceOwner=a.id; a.state=PUSHBACK; a.elapsed=0; a.force=false;
        }
        break;
    case PUSHBACK: {
        // Reverse straight first, then through a quarter-circle onto the taxiway.
        a.elapsed+=dt;
        float straight=std::min(10.f,3*a.elapsed);
        float theta=std::min(90*RAD,std::max(0.f,3*a.elapsed-10)/9);
        a.x=bayX[a.bay]+9*(1-std::cos(theta)); a.z=STAND-straight-9*std::sin(theta);
        a.yaw=-90-theta/RAD;
        if(theta>=90*RAD) {
            bayOwner[a.bay]=-1; a.state=TAXI_OUT; a.speed=0; a.target=0;
            a.path={{-64,-15,PASS},{-70,-21,CLAIM_R2},{-70,-30,PASS},
                    {-66,RW2,PASS},{-60,RW2,LINE_UP}};
        }
        break;
    }
    case TAKEOFF:
        a.speed+=16*dt; a.x+=a.speed*dt;
        if(a.speed>54) a.pitch=std::min(8.f,a.pitch+13*dt);
        if(a.pitch>=8) a.state=CLIMB;
        break;
    case CLIMB:
        a.speed=std::min(95.f,a.speed+10*dt); a.pitch=std::min(13.f,a.pitch+3*dt);
        a.x+=a.speed*std::cos(a.pitch*RAD)*dt; a.y+=a.speed*std::sin(a.pitch*RAD)*dt;
        if(a.x>100 && r2Owner==a.id) r2Owner=-1;
        if(a.x>260) {a.finished=true; ++departed;}
        break;
    }
}

// ------------------------------------------------------------ baggage tugs
static const P2 EAST_LOOP[7]={{68,LANE_E},{70.5f,13.3f},{72.6f,12.9f},{73.3f,11.65f},
                              {72.6f,10.4f},{70.5f,10.0f},{68,LANE_W}};
static const P2 WEST_LOOP[7]={{-38,LANE_W},{-40.5f,10.0f},{-42.5f,10.4f},{-43.2f,11.65f},
                              {-42.5f,12.9f},{-40.5f,13.3f},{-38,LANE_E}};
static void addPts(std::vector<P2>& p,const P2* q,int n) { p.insert(p.end(),q,q+n); }
static void addPts(std::vector<P2>& p,std::initializer_list<P2> q) { p.insert(p.end(),q.begin(),q.end()); }

// hall rank -> east loop -> west-bound lane -> beside the aircraft nose
static void pathToStand(Tug& t) {
    const float bx=bayX[t.bay];
    t.path.clear(); t.target=0;
    addPts(t.path,EAST_LOOP,7);
    addPts(t.path,{{bx+11,LANE_W},{bx+8.6f,8.6f},{bx+6.5f,STAND_Z},{bx+3.2f,STAND_Z}});
}
// stand (or rank) -> west loop -> east-bound lane -> baggage hall dock
static void pathToHall(Tug& t,bool fromStand) {
    t.path.clear(); t.target=0;
    if(fromStand) {const float bx=bayX[t.bay]; addPts(t.path,{{bx+2.1f,9.3f},{bx+1.0f,LANE_W}});}
    else addPts(t.path,EAST_LOOP,7);
    addPts(t.path,WEST_LOOP,7);
    addPts(t.path,{{DOCK_X,LANE_E}});
}
static void followCart(Tug& t) {        // the cart trails the tug's hitch point
    const float dx=std::cos(t.yaw*RAD), dz=-std::sin(t.yaw*RAD);
    const float hx=t.x-dx*(TUG_HALF+CART_GAP*.5f), hz=t.z-dz*(TUG_HALF+CART_GAP*.5f);
    float ux=hx-t.cx, uz=hz-t.cz, d=std::hypot(ux,uz);
    if(d<1e-4f) return;
    ux/=d; uz/=d;
    t.cx=hx-ux*(CART_HALF+CART_GAP*.5f); t.cz=hz-uz*(CART_HALF+CART_GAP*.5f);
    t.cyaw=std::atan2(-uz,ux)/RAD;
}
// true if another tug or its cart is just ahead (simple collision avoidance)
static bool tugBlocked(const Tug& t) {
    const float dx=std::cos(t.yaw*RAD), dz=-std::sin(t.yaw*RAD);
    for(const Tug& o:tugs) {
        if(&o==&t) continue;
        const float px[2]={o.x,o.cx}, pz[2]={o.z,o.cz}, half[2]={TUG_HALF,CART_HALF};
        for(int k=0;k<2;++k) {
            float rx=px[k]-t.x, rz=pz[k]-t.z, fwd=rx*dx+rz*dz, lat=std::fabs(rx*dz-rz*dx);
            if(fwd>0 && fwd<TUG_HALF+half[k]+.9f && lat<1.15f) return true;
        }
    }
    return false;
}
// drive along t.path; returns true once the last point is reached
static bool driveTug(Tug& t,float dt) {
    if(t.target>=t.path.size()) {t.speed=0; return true;}
    if(tugBlocked(t)) {
        t.blocked+=dt;
        if(t.blocked<15.f) {t.speed=0; return false;}     // after 15 s: creep past (no deadlock)
    } else t.blocked=0;
    const P2& w=t.path[t.target];
    float dx=w.x-t.x, dz=w.z-t.z, dist=std::hypot(dx,dz), rem=dist;
    for(size_t i=t.target+1;i<t.path.size();++i)
        rem+=std::hypot(t.path[i].x-t.path[i-1].x,t.path[i].z-t.path[i-1].z);
    float want=std::min(7.5f,1.2f+1.1f*rem);
    if(dist>.05f) {
        float heading=std::atan2(-dz,dx)/RAD, d=angleDiff(heading-t.yaw), rate=220*dt;
        t.yaw=angleDiff(t.yaw+std::max(-rate,std::min(rate,d)));
        if(std::fabs(d)>35) want=std::min(want,3.f);
    }
    t.speed=t.speed<want?std::min(want,t.speed+5*dt):std::max(want,t.speed-10*dt);
    float travel=t.speed*dt;
    while(travel>0 && t.target<t.path.size()) {
        const P2& p=t.path[t.target];
        float ex=p.x-t.x, ez=p.z-t.z, d=std::hypot(ex,ez);
        if(travel>=d) {t.x=p.x; t.z=p.z; travel-=d; ++t.target;}
        else {t.x+=ex*(travel/d); t.z+=ez*(travel/d); travel=0;}
    }
    followCart(t);
    if(t.target>=t.path.size()) {t.speed=0; return true;}
    return false;
}
static int rankIndex(int id) {
    for(size_t i=0;i<tugRank.size();++i) if(tugRank[i]==id) return int(i);
    return -1;
}
static void leaveRank(Tug& t) {
    int i=rankIndex(t.id);
    if(i>=0) tugRank.erase(tugRank.begin()+i);
}
static void joinRank(Tug& t) {
    t.state=T_RANK; t.flight=-1; t.bay=-1;
    if(rankIndex(t.id)<0) tugRank.push_back(t.id);
}
static void leaveStand(Tug& t) {
    t.state=T_TO_DOCK; t.xfer=0;
    pathToHall(t,true);
}
// take an outbound bag off the carousel (the one closest to the outbound belt)
static int takeOutboundBag() {
    int best=-1; float bestGap=1e9f;
    for(size_t i=0;i<carBags.size();++i) if(!carBags[i].inbound) {
        float gap=std::fmod(CAR_OUT_S-carBags[i].s+CAR_P,CAR_P);
        if(gap<bestGap) {bestGap=gap; best=int(i);}
    }
    if(best<0) return bagColor(++bagSeed);
    int c=carBags[best].color; carBags.erase(carBags.begin()+best);
    return c;
}
// The front tug of the hall rank takes the next job: emergencies first.
static void dispatchTugs() {
    if(tugRank.empty()) return;
    Tug& t=tugs[tugRank[0]];
    if(t.state!=T_RANK || std::fabs(t.x-rankX[0])>.05f || std::fabs(t.z-LANE_E)>.05f) return;
    int best=-1;
    for(size_t i=0;i<bagJobs.size();) {
        const BagJob& j=bagJobs[i];
        Aircraft* a=findAircraft(j.flight);
        bool valid=a && a->state==PARKED &&
                   a->bagPhase==(j.load?BAG_LOAD_WAIT:BAG_UNLOAD_WAIT);
        if(!valid) {bagJobs.erase(bagJobs.begin()+i); if(best>=int(i)) --best; continue;}
        if(best<0 || (j.emergency && !bagJobs[best].emergency) ||
           (j.emergency==bagJobs[best].emergency && j.t<bagJobs[best].t)) best=int(i);
        ++i;
    }
    if(best<0) return;
    BagJob job=bagJobs[best]; bagJobs.erase(bagJobs.begin()+best);
    Aircraft* a=findAircraft(job.flight);
    t.flight=a->id; t.bay=a->bay; t.load=job.load; ++t.trip; t.xfer=0; t.timer=0;
    if(job.load) {
        a->bagPhase=BAG_LOADING;
        t.state=T_PICKUP; t.requested=0;
        t.need=std::min(CART_CAP-t.bags,a->bagsOutTarget-a->bagsOut);
    } else {
        a->bagPhase=BAG_UNLOADING;
        leaveRank(t); t.state=T_TO_STAND; pathToStand(t);
    }
}
static void stepTug(Tug& t,float dt) {
    switch(t.state) {
    case T_RANK: {                                    // queue up at the hall
        int i=std::max(0,rankIndex(t.id));
        t.path.assign(1,P2{rankX[i],LANE_E}); t.target=0;
        driveTug(t,dt);
        break;
    }
    case T_PICKUP: {                                  // outbound belt fills the cart
        t.speed=0;
        Aircraft* a=findAircraft(t.flight);
        bool wanted=a && a->state==PARKED && a->bagPhase==BAG_LOADING;
        if(!wanted) t.need=t.requested;
        t.timer+=dt;
        if(t.requested<t.need && t.timer>=OUT_T) {
            t.timer=0; ++t.requested;
            BeltBag b={0,takeOutboundBag(),t.id}; outBags.push_back(b);
        }
        bool onBelt=false;
        for(const BeltBag& b:outBags) if(b.tug==t.id) onBelt=true;
        if(t.requested>=t.need && !onBelt) {
            if(wanted) {leaveRank(t); t.state=T_TO_STAND; pathToStand(t);}
            else if(t.bags>0) {leaveRank(t); t.state=T_TO_DOCK; pathToHall(t,false);}
            else joinRank(t);
        }
        break;
    }
    case T_TO_STAND:
        if(driveTug(t,dt)) {t.state=T_AT_STAND; t.xfer=0;}
        break;
    case T_AT_STAND: {                                // belt loader transfer
        t.speed=0;
        Aircraft* a=findAircraft(t.flight);
        bool ok=a && a->state==PARKED && a->bay==t.bay &&
                a->bagPhase==(t.load?BAG_LOADING:BAG_UNLOADING);
        if(!ok) {leaveStand(t); break;}
        if(loaderExt[t.bay]<.99f) break;               // wait for the belt loader
        bool more=t.load ? (t.bags>0 && a->bagsOut<a->bagsOutTarget)
                         : (a->bagsIn>0 && t.bags<CART_CAP);
        if(more) {
            t.xfer+=dt;
            if(t.xfer>=XFER_T) {
                t.xfer=0;
                if(t.load) {--t.bags; ++a->bagsOut; ++bagsLoaded;}
                else {t.cart[t.bags]=inboundColor(t); ++t.bags; --a->bagsIn; ++bagsUnloaded;}
            }
            break;
        }
        if(t.load) {
            if(a->bagsOut>=a->bagsOutTarget) a->bagPhase=BAG_READY;
            else {a->bagPhase=BAG_LOAD_WAIT; queueBagJob(*a,true);}
        } else if(a->bagsIn==0) {a->bagPhase=BAG_LOAD_WAIT; queueBagJob(*a,true);}
        else {a->bagPhase=BAG_UNLOAD_WAIT; queueBagJob(*a,false);}   // cart full: another trip
        leaveStand(t);
        break;
    }
    case T_TO_DOCK:
        if(driveTug(t,dt)) {
            if(t.bags>0) {t.state=T_DROP; t.timer=0;}
            else joinRank(t);
        }
        break;
    case T_DROP:                                      // cart -> inbound belt
        t.speed=0; t.timer+=dt;
        if(t.bags>0 && t.timer>=DROP_T) {
            t.timer=0; --t.bags;
            BeltBag b={0,t.cart[t.bags],-1}; inBags.push_back(b);
        }
        if(t.bags==0) joinRank(t);
        break;
    }
}
static void stepBaggage(float dt) {
    dispatchTugs();
    for(Tug& t:tugs) stepTug(t,dt);
    for(int b=0;b<5;++b) {                            // belt loaders dock / withdraw
        bool want=false;
        for(const Tug& t:tugs) if(t.state==T_AT_STAND && t.bay==b) want=true;
        float d=(want?1.f:0.f)-loaderExt[b], r=.8f*dt;
        loaderExt[b]+=std::max(-r,std::min(r,d));
    }
    static const float inLen=polyLen(IN_BELT,2), outLen=polyLen(OUT_BELT,4), feedLen=polyLen(FEED_BELT,2);
    for(size_t i=0;i<inBags.size();) {                // inbound belt -> carousel
        inBags[i].s+=IN_SPEED*dt;
        if(inBags[i].s>=inLen) {
            CarouselBag c={CAR_IN_S,inBags[i].color,true,16+18*hash01(++bagSeed)};
            carBags.push_back(c); inBags.erase(inBags.begin()+i);
        } else ++i;
    }
    for(size_t i=0;i<outBags.size();) {               // outbound belt -> tug cart
        outBags[i].s+=OUT_SPEED*dt;
        if(outBags[i].s>=outLen) {
            Tug& t=tugs[outBags[i].tug];
            if(t.state==T_PICKUP && t.bags<CART_CAP) t.cart[t.bags++]=outBags[i].color;
            outBags.erase(outBags.begin()+i);
        } else ++i;
    }
    int pool=int(feedBags.size());                    // check-in keeps bags coming
    for(const CarouselBag& c:carBags) if(!c.inbound) ++pool;
    feedTimer+=dt;
    if(feedTimer>=FEED_T) {
        feedTimer=0;
        if(pool<10) {BeltBag b={0,bagColor(++bagSeed),-1}; feedBags.push_back(b);}
    }
    for(size_t i=0;i<feedBags.size();) {
        feedBags[i].s+=FEED_SPEED*dt;
        if(feedBags[i].s>=feedLen) {
            CarouselBag c={CAR_FEED_S,feedBags[i].color,false,0};
            carBags.push_back(c); feedBags.erase(feedBags.begin()+i);
        } else ++i;
    }
    for(size_t i=0;i<carBags.size();) {               // carousel turns; passengers collect
        CarouselBag& c=carBags[i];
        c.s=std::fmod(c.s+CAR_SPEED*dt,CAR_P);
        if(c.inbound && (c.life-=dt)<=0) {++bagsClaimed; carBags.erase(carBags.begin()+i);}
        else ++i;
    }
}
static int bagsInHall() {
    int n=int(inBags.size());
    for(const CarouselBag& c:carBags) if(c.inbound) ++n;
    return n;
}

static void stepSim(float dt) {
    if(paused) return;
    simTime+=dt;
    if(autoMode) {
        if(simTime>=nextAuto) {
            if(waiting.size()<6) addFlight(false);
            nextAuto=simTime+22+nextId%11;
        }
        if(assigned.id<0 && !waiting.empty() && r1Owner<0 && freeBay()>=0) assignRunway();
        if(assigned.id>=0) clearLanding();
    }
    for(Aircraft& a:fleet) moveAircraft(a,dt);
    fleet.erase(std::remove_if(fleet.begin(),fleet.end(),[](const Aircraft& a){return a.finished;}),fleet.end());
    stepBaggage(dt);
}

#ifndef AIRPORT_HEADLESS
static int followId=-1;
static const char* phase(const Aircraft& a) {
    static const char* names[]={"Final approach","Landing roll","Taxi to gate","Parked",
                               "Pushback","Taxi to runway","Take-off roll","Climb out"};
    return names[a.state];
}
static const char* bagText(const Aircraft& a) {
    static char b[48];
    switch(a.bagPhase) {
    case BAG_INFLIGHT:    std::snprintf(b,sizeof(b),"%d bags aboard",a.bagsIn); break;
    case BAG_UNLOAD_WAIT: std::snprintf(b,sizeof(b),"Bags: wait tug (%d)",a.bagsIn); break;
    case BAG_UNLOADING:   std::snprintf(b,sizeof(b),"Unloading, %d left",a.bagsIn); break;
    case BAG_LOAD_WAIT:   std::snprintf(b,sizeof(b),"Bags: wait load"); break;
    case BAG_LOADING:     std::snprintf(b,sizeof(b),"Loading %d/%d",a.bagsOut,a.bagsOutTarget); break;
    case BAG_READY:       std::snprintf(b,sizeof(b),"Bags loaded %d",a.bagsOut); break;
    }
    return b;
}
static const char* tugText(const Tug& t) {
    static char b[64];
    switch(t.state) {
    case T_RANK:     std::snprintf(b,sizeof(b),"Standby at baggage hall"); break;
    case T_PICKUP:   std::snprintf(b,sizeof(b),"Collecting bags for BG%d",t.flight); break;
    case T_TO_STAND: std::snprintf(b,sizeof(b),"To bay %d (%s BG%d)",t.bay+1,t.load?"load":"unload",t.flight); break;
    case T_AT_STAND: std::snprintf(b,sizeof(b),"%s BG%d at bay %d",t.load?"Loading":"Unloading",t.flight,t.bay+1); break;
    case T_TO_DOCK:  std::snprintf(b,sizeof(b),"Returning to baggage hall"); break;
    case T_DROP:     std::snprintf(b,sizeof(b),"Unloading at baggage hall"); break;
    }
    return b;
}

// ------------------------------------------------------ baggage drawing
static const float bagPal[8][3]={{.75f,.12f,.12f},{.12f,.22f,.55f},{.10f,.10f,.11f},{.15f,.50f,.22f},
                                 {.92f,.52f,.08f},{.48f,.20f,.58f},{.60f,.60f,.63f},{.08f,.52f,.56f}};
static void bagBox(float x,float y,float z,float yaw,int c) {
    glPushMatrix(); glTranslatef(x,y,z); glRotatef(yaw,0,1,0);
    col(bagPal[c][0],bagPal[c][1],bagPal[c][2]); box(0,0,0,.5f,.32f,.4f);
    col(.08f,.08f,.08f); box(0,.32f,0,.18f,.05f,.06f);        // handle
    glPopMatrix();
}
static void cartSlot(const Tug& t,int i,float& x,float& y,float& z) {
    int tier=i/10, k=i%10;
    float lx=1.2f-(k/2)*.6f, lz=(k%2)?.34f:-.34f, c=std::cos(t.cyaw*RAD), s=std::sin(t.cyaw*RAD);
    y=.5f+tier*.34f; x=t.cx+lx*c+lz*s; z=t.cz-lx*s+lz*c;
}
static void drawTug(const Tug& t) {
    glPushMatrix(); glTranslatef(t.x,0,t.z); glRotatef(t.yaw,0,1,0);
    col(.97f,.76f,.07f); box(0,.25f,0,2.4f,.5f,1.2f);           // chassis
    box(.75f,.75f,0,.85f,.3f,1.15f);                           // engine hood
    col(.12f,.12f,.14f); box(-.3f,.75f,0,.55f,.42f,.9f);        // seat
    for(int sx=0;sx<2;++sx) for(int sz=-1;sz<=1;sz+=2)
        box(sx?.25f:-.85f,.75f,sz*.52f,.06f,.85f,.06f);         // cab posts
    col(.95f,.95f,.97f); box(-.3f,1.6f,0,1.35f,.07f,1.2f);       // roof
    col(.2f,.2f,.22f);   box(-1.3f,.3f,0,.25f,.08f,.14f);        // hitch
    col(.05f,.05f,.05f);
    wheel(.75f,.28f,.62f,.28f,.22f);  wheel(.75f,.28f,-.62f,.28f,.22f);
    wheel(-.75f,.28f,.62f,.28f,.22f); wheel(-.75f,.28f,-.62f,.28f,.22f);
    glDisable(GL_LIGHTING);                                    // amber beacon
    bool on=std::fmod(simTime*2.5+t.id*.37,1.0)<.5;
    col(on?1.f:.45f,on?.62f:.28f,0); sph(-.3f,1.75f,0,.1f,8);
    glEnable(GL_LIGHTING);
    glPopMatrix();
}
static void drawTugCart(const Tug& t,int shown) {
    glPushMatrix(); glTranslatef(t.cx,0,t.cz); glRotatef(t.cyaw,0,1,0);
    col(.30f,.31f,.35f); box(0,.38f,0,3.0f,.12f,1.5f);          // bed
    col(.85f,.85f,.88f);                                       // rails
    box(0,.5f,.72f,3.0f,.18f,.05f); box(0,.5f,-.72f,3.0f,.18f,.05f); box(-1.48f,.5f,0,.05f,.5f,1.5f);
    col(.25f,.25f,.28f); box(1.75f,.3f,0,.6f,.06f,.12f);        // drawbar
    col(.05f,.05f,.05f);
    wheel(1.0f,.24f,.62f,.24f,.18f);  wheel(1.0f,.24f,-.62f,.24f,.18f);
    wheel(-1.0f,.24f,.62f,.24f,.18f); wheel(-1.0f,.24f,-.62f,.24f,.18f);
    glPopMatrix();
    for(int i=0;i<shown;++i) {float x,y,z; cartSlot(t,i,x,y,z); bagBox(x,y,z,t.cyaw,t.cart[i]);}
}
// belt loader of bay b; also returns the top (door) and bottom points of the belt
static void loaderEnds(int b,float& hx,float& hy,float& lx,float& ly) {
    float e=loaderExt[b], sh=(1-e)*1.2f;
    hx=bayX[b]+1.0f+sh; hy=.95f+.38f*e; lx=bayX[b]+5.7f+sh; ly=.78f;
}
static void drawBeltLoader(int b) {
    float hx,hy,lx,ly; loaderEnds(b,hx,hy,lx,ly);
    const float z=LOADER_Z, sh=lx-(bayX[b]+5.7f);
    col(.93f,.93f,.95f); box(bayX[b]+4.1f+sh,.28f,z,3.6f,.42f,1.1f);   // chassis
    col(.97f,.76f,.07f); box(bayX[b]+5.0f+sh,.7f,z-.82f,1.0f,.75f,.5f); // driver cab
    col(.45f,.45f,.5f);  box(bayX[b]+3.3f+sh,.6f,z,.16f,(hy+ly)/2-.55f,.16f); // lift strut
    col(.05f,.05f,.05f);
    wheel(bayX[b]+2.8f+sh,.25f,z+.56f,.25f,.2f); wheel(bayX[b]+2.8f+sh,.25f,z-.56f,.25f,.2f);
    wheel(bayX[b]+5.4f+sh,.25f,z+.56f,.25f,.2f); wheel(bayX[b]+5.4f+sh,.25f,z-.56f,.25f,.2f);
    float len=std::hypot(lx-hx,ly-hy), ang=std::atan2(ly-hy,lx-hx)/RAD;
    glPushMatrix(); glTranslatef(hx,hy,z); glRotatef(ang,0,0,1);
    col(.12f,.12f,.13f); box(len/2,-.1f,0,len,.1f,.66f);                // belt
    col(.97f,.76f,.07f);
    box(len/2,-.12f,.37f,len,.22f,.06f); box(len/2,-.12f,-.37f,len,.22f,.06f);
    glPopMatrix();
    if(loaderExt[b]>.6f && bayOwner[b]>=0) {                             // open hold door
        col(.06f,.06f,.08f); box(bayX[b]+.8f,.95f,z,.08f,.55f,.75f);
    }
}
// ---------------------------------------- scanner / sorter indicators (live)
static void laserPlane(float x0, float x1, float z, bool active)
{
    glDisable(GL_LIGHTING); glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glDepthMask(GL_FALSE);
    float f = active ? .35f + .25f * std::sin(float(simTime) * 40.f) : 0.f;
    if (active) {
        glColor4f(1, .08f, .08f, f);
        glBegin(GL_QUADS);
        glVertex3f(x0, .80f, z); glVertex3f(x1, .80f, z); glVertex3f(x1, 1.60f, z); glVertex3f(x0, 1.60f, z);
        glEnd();
    }
    glColor4f(1, .15f, .15f, active ? .95f : .45f);                 // scan line under the camera heads
    glBegin(GL_QUADS);
    glVertex3f(x0, 1.50f, z); glVertex3f(x1, 1.50f, z); glVertex3f(x1, 1.54f, z); glVertex3f(x0, 1.54f, z);
    glEnd();
    glDepthMask(GL_TRUE); glDisable(GL_BLEND); glEnable(GL_LIGHTING);
}
static void drawSortingFx()
{
    // X-ray tunnel lamp: green when idle, amber while a check-in bag is inside
    bool inTunnel = false;
    for (const BeltBag& b : feedBags) if (b.s > .4f && b.s < 2.9f) inTunnel = true;
    glDisable(GL_LIGHTING);
    if (inTunnel && int(simTime * 6) % 2 == 0) col(1, .65f, .05f); else if (inTunnel) col(.5f, .3f, .02f); else col(.1f, 1, .2f);
    sph(TUNNEL_X, 2.2f, 24.8f, .13f, 8);
    glEnable(GL_LIGHTING);

    // arrival scanner
    const float sIn = IN_SCAN_Z - IN_BELT[0].z;
    bool inScan = false;
    for (const BeltBag& b : inBags) if (std::fabs(b.s - sIn) < .8f) inScan = true;
    laserPlane(40.1f, 41.9f, IN_SCAN_Z, inScan);

    // outbound scanner + sorter: the tag reader gives the gate, the matching chute lights up
    const float sScan = polyLen(OUT_BELT, 2) + (OUT_BELT[1].z - SCAN_Z), sEnd = polyLen(OUT_BELT, 3);
    bool lit[5] = { false, false, false, false, false }, underScan = false;
    int gate = -1, flight = -1;
    for (const BeltBag& b : outBags) {
        if (b.tug < 0 || b.tug >= NTUG) continue;
        const Tug& t = tugs[b.tug];
        if (t.bay < 0 || t.bay > 4) continue;
        if (std::fabs(b.s - sScan) < .8f) { underScan = true; gate = t.bay; flight = t.flight; }
        if (b.s >= sScan - .3f && b.s <= sEnd) lit[t.bay] = true;
    }
    laserPlane(50.6f, 52.4f, SCAN_Z, underScan);
    char buf[32];
    if (underScan) { std::snprintf(buf, sizeof(buf), "BG%d > GATE %d", flight, gate + 1); signBoard(buf, 51.5f, 2.55f, SCAN_Z, 90, 1.5f, .55f, GREEN_WHITE); }
    else signBoard("SCANNING", 51.5f, 2.55f, SCAN_Z, 90, 1.5f, .55f, BLACK_YELLOW);
    glDisable(GL_LIGHTING);
    for (int i = 0; i < 5; i++) {                                    // chute status lamps
        if (lit[i]) col(.1f, 1, .2f); else col(.40f, .10f, .10f);
        sph(PLATE_X, 3.5f, CHUTE_Z[i], .2f, 8);
        if (lit[i]) { col(.1f, 1, .2f); sph(51.0f, 1.15f, CHUTE_Z[i] + .5f, .09f, 6); }
    }
    glEnable(GL_LIGHTING);
}

static void drawBaggage() {
    for(int b=0;b<5;++b) drawBeltLoader(b);
    for(const Tug& t:tugs) {
        drawTug(t);
        bool movingOut=t.state==T_AT_STAND && t.load && t.xfer>0 && t.bags>0;
        drawTugCart(t,t.bags-(movingOut?1:0));
        if(t.state!=T_AT_STAND || t.xfer<=0 || t.bay<0) continue;
        // the bag currently travelling on the belt loader
        float hx,hy,lx,ly,x,y,z,f=t.xfer/XFER_T, sx,sy,sz;
        loaderEnds(t.bay,hx,hy,lx,ly);
        if(!t.load) {
            if(t.bags>=CART_CAP) continue;
            cartSlot(t,t.bags,sx,sy,sz);
            if(f<.7f) {float u=f/.7f; x=hx+(lx-hx)*u; y=hy+(ly-hy)*u; z=LOADER_Z;}
            else {float u=(f-.7f)/.3f; x=lx+(sx-lx)*u; y=ly+(sy-ly)*u+.5f*std::sin(PI_F*u); z=LOADER_Z+(sz-LOADER_Z)*u;}
            bagBox(x,y,z,0,inboundColor(t));
        } else {
            if(t.bags<=0) continue;
            cartSlot(t,t.bags-1,sx,sy,sz);
            if(f<.3f) {float u=f/.3f; x=sx+(lx-sx)*u; y=sy+(ly-sy)*u+.5f*std::sin(PI_F*u); z=sz+(LOADER_Z-sz)*u;}
            else {float u=(f-.3f)/.7f; x=lx+(hx-lx)*u; y=ly+(hy-ly)*u; z=LOADER_Z;}
            bagBox(x,y,z,0,t.cart[t.bags-1]);
        }
    }
    float x,z,yaw;
    for(const BeltBag& b:inBags)   {polyAt(IN_BELT,2,b.s,x,z,yaw);   bagBox(x,.78f,z,yaw,b.color);}
    for(const BeltBag& b:outBags)  {polyAt(OUT_BELT,4,b.s,x,z,yaw);  bagBox(x,.78f,z,yaw,b.color);}
    for(const BeltBag& b:feedBags) {polyAt(FEED_BELT,2,b.s,x,z,yaw); bagBox(x,.78f,z,yaw,b.color);}
    for(const CarouselBag& c:carBags) {carouselAt(c.s,0,x,z,yaw); bagBox(x,.78f,z,yaw,c.color);}
    drawSortingFx();
}

static void drawFlights() {
    const float palette[][3]={{.8f,.1f,.1f},{.1f,.3f,.75f},{.1f,.60f,.25f},
                              {.9f,.55f,.05f},{.55f,.15f,.65f}};
    for(const Aircraft& a:fleet) {
        const float* c=palette[(a.id-101)%5];
        drawAircraft(a.x,a.y+.09f,a.z,a.yaw,a.pitch,0,.56f,c[0],c[1],c[2],a.y<3);
        if(a.y>.3f && std::fabs(a.x)<110) {
            glDisable(GL_LIGHTING); glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
            glColor4f(0,0,0,.24f);
            glPushMatrix(); glTranslatef(a.x+a.y*.45f,.13f,a.z+a.y*.25f);
            glRotatef(a.yaw,0,1,0); rect(-9,-.9f,6,.9f,0); rect(-4,-6,0,6,0);
            glPopMatrix(); glDisable(GL_BLEND); glEnable(GL_LIGHTING);
        }
    }
}
static void drawLights() {
    glPushMatrix(); glTranslatef(52,26.8f,2); glRotatef(radarAngle,0,1,0);
    col(.75f,.75f,.78f); box(0,.7f,0,4,.35f,.6f); box(1.9f,.7f,0,.2f,1,.6f); glPopMatrix();
    glDisable(GL_LIGHTING);
    if(beaconOn) { col(1,.1f,.1f); sph(52,28.2f,2,night?.9f:.6f); }
    float r=night?.4f:.22f;
    for(int i=0;i<2;++i) {
        float cz=i?RW2:RW1; col(1,1,.85f);
        for(int x=-88;x<=88;x+=11) {sph(x,.25f,cz-6.4f,r,6); sph(x,.25f,cz+6.4f,r,6);}
        for(int k=0;k<7;++k) {
            float z=cz-5.4f+k*1.8f;
            col(.1f,1,.2f); sph(-90.6f,.25f,z,r,6); col(1,.1f,.1f); sph(90.6f,.25f,z,r,6);
        }
    }
    col(.2f,.4f,1); for(int x=-72;x<=72;x+=8) sph(x,.2f,-18.4f,r,6);
    col(1,.95f,.65f); for(const Lamp& l:lamps) sph(l.x,l.h+.3f,l.z,night?.7f:.4f,8);
    if(night) {
        col(1,.88f,.50f); box(0,1,29.22f,55,4.8f,.05f); box(0,1,14.77f,55,4.8f,.05f);
        cyl(52,21,2,4.62f,2.6f,24);
        box(41,5.92f,17.5f,14,.06f,.5f); box(41,5.92f,28.75f,14,.06f,.5f);   // hall lights
    }
    glEnable(GL_LIGHTING);
}
static void textLine(int x,int y,const char* format,...) {
    char buf[200]; va_list args; va_start(args,format); vsnprintf(buf,sizeof(buf),format,args); va_end(args);
    glRasterPos2i(x,y); glutBitmapString(GLUT_BITMAP_HELVETICA_12,(const unsigned char*)buf);
}
static void panel(int x,int top,int width,int height) {
    glEnable(GL_BLEND); glBlendFunc(GL_SRC_ALPHA,GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(.025f,.055f,.09f,.89f);
    glRecti(x,top-height,x+width,top); glDisable(GL_BLEND); col(.91f,.95f,1);
}
static void hud() {
    glDisable(GL_LIGHTING); glDisable(GL_DEPTH_TEST);
    glMatrixMode(GL_PROJECTION); glPushMatrix(); glLoadIdentity(); gluOrtho2D(0,winW,0,winH);
    glMatrixMode(GL_MODELVIEW); glPushMatrix(); glLoadIdentity();
    int top=winH-10;
    panel(10,top,410,111);
    textLine(21,top-18,"AIRPORT CONTROL   |   %s   |   %s   |   %.2gx   |   %.0f FPS",
             autoMode?"AUTO":"MANUAL",paused?"PAUSED":"LIVE",simSpeed,fps);
    textLine(21,top-38,"A Routine   E Emergency   R Assign runway   L Clear landing");
    textLine(21,top-54,"D Depart   M Auto/manual   P Pause   S Resume   T Board");
    textLine(21,top-70,"1-5 Views (5 Baggage)   F Follow   N Day/night   [ ] Speed");
    col(.96f,.84f,.49f); textLine(21,top-96,"%s",notice);
    if(showBoard) {
        const int W=420;
        int left=winW>=850?winW-W-10:10;
        if(winW<850) top-=119;
        int queued=int(std::min<size_t>(3,waiting.size()));
        int height=158+16*(queued+int(fleet.size()))+38+16*NTUG;
        panel(left,top,W,height);
        int x=left+11,y=top-18;
        textLine(x,y,"FLIGHT BOARD    |    T + %.0f s",simTime); y-=20;
        textLine(x,y,"R1 %s   R2 %s   |   Landed %d   Departed %d",
                 r1Owner<0?"FREE":"BUSY",r2Owner<0?"FREE":"BUSY",landed,departed); y-=16;
        int occupied=0; for(int id:bayOwner) if(id>=0) ++occupied;
        textLine(x,y,"Bays reserved/occupied %d/5   |   Queued %u",occupied,unsigned(waiting.size())); y-=16;
        textLine(x,y,"Queue to clearance: avg %.1f s   max %.1f s   (n=%d)",
                 cleared?totalWait/cleared:0,maxWait,cleared); y-=18;
        if(assigned.id>=0) textLine(x,y,"Assigned BG%d  |  R1, bay %d  |  Awaiting L",assigned.id,assignedBay+1);
        else if(waiting.empty()) textLine(x,y,"No arrivals waiting for assignment");
        else if(r1Owner>=0) textLine(x,y,"Waiting for runway 1 to clear");
        else if(freeBay()<0) textLine(x,y,"Waiting for a free bay");
        else textLine(x,y,"Next arrival ready for assignment");
        y-=18;
        // The normal auto queue is bounded. Copy only while displaying a nonempty queue.
        ArrivalQueue preview=waiting;
        for(int i=0;i<3 && !preview.empty();++i) {
            Request r=preview.top(); preview.pop();
            if(r.emergency) col(1,.60f,.35f); else col(.78f,.87f,.96f);
            textLine(x,y,"%d. BG%d   %s   |   waiting %.1f s",i+1,r.id,r.emergency?"EMERGENCY":"Routine",simTime-r.entered); y-=16;
        }
        col(1,1,1); textLine(x,y,"ACTIVE FLIGHTS"); y-=17;
        for(const Aircraft& a:fleet) {
            if(a.id==followId) col(1,.83f,.35f); else if(a.emergency) col(1,.66f,.45f); else col(.89f,.94f,1);
            if(a.state==PARKED || a.state==TAXI_IN || a.state==ROLLOUT || a.state==APPROACH)
                textLine(x,y,"BG%d  %s  |  Bay %d  |  %s%s",a.id,phase(a),a.bay+1,bagText(a),a.emergency?"  EMG":"");
            else textLine(x,y,"BG%d  %s  |  Bay %d  |  Bags loaded %d%s",a.id,phase(a),a.bay+1,a.bagsOut,a.emergency?"  EMG":"");
            y-=16;
        }
        y-=4; col(1,1,1);
        textLine(x,y,"BAGGAGE   |   In hall %d   Claimed %d   Loaded %d   Jobs %u",
                 bagsInHall(),bagsClaimed,bagsLoaded,unsigned(bagJobs.size())); y-=17;
        for(const Tug& t:tugs) {
            col(.98f,.82f,.30f);
            textLine(x,y,"TUG %d   %s   |   cart %d/%d",t.id+1,tugText(t),t.bags,CART_CAP); y-=16;
        }
    }
    glPopMatrix(); glMatrixMode(GL_PROJECTION); glPopMatrix(); glMatrixMode(GL_MODELVIEW);
    glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING);
}
static void setView(int v) {
    // yaw, pitch, distance, target x, target z
    static const float views[][5]={{35,32,230,0,8},{0,88,270,0,0},{-22,17,125,0,12},
                                   {78,11,160,0,-35},{170,30,92,20,10}};
    const float* p=views[v-1]; camYaw=p[0]; camPitch=p[1]; camDist=p[2];
    camTX=p[3];camTY=0;camTZ=p[4];followId=-1;
}
static void cycleFollow() {
    size_t i=0;
    if(followId>=0) {while(i<fleet.size() && fleet[i].id!=followId) ++i; ++i;}
    if(i>=fleet.size()) {setView(1);return;}
    followId=fleet[i].id; camDist=std::min(camDist,110.f);
}
static void display() {
    if(night) glClearColor(.03f,.05f,.12f,1); else glClearColor(.55f,.76f,.95f,1);
    glClear(GL_COLOR_BUFFER_BIT|GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW); glLoadIdentity();
    if(followId>=0) {
        bool found=false;
        for(const Aircraft& a:fleet) if(a.id==followId) {
            camTX+=(a.x-camTX)*.08f; camTY+=(a.y-camTY)*.08f; camTZ+=(a.z-camTZ)*.08f; found=true; break;
        }
        if(!found) setView(1);
    }
    float ground=camDist*std::cos(camPitch*RAD);
    gluLookAt(camTX+ground*std::sin(camYaw*RAD),camTY+camDist*std::sin(camPitch*RAD),
              camTZ+ground*std::cos(camYaw*RAD),camTX,camTY,camTZ,0,1,0);
    const GLfloat amb[][4]={{.4f,.4f,.44f,1},{.14f,.15f,.24f,1}};
    const GLfloat sun[][4]={{.85f,.82f,.75f,1},{.22f,.26f,.42f,1}};
    const GLfloat fill[][4]={{.18f,.2f,.28f,1},{.05f,.05f,.08f,1}};
    const GLfloat p0[]={.45f,.85f,.35f,0},p1[]={-.5f,.3f,-.6f,0};
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT,amb[night]); glLightfv(GL_LIGHT0,GL_DIFFUSE,sun[night]);
    glLightfv(GL_LIGHT1,GL_DIFFUSE,fill[night]); glLightfv(GL_LIGHT0,GL_POSITION,p0); glLightfv(GL_LIGHT1,GL_POSITION,p1);
    glCallList(sceneList); drawFlights(); drawBaggage(); drawLights(); hud(); glutSwapBuffers();
    static int frames=0,last=0; ++frames;
    int now=glutGet(GLUT_ELAPSED_TIME); if(now-last>=1000) {fps=frames*1000.f/(now-last);frames=0;last=now;}
}
static void reshape(int w,int h) {
    winW=std::max(1,w); winH=std::max(1,h); glViewport(0,0,winW,winH);
    glMatrixMode(GL_PROJECTION); glLoadIdentity(); gluPerspective(48,double(winW)/winH,1,700);
    glMatrixMode(GL_MODELVIEW);
}
static void timer(int) {
    static int last=glutGet(GLUT_ELAPSED_TIME);
    int now=glutGet(GLUT_ELAPSED_TIME);
    float elapsed=std::min(.1f,(now-last)/1000.f)*simSpeed; last=now;
    while(elapsed>0) {float dt=std::min(elapsed,.02f); stepSim(dt); elapsed-=dt;}
    radarAngle=float(std::fmod(simTime*120,360.)); beaconOn=int(simTime*2)%2==0;
    glutPostRedisplay(); glutTimerFunc(16,timer,0);
}
static void clampCam() {
    camDist=std::max(40.f,std::min(camDist,400.f)); camPitch=std::max(8.f,std::min(camPitch,88.f));
}
static void keyboard(unsigned char key,int,int) {
    switch(std::tolower(key)) {
    case 27: glutLeaveMainLoop(); break;
    case '1': case '2': case '3': case '4': case '5': setView(key-'0'); break;
    case 'a':addFlight(false);break;
    case 'e':addFlight(true);break;
    case 'r':assignRunway();break;
    case 'l':clearLanding();break;
    case 'd':forceDepart();break;
    case 'm':autoMode=!autoMode; if(autoMode) nextAuto=simTime+3;
             report("%s dispatch.",autoMode?"Automatic":"Manual");break;
    case 'p':paused=!paused;break;
    case 's':paused=false;break;
    case 't':showBoard=!showBoard;break;
    case 'n':night=!night;break;
    case 'f':cycleFollow();break;
    case '[':simSpeed=std::max(.25f,simSpeed*.5f);break;
    case ']':simSpeed=std::min(4.f,simSpeed*2);break;
    case '+': case '=':camDist-=10;break;
    case '-': case '_':camDist+=10;break;
    }
    clampCam(); glutPostRedisplay();
}
static void special(int k,int,int) {
    switch(k) {
    case GLUT_KEY_LEFT:camYaw-=4;break; case GLUT_KEY_RIGHT:camYaw+=4;break;
    case GLUT_KEY_UP:camPitch+=3;break; case GLUT_KEY_DOWN:camPitch-=3;break;
    case GLUT_KEY_PAGE_UP:camDist-=10;break; case GLUT_KEY_PAGE_DOWN:camDist+=10;break;
    }
    clampCam();
}
static void mouse(int b,int s,int x,int y) {if(b==GLUT_LEFT_BUTTON){dragging=s==GLUT_DOWN;lastX=x;lastY=y;}}
static void motion(int x,int y) {
    if(dragging){camYaw-=(x-lastX)*.4f;camPitch+=(y-lastY)*.4f;lastX=x;lastY=y;clampCam();}
}
static void wheelFn(int,int dir,int,int){camDist-=12*dir;clampCam();}
static void menu(int id){keyboard(static_cast<unsigned char>(id),0,0);}
int main(int argc,char** argv) {
    glutInit(&argc,argv); glutInitDisplayMode(GLUT_DOUBLE|GLUT_RGB|GLUT_DEPTH);
    glutInitWindowSize(winW,winH); glutInitWindowPosition(80,40);
    glutCreateWindow("Airport Management System - Runway & Baggage Control");
    glEnable(GL_DEPTH_TEST); glEnable(GL_LIGHTING); glEnable(GL_LIGHT0); glEnable(GL_LIGHT1);
    glEnable(GL_COLOR_MATERIAL); glColorMaterial(GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE); glShadeModel(GL_SMOOTH); Q=gluNewQuadric(); gluQuadricNormals(Q,GLU_SMOOTH);
    for(int x=-110;x<=110;x+=20) lamps.push_back({float(x),57.2f,6});
    for(int x=-40;x<=40;x+=16) lamps.push_back({float(x),38.4f,5});
    for(int x=-44;x<=44;x+=88){lamps.push_back({float(x),-11,12});lamps.push_back({float(x),14.3f,12});}
    for(int x=62;x<=98;x+=12) lamps.push_back({float(x),9.5f,7});
    initFleet(); sceneList=glGenLists(1); glNewList(sceneList,GL_COMPILE); drawStaticScene(); glEndList();
    glutCreateMenu(menu);
    glutAddMenuEntry("Routine arrival (A)",'a'); glutAddMenuEntry("Emergency arrival (E)",'e');
    glutAddMenuEntry("Assign runway (R)",'r'); glutAddMenuEntry("Clear landing (L)",'l');
    glutAddMenuEntry("Request departure (D)",'d'); glutAddMenuEntry("Auto / manual (M)",'m');
    glutAddMenuEntry("Pause / resume (P)",'p'); glutAddMenuEntry("Flight + baggage board (T)",'t');
    glutAddMenuEntry("Isometric view (1)",'1'); glutAddMenuEntry("Top view (2)",'2');
    glutAddMenuEntry("Terminal view (3)",'3'); glutAddMenuEntry("Runway view (4)",'4');
    glutAddMenuEntry("Baggage view (5)",'5');
    glutAddMenuEntry("Day / night (N)",'n'); glutAddMenuEntry("Exit",27); glutAttachMenu(GLUT_RIGHT_BUTTON);
    glutDisplayFunc(display); glutReshapeFunc(reshape); glutKeyboardFunc(keyboard); glutSpecialFunc(special);
    glutMouseFunc(mouse); glutMotionFunc(motion); glutMouseWheelFunc(wheelFn); glutTimerFunc(16,timer,0);
    glutMainLoop(); return 0;
}
#endif
