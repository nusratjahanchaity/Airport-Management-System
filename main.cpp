#include <GL/freeglut.h>
#include <algorithm>
#include <cmath>
#include <cstdio>
#include <queue>
#include <string>
#include <vector>

inline std::string toStr(int n) {
    char b[16];
    std::snprintf(b, sizeof(b), "%d", n);
    return std::string(b);
}

// ============================================================================
//  Airport Management System — 50% Milestone Implementation (Steps 1, 2 & 3)
//  Course: CSE-4742 Computer Graphics Lab | IIUC
//
//  COMPLETED (50%):
//   [Step 1] 3D Airport Environment, Lighting, Sky Dome, Fog & Orbit Camera
//   [Step 2] 3D Aircraft Modeling & Transformations (Landing, Taxi, Park, Takeoff)
//   [Step 3] Smart Runway Allocation & Priority-Queue Flight Scheduling
//   [Core UI] Keyboard Shortcuts (A, L, R, S, P, T), Right-Click Menu & HUD
// ============================================================================

// ================== Scene & Camera Configuration ==================
namespace Config {
    float yaw = 0.60f,       targetYaw = 0.60f;
    float pitch = 0.58f,     targetPitch = 0.58f;
    float distance = 185.0f, targetDistance = 185.0f;
    float targetX = 0.0f, targetY = 2.0f, targetZ = -20.0f;

    int lastMouseX = 0, lastMouseY = 0;
    bool isDragging = false;

    int windowW = 1280, windowH = 780;
    int timeMode = 0;       // 0 = Day, 1 = Sunset, 2 = Night
    bool isRunning = true;  // S = Start, P = Pause
    bool showStats = true;  // T = Toggle On-Screen Status Panel
    float simTime = 0.0f;

    inline float clampf(float v, float lo, float hi) {
        return (v < lo) ? lo : (v > hi ? hi : v);
    }
}

static GLUquadric* g_Quadric = nullptr;

// ================== Step 3: Priority-Queue Runway Scheduler ==================

enum FlightType { FLIGHT_LANDING = 0, FLIGHT_TAKEOFF = 1 };
enum FlightPhase {
    PHASE_QUEUED = 0,
    PHASE_APPROACH = 1,
    PHASE_LANDING_ROLL = 2,
    PHASE_TAXI_IN = 3,
    PHASE_PARKED = 4,
    PHASE_TAXI_OUT = 5,
    PHASE_TAKEOFF_ROLL = 6,
    PHASE_COMPLETED = 7
};

struct FlightRequest {
    int id;
    std::string callsign;
    FlightType type;
    int priority;          // Higher = more urgent (Emergency=3, Landing=2, Takeoff=1)
    float arrivalTime;
    int preferredBay;

    bool operator<(const FlightRequest& other) const {
        if (priority != other.priority) return priority < other.priority;
        return arrivalTime > other.arrivalTime;
    }
};

struct Aircraft {
    int id;
    std::string callsign;
    FlightType type;
    FlightPhase phase;
    int assignedRunway;    // 0 = Runway 1 (z=15), 1 = Runway 2 (z=40), -1 = None
    int assignedBay;       // 0..3 -> x = {-32, -11, 11, 32}

    float x, y, z;
    float yawDeg, pitchDeg;
    float speed;
    float stateTimer;
    float rColor, gColor, bColor;
};

namespace AirportManager {
    std::priority_queue<FlightRequest> flightQueue;
    std::vector<Aircraft> activeAircraft;
    bool runwayOccupied[2] = { false, false };
    bool bayOccupied[4]    = { false, false, false, false };
    const float bayX[4]    = { -32.0f, -11.0f, 11.0f, 32.0f };
    const float runwayZ[2] = { 15.0f, 40.0f };

    int nextFlightId = 101;
    int totalCompleted = 0;
    std::string lastEventMsg = "System Ready - Runways 1 & 2 Clear";

    int findFreeBay() {
        for (int i = 0; i < 4; ++i) {
            if (!bayOccupied[i]) return i;
        }
        return -1;
    }

    int findFreeRunway() {
        if (!runwayOccupied[0]) return 0;
        if (!runwayOccupied[1]) return 1;
        return -1;
    }

    void enqueueFlight(FlightType type, int priority) {
        FlightRequest req;
        req.id = nextFlightId++;
        char buf[16];
        std::snprintf(buf, sizeof(buf), "%s-%d", (type == FLIGHT_LANDING ? "BG" : "QR"), req.id);
        req.callsign = buf;
        req.type = type;
        req.priority = priority;
        req.arrivalTime = Config::simTime;
        req.preferredBay = findFreeBay();
        flightQueue.push(req);

        lastEventMsg = "Queued " + req.callsign + (type == FLIGHT_LANDING ? " (Landing)" : " (Takeoff)");
    }

    void processRunwayQueue() {
        if (flightQueue.empty()) return;

        int rwy = findFreeRunway();
        if (rwy == -1) {
            lastEventMsg = "Runways Busy - " + flightQueue.top().callsign + " Holding in Queue";
            return;
        }

        FlightRequest req = flightQueue.top();
        if (req.type == FLIGHT_LANDING) {
            int bay = findFreeBay();
            if (bay == -1) {
                lastEventMsg = "All 4 Bays Full - " + req.callsign + " Holding";
                return;
            }
            flightQueue.pop();
            runwayOccupied[rwy] = true;
            bayOccupied[bay] = true;

            Aircraft ac;
            ac.id = req.id;
            ac.callsign = req.callsign;
            ac.type = FLIGHT_LANDING;
            ac.phase = PHASE_APPROACH;
            ac.assignedRunway = rwy;
            ac.assignedBay = bay;
            ac.x = -125.0f;
            ac.y = 22.0f;
            ac.z = runwayZ[rwy];
            ac.yawDeg = 0.0f;
            ac.pitchDeg = -8.0f;
            ac.speed = 32.0f;
            ac.stateTimer = 0.0f;
            ac.rColor = 0.12f; ac.gColor = 0.45f; ac.bColor = 0.85f;
            activeAircraft.push_back(ac);

            lastEventMsg = "Assigned Runway " + toStr(rwy + 1) + " to " + ac.callsign + " (Landing)";
        } else {
            int bay = -1;
            for (size_t i = 0; i < activeAircraft.size(); ++i) {
                if (activeAircraft[i].phase == PHASE_PARKED) {
                    bay = activeAircraft[i].assignedBay;
                    flightQueue.pop();
                    runwayOccupied[rwy] = true;
                    activeAircraft[i].phase = PHASE_TAXI_OUT;
                    activeAircraft[i].assignedRunway = rwy;
                    activeAircraft[i].type = FLIGHT_TAKEOFF;
                    activeAircraft[i].speed = 14.0f;
                    lastEventMsg = "Cleared " + activeAircraft[i].callsign + " for Takeoff on Rwy " + toStr(rwy + 1);
                    return;
                }
            }

            bay = findFreeBay();
            if (bay == -1) return;
            flightQueue.pop();
            runwayOccupied[rwy] = true;
            bayOccupied[bay] = true;

            Aircraft ac;
            ac.id = req.id;
            ac.callsign = req.callsign;
            ac.type = FLIGHT_TAKEOFF;
            ac.phase = PHASE_TAXI_OUT;
            ac.assignedRunway = rwy;
            ac.assignedBay = bay;
            ac.x = bayX[bay];
            ac.y = 0.1f;
            ac.z = -18.5f;
            ac.yawDeg = -90.0f;
            ac.pitchDeg = 0.0f;
            ac.speed = 14.0f;
            ac.stateTimer = 0.0f;
            ac.rColor = 0.88f; ac.gColor = 0.25f; ac.bColor = 0.18f;
            activeAircraft.push_back(ac);

            lastEventMsg = "Assigned Runway " + toStr(rwy + 1) + " to " + ac.callsign + " (Takeoff)";
        }
    }

    void updateSimulation(float dt) {
        if (!Config::isRunning) return;

        processRunwayQueue();

        for (size_t i = 0; i < activeAircraft.size(); ++i) {
            Aircraft& ac = activeAircraft[i];
            float rwyZ = (ac.assignedRunway >= 0) ? runwayZ[ac.assignedRunway] : 15.0f;
            float targetBayX = bayX[ac.assignedBay];

            switch (ac.phase) {
                case PHASE_APPROACH: {
                    ac.x += ac.speed * dt;
                    float progress = Config::clampf((ac.x + 125.0f) / 65.0f, 0.0f, 1.0f);
                    ac.y = 22.0f * (1.0f - progress) + 0.1f;
                    ac.pitchDeg = -8.0f * (1.0f - progress);
                    if (ac.x >= -60.0f) {
                        ac.y = 0.1f;
                        ac.pitchDeg = 0.0f;
                        ac.phase = PHASE_LANDING_ROLL;
                    }
                    break;
                }
                case PHASE_LANDING_ROLL: {
                    ac.speed = std::max(14.0f, ac.speed - 7.5f * dt);
                    ac.x += ac.speed * dt;
                    if (ac.x >= 70.0f) {
                        ac.x = 70.0f;
                        ac.phase = PHASE_TAXI_IN;
                        ac.yawDeg = 90.0f;
                        if (ac.assignedRunway >= 0) {
                            runwayOccupied[ac.assignedRunway] = false;
                            ac.assignedRunway = -1;
                        }
                        lastEventMsg = ac.callsign + " vacated runway, taxiing to Bay " + toStr(ac.assignedBay + 1);
                    }
                    break;
                }
                case PHASE_TAXI_IN: {
                    float taxSpeed = 14.0f;
                    if (ac.z > -4.0f && fabsf(ac.x - 70.0f) < 1.0f) {
                        ac.yawDeg = 90.0f;
                        ac.z -= taxSpeed * dt;
                    } else if (fabsf(ac.x - targetBayX) > 1.0f) {
                        ac.z = -4.0f;
                        ac.yawDeg = 180.0f;
                        ac.x -= taxSpeed * dt;
                    } else {
                        ac.x = targetBayX;
                        ac.yawDeg = 90.0f;
                        ac.z -= taxSpeed * dt;
                        if (ac.z <= -18.5f) {
                            ac.z = -18.5f;
                            ac.phase = PHASE_PARKED;
                            ac.stateTimer = 0.0f;
                            lastEventMsg = ac.callsign + " parked at Bay " + toStr(ac.assignedBay + 1);
                        }
                    }
                    break;
                }
                case PHASE_PARKED: {
                    ac.stateTimer += dt;
                    if (ac.stateTimer >= 6.0f) {
                        ac.stateTimer = -999.0f;
                        FlightRequest req;
                        req.id = ac.id;
                        req.callsign = ac.callsign;
                        req.type = FLIGHT_TAKEOFF;
                        req.priority = 1;
                        req.arrivalTime = Config::simTime;
                        req.preferredBay = ac.assignedBay;
                        flightQueue.push(req);
                    }
                    break;
                }
                case PHASE_TAXI_OUT: {
                    float taxSpeed = 15.0f;
                    if (ac.z < -4.0f && fabsf(ac.x - targetBayX) < 1.0f) {
                        ac.yawDeg = -90.0f;
                        ac.z += taxSpeed * dt;
                    } else if (ac.x > -70.0f && fabsf(ac.z + 4.0f) < 1.0f) {
                        ac.z = -4.0f;
                        ac.yawDeg = 180.0f;
                        ac.x -= taxSpeed * dt;
                    } else if (ac.z < rwyZ) {
                        ac.x = -70.0f;
                        ac.yawDeg = -90.0f;
                        ac.z += taxSpeed * dt;
                        if (ac.z >= rwyZ) {
                            ac.z = rwyZ;
                            ac.yawDeg = 0.0f;
                            ac.speed = 8.0f;
                            ac.phase = PHASE_TAKEOFF_ROLL;
                            bayOccupied[ac.assignedBay] = false;
                        }
                    }
                    break;
                }
                case PHASE_TAKEOFF_ROLL: {
                    ac.speed = std::min(46.0f, ac.speed + 12.0f * dt);
                    ac.x += ac.speed * dt;
                    if (ac.x > 10.0f) {
                        ac.pitchDeg = std::min(15.0f, (ac.x - 10.0f) * 0.25f);
                        ac.y += (ac.x - 10.0f) * 0.22f * dt;
                    }
                    if (ac.x > 135.0f) {
                        if (ac.assignedRunway >= 0) {
                            runwayOccupied[ac.assignedRunway] = false;
                        }
                        ac.phase = PHASE_COMPLETED;
                        totalCompleted++;
                        lastEventMsg = ac.callsign + " departed cleanly";
                    }
                    break;
                }
                case PHASE_COMPLETED:
                    break;
            }
        }

        for (int i = (int)activeAircraft.size() - 1; i >= 0; --i) {
            if (activeAircraft[i].phase == PHASE_COMPLETED) {
                activeAircraft.erase(activeAircraft.begin() + i);
            }
        }
    }
}

// ================== Material & Lighting Helpers ==================

inline void setMat(float r, float g, float b, float shininess = 24.0f,
                   float spec = 0.25f, float emit = 0.0f) {
    glColor3f(r, g, b);
    GLfloat specular[] = { spec, spec, spec, 1.0f };
    GLfloat emission[] = { r * emit, g * emit, b * emit, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, emission);
    glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shininess);
}

inline void resetEmit() {
    GLfloat zero[] = { 0.0f, 0.0f, 0.0f, 1.0f };
    glMaterialfv(GL_FRONT_AND_BACK, GL_EMISSION, zero);
}

// ================== Geometric Primitives ==================

void drawQuadXZ(float x1, float z1, float x2, float z2, float y) {
    glBegin(GL_QUADS);
    glNormal3f(0.0f, 1.0f, 0.0f);
    glVertex3f(x1, y, z1);
    glVertex3f(x1, y, z2);
    glVertex3f(x2, y, z2);
    glVertex3f(x2, y, z1);
    glEnd();
}

void drawRectOutline(float x1, float z1, float x2, float z2, float thickness, float y) {
    drawQuadXZ(x1, z1, x2, z1 + thickness, y);
    drawQuadXZ(x1, z2 - thickness, x2, z2, y);
    drawQuadXZ(x1, z1, x1 + thickness, z2, y);
    drawQuadXZ(x2 - thickness, z1, x2, z2, y);
}

void drawBox(float cx, float cy, float cz, float sx, float sy, float sz) {
    glPushMatrix();
    glTranslatef(cx, cy, cz);
    glScalef(sx, sy, sz);
    glutSolidCube(1.0);
    glPopMatrix();
}

void drawCylinderY(float x, float yBase, float z, float rBot, float rTop, float height, int slices = 28) {
    if (!g_Quadric) return;
    glPushMatrix();
    glTranslatef(x, yBase, z);
    glRotatef(-90.0f, 1.0f, 0.0f, 0.0f);
    gluCylinder(g_Quadric, rBot, rTop, height, slices, 1);
    glTranslatef(0.0f, 0.0f, height);
    gluDisk(g_Quadric, 0.0, rTop, slices, 1);
    glPopMatrix();
}

void drawLightBulb(float x, float y, float z, float radius, float r, float g, float b, float glow) {
    glPushMatrix();
    glTranslatef(x, y, z);
    setMat(r, g, b, 10.0f, 0.1f, glow);
    glutSolidSphere(radius, 10, 10);
    resetEmit();
    glPopMatrix();
}

// ================== Sky Dome & Atmosphere ==================

void applyAtmosphere() {
    GLfloat fogColor[4], amb[4], dif[4], spec[4], pos[4];
    if (Config::timeMode == 0) {
        fogColor[0] = 0.68f; fogColor[1] = 0.85f; fogColor[2] = 0.96f; fogColor[3] = 1.0f;
        amb[0] = 0.36f; amb[1] = 0.39f; amb[2] = 0.45f; amb[3] = 1.0f;
        dif[0] = 0.98f; dif[1] = 0.95f; dif[2] = 0.88f; dif[3] = 1.0f;
        spec[0] = 0.85f; spec[1] = 0.85f; spec[2] = 0.80f; spec[3] = 1.0f;
        pos[0] = -85.0f; pos[1] = 145.0f; pos[2] = 90.0f; pos[3] = 0.0f;
    } else if (Config::timeMode == 1) {
        fogColor[0] = 0.86f; fogColor[1] = 0.56f; fogColor[2] = 0.42f; fogColor[3] = 1.0f;
        amb[0] = 0.38f; amb[1] = 0.26f; amb[2] = 0.28f; amb[3] = 1.0f;
        dif[0] = 1.00f; dif[1] = 0.62f; dif[2] = 0.36f; dif[3] = 1.0f;
        spec[0] = 0.95f; spec[1] = 0.65f; spec[2] = 0.42f; spec[3] = 1.0f;
        pos[0] = -140.0f; pos[1] = 45.0f; pos[2] = 75.0f; pos[3] = 0.0f;
    } else {
        fogColor[0] = 0.05f; fogColor[1] = 0.08f; fogColor[2] = 0.16f; fogColor[3] = 1.0f;
        amb[0] = 0.14f; amb[1] = 0.18f; amb[2] = 0.28f; amb[3] = 1.0f;
        dif[0] = 0.30f; dif[1] = 0.40f; dif[2] = 0.62f; dif[3] = 1.0f;
        spec[0] = 0.45f; spec[1] = 0.52f; spec[2] = 0.70f; spec[3] = 1.0f;
        pos[0] = 70.0f; pos[1] = 110.0f; pos[2] = -60.0f; pos[3] = 0.0f;
    }

    glClearColor(fogColor[0], fogColor[1], fogColor[2], 1.0f);
    glFogfv(GL_FOG_COLOR, fogColor);
    glFogf(GL_FOG_DENSITY, Config::timeMode == 2 ? 0.0024f : 0.0017f);
    glLightfv(GL_LIGHT0, GL_AMBIENT, amb);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, dif);
    glLightfv(GL_LIGHT0, GL_SPECULAR, spec);
    glLightfv(GL_LIGHT0, GL_POSITION, pos);
}

void renderSkyDome() {
    glDisable(GL_LIGHTING);
    glDisable(GL_FOG);
    glDepthMask(GL_FALSE);

    float top[3], hor[3];
    if (Config::timeMode == 0) {
        top[0] = 0.24f; top[1] = 0.54f; top[2] = 0.88f;
        hor[0] = 0.72f; hor[1] = 0.87f; hor[2] = 0.97f;
    } else if (Config::timeMode == 1) {
        top[0] = 0.20f; top[1] = 0.25f; top[2] = 0.48f;
        hor[0] = 0.94f; hor[1] = 0.54f; hor[2] = 0.36f;
    } else {
        top[0] = 0.02f; top[1] = 0.04f; top[2] = 0.10f;
        hor[0] = 0.07f; hor[1] = 0.11f; hor[2] = 0.22f;
    }

    const float R = 480.0f, PI = 3.14159265f;
    for (int i = 0; i < 12; ++i) {
        float t0 = (float)i / 12.0f, t1 = (float)(i + 1) / 12.0f;
        float p0 = t0 * (PI * 0.5f), p1 = t1 * (PI * 0.5f);
        glBegin(GL_QUAD_STRIP);
        for (int j = 0; j <= 32; ++j) {
            float th = (float)j / 32.0f * 2.0f * PI;
            float ct = cosf(th), st = sinf(th);
            glColor3f(hor[0] + (top[0] - hor[0]) * t1, hor[1] + (top[1] - hor[1]) * t1, hor[2] + (top[2] - hor[2]) * t1);
            glVertex3f(R * cosf(p1) * ct, R * sinf(p1), R * cosf(p1) * st);
            glColor3f(hor[0] + (top[0] - hor[0]) * t0, hor[1] + (top[1] - hor[1]) * t0, hor[2] + (top[2] - hor[2]) * t0);
            glVertex3f(R * cosf(p0) * ct, R * sinf(p0), R * cosf(p0) * st);
        }
        glEnd();
    }

    glDepthMask(GL_TRUE);
    glEnable(GL_FOG);
    glEnable(GL_LIGHTING);
}

// ================== Step 1: 3D Airport Environment ==================

void renderGround() {
    setMat(0.22f, 0.46f, 0.21f, 4.0f, 0.04f);
    drawQuadXZ(-360.0f, -360.0f, 360.0f, 360.0f, -0.02f);

    setMat(0.26f, 0.55f, 0.24f, 6.0f, 0.05f);
    drawQuadXZ(-120.0f, -120.0f, 120.0f, 80.0f, 0.0f);

    setMat(0.23f, 0.50f, 0.22f, 6.0f, 0.04f);
    for (float x = -110.0f; x < 110.0f; x += 20.0f) {
        drawQuadXZ(x, 22.0f, x + 10.0f, 33.0f, 0.01f);
        drawQuadXZ(x, 47.0f, x + 10.0f, 65.0f, 0.01f);
    }
}

void renderRunway(float zCenter, bool occupied) {
    float glow = (Config::timeMode == 0) ? 0.15f : 0.95f;

    setMat(0.23f, 0.24f, 0.27f, 10.0f, 0.08f);
    drawQuadXZ(-98.0f, zCenter - 6.2f, 98.0f, zCenter + 6.2f, 0.03f);

    setMat(0.14f, 0.15f, 0.18f, 18.0f, 0.16f);
    drawQuadXZ(-95.0f, zCenter - 5.0f, 95.0f, zCenter + 5.0f, 0.05f);

    setMat(0.95f, 0.95f, 0.96f, 12.0f, 0.12f);
    for (float x = -80.0f; x < 80.0f; x += 10.0f) {
        drawQuadXZ(x, zCenter - 0.32f, x + 5.0f, zCenter + 0.32f, 0.08f);
    }
    drawRectOutline(-95.0f, zCenter - 4.85f, 95.0f, zCenter + 4.85f, 0.30f, 0.08f);

    for (int i = -2; i <= 2; ++i) {
        drawQuadXZ(-92.0f, zCenter + i * 1.6f - 0.38f, -87.0f, zCenter + i * 1.6f + 0.38f, 0.08f);
        drawQuadXZ(87.0f,  zCenter + i * 1.6f - 0.38f, 92.0f,  zCenter + i * 1.6f + 0.38f, 0.08f);
    }

    for (int side = -1; side <= 1; side += 2) {
        float ax = side * 66.0f;
        drawQuadXZ(ax - 3.0f, zCenter - 2.6f, ax + 3.0f, zCenter - 1.5f, 0.08f);
        drawQuadXZ(ax - 3.0f, zCenter + 1.5f, ax + 3.0f, zCenter + 2.6f, 0.08f);
    }

    for (float x = -90.0f; x <= 90.0f; x += 15.0f) {
        drawLightBulb(x, 0.25f, zCenter - 5.2f, 0.20f, 1.0f, 0.96f, 0.80f, glow);
        drawLightBulb(x, 0.25f, zCenter + 5.2f, 0.20f, 1.0f, 0.96f, 0.80f, glow);
    }
    for (int i = -2; i <= 2; ++i) {
        if (occupied) {
            drawLightBulb(-95.3f, 0.30f, zCenter + i * 1.8f, 0.24f, 1.0f, 0.20f, 0.20f, 0.95f);
        } else {
            drawLightBulb(-95.3f, 0.30f, zCenter + i * 1.8f, 0.24f, 0.20f, 1.0f, 0.45f, glow);
        }
        drawLightBulb(95.3f, 0.25f, zCenter + i * 1.8f, 0.22f, 1.0f, 0.25f, 0.25f, glow);
    }
}

void renderTaxiways() {
    float blueGlow = (Config::timeMode == 0) ? 0.15f : 0.95f;
    setMat(0.21f, 0.22f, 0.25f, 14.0f, 0.12f);
    drawQuadXZ(-95.0f, -8.2f, 95.0f, 0.2f, 0.05f);

    for (int s = -1; s <= 1; s += 2) {
        float xPos = 70.0f * s;
        drawQuadXZ(xPos - 4.0f, 0.2f, xPos + 4.0f, 10.0f, 0.05f);
        drawQuadXZ(xPos - 4.0f, 20.0f, xPos + 4.0f, 35.0f, 0.05f);
    }

    setMat(1.0f, 0.82f, 0.08f, 10.0f, 0.10f);
    drawQuadXZ(-95.0f, -4.15f, 95.0f, -3.85f, 0.08f);
    for (int s = -1; s <= 1; s += 2) {
        float xPos = 70.0f * s;
        drawQuadXZ(xPos - 0.16f, -4.0f, xPos + 0.16f, 10.0f, 0.08f);
        drawQuadXZ(xPos - 0.16f, 20.0f, xPos + 0.16f, 35.0f, 0.08f);
    }

    for (float x = -90.0f; x <= 90.0f; x += 15.0f) {
        drawLightBulb(x, 0.22f, -8.4f, 0.16f, 0.22f, 0.65f, 1.0f, blueGlow);
        drawLightBulb(x, 0.22f,  0.4f, 0.16f, 0.22f, 0.65f, 1.0f, blueGlow);
    }
}

void renderApronAndBays() {
    setMat(0.40f, 0.41f, 0.44f, 12.0f, 0.10f);
    drawQuadXZ(-52.0f, -34.0f, 52.0f, -8.2f, 0.05f);

    for (int i = 0; i < 4; ++i) {
        float bx = AirportManager::bayX[i];
        setMat(1.0f, 0.82f, 0.08f, 10.0f, 0.08f);
        drawRectOutline(bx - 7.5f, -30.0f, bx + 7.5f, -10.0f, 0.30f, 0.08f);
        drawQuadXZ(bx - 0.16f, -26.0f, bx + 0.16f, -8.2f, 0.08f);
        drawQuadXZ(bx - 1.8f, -25.2f, bx + 1.8f, -24.8f, 0.08f);

        setMat(0.75f, 0.77f, 0.82f, 36.0f, 0.35f);
        drawCylinderY(bx, 0.0f, -33.2f, 1.25f, 1.25f, 6.0f, 20);
        drawBox(bx, 4.9f, -28.6f, 2.8f, 2.1f, 9.0f);

        setMat(0.24f, 0.28f, 0.34f, 28.0f, 0.30f);
        drawBox(bx, 4.9f, -23.8f, 3.1f, 2.3f, 1.2f);
        drawCylinderY(bx, 0.0f, -25.6f, 0.42f, 0.42f, 3.8f, 12);
    }
}

void renderTerminal() {
    float glassGlow = (Config::timeMode == 0) ? 0.08f : 0.65f;

    setMat(0.89f, 0.90f, 0.93f, 28.0f, 0.25f);
    drawBox(0.0f, 4.2f, -42.0f, 74.0f, 8.4f, 16.0f);

    setMat(0.20f, 0.25f, 0.33f, 48.0f, 0.45f);
    drawBox(0.0f, 8.8f, -42.0f, 78.0f, 0.85f, 19.5f);
    setMat(0.55f, 0.58f, 0.64f, 52.0f, 0.55f);
    for (float x = -35.0f; x <= 35.0f; x += 10.0f) {
        drawBox(x, 9.4f, -42.0f, 0.5f, 0.4f, 19.8f);
    }

    setMat(0.38f, 0.67f, 0.88f, 96.0f, 0.85f, glassGlow);
    drawBox(0.0f, 4.2f, -33.85f, 70.0f, 6.4f, 0.32f);
    resetEmit();

    setMat(0.22f, 0.25f, 0.30f, 40.0f, 0.50f);
    for (float x = -35.0f; x <= 35.0f; x += 5.0f) {
        drawBox(x, 4.2f, -33.65f, 0.26f, 6.6f, 0.40f);
    }

    setMat(0.24f, 0.28f, 0.34f, 36.0f, 0.40f);
    drawBox(0.0f, 3.6f, -52.0f, 14.0f, 0.45f, 4.2f);
    setMat(0.84f, 0.85f, 0.88f, 30.0f, 0.30f);
    drawCylinderY(-6.0f, 0.0f, -53.5f, 0.28f, 0.28f, 3.6f, 12);
    drawCylinderY( 6.0f, 0.0f, -53.5f, 0.28f, 0.28f, 3.6f, 12);
}

void renderControlTower() {
    float glassGlow = (Config::timeMode == 0) ? 0.12f : 0.75f;
    const float tx = 58.0f, tz = -42.0f;

    setMat(0.84f, 0.85f, 0.88f, 24.0f, 0.25f);
    drawCylinderY(tx, 0.0f, tz, 3.3f, 2.2f, 22.5f, 28);
    setMat(0.28f, 0.32f, 0.38f, 36.0f, 0.40f);
    drawCylinderY(tx, 22.2f, tz, 2.4f, 4.5f, 1.8f, 24);

    setMat(0.30f, 0.62f, 0.84f, 96.0f, 0.85f, glassGlow);
    drawCylinderY(tx, 24.0f, tz, 4.3f, 5.1f, 4.2f, 8);
    resetEmit();

    setMat(0.75f, 0.16f, 0.16f, 40.0f, 0.45f);
    drawCylinderY(tx, 28.2f, tz, 5.5f, 4.7f, 0.85f, 8);

    setMat(0.92f, 0.92f, 0.94f, 48.0f, 0.60f);
    drawCylinderY(tx, 29.0f, tz, 0.20f, 0.15f, 5.2f, 14);

    glPushMatrix();
    glTranslatef(tx, 33.4f, tz);
    glRotatef(Config::simTime * 55.0f, 0.0f, 1.0f, 0.0f);
    setMat(0.95f, 0.95f, 0.98f, 40.0f, 0.55f);
    drawBox(0.0f, 0.0f, 0.0f, 4.2f, 0.55f, 0.35f);
    glPopMatrix();

    float blink = (sinf(Config::simTime * 4.5f) > 0.0f) ? 1.0f : 0.15f;
    drawLightBulb(tx, 34.4f, tz, 0.30f, 1.0f, 0.12f, 0.12f, blink);
}

void drawSedan(float x, float z, float r, float g, float b) {
    setMat(r, g, b, 64.0f, 0.65f);
    drawBox(x, 0.85f, z, 3.2f, 1.15f, 6.0f);
    setMat(0.16f, 0.24f, 0.34f, 96.0f, 0.85f);
    drawBox(x, 1.75f, z - 0.15f, 2.55f, 0.80f, 3.1f);
    setMat(0.10f, 0.10f, 0.12f, 12.0f, 0.15f);
    for (int wx = -1; wx <= 1; wx += 2) {
        for (int wz = -1; wz <= 1; wz += 2) {
            drawBox(x + wx * 1.55f, 0.42f, z + wz * 1.85f, 0.34f, 0.84f, 0.84f);
        }
    }
}

void renderRoadsAndParking() {
    setMat(0.17f, 0.18f, 0.21f, 12.0f, 0.10f);
    drawQuadXZ(-100.0f, -68.0f, 100.0f, -61.0f, 0.05f);
    drawQuadXZ(-3.8f, -61.0f, 3.8f, -50.0f, 0.05f);

    setMat(0.95f, 0.95f, 0.96f, 10.0f, 0.08f);
    for (float x = -95.0f; x < 95.0f; x += 9.0f) {
        drawQuadXZ(x, -64.65f, x + 4.5f, -64.25f, 0.08f);
    }

    setMat(0.28f, 0.29f, 0.32f, 10.0f, 0.08f);
    drawQuadXZ(-92.0f, -98.0f, -32.0f, -70.0f, 0.05f);

    setMat(0.94f, 0.94f, 0.94f, 8.0f, 0.06f);
    for (float x = -89.0f; x <= -32.0f; x += 5.5f) {
        drawQuadXZ(x, -95.0f, x + 0.18f, -84.0f, 0.08f);
        drawQuadXZ(x, -82.0f, x + 0.18f, -72.0f, 0.08f);
    }

    float carR[5] = { 0.85f, 0.14f, 0.22f, 0.92f, 0.28f };
    float carG[5] = { 0.14f, 0.45f, 0.65f, 0.72f, 0.30f };
    float carB[5] = { 0.14f, 0.85f, 0.24f, 0.14f, 0.32f };
    for (int i = 0; i < 5; ++i) {
        drawSedan(-86.0f + i * 11.0f, -89.5f, carR[i], carG[i], carB[i]);
    }
}

void renderServiceZone() {
    setMat(0.42f, 0.42f, 0.40f, 10.0f, 0.08f);
    drawQuadXZ(42.0f, -98.0f, 95.0f, -70.0f, 0.05f);

    setMat(0.64f, 0.66f, 0.72f, 32.0f, 0.35f);
    drawBox(60.0f, 5.2f, -84.0f, 22.0f, 10.5f, 17.0f);
    setMat(0.26f, 0.30f, 0.36f, 44.0f, 0.45f);
    drawBox(60.0f, 10.8f, -84.0f, 23.4f, 0.85f, 18.4f);
    setMat(0.14f, 0.15f, 0.18f, 10.0f, 0.10f);
    drawBox(60.0f, 4.0f, -75.3f, 15.0f, 7.5f, 0.45f);

    setMat(0.80f, 0.58f, 0.22f, 24.0f, 0.25f);
    drawBox(84.0f, 3.2f, -82.0f, 13.0f, 6.4f, 10.5f);
    setMat(0.28f, 0.30f, 0.34f, 32.0f, 0.35f);
    drawBox(84.0f, 6.6f, -82.0f, 13.8f, 0.55f, 11.2f);

    for (int i = 0; i < 2; ++i) {
        float fx = 48.0f + i * 7.0f;
        setMat(0.92f, 0.93f, 0.95f, 56.0f, 0.55f);
        drawCylinderY(fx, 0.0f, -74.0f, 2.3f, 2.3f, 5.2f, 28);
        setMat(0.24f, 0.26f, 0.30f, 40.0f, 0.45f);
        drawCylinderY(fx, 2.4f, -74.0f, 2.36f, 2.36f, 0.32f, 28);
    }
}

// ================== Step 2: 3D Aircraft Rendering ==================

void drawAirliner(const Aircraft& ac) {
    if (!g_Quadric) return;
    glPushMatrix();
    glTranslatef(ac.x, ac.y + 1.8f, ac.z);
    glRotatef(ac.yawDeg, 0.0f, 1.0f, 0.0f);
    glRotatef(ac.pitchDeg, 0.0f, 0.0f, 1.0f);

    setMat(0.95f, 0.96f, 0.98f, 64.0f, 0.60f);
    glPushMatrix();
    glTranslatef(-5.0f, 0.0f, 0.0f);
    glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
    gluCylinder(g_Quadric, 1.15f, 1.15f, 10.5f, 22, 1);
    glTranslatef(0.0f, 0.0f, 10.5f);
    gluCylinder(g_Quadric, 1.15f, 0.16f, 3.0f, 22, 1);
    glPopMatrix();

    glPushMatrix();
    glTranslatef(-5.0f, 0.0f, 0.0f);
    glRotatef(-90.0f, 0.0f, 1.0f, 0.0f);
    gluCylinder(g_Quadric, 1.15f, 0.22f, 3.4f, 22, 1);
    glPopMatrix();

    setMat(0.10f, 0.18f, 0.28f, 96.0f, 0.90f);
    drawBox(5.6f, 0.55f, 0.0f, 1.1f, 0.5f, 1.25f);

    setMat(0.84f, 0.86f, 0.90f, 48.0f, 0.50f);
    for (int side = -1; side <= 1; side += 2) {
        glPushMatrix();
        glTranslatef(0.3f, -0.25f, side * 4.6f);
        glRotatef(side * -18.0f, 0.0f, 1.0f, 0.0f);
        drawBox(0.0f, 0.0f, 0.0f, 2.5f, 0.20f, 8.4f);
        glPopMatrix();

        setMat(0.30f, 0.33f, 0.38f, 64.0f, 0.65f);
        glPushMatrix();
        glTranslatef(0.2f, -0.82f, side * 3.0f);
        glRotatef(90.0f, 0.0f, 1.0f, 0.0f);
        gluCylinder(g_Quadric, 0.48f, 0.42f, 2.0f, 16, 1);
        glPopMatrix();

        setMat(0.84f, 0.86f, 0.90f, 48.0f, 0.50f);
        drawBox(-6.6f, 0.35f, side * 1.7f, 1.3f, 0.16f, 2.9f);
    }

    setMat(ac.rColor, ac.gColor, ac.bColor, 40.0f, 0.45f);
    drawBox(-6.4f, 1.9f, 0.0f, 2.0f, 2.9f, 0.22f);

    glPopMatrix();
}

void renderAllAircraft() {
    for (size_t i = 0; i < AirportManager::activeAircraft.size(); ++i) {
        drawAirliner(AirportManager::activeAircraft[i]);
    }
}

// ================== On-Screen Status HUD (Crash-Proof Vector Font) ==================

inline void drawSeg(float ox, float oy, float s, float x1, float y1, float x2, float y2) {
    glVertex2f(ox + x1 * s, oy + y1 * s);
    glVertex2f(ox + x2 * s, oy + y2 * s);
}

void drawVectorChar(float x, float y, float s, char ch) {
    if (ch >= 'a' && ch <= 'z') ch = ch - 'a' + 'A';
    switch (ch) {
        case 'A': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0); drawSeg(x,y,s,0,0.7f,1,0.7f); break;
        case 'B': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,0.8f,1.4f); drawSeg(x,y,s,0.8f,1.4f,0.8f,0.7f); drawSeg(x,y,s,0,0.7f,0.9f,0.7f); drawSeg(x,y,s,0.9f,0.7f,0.9f,0); drawSeg(x,y,s,0,0,0.9f,0); break;
        case 'C': drawSeg(x,y,s,1,1.4f,0,1.4f); drawSeg(x,y,s,0,1.4f,0,0); drawSeg(x,y,s,0,0,1,0); break;
        case 'D': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,0.8f,1.2f); drawSeg(x,y,s,0.8f,1.2f,0.8f,0.2f); drawSeg(x,y,s,0.8f,0.2f,0,0); break;
        case 'E': drawSeg(x,y,s,1,1.4f,0,1.4f); drawSeg(x,y,s,0,1.4f,0,0); drawSeg(x,y,s,0,0,1,0); drawSeg(x,y,s,0,0.7f,0.8f,0.7f); break;
        case 'F': drawSeg(x,y,s,1,1.4f,0,1.4f); drawSeg(x,y,s,0,1.4f,0,0); drawSeg(x,y,s,0,0.7f,0.8f,0.7f); break;
        case 'G': drawSeg(x,y,s,1,1.4f,0,1.4f); drawSeg(x,y,s,0,1.4f,0,0); drawSeg(x,y,s,0,0,1,0); drawSeg(x,y,s,1,0,1,0.7f); drawSeg(x,y,s,1,0.7f,0.5f,0.7f); break;
        case 'H': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,1,0,1,1.4f); drawSeg(x,y,s,0,0.7f,1,0.7f); break;
        case 'I': drawSeg(x,y,s,0.2f,1.4f,0.8f,1.4f); drawSeg(x,y,s,0.5f,1.4f,0.5f,0); drawSeg(x,y,s,0.2f,0,0.8f,0); break;
        case 'J': drawSeg(x,y,s,1,1.4f,1,0); drawSeg(x,y,s,1,0,0,0); drawSeg(x,y,s,0,0,0,0.4f); break;
        case 'K': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,1,1.4f,0,0.7f); drawSeg(x,y,s,0,0.7f,1,0); break;
        case 'L': drawSeg(x,y,s,0,1.4f,0,0); drawSeg(x,y,s,0,0,1,0); break;
        case 'M': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,0.5f,0.7f); drawSeg(x,y,s,0.5f,0.7f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0); break;
        case 'N': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,1,0); drawSeg(x,y,s,1,0,1,1.4f); break;
        case 'O': case '0': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0); drawSeg(x,y,s,1,0,0,0); break;
        case 'P': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0.7f); drawSeg(x,y,s,1,0.7f,0,0.7f); break;
        case 'Q': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0); drawSeg(x,y,s,1,0,0,0); drawSeg(x,y,s,0.5f,0.4f,1.1f,-0.1f); break;
        case 'R': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0.7f); drawSeg(x,y,s,1,0.7f,0,0.7f); drawSeg(x,y,s,0.3f,0.7f,1,0); break;
        case 'S': case '5': drawSeg(x,y,s,1,1.4f,0,1.4f); drawSeg(x,y,s,0,1.4f,0,0.7f); drawSeg(x,y,s,0,0.7f,1,0.7f); drawSeg(x,y,s,1,0.7f,1,0); drawSeg(x,y,s,1,0,0,0); break;
        case 'T': drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,0.5f,1.4f,0.5f,0); break;
        case 'U': drawSeg(x,y,s,0,1.4f,0,0); drawSeg(x,y,s,0,0,1,0); drawSeg(x,y,s,1,0,1,1.4f); break;
        case 'V': drawSeg(x,y,s,0,1.4f,0.5f,0); drawSeg(x,y,s,0.5f,0,1,1.4f); break;
        case 'W': drawSeg(x,y,s,0,1.4f,0.25f,0); drawSeg(x,y,s,0.25f,0,0.5f,0.8f); drawSeg(x,y,s,0.5f,0.8f,0.75f,0); drawSeg(x,y,s,0.75f,0,1,1.4f); break;
        case 'X': drawSeg(x,y,s,0,1.4f,1,0); drawSeg(x,y,s,1,1.4f,0,0); break;
        case 'Y': drawSeg(x,y,s,0,1.4f,0.5f,0.7f); drawSeg(x,y,s,1,1.4f,0.5f,0.7f); drawSeg(x,y,s,0.5f,0.7f,0.5f,0); break;
        case 'Z': case '2': drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0.7f); drawSeg(x,y,s,1,0.7f,0,0.7f); drawSeg(x,y,s,0,0.7f,0,0); drawSeg(x,y,s,0,0,1,0); break;
        case '1': drawSeg(x,y,s,0.5f,1.4f,0.5f,0); drawSeg(x,y,s,0.2f,1.1f,0.5f,1.4f); drawSeg(x,y,s,0.2f,0,0.8f,0); break;
        case '3': drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0); drawSeg(x,y,s,0,0.7f,1,0.7f); drawSeg(x,y,s,0,0,1,0); break;
        case '4': drawSeg(x,y,s,0,1.4f,0,0.7f); drawSeg(x,y,s,0,0.7f,1,0.7f); drawSeg(x,y,s,1,1.4f,1,0); break;
        case '6': drawSeg(x,y,s,1,1.4f,0,1.4f); drawSeg(x,y,s,0,1.4f,0,0); drawSeg(x,y,s,0,0,1,0); drawSeg(x,y,s,1,0,1,0.7f); drawSeg(x,y,s,1,0.7f,0,0.7f); break;
        case '7': drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,0.4f,0); break;
        case '8': drawSeg(x,y,s,0,0,0,1.4f); drawSeg(x,y,s,0,1.4f,1,1.4f); drawSeg(x,y,s,1,1.4f,1,0); drawSeg(x,y,s,1,0,0,0); drawSeg(x,y,s,0,0.7f,1,0.7f); break;
        case '9': drawSeg(x,y,s,0,0,1,0); drawSeg(x,y,s,1,0,1,1.4f); drawSeg(x,y,s,1,1.4f,0,1.4f); drawSeg(x,y,s,0,1.4f,0,0.7f); drawSeg(x,y,s,0,0.7f,1,0.7f); break;
        case ':': drawSeg(x,y,s,0.5f,1.0f,0.5f,0.85f); drawSeg(x,y,s,0.5f,0.35f,0.5f,0.2f); break;
        case '-': drawSeg(x,y,s,0.15f,0.7f,0.85f,0.7f); break;
        case '/': drawSeg(x,y,s,0.1f,0,0.9f,1.4f); break;
        case '|': drawSeg(x,y,s,0.5f,0,0.5f,1.4f); break;
        case '[': drawSeg(x,y,s,0.7f,1.4f,0.2f,1.4f); drawSeg(x,y,s,0.2f,1.4f,0.2f,0); drawSeg(x,y,s,0.2f,0,0.7f,0); break;
        case ']': drawSeg(x,y,s,0.3f,1.4f,0.8f,1.4f); drawSeg(x,y,s,0.8f,1.4f,0.8f,0); drawSeg(x,y,s,0.8f,0,0.3f,0); break;
        case '(': drawSeg(x,y,s,0.7f,1.4f,0.3f,0.9f); drawSeg(x,y,s,0.3f,0.9f,0.3f,0.5f); drawSeg(x,y,s,0.3f,0.5f,0.7f,0); break;
        case ')': drawSeg(x,y,s,0.3f,1.4f,0.7f,0.9f); drawSeg(x,y,s,0.7f,0.9f,0.7f,0.5f); drawSeg(x,y,s,0.7f,0.5f,0.3f,0); break;
        case '%': drawSeg(x,y,s,0.1f,0,0.9f,1.4f); drawSeg(x,y,s,0.1f,1.1f,0.3f,1.3f); drawSeg(x,y,s,0.7f,0.1f,0.9f,0.3f); break;
        case ',': drawSeg(x,y,s,0.5f,0.2f,0.3f,-0.2f); break;
        case '.': drawSeg(x,y,s,0.45f,0.0f,0.55f,0.15f); break;
        case '+': drawSeg(x,y,s,0.15f,0.7f,0.85f,0.7f); drawSeg(x,y,s,0.5f,0.35f,0.5f,1.05f); break;
        case '=': drawSeg(x,y,s,0.15f,0.9f,0.85f,0.9f); drawSeg(x,y,s,0.15f,0.5f,0.85f,0.5f); break;
        default: break;
    }
}

void drawText2D(float x, float y, const char* text, float scale = 5.5f) {
    glLineWidth(1.2f);
    glBegin(GL_LINES);
    for (int i = 0; text[i] != '\0'; ++i) {
        drawVectorChar(x + i * (scale * 1.45f), y, scale, text[i]);
    }
    glEnd();
}

void renderOverlayHUD() {
    if (!Config::showStats) return;

    glDisable(GL_LIGHTING);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_FOG);

    glMatrixMode(GL_PROJECTION);
    glPushMatrix();
    glLoadIdentity();
    gluOrtho2D(0, Config::windowW, 0, Config::windowH);

    glMatrixMode(GL_MODELVIEW);
    glPushMatrix();
    glLoadIdentity();

    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);
    glColor4f(0.05f, 0.08f, 0.14f, 0.80f);
    glBegin(GL_QUADS);
    glVertex2f(16, Config::windowH - 16);
    glVertex2f(640, Config::windowH - 16);
    glVertex2f(640, Config::windowH - 154);
    glVertex2f(16, Config::windowH - 154);
    glEnd();
    glDisable(GL_BLEND);

    glColor3f(0.95f, 0.97f, 1.0f);
    drawText2D(28, Config::windowH - 40, "AIRPORT MANAGEMENT SYSTEM (50% MILESTONE: STEPS 1-3)");

    char line1[128], line2[128], line3[128], line4[128];
    std::snprintf(line1, sizeof(line1), "SIM: %s | ACTIVE: %d | QUEUED: %d | DEPARTED: %d",
                  Config::isRunning ? "RUNNING" : "PAUSED",
                  (int)AirportManager::activeAircraft.size(),
                  (int)AirportManager::flightQueue.size(),
                  AirportManager::totalCompleted);
    std::snprintf(line2, sizeof(line2), "RUNWAY 1: %s | RUNWAY 2: %s",
                  AirportManager::runwayOccupied[0] ? "OCCUPIED" : "FREE",
                  AirportManager::runwayOccupied[1] ? "OCCUPIED" : "FREE");
    std::snprintf(line3, sizeof(line3), "EVENT: %s", AirportManager::lastEventMsg.c_str());
    std::snprintf(line4, sizeof(line4), "KEYS: [A] ADD FLIGHT [L] LANDING [R] ASSIGN [S/P] RUN/PAUSE [1/2/3] SKY");

    glColor3f(0.75f, 0.88f, 1.0f);
    drawText2D(28, Config::windowH - 64, line1);
    glColor3f(0.45f, 0.95f, 0.65f);
    drawText2D(28, Config::windowH - 86, line2);
    glColor3f(1.0f, 0.85f, 0.45f);
    drawText2D(28, Config::windowH - 108, line3);
    glColor3f(0.75f, 0.78f, 0.84f);
    drawText2D(28, Config::windowH - 134, line4);

    glPopMatrix();
    glMatrixMode(GL_PROJECTION);
    glPopMatrix();
    glMatrixMode(GL_MODELVIEW);

    glEnable(GL_FOG);
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
}

// ================== GLUT Callbacks & Right-Click Menu ==================

void display() {
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glMatrixMode(GL_MODELVIEW);
    glLoadIdentity();

    float eyeX = Config::targetX + Config::distance * cosf(Config::pitch) * sinf(Config::yaw);
    float eyeY = Config::targetY + Config::distance * sinf(Config::pitch);
    float eyeZ = Config::targetZ + Config::distance * cosf(Config::pitch) * cosf(Config::yaw);

    gluLookAt(eyeX, eyeY, eyeZ, Config::targetX, Config::targetY, Config::targetZ, 0.0f, 1.0f, 0.0f);

    applyAtmosphere();
    renderSkyDome();
    renderGround();
    renderRunway(15.0f, AirportManager::runwayOccupied[0]);
    renderRunway(40.0f, AirportManager::runwayOccupied[1]);
    renderTaxiways();
    renderApronAndBays();
    renderTerminal();
    renderControlTower();
    renderRoadsAndParking();
    renderServiceZone();
    renderAllAircraft();
    renderOverlayHUD();

    glutSwapBuffers();
}

void reshape(int width, int height) {
    if (height == 0) height = 1;
    Config::windowW = width;
    Config::windowH = height;
    glViewport(0, 0, width, height);
    glMatrixMode(GL_PROJECTION);
    glLoadIdentity();
    gluPerspective(44.0, (double)width / height, 1.0, 1200.0);
    glMatrixMode(GL_MODELVIEW);
}

void keyboard(unsigned char key, int, int) {
    switch (key) {
        case 27: if (g_Quadric) gluDeleteQuadric(g_Quadric); exit(0);
        case 'a': case 'A': AirportManager::enqueueFlight(FLIGHT_TAKEOFF, 1); break;
        case 'l': case 'L': AirportManager::enqueueFlight(FLIGHT_LANDING, 2); break;
        case 'r': case 'R': AirportManager::processRunwayQueue(); break;
        case 's': case 'S': Config::isRunning = true; break;
        case 'p': case 'P': Config::isRunning = false; break;
        case 't': case 'T': Config::showStats = !Config::showStats; break;
        case '1': Config::timeMode = 0; break;
        case '2': Config::timeMode = 1; break;
        case '3': Config::timeMode = 2; break;
        case '+': case '=': Config::targetDistance -= 10.0f; break;
        case '-': case '_': Config::targetDistance += 10.0f; break;
        case 'c': case 'C':
            Config::targetYaw = 0.60f; Config::targetPitch = 0.58f; Config::targetDistance = 185.0f;
            break;
    }
    Config::targetDistance = Config::clampf(Config::targetDistance, 40.0f, 450.0f);
}

void specialKeys(int key, int, int) {
    if (key == GLUT_KEY_LEFT)  Config::targetYaw -= 0.06f;
    if (key == GLUT_KEY_RIGHT) Config::targetYaw += 0.06f;
    if (key == GLUT_KEY_UP)    Config::targetPitch += 0.05f;
    if (key == GLUT_KEY_DOWN)  Config::targetPitch -= 0.05f;
    Config::targetPitch = Config::clampf(Config::targetPitch, 0.08f, 1.50f);
}

void mouse(int button, int state, int x, int y) {
    if (button == GLUT_LEFT_BUTTON) {
        Config::isDragging = (state == GLUT_DOWN);
        Config::lastMouseX = x;
        Config::lastMouseY = y;
    }
}

void mouseWheel(int, int dir, int, int) {
    Config::targetDistance -= dir * 12.0f;
    Config::targetDistance = Config::clampf(Config::targetDistance, 40.0f, 450.0f);
}

void motion(int x, int y) {
    if (Config::isDragging) {
        Config::targetYaw   -= (float)(x - Config::lastMouseX) * 0.005f;
        Config::targetPitch += (float)(y - Config::lastMouseY) * 0.005f;
        Config::targetPitch = Config::clampf(Config::targetPitch, 0.08f, 1.50f);
        Config::lastMouseX = x;
        Config::lastMouseY = y;
    }
}

void menuCallback(int item) {
    switch (item) {
        case 1: AirportManager::enqueueFlight(FLIGHT_TAKEOFF, 1); break;
        case 2: AirportManager::enqueueFlight(FLIGHT_LANDING, 2); break;
        case 3: AirportManager::processRunwayQueue(); break;
        case 4: Config::isRunning = !Config::isRunning; break;
        case 5: Config::showStats = !Config::showStats; break;
        case 6: Config::timeMode = 0; break;
        case 7: Config::timeMode = 1; break;
        case 8: Config::timeMode = 2; break;
    }
}

void createRightClickMenu() {
    glutCreateMenu(menuCallback);
    glutAddMenuEntry("Add Takeoff Flight (A)", 1);
    glutAddMenuEntry("Start Landing (L)", 2);
    glutAddMenuEntry("Assign Runway (R)", 3);
    glutAddMenuEntry("Start / Pause Simulation (S/P)", 4);
    glutAddMenuEntry("Toggle Statistics HUD (T)", 5);
    glutAddMenuEntry("Atmosphere: Day (1)", 6);
    glutAddMenuEntry("Atmosphere: Sunset (2)", 7);
    glutAddMenuEntry("Atmosphere: Night (3)", 8);
    glutAttachMenu(GLUT_RIGHT_BUTTON);
}

void timer(int) {
    const float dt = 0.016f;
    Config::simTime += dt;

    AirportManager::updateSimulation(dt);

    const float k = 0.14f;
    Config::yaw      += (Config::targetYaw      - Config::yaw)      * k;
    Config::pitch    += (Config::targetPitch    - Config::pitch)    * k;
    Config::distance += (Config::targetDistance - Config::distance) * k;

    glutPostRedisplay();
    glutTimerFunc(16, timer, 0);
}

void initOpenGL() {
    glEnable(GL_DEPTH_TEST);
    glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);
    glEnable(GL_FOG);
    glFogi(GL_FOG_MODE, GL_EXP2);
    glEnable(GL_COLOR_MATERIAL);
    glColorMaterial(GL_FRONT_AND_BACK, GL_AMBIENT_AND_DIFFUSE);
    glEnable(GL_NORMALIZE);
    glShadeModel(GL_SMOOTH);

    g_Quadric = gluNewQuadric();
    if (g_Quadric) gluQuadricNormals(g_Quadric, GLU_SMOOTH);

    AirportManager::enqueueFlight(FLIGHT_LANDING, 2);
    AirportManager::enqueueFlight(FLIGHT_LANDING, 2);
    AirportManager::enqueueFlight(FLIGHT_TAKEOFF, 1);
}

int main(int argc, char** argv) {
    glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
    glutInitWindowSize(Config::windowW, Config::windowH);
    glutInitWindowPosition(60, 40);
    glutCreateWindow("Airport Management System - 50% Milestone (CSE-4742)");

    initOpenGL();
    createRightClickMenu();

    glutDisplayFunc(display);
    glutReshapeFunc(reshape);
    glutKeyboardFunc(keyboard);
    glutSpecialFunc(specialKeys);
    glutMouseFunc(mouse);
    glutMouseWheelFunc(mouseWheel);
    glutMotionFunc(motion);
    glutTimerFunc(16, timer, 0);

    glutMainLoop();
    return 0;
}
