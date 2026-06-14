#include "stewart_kinematics.h"
#include <math.h>

#ifndef M_PI
#define M_PI 3.14159265358979323846f
#endif

#define DEG2RAD(d) ((d) * (M_PI / 180.0f))
#define RAD2DEG(r) ((r) * (180.0f / M_PI))

// Base servo-axis angles (deg) on the lid circle, CCW from +X.
static const float phi_base_deg[STEWART_NUM_LEGS] = {
    0.0f, 60.0f, 120.0f, 180.0f, 240.0f, 300.0f
};

// Platform joint angles (deg), CCW from +X, in the platform's own frame.
// Paired config: the 6 joints are grouped around 3 bisectors (30, 150, 270),
// each pair separated by 2*9.17 = 18.34 deg. This pulls neighboring legs closer
// so the mirrored cranks don't collide.
static const float phi_plat_deg[STEWART_NUM_LEGS] = {
    30.0f  -  9.17f, 30.0f  +  9.17f,   // pair 1 -> servos S0, S1
    150.0f -  9.17f, 150.0f +  9.17f,   // pair 2 -> servos S2, S3
    270.0f -  9.17f, 270.0f +  9.17f    // pair 3 -> servos S4, S5
};

// Crank rotation sense per servo. +1 = CCW tangential (standard), -1 = CW
// (mirrored). Paired config has the second servo of each pair mirrored so the
// two cranks in a pair "look toward each other" instead of both going CCW.
static const float servo_sens[STEWART_NUM_LEGS] = {
    +1.0f, -1.0f, +1.0f, -1.0f, +1.0f, -1.0f
};

// Rotation matrix R = Rz(yaw) * Ry(pitch) * Rx(roll), applied to a point in
// the platform frame to bring it into the world frame.
static void rot_zyx(float roll, float pitch, float yaw, float R[3][3]) {
    float cr = cosf(roll),  sr = sinf(roll);
    float cp = cosf(pitch), sp = sinf(pitch);
    float cy = cosf(yaw),   sy = sinf(yaw);

    R[0][0] = cy * cp;
    R[0][1] = cy * sp * sr - sy * cr;
    R[0][2] = cy * sp * cr + sy * sr;

    R[1][0] = sy * cp;
    R[1][1] = sy * sp * sr + cy * cr;
    R[1][2] = sy * sp * cr - cy * sr;

    R[2][0] = -sp;
    R[2][1] = cp * sr;
    R[2][2] = cp * cr;
}

int stewart_ik(float x, float y, float z,
               float roll_deg, float pitch_deg, float yaw_deg,
               float crank_deg_out[STEWART_NUM_LEGS],
               unsigned int *unreachable_mask_out) {
    int status = STEWART_OK;
    unsigned int unreachable_mask = 0;

    float R[3][3];
    rot_zyx(DEG2RAD(roll_deg), DEG2RAD(pitch_deg), DEG2RAD(yaw_deg), R);

    for (int i = 0; i < STEWART_NUM_LEGS; ++i) {
        float pb = DEG2RAD(phi_base_deg[i]);
        float pp = DEG2RAD(phi_plat_deg[i]);

        // Base joint (servo axis center) in world frame.
        float Bx = STEWART_RB * cosf(pb);
        float By = STEWART_RB * sinf(pb);
        float Bz = STEWART_ZAX;

        // Platform joint in the platform's local frame, then rotated + translated.
        float plx = STEWART_RP * cosf(pp);
        float ply = STEWART_RP * sinf(pp);
        float plz = 0.0f;

        float Px = R[0][0]*plx + R[0][1]*ply + R[0][2]*plz + x;
        float Py = R[1][0]*plx + R[1][1]*ply + R[1][2]*plz + y;
        float Pz = R[2][0]*plx + R[2][1]*ply + R[2][2]*plz + z;

        // L = P - B, expressed in the servo's local frame (radial, tangential, z).
        float Lx = Px - Bx, Ly = Py - By, Lz = Pz - Bz;
        // Radial unit vector points outward; tangential is CCW (same sense for all 6).
        float ur_x = cosf(pb),  ur_y = sinf(pb);
        float ut_x = -sinf(pb), ut_y = cosf(pb);
        float Lr = Lx * ur_x + Ly * ur_y;
        float Lt = Lx * ut_x + Ly * ut_y;
        // Lz unchanged.

        // The crank end relative to B is (radial=0, tangential=sens*La*cos(a),
        // z=La*sin(a)), where sens = +1 for CCW-mounted cranks and -1 for the
        // mirrored ones in a pair. Constraint |L - crank_end|^2 = Ls^2 gives:
        //   A*cos(a) + B*sin(a) = C
        // with A = 2*La*sens*Lt, B = 2*La*Lz, C = |L|^2 + La^2 - Ls^2.
        float A = 2.0f * STEWART_LA * servo_sens[i] * Lt;
        float B = 2.0f * STEWART_LA * Lz;
        float Lsq = Lr*Lr + Lt*Lt + Lz*Lz;
        float C = Lsq + STEWART_LA*STEWART_LA - STEWART_LS*STEWART_LS;

        float Rmag = sqrtf(A*A + B*B);
        float alpha_deg;
        if (Rmag < 1e-6f) {
            // Degenerate; B and Lt and Lz both ~0. Treat as unreachable.
            alpha_deg = 0.0f;
            unreachable_mask |= (1u << i);
            status |= STEWART_ERR_UNREACHABLE;
        } else {
            float ratio = C / Rmag;
            if (ratio > 1.0f)  { ratio = 1.0f;  unreachable_mask |= (1u << i); status |= STEWART_ERR_UNREACHABLE; }
            if (ratio < -1.0f) { ratio = -1.0f; unreachable_mask |= (1u << i); status |= STEWART_ERR_UNREACHABLE; }
            // Two solutions: alpha = atan2(B, A) +/- acos(C/R). The "elbow up"
            // solution (rod end above servo, radial leg pulled inward) is the
            // physical one; with our sign convention that is the minus branch.
            float alpha = atan2f(B, A) - acosf(ratio);
            alpha_deg = RAD2DEG(alpha);
        }

        // Clamp to mechanical servo travel.
        if (alpha_deg > 90.0f)  { alpha_deg = 90.0f;  status |= STEWART_ERR_CLAMPED; }
        if (alpha_deg < -90.0f) { alpha_deg = -90.0f; status |= STEWART_ERR_CLAMPED; }

        crank_deg_out[i] = alpha_deg;
    }

    if (unreachable_mask_out) *unreachable_mask_out = unreachable_mask;
    return status;
}

int stewart_home_angles(float crank_deg_out[STEWART_NUM_LEGS]) {
    return stewart_ik(0.0f, 0.0f, STEWART_H0, 0.0f, 0.0f, 0.0f,
                      crank_deg_out, NULL);
}
