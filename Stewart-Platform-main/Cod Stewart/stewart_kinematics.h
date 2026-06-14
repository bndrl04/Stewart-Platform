#ifndef STEWART_KINEMATICS_H
#define STEWART_KINEMATICS_H

#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

// Geometry (mm / deg). Matches the assembled hardware.
#define STEWART_RB     100.0f  // base servo-axis circle radius
#define STEWART_RP      80.0f  // platform joint circle radius
#define STEWART_LA      30.0f  // crank arm length (servo spline to rod end)
#define STEWART_LS     106.3f  // connecting rod length (ball center to ball center)
#define STEWART_H0     125.0f  // HOME platform height above the lid (paired config)
#define STEWART_ZAX     21.75f // servo-axis height above the lid

#define STEWART_NUM_LEGS 6

// Status flags for stewart_ik (bitmask).
#define STEWART_OK              0
#define STEWART_ERR_UNREACHABLE 0x01  // at least one leg was clamped (out of range)
#define STEWART_ERR_CLAMPED     0x02  // crank angle was clamped to +-90 deg

// Inverse kinematics for a 6-RSS Stewart platform.
//
//   x, y, z      : translation of the platform center, mm. z is the height above
//                  the lid; HOME is x=0, y=0, z=STEWART_H0.
//   roll, pitch, yaw : intrinsic ZYX Euler angles in degrees (roll about X,
//                  pitch about Y, yaw about Z).
//   crank_deg_out: array of 6 absolute crank angles (deg). 0 deg = crank lying
//                  horizontally in the leg's tangential direction (CCW for legs
//                  with servo_sens=+1, CW for legs with servo_sens=-1, i.e. the
//                  mirrored half in the paired config); +90 deg = rod end up.
//   unreachable_mask_out: optional, may be NULL. Bit i set if leg i was clamped.
//
// Returns a bitmask of STEWART_ERR_* flags (0 == STEWART_OK).
int stewart_ik(float x, float y, float z,
               float roll_deg, float pitch_deg, float yaw_deg,
               float crank_deg_out[STEWART_NUM_LEGS],
               unsigned int *unreachable_mask_out);

// Convenience: compute the HOME crank angles (call at boot).
int stewart_home_angles(float crank_deg_out[STEWART_NUM_LEGS]);

#ifdef __cplusplus
}
#endif

#endif
