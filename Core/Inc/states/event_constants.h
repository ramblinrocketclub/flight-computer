#ifndef EVENT_CONSTANTS_H
#define EVENT_CONSTANTS_H

#define GRAVITY_CONSTANT_MSEC2                  9.81

#define BOOSTER_IGNITION_ACCEL_THRESH_MSEC2     (0.05 * GRAVITY_CONSTANT_MSEC2)

#define BOOSTER_BURNOUT_TIME_SEC                9.0

#define STAGING_TIME_THRESH_SEC                 7.0

#define STAGING_TIME_LIMIT_SEC                  30.0

#define SUSTAINER_IGNITION_TIME_OFFSET_SEC      8.0
#define MAX_SUSTAINER_IGN_TILT                  PI/4.0

#define SUSTAINER_BURNOUT_TIME_SEC              9.0

#define CHUTE_DEPLOY_TIME_SEC                   150.0

#define APOGEE_HEIGHT_THRESH_M                  10000

#define APOGEE_HEIGHT_DIFF_THRESH_M             5
#define FALLING_VEL_THRESH_MSEC                 -20

#define LANDED_HEIGHT_THRESH_M                  50

#endif /* EVENT_CONSTANTS_H */