#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <math.h>
#include <float.h>
#include <pthread.h>

#ifndef _WIN32
/* https://stackoverflow.com/a/3437484 */
#define max(a, b) \
  ({ __typeof__ (a) _a = (a);			\
    __typeof__ (b) _b = (b);			\
    _a > _b ? _a : _b; })

#define min(a, b) \
  ({ __typeof__ (a) _a = (a);			\
    __typeof__ (b) _b = (b);			\
    _a < _b ? _a : _b; })
#endif

typedef struct
{
  float xmin;
  float ymin;
  float xmax;
  float ymax;
} Rect;

typedef struct
{
  float maxSpeed;
  float minSpeed;
  float maxYawrate;
  float maxAccel;
  float maxdYawrate;
  float velocityResolution;
  float yawrateResolution;
  float dt;
  float predictTime;
  float saturationMaxRadius;
  int useThreads;
  float heading;
  float saturation;
  float clearance;
  float velocity;
  Rect base;
} Config;

float *threadedCosts;

typedef struct
{
  float linearVelocity;
  float angularVelocity;
} Velocity;

typedef struct
{
  float x;
  float y;
} Point;

typedef struct
{
  int size;
  Point *points;
} PointCloud;

typedef struct
{
  float x;
  float y;
  float saturation;
} SaturationPoint;

typedef struct
{
  int size;
  SaturationPoint *saturationPoints;
} SaturationPointCloud;

typedef struct
{
  Point point;
  float yaw;
} Pose;

typedef struct
{
  int nPossibleV;
  float *possibleV;
  int nPossibleW;
  float *possibleW;
} DynamicWindow;

typedef struct threadedPlanningArgs
{
  int thread_id;
  DynamicWindow *dw;
  int possibleVIndex;
  int possibleWIndex;
  Pose pose;
  Velocity velocity;
  Point goal;
  PointCloud *pointCloud;
  Config config;
  SaturationPointCloud *saturationCloud;
} threadedPlanningArgs;

void createDynamicWindow(Velocity velocity, Config config, DynamicWindow **dynamicWindow);
Pose motion(Pose pose, Velocity velocity, float dt);
float calculateVelocityCost(Velocity velocity, Config config);
float calculateHeadingCost(Pose pose, Point goal);
float calculateClearanceCost(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config);
float calculateSaturationCost(Pose pose, Velocity velocity, SaturationPointCloud *saturationCloud, Config config);
Velocity
planning(Pose pose, Velocity velocity, Point goal, PointCloud *pointCloud, Config config, SaturationPointCloud *saturationCloud);
PointCloud *createPointCloud(int size);
void freePointCloud(PointCloud *pointCloud);
SaturationPointCloud *createSaturationPointCloud(int size);
void freeSaturationPointCloud(SaturationPointCloud *saturationPointCloud);
void freeDynamicWindow(DynamicWindow *dynamicWindow);
void * threadedPlanning(void * input);
