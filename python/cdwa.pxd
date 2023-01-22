cdef extern from "dwa.h":
    ctypedef struct Rect:
        float xmin
        float ymin
        float xmax
        float ymax
    ctypedef struct Config:
        float maxSpeed
        float minSpeed
        float maxYawrate
        float maxAccel
        float maxdYawrate
        float velocityResolution
        float yawrateResolution
        float dt
        float predictTime
        float saturationMaxRadius
        float heading
        float saturation
        float clearance
        float velocity
        Rect base
    ctypedef struct Point:
        float x
        float y
    ctypedef struct SaturationPoint:
        float x
        float y
        float saturation
    ctypedef struct SaturationPointCloud:
        int size;
        SaturationPoint *saturationPoints;
    ctypedef struct PointCloud:
        int size;
        Point *points;
    ctypedef struct Pose:
        Point point
        float yaw
    ctypedef struct Velocity:
        float linearVelocity
        float angularVelocity
    ctypedef struct DynamicWindow:
        int nPossibleV;
        float *possibleV;
        int nPossibleW;
        float *possibleW;

    Pose motion(Pose pose, Velocity velocity, float dt);
    Velocity planning(
        Pose pose, Velocity velocity, Point goal,
        PointCloud *pointCloud, Config config, SaturationPointCloud *saturationCloud);
    PointCloud* createPointCloud(int size);
    void freePointCloud(PointCloud* pointCloud);
    SaturationPointCloud* createSaturationPointCloud(int size);
    void freeSaturationPointCloud(SaturationPointCloud* saturationPointCloud);
    void freeDynamicWindow(DynamicWindow *dynamicWindow);
    void createDynamicWindow(
        Velocity velocity, Config config, DynamicWindow **dynamicWindow);
    float calculateVelocityCost(Velocity velocity, Config config);
    float calculateHeadingCost(Pose pose, Point goal);
    float calculateSaturationCost(Pose pose, Velocity velocity, SaturationPointCloud *saturationCloud, Config config);
    float calculateClearanceCost(
        Pose pose, Velocity velocity, PointCloud *pointCloud, Config config);
