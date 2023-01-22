#include "dwa.h"

void
createDynamicWindow(Velocity velocity, Config config, DynamicWindow **dynamicWindow) {
  float
    minV = max(config.minSpeed, velocity.linearVelocity - config.maxAccel * config.dt);
  float
    maxV = min(config.maxSpeed, velocity.linearVelocity + config.maxAccel * config.dt);
  float minW =
    max(-config.maxYawrate, velocity.angularVelocity - config.maxdYawrate * config.dt);
  float maxW =
    min(config.maxYawrate, velocity.angularVelocity + config.maxdYawrate * config.dt);

  int nPossibleV = (maxV - minV) / config.velocityResolution;
  int nPossibleW = (maxW - minW) / config.yawrateResolution;
  *dynamicWindow = malloc(sizeof(DynamicWindow));

  (*dynamicWindow)->possibleV = malloc(nPossibleV * sizeof(float));
  (*dynamicWindow)->possibleW = malloc(nPossibleW * sizeof(float));
  (*dynamicWindow)->nPossibleV = nPossibleV;
  (*dynamicWindow)->nPossibleW = nPossibleW;

  for(int i=0; i < nPossibleV; i++) {
    (*dynamicWindow)->possibleV[i] = minV + (float)i * config.velocityResolution;
  }

  for(int i=0; i < nPossibleW; i++) {
    (*dynamicWindow)->possibleW[i] = minW + (float)i * config.yawrateResolution;
  }
}

void freeDynamicWindow(DynamicWindow *dynamicWindow){
  free(dynamicWindow->possibleV);
  free(dynamicWindow->possibleW);
  free(dynamicWindow);
}

PointCloud* createPointCloud(int size){
  PointCloud* pointCloud = malloc(sizeof(PointCloud));
  pointCloud->points = malloc(size * sizeof(Point));
  pointCloud->size = size;
  return pointCloud;
}

void freePointCloud(PointCloud* pointCloud){
  free(pointCloud->points);
  free(pointCloud);
}

SaturationPointCloud* createSaturationPointCloud(int size){
  SaturationPointCloud* saturationPointCloud = malloc(sizeof(SaturationPointCloud));
  saturationPointCloud->saturationPoints = malloc(size * sizeof(SaturationPoint));
  saturationPointCloud->size = size;
  return saturationPointCloud;
}

void freeSaturationPointCloud(SaturationPointCloud* saturationPointCloud){
  free(saturationPointCloud->saturationPoints);
  free(saturationPointCloud);
}

Pose motion(Pose pose, Velocity velocity, float dt){
  Pose new_pose;
  new_pose.yaw = pose.yaw + velocity.angularVelocity * dt;
  new_pose.point.x = pose.point.x + velocity.linearVelocity * cos(new_pose.yaw) * dt;
  new_pose.point.y = pose.point.y + velocity.linearVelocity * sin(new_pose.yaw) * dt;
  return new_pose;
}

float calculateVelocityCost(Velocity velocity, Config config) {
  return config.maxSpeed - velocity.linearVelocity;
}

float calculateHeadingCost(Pose pose, Point goal) {
  float dx = goal.x - pose.point.x;
  float dy = goal.y - pose.point.y;
  float angleError = atan2(dy, dx);
  float angleCost = angleError - pose.yaw;
  return fabs(atan2(sin(angleCost), cos(angleCost)));
}

float
calculateClearanceCost
(Pose pose, Velocity velocity, PointCloud *pointCloud, Config config) {
  Pose pPose = pose;
  float time = 0.0;
  float minr = FLT_MAX;
  float r;
  float dx;
  float dy;

  float x;
  float y;

  while (time < config.predictTime) {
    pPose = motion(pPose, velocity, config.dt);
      
    for(int i = 0; i < pointCloud->size; ++i) {
      dx = pPose.point.x - pointCloud->points[i].x;
      dy = pPose.point.y - pointCloud->points[i].y;
      x = -dx * cos(pPose.yaw) + -dy * sin(pPose.yaw);
      y = -dx * -sin(pPose.yaw) + -dy * cos(pPose.yaw);
      if (x <= config.base.xmax &&
          x >= config.base.xmin &&
          y <= config.base.ymax &&
          y >= config.base.ymin){
        return FLT_MAX;
      }
      r = sqrtf(dx*dx + dy*dy);
      if (r < minr)
        minr = r;
    }
    time += config.dt;
  }
  return 1.0 / minr;
}

float
calculateSaturationCost
(Pose pose, Velocity velocity, SaturationPointCloud *saturationCloud, Config config) {
  Pose pPose = pose;
  float time = 0.0;
  float minr = config.saturationMaxRadius;
  float closest_saturation = 0;
  float dx;
  float r;
  float dy;

  float x;
  float y;
  float saturation;

  float basex = config.base.xmax - config.base.xmin;
  float basey = config.base.ymax - config.base.ymin;

  while (time < config.predictTime) {
    pPose = motion(pPose, velocity, config.dt);
      
    for(int i = 0; i < saturationCloud->size; ++i) {
      dx = pPose.point.x - saturationCloud->saturationPoints[i].x;
      dy = pPose.point.y - saturationCloud->saturationPoints[i].y;
      saturation = saturationCloud->saturationPoints[i].saturation;
      x = -dx * cos(pPose.yaw) + -dy * sin(pPose.yaw);
      y = -dx * -sin(pPose.yaw) + -dy * cos(pPose.yaw);
      // if (x <= config.base.xmax &&
      //     x >= config.base.xmin &&
      //     y <= config.base.ymax &&
      //     y >= config.base.ymin){
      //   return FLT_MAX;
      // }
      r = sqrtf(dx*dx + dy*dy);
      
      if (r < minr){
        minr = r;
        closest_saturation = saturation;
      }
        
    }
    time += config.dt;
  }
  return closest_saturation * ((config.saturationMaxRadius - minr)/config.saturationMaxRadius);
  // return 1.0 / minr;
}

void * threadedPlanning(void * input){
      float cost;
      Velocity pVelocity;
      Pose pPose = ((struct threadedPlanningArgs*) input)->pose;
      pVelocity.linearVelocity = ((struct threadedPlanningArgs*) input)->dw->possibleV[((struct threadedPlanningArgs*) input)->possibleVIndex];
      pVelocity.angularVelocity = ((struct threadedPlanningArgs*) input)->dw->possibleW[((struct threadedPlanningArgs*) input)->possibleWIndex];
      pPose = motion(pPose, pVelocity, 
      ((struct threadedPlanningArgs*) input)->config.predictTime);
      cost = 
        ((struct threadedPlanningArgs*) input)->config.velocity * calculateVelocityCost(pVelocity, ((struct threadedPlanningArgs*) input)->config) +
        ((struct threadedPlanningArgs*) input)->config.heading * calculateHeadingCost(pPose, ((struct threadedPlanningArgs*) input)->goal) +
        ((struct threadedPlanningArgs*) input)->config.saturation * calculateSaturationCost(((struct threadedPlanningArgs*) input)->pose, pVelocity,
                                                  ((struct threadedPlanningArgs*) input)->saturationCloud, 
                                                  ((struct threadedPlanningArgs*) input)->config) +
        ((struct threadedPlanningArgs*) input)->config.clearance * calculateClearanceCost(((struct threadedPlanningArgs*) input)->pose, pVelocity,
                                                  ((struct threadedPlanningArgs*) input)->pointCloud, 
                                                  ((struct threadedPlanningArgs*) input)->config);
      threadedCosts[((struct threadedPlanningArgs*) input)->thread_id] = cost;
}

Velocity
planning(Pose pose, Velocity velocity, Point goal,
         PointCloud *pointCloud, Config config, SaturationPointCloud *saturationCloud) {
  DynamicWindow *dw;
  createDynamicWindow(velocity, config, &dw);
  Velocity pVelocity;
  Pose pPose = pose;
  float total_cost = FLT_MAX;
  float cost;
  Velocity bestVelocity;
  int thread_count = dw->nPossibleV * dw->nPossibleW;
  int thread_id = 0;

  pthread_t * pthreads = (pthread_t*)malloc(thread_count * sizeof(pthread_t));
  threadedCosts = malloc(thread_count * sizeof(float));
  threadedPlanningArgs * threadedArgs = (threadedPlanningArgs *)malloc(thread_count * sizeof(threadedPlanningArgs));
  for (int i = 0; i < dw->nPossibleV; ++i) {
    for (int j = 0; j < dw->nPossibleW; ++j) {

      threadedArgs[thread_id].thread_id = thread_id;
      threadedArgs[thread_id].dw = dw;
      threadedArgs[thread_id].possibleVIndex = i;
      threadedArgs[thread_id].possibleWIndex = j;
      threadedArgs[thread_id].pose = pose;
      threadedArgs[thread_id].velocity = velocity;
      threadedArgs[thread_id].goal = goal;
      threadedArgs[thread_id].pointCloud = pointCloud;
      threadedArgs[thread_id].config = config;
      threadedArgs[thread_id].saturationCloud = saturationCloud;

      pthread_create(&pthreads[thread_id], NULL, threadedPlanning, &threadedArgs[thread_id]);
    }
  }

  for (int t=0; t<thread_count; t++){
    pthread_join(pthreads[t], NULL);
  }

  for (int t=0; t<thread_count; t++){
    cost = threadedCosts[t];
    if (cost < total_cost) {
      total_cost = cost;
      bestVelocity = pVelocity;
    }
  }

  free(pthreads);
  free(threadedCosts);
  free(threadedArgs);

  freeDynamicWindow(dw);
  return bestVelocity;
}
