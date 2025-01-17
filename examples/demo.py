#!/usr/bin/env python3

import time

import numpy as np
import cv2
import dwa


class Demo(object):
    def __init__(self):
        # 1 px = 0.1 m
        # That's why everything is multiplied or divided by 10.
        cv2.namedWindow('cvwindow')
        cv2.setMouseCallback('cvwindow', self.callback)
        self.drawing = False
        self.drawing_saturation = False

        self.point_cloud = []
        self.saturation_cloud = []
        self.draw_points = []
        self.draw_saturation_points = []

        # Planner Settings
        self.vel = (0.0, 0.0)
        self.pose = (30.0, 30.0, 0)
        self.goal = None
        self.base = [-3.0, -2.5, +3.0, +2.5]
        self.config = dwa.Config(
                max_speed = 3.0,
                min_speed = -1.0,
                max_yawrate = np.radians(40.0),
                max_accel = 15.0,
                max_dyawrate = np.radians(110.0),
                velocity_resolution = 0.1,
                yawrate_resolution = np.radians(1.0),
                dt = 0.1,
                predict_time = 3.0,
                heading = 0.1,
                clearance = 1.5,
                velocity = 0.80,
                base = self.base,
                saturation=0.06,
                saturation_max_radius = 100,
                use_threads=1)

    def callback(self, event, x, y, flags, param):
        if event == cv2.EVENT_LBUTTONDOWN:
            self.drawing = True
        elif event == cv2.EVENT_MOUSEMOVE:
            if not self.drawing and not self.drawing_saturation:
                self.goal = (x/10, y/10)

            if self.drawing:
                if [x, y] not in self.draw_points:
                    self.draw_points.append([x, y])
                    self.point_cloud.append([x/10, y/10])
                    self.goal = None
            if self.drawing_saturation:
                if [x, y] not in self.draw_saturation_points:
                    self.draw_saturation_points.append([x, y])
                    self.saturation_cloud.append([x/10, y/10, 1])
                    self.goal = None
            
        elif event == cv2.EVENT_LBUTTONUP:
            print("no longer drawing")
            self.drawing = False
        
        if event == cv2.EVENT_RBUTTONDOWN:
            self.drawing_saturation = True
            # if not self.drawing and not self.drawing_saturation:
            #     self.goal = (x/10, y/10)
            pass
        elif event == cv2.EVENT_RBUTTONUP:
            self.drawing_saturation = False
            pass

    def main(self):
        import argparse
        parser = argparse.ArgumentParser(description='DWA Demo')
        parser.add_argument('--save', dest='save', action='store_true')
        parser.set_defaults(save=False)
        args = parser.parse_args()
        if args.save:
            import imageio
            writer = imageio.get_writer('./dwa.gif', mode='I', duration=0.05)
        while True:
            prev_time = time.time()
            self.map = np.zeros((1200, 900, 3), dtype=np.uint8)
            for point in self.draw_points:
                cv2.circle(self.map, tuple(point), 4, (0, 255, 255, 0.5), -1)
            for point in self.draw_saturation_points:
                cv2.circle(self.map, tuple(point), 3, (255, 0, 0, 0.5), 1)
            if self.goal is not None:
                cv2.circle(self.map, (int(self.goal[0]*10), int(self.goal[1]*10)),
                        4, (0, 255, 0), -1)
                if len(self.point_cloud):
                    # Planning
                    if(len(self.saturation_cloud) == 0):
                        self.saturation_cloud = [[-5,-5, 1]]
                    self.vel = dwa.planning(self.pose, self.vel, self.goal,
                            np.array(self.point_cloud, np.float32), self.config
                            , np.array(self.saturation_cloud, np.float32)
                            )
                    # Simulate motion
                    # print("vel", self.vel)
                    self.pose = dwa.motion(self.pose, self.vel, self.config.dt)

                    # print("pose", self.pose)

            pose = np.ndarray((3,))
            pose[0:2] = np.array(self.pose[0:2]) * 10
            pose[2] = self.pose[2]

            base = np.array(self.base) * 10
            base[0:2] += pose[0:2]
            base[2:4] += pose[0:2]

            # Not the correct rectangle but good enough for the demo
            width = base[2] - base[0]
            height = base[3] - base[1]
            rect = ((pose[0], pose[1]), (width, height), np.degrees(pose[2]))
            box = cv2.boxPoints(rect)
            box = np.intp(box)
            cv2.drawContours(self.map,[box],0,(0,0,255),-1)

            # Prevent divide by zero
            fps = int(1.0 / (time.time() - prev_time + 1e-10))
            cv2.putText(self.map, f'FPS: {fps}', (20, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            cv2.putText(self.map, f'Point Cloud Size: {len(self.point_cloud)}',
                    (20, 60), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)

            if args.save:
                writer.append_data(self.map)
            cv2.imshow('cvwindow', self.map)
            key = cv2.waitKey(1)
            if key == 27:
                # break
                pass
            elif key == ord('r'):
                self.point_cloud = []
                self.draw_points = []
            elif key == ord('b'):
                self.saturation_cloud = []
                self.draw_saturation_points = []
        if args.save:
            writer.close()

if __name__ == '__main__':
    Demo().main()


