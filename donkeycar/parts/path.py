import pickle
import math
import logging
import cv2

import numpy
from PIL import Image, ImageDraw

from donkeycar.utils import norm_deg, dist, deg2rad, arr_to_img


class Path(object):
    def __init__(self, min_dist = 1.):
        self.path = []
        self.min_dist = min_dist
        self.x = math.inf
        self.y = math.inf
        self.recording = True

    def run(self, x, y):
        d = dist(x, y, self.x, self.y)
        if self.recording and d > self.min_dist:
            self.path.append((x, y))
            logging.info("path point (%f, %f)" % ( x, y))
            self.x = x
            self.y = y
        return self.path

    def save(self, filename):
        outfile = open(filename, 'wb')
        pickle.dump(self.path, outfile)
    
    def load(self, filename):
        infile = open(filename, 'rb')
        self.path = pickle.load(infile)
        self.recording = False

class PImage(object):
    def __init__(self, resolution=(500, 500), color="white", clear_each_frame=False):
        self.resolution = resolution
        self.color = color
        self.img = Image.new('RGB', resolution, color=color)
        self.clear_each_frame = clear_each_frame

    def run(self):
        if self.clear_each_frame:
            self.img = Image.new('RGB', self.resolution, color=self.color)

        return self.img


class OriginOffset(object):
    '''
    Use this to set the car back to the origin without restarting it.
    '''

    def __init__(self):
        self.ox = 0.0
        self.oy = 0.0
        self.last_x = 0.
        self.last_y = 0.

    def run(self, x, y):
        self.last_x = x
        self.last_y = y

        return x + self.ox, y + self.oy

    def init_to_last(self):
        self.ox = -self.last_x
        self.oy = -self.last_y


class PathPlot(object):
    '''
    draw a path plot to an image
    '''
    def __init__(self, scale=1.0, offset=(0., 0.0), color=(255, 0, 0)):
        self.scale = scale
        self.offset = offset
        self.color = color

    def plot_line(self, sx, sy, ex, ey, draw, color):
        '''
        scale dist so that max_dist is edge of img (mm)
        and img is PIL Image, draw the line using the draw ImageDraw object
        '''
        draw.line((sx,sy, ex, ey), fill=color, width=1)

    def run(self, img, path):
        
        if type(img) is numpy.ndarray:
            stacked_img = numpy.stack((img,)*3, axis=-1)
            img = arr_to_img(stacked_img)

        draw = ImageDraw.Draw(img)
        for iP in range(0, len(path) - 1):
            ax, ay = path[iP]
            bx, by = path[iP + 1]
            # flip y because on image y increases downwards
            self.plot_line(ax * self.scale + self.offset[0],
                        -ay * self.scale + self.offset[1], 
                        bx * self.scale + self.offset[0], 
                        -by * self.scale + self.offset[1], 
                        draw, 
                        self.color)

        return img


class PlotCircle(object):
    '''
    draw a circle plot to an image
    '''
    def __init__(self,  scale=1.0, offset=(0., 0.0), radius=4, color = (0, 255, 0)):
        self.scale = scale
        self.offset = offset
        self.radius = radius
        self.color = color

    def plot_circle(self, x, y, rad, draw, color, width=1):
        '''
        scale dist so that max_dist is edge of img (mm)
        and img is PIL Image, draw the circle using the draw ImageDraw object
        '''
        sx = x - rad
        sy = y - rad
        ex = x + rad
        ey = y + rad

        draw.ellipse([(sx, sy), (ex, ey)], fill=color)


    def run(self, img, x, y):
        draw = ImageDraw.Draw(img)
        self.plot_circle(x * self.scale + self.offset[0],
                        y * self.scale + self.offset[1], 
                        self.radius,
                        draw, 
                        self.color)

        return img
    
class PlotPose(object):
    '''
    draw an arrow plot to an image
    '''
    def __init__(self,  scale=1.0, offset=(0., 0.), radius=4, color = (0, 255, 0)):
        self.scale = scale
        self.offset = offset
        self.radius = radius
        self.color = color

    def plot_pose(self, x, y, rad, yaw, draw, color, width=1):
        '''
        scale dist so that max_dist is edge of img (mm)
        and img is PIL Image, draw the circle using the draw ImageDraw object
        '''

        sx = x - rad
        sy = y - rad
        ex = x + rad
        ey = y + rad

        draw.ellipse([(sx, sy), (ex, ey)], fill=color)
        dx = self.radius * 3. * math.cos(math.radians(yaw))
        dy = self.radius * 3. * math.sin(math.radians(yaw))
        draw.line([(x,y),(x+dx, y+dy)], fill=(0,0,255))


    def run(self, img, x, y, yaw):
        draw = ImageDraw.Draw(img)
        # flip y because on image y increases from top to bottom
        self.plot_pose(x * self.scale + self.offset[0],
                        -y * self.scale + self.offset[1], 
                        self.radius,
                        yaw,
                        draw, 
                        self.color)

        return img

from donkeycar.la import Line3D, Vec3

class CTE(object):

    def nearest_two_pts(self, path, x, y):
        if len(path) < 2:
            return None, None

        distances = []
        for iP, p in enumerate(path):
            d = dist(p[0], p[1], x, y)
            distances.append((d, iP, p))
        distances.sort(key=lambda elem : elem[0])
        iA = (distances[0][1] - 1) % len(path)
        a = path[iA]
        #iB is the next element in the path, wrapping around..
        iB = (iA + 2) % len(path)
        b = path[iB]
        
        return a, b

    def run(self, path, x, y):
        cte = 0.

        a, b = self.nearest_two_pts(path, x, y)
        
        if a and b:
            #logging.info("nearest: (%f, %f) to (%f, %f)" % ( a[0], a[1], x, y))
            a_v = Vec3(a[0], 0., a[1])
            b_v = Vec3(b[0], 0., b[1])
            p_v = Vec3(x, 0., y)
            line = Line3D(a_v, b_v)
            err = line.vector_to(p_v)
            sign = 1.0
            cp = line.dir.cross(err.normalized())
            if cp.y > 0.0 :
                sign = -1.0
            cte = err.mag() * sign            

        return cte

class CTE2(object):

    def __init__(self):
        self.counter = 0
        self.cte = 0.

    def nearest_two_pts(self, path, x, y):
        if len(path) < 2:
            return None, None

        min_dist = 1000000
        min_idx = 0
        for i, p in enumerate(path):
            d = dist(p[0], p[1], x, y)
            if d < min_dist:
                min_dist = d
                min_idx = i
        iA = min_idx % len(path)
        a = path[iA]
        #iB is the next element in the path, wrapping around..
        iB = (iA + 1) % len(path)
        b = path[iB]
        
        return a, b

    def run(self, path, x, y):

        a, b = self.nearest_two_pts(path, x, y)
        
        if a and b:
            #logging.info("nearest: (%f, %f) to (%f, %f)" % ( a[0], a[1], x, y))
            a_v = Vec3(a[0], 0., a[1])
            b_v = Vec3(b[0], 0., b[1])
            p_v = Vec3(x, 0., y)
            line = Line3D(a_v, b_v)
            err = line.vector_to(p_v)
            sign = -1.0
            cp = line.dir.cross(err.normalized())
            if cp.y > 0.0 :
                sign = 1.0
            self.cte = err.mag() * sign
        return self.cte

class PID_Pilot(object):

    def __init__(self, pid, throttle):
        self.pid = pid
        self.throttle = throttle

    def run(self, cte):
        steer = self.pid.run(cte)
        logging.info("CTE: %f steer: %f" % (cte, steer))
        return steer, self.throttle

class PosStream:
    def run(self, pos, yaw):
        #RS_t265: y is up, x is right, z is backwards/forwards
        return -pos.z, -pos.x, yaw

class gray2color:

    def __init__(self):
        self.img = None

    def run(self, img_gray):
        self.img = cv2.cvtColor(img_gray,cv2.COLOR_GRAY2RGB)

    def update(self):
        pass

    def run_threaded(self, img_gray):
        return self.run(img_gray)