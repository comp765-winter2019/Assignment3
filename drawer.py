'''
Copyright (c) 2019, Travis Manderson
All rights reserved.
Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the <organization> nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.
THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
'''

import math
import tkinter as tk
from PIL import Image, ImageTk
import os
import numpy as np
from geometry_tools import *
import networkx as nx
from dubins import *
from map import *


class TKDrawer:
    def __init__(self, width, height):
        self.width = width
        self.height = height
        self.master = tk.Tk()

        self.canvas = tk.Canvas(self.master, width=self.width, height=self.height)
        self.canvas.pack()
        self.update_canvas_text = None
        return

    def update_text(self, text):
        if self.update_canvas_text is not None:
            self.canvas.delete(self.update_canvas_text)
        self.update_canvas_text = self.canvas.create_text(150, 50, text=text, fill='black')
        return

    def draw_image(self, image_name):
        # self.image = Image.open(os.path.join("maps", self.map_name + '.bmp'))
        self.image = Image.open(image_name)
        self.tkimage = ImageTk.PhotoImage(self.image)
        self.canvas.create_image((self.width / 2, self.height / 2), image=self.tkimage)
        return


    def draw_edges_and_poly(self, edges):
        # draw closed polygons
        G = nx.DiGraph()
        for edge in edges:
            G.add_edge(edge[0], edge[1])
        cycles = list(nx.simple_cycles(G))
        for cycle in cycles:
            px = list()
            for node in cycle:
                px.append(node[0])
                px.append(self.height - node[1])
            self.canvas.create_polygon(px, fill='gray41')
        for edge in edges:
            p0 = edge[0]
            p1 = edge[1]
            self.draw_line(p0, p1, width=1, fill='red')

    def draw_map(self, map):
        self.draw_edges_and_poly(map.edges)
        return

    def draw_arrow(self, pose, length=25, a_width=10, l_width=1, fill='red'):  # pragma: no cover
        x = pose[0]
        y = pose[1]
        yaw = pose[2]
        l = length
        hl = 0.4 * l
        w = a_width
        p1 = [0.0, 0.0]
        p2 = [l - hl, 0.0]
        p3 = [l - hl, w / 2]
        p4 = [l - hl, -w / 2]
        p5 = [l, 0.0]

        t = [x, y, yaw]
        p1 = transform_2d(p1, t)
        p2 = transform_2d(p2, t)
        p3 = transform_2d(p3, t)
        p4 = transform_2d(p4, t)
        p5 = transform_2d(p5, t)

        h = self.height

        self.canvas.create_polygon(*tuple([p3[0], h - p3[1], p4[0], h - p4[1], p5[0], h - p5[1]]), fill=fill)
        self.canvas.create_line(p1[0], h - p1[1], p2[0], h - p2[1], fill='black', width=l_width)
        self.canvas.create_line(p3[0], h - p3[1], p4[0], h - p4[1], fill='black', width=l_width)
        self.canvas.create_line(p3[0], h - p3[1], p5[0], h - p5[1], fill='black', width=l_width)
        self.canvas.create_line(p4[0], h - p4[1], p5[0], h - p5[1], fill='black', width=l_width)
        return

    def draw_circle(self, pose, r, width=1, fill='', outline='black'):
        x = pose[0]
        y = pose[1]
        h = self.height
        x0 = x-r
        y0 = y-r
        x1 = x+r
        y1 = y+r
        canvas_circle = self.canvas.create_oval(x0, h - y0, x1, h - y1, width=width, fill=fill, outline=outline)
        return canvas_circle

    def draw_line(self, pose0, pose1, width=1, fill='black'):
        h = self.height
        x0 = pose0[0]
        y0 = pose0[1]
        x1 = pose1[0]
        y1 = pose1[1]
        canvas_line = self.canvas.create_line(x0, h - y0, x1, h - y1, width=width, fill=fill)
        return canvas_line

    def plot_path_from_x_y(self, px, py, pyaw, width=1):
        points = [None] * (len(px) + len(py))
        points[::2] = px
        points[1::2] = py

        points = points * 100
        self.canvas.create_line(*points, fill='blue', width=2)
        self.canvas.create_line(100, 100, 150, 155, 210, 40, 400, 400, width=2)
        return

    def plot_path(self, poses, width=1):
        poses_array = np.array(poses)
        points = [None] * len(poses)*2
        points[::2] = poses_array[:, 0]
        points[1::2] = self.height - poses_array[:, 1]

        # points = points * 100
        self.canvas.create_line(*points, fill='blue', width=2)
        return

    def draw_arc(self, center, angle_i, angle_f, radius, direction='L', outline='black', width=1):
        h = self.height
        x, y = center[0], center[1]
        x0, x1, y0, y1 = x - radius, x + radius, y - radius, y + radius
        d_angle = 0
        angle_start = 0
        if direction == 'L':
            if angle_f > angle_i:
                d_angle = angle_f - angle_i
                angle_start = angle_i
            else:
                d_angle = 2 * math.pi - (angle_i - angle_f)
                angle_start = angle_i
        if direction == 'R':
            if angle_i > angle_f:
                d_angle = angle_i - angle_f
                angle_start = angle_f
            else:
                d_angle = 2 * math.pi - (angle_f - angle_i)
                angle_start = angle_f
        angle_start = angle_start*180.0/math.pi
        d_angle = d_angle * 180.0 / math.pi
        # fix for tk arc issue. A very small angle causes the arc to go the wrong direction
        if 2*math.pi*radius * d_angle/360.0 < 1:
            d_angle = 2.1 * 360.0 / (2*math.pi*radius)
        self.canvas.create_arc(x0, h-y0, x1, h-y1, start=angle_start, extent=d_angle, fill="green", style=tk.ARC, outline=outline, width=width)
        return

    def draw_dubins_path(self, dubins, width=1, color='blue', car_width=0):
        offsets = [0]
        if car_width > 0:
            offsets = [car_width/2.0,  -car_width/2.0]

        for offset in offsets:
            radius = dubins.rho + offset

            si = 1 if dubins.dir_i == 'R' else -1
            sf = 1 if dubins.dir_f == 'R' else -1
            point0_radius, point1_radius = (dubins.rho + si * offset), (dubins.rho + sf * offset)

            self.draw_arc(dubins.c_i, dubins.gamma_ii, dubins.gamma_if, radius, direction=dubins.dir_i, outline=color, width=width)
            self.draw_arc(dubins.c_f, dubins.gamma_fi, dubins.gamma_ff, radius, direction=dubins.dir_f, outline=color, width=width)

            point0 = (dubins.c_i[0] + point0_radius * math.cos(dubins.gamma_if), dubins.c_i[1] + point0_radius * math.sin(dubins.gamma_if))
            point1 = (dubins.c_f[0] + point1_radius * math.cos(dubins.gamma_fi), dubins.c_f[1] + point1_radius * math.sin(dubins.gamma_fi))
            self.draw_line(point0, point1, fill=color, width=width)
        return

    def runMainLoop(self):
        self.master.mainloop()
        return


def draw_dubins(drawer, qStart, qFinal, rho=40):
    """ example to test dubins logic"""
    drawer.draw_circle(qStart, 5, fill='green') #draw start
    drawer.draw_circle(qFinal, 5, fill='red') #draw end

    drawer.draw_arrow(qStart, fill='green')
    drawer.draw_arrow(qFinal, fill='red')

    dubins = DubinsPath(qStart, qFinal, rho)

    drawer.draw_circle(dubins.cr_i, rho, fill='')
    drawer.draw_circle(dubins.cr_i, 4, fill='black')
    drawer.draw_circle(dubins.cl_i, rho, fill='')
    drawer.draw_circle(dubins.cl_i, 4, fill='black')
    drawer.draw_circle(dubins.cr_f, rho, fill='')
    drawer.draw_circle(dubins.cr_f, 4, fill='black')
    drawer.draw_circle(dubins.cl_f, rho, fill='')
    drawer.draw_circle(dubins.cl_f, 4, fill='black')

    # draw tangent lines
    drawer.draw_line(dubins.rsr_tang_i, dubins.rsr_tang_f, fill='red') #RSR
    drawer.draw_line(dubins.rsl_tang_i, dubins.rsl_tang_f, fill='green')  # RSL
    drawer.draw_line(dubins.lsl_tang_i, dubins.lsl_tang_f, fill='blue')  # LSL
    drawer.draw_line(dubins.lsr_tang_i, dubins.lsr_tang_f, fill='purple')  # LSR

    print("rot: {}".format(dubins.abs_rotation))
    drawer.draw_dubins_path(dubins, width=2)

    # show a specific path by uncommenting a path below
    #draw RSR
    # drawer.draw_arc(dubins.cr_i, dubins.rsr_gamma_ii, dubins.rsr_gamma_if, rho, direction='R', outline='red', width=2)
    # drawer.draw_line(dubins.rsr_tang_i, dubins.rsr_tang_f, fill='red', width=2)
    # drawer.draw_arc(dubins.cr_f, dubins.rsr_gamma_fi, dubins.rsr_gamma_ff, rho, direction='R', outline='red', width=2)
    # (text, tcolor) = ('INVALID', 'red') if dubins.rsr_valid else ('VALID length: {}'.format(dubins.rsr_dist), 'green')
    # drawer.canvas.create_text(150, 50, text=text, fill=tcolor)

    # draw RSL
    # drawer.draw_arc(dubins.cr_i, dubins.rsl_gamma_ii, dubins.rsl_gamma_if, rho, direction='R', outline='red', width=2)
    # drawer.draw_line(dubins.rsl_tang_i, dubins.rsl_tang_f, fill='red', width=2)
    # drawer.draw_arc(dubins.cl_f, dubins.rsl_gamma_fi, dubins.rsl_gamma_ff, rho, direction='L', outline='red', width=2)
    # (text, tcolor) = ('INVALID', 'red') if dubins.rsl_valid else ('VALID length: {}'.format(dubins.rsl_dist), 'green')
    # drawer.canvas.create_text(50, 50, text=text, fill=tcolor)


    #draw LSL
    # drawer.draw_arc(dubins.cl_i, dubins.lsl_gamma_ii, dubins.lsl_gamma_if, rho, direction='L', outline='green', width=2)
    # drawer.draw_line(dubins.lsl_tang_i, dubins.lsl_tang_f, fill='green', width=2)
    # drawer.draw_arc(dubins.cl_f, dubins.lsl_gamma_fi, dubins.lsl_gamma_ff, rho, direction='L', outline='green', width=2)
    # (text, tcolor) = ('INVALID', 'red') if dubins.lsl_valid else ('VALID length: {}'.format(dubins.lsl_dist), 'green')
    # drawer.canvas.create_text(50, 50, text=text, fill=tcolor)

    #draw LSR
    # drawer.draw_arc(dubins.cl_i, dubins.lsr_gamma_ii, dubins.lsr_gamma_if, rho, direction='L', outline='green', width=2)
    # drawer.draw_line(dubins.lsr_tang_i, dubins.lsr_tang_f, fill='green', width=2)
    # drawer.draw_arc(dubins.cr_f, dubins.lsr_gamma_fi, dubins.lsr_gamma_ff, rho, direction='R', outline='green', width=2)
    # (text, tcolor) = ('INVALID', 'red') if dubins.lsr_valid else ('VALID length: {}'.format(dubins.lsr_dist), 'green')
    # drawer.canvas.create_text(50, 50, text=text, fill=tcolor)

    # p_n = dubins.rsr_path()
    # drawer.plot_path(p_n)
    return

def test_dubins():
    image_path = os.path.join("maps", "map0_1000_600" + '.bmp')
    x1, y1, x2, y2 = 200, 400, 220, 380
    thetaStart = math.pi*0.3
    thetaFinal = math.pi*0
    drawer = TKDrawer(1000, 600)
    drawer.draw_image(image_path)
    qStart = [x1, y1, thetaStart]
    qFinal = [x2, y2, thetaFinal]
    draw_dubins(drawer, qStart, qFinal)
    drawer.runMainLoop()

    drawer.runMainLoop()
    return

if __name__ == '__main__':
    test_dubins()
