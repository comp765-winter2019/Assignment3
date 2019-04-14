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

# import Tkinter as tk
import tkinter as tk
from PIL import Image, ImageTk
import numpy as np
import os
import math
import networkx as nx
import pickle

'''
u - undo
s - save
o - load
e - edge_mode
v - vertex mode
a - append vertex mode on
z - append vertex mode off
'''
class MapEditor:
    # radius of drawn points on canvas
    RADIUS = 3

    # flag to lock the canvas when drawn
    LOCK_FLAG = False

    def __init__(self, map_name="temp", use_image=False, width=1000, height=600):
        self.master = tk.Tk()
        self.width = width
        self.height = height

        self.map_name = map_name
        self.map_directory = 'maps'
        self.use_image = use_image
        if use_image:
            self.image_file = map_name + ".bmp"
            self.image_path = os.path.join(self.map_directory, self.image_file)
            self.image = Image.open(self.image_path)
            self.tkimage = ImageTk.PhotoImage(self.image)
        # Tkinter.Label(root, image=tkimage).pack()
            self.width = self.tkimage.width()
            self.height = self.tkimage.height()
        self.canvas = tk.Canvas(self.master, width=self.width, height=self.height)

        # self.w.config(background='white')
        self.canvas.bind('<Double-1>', self.onDoubleClick)
        self.canvas.bind('<Double-3>', self.on_double_right_click)
        self.canvas.pack()
        self.canvas.bind("<KeyRelease>", self.key)
        self.canvas.focus_set()
        #self.w.bind("<1>", lambda print: self.w.focus_set())
        self.canvas.pack()

        self.previousPoint=None
        self.edges = list()

        self.mode = "edge"
        self.append = False

        self.new_vertex = False

        self.re_draw()

    def add_edge(self, p0, p1):
        self.edges.append((p0, p1))
        return

    def save_map(self):
        print("saving")
        save_file_name = self.map_name + ".p"
        save_file_path = os.path.join(self.map_directory, save_file_name)
        directory = os.path.dirname(save_file_path)
        if not os.path.exists(directory):
            os.makedirs(directory)
        pickle.dump(self.edges, open(save_file_path, "wb"))
        return

    def load_map(self):
        print("loading")
        directory = self.map_directory
        save_file_name = self.map_name + ".p"
        save_file_path = os.path.join(directory, save_file_name)
        self.edges = pickle.load(open(save_file_path, "rb"))
        if len(self.edges) > 1:
            # self.edges.pop()
            self.previousPoint = self.edges[len(self.edges) - 1][1]
        elif len(self.edges) == 1:
            self.previousPoint = self.edges[len(self.edges) - 1][0]
            self.edges.pop()
        else:
            self.previousPoint = None
        self.re_draw()

    def draw_vertex(self, point, fill="blue"):
        if not self.LOCK_FLAG:
            x = point[0]
            y = point[1]
            self.canvas.create_oval(x - self.RADIUS, self.height - (y - self.RADIUS), x + self.RADIUS,
                                    self.height - (y + self.RADIUS), fill=fill)
        return

    def draw_edge(self, edge):
        if not self.LOCK_FLAG:
            p0 = edge[0]
            p1 = edge[1]
            self.canvas.create_line(p0[0], self.height - p0[1], p1[0], self.height - p1[1], fill='red')
        return

    def re_draw(self):
        self.canvas.delete("all")
        # self.canvas = tk.Canvas(self.master, width=self.width + 2, height=self.height + 2)
        if self.use_image:
            self.canvas.create_image(((self.width + 2) / 2, (self.height + 2) / 2), image=self.tkimage)
        self.canvas.create_text(180, 80,
                                text="u - undo\n"
                                     "s - save\n"
                                     "o - load\n"
                                     "e - edge_mode - add an edge\n"
                                     "v - vertex mode - add a vertex to an edge\n"
                                     "a - append vertex mode on - connect to an existing vertex\n"
                                     "z - append vertex mode off - create new vertex and edge\n"
                                     "c - create new vertex in free space\np - show enclosed polygons",
                                fill='green')
        self.canvas.pack()
        if len(self.edges) == 0:
            if self.previousPoint is not None:
                self.draw_vertex(self.previousPoint)
        for edge in self.edges:
            p0 = edge[0]
            p1 = edge[1]
            self.draw_vertex(p0)
            self.draw_vertex(p1)
            self.draw_edge(edge)

        #draw closed polygons
        G = nx.DiGraph()
        for edge in self.edges:
            G.add_edge(edge[0], edge[1])
        cycles = list(nx.simple_cycles(G))
        return


    def key(self, event):
        # print("pressed", repr(event.char))
        if event.char == 'u':
            print("pressed u")
            if len(self.edges) > 1:
                self.edges.pop()
                self.previousPoint = self.edges[len(self.edges)-1][1]
            elif len(self.edges) == 1:
                self.previousPoint = self.edges[len(self.edges) - 1][0]
                self.edges.pop()
            else:
                self.previousPoint = None
            self.re_draw()
        if event.char == 's':
            self.save_map()
        if event.char == 'o':
            self.load_map()
        if event.char == 'e':
            print("edge mode")
            self.mode = "edge"
        if event.char == 'v':
            print("vertex mode")
            self.mode = "vertex"
        if event.char == 'a':
            print("append vertex mode")
            self.append = True
        if event.char == 'z':
            print("append vertex mode off")
            self.append = False
        if event.char == 'c':
            print("create new vertex")
            self.new_vertex = True
        if event.char == 'p':
            G = nx.DiGraph()
            for edge in self.edges:
                G.add_edge(edge[0], edge[1])
            cycles = list(nx.simple_cycles(G))

            for cycle in cycles:
                px = list()
                for node in cycle:
                    px.append(node[0])
                    px.append(self.height - node[1])
                self.canvas.create_polygon(px, fill='green')

        return

    def on_double_right_click(self, event):
        clickedPoint = (event.x, self.height - event.y)
        closestDistance = 10e6
        if len(self.edges)>=1: #set it to first point of the first edge. afterwards we only look at the second edge
            point = self.edges[0][0]
            distance = np.linalg.norm(np.array(clickedPoint) - np.array(point))
            if distance < closestDistance:
                closestDistance = distance
                closestPoint = point
        for edge in self.edges:
            point=edge[1]
            distance = np.linalg.norm(np.array(clickedPoint) - np.array(point))
            if distance < closestDistance:
                closestDistance = distance
                closestPoint = point
        print("closest right clicked distance: {}".format(closestDistance))
        if closestDistance < 10:
            self.previousPoint = closestPoint
        return


    def onDoubleClick(self, event):
        print("clicked x: {}, y: {}".format(event.x, self.height - event.y))
        clickedPoint = (event.x, self.height - event.y)
        newPoint = clickedPoint
        if self.new_vertex == True:
            self.draw_vertex(clickedPoint)
            self.previousPoint = newPoint
            self.new_vertex = False
            self.append = False
        else:
            closestDistance = 10e6
            if self.append:
                closestPoint = None

                for edge in self.edges:
                    point=edge[0]
                    distance = np.linalg.norm(np.array(clickedPoint) - np.array(point))
                    if distance < closestDistance:
                        closestDistance = distance
                        closestPoint = point
                    point = edge[1]
                    distance = np.linalg.norm(np.array(clickedPoint) - np.array(point))
                    if distance < closestDistance:
                        closestDistance = distance
                        closestPoint = point

                if closestDistance <= 10:
                    if self.previousPoint is not None:
                        pointToConnect = self.previousPoint

                        p0 = (pointToConnect[0], pointToConnect[1])
                        p1 = (closestPoint[0], closestPoint[1])
                        self.draw_edge((p0, p1))
                        self.add_edge(p0, p1)


                        self.previousPoint = closestPoint
                        return
                else:
                    print("click closer to existing vertex")
                return

            else:
                if self.mode=="edge":
                    self.draw_vertex((event.x, self.height - event.y))
                    if self.previousPoint is not None:
                        pointToConnect=self.previousPoint
                        closestPoint = None
                        closestDistance = 10e6 #something big
                        for edge in self.edges:
                            point = edge[1]
                            distance = np.linalg.norm(np.array(newPoint)-np.array(point))
                            if distance < closestDistance:
                                closestDistance = distance
                                closestPoint = point
                        # print("closest distance: {}".format(closestDistance))
                        # if closestDistance < 50:
                        #     pointToConnect = closestPoint

                        p0 = (pointToConnect[0], pointToConnect[1])
                        p1 = (newPoint[0], newPoint[1])
                        self.draw_edge((p0, p1))
                        self.add_edge(p0, p1)

                if self.mode == "vertex":
                    #find nearest point on edge
                    closestDistance = 10e6 #something big
                    closestEdgeIndex = None
                    for e in range(0, len(self.edges)):
                    # for edge in self.edges:
                        edge = self.edges[e]
                        p0 = np.array(edge[0])
                        p1 = np.array(edge[1])
                        p2 = np.array(clickedPoint)

                        # l1 = np.array([p1[0], p1[1], 0]) - np.array([p0[0], p0[1], 0])
                        # l2 = np.array([p2[0], p2[1], 0]) - np.array([p1[0], p1[1], 0])

                        line = p1-p0
                        distance = np.linalg.norm(np.cross(p1-p0, p2-p0))/np.linalg.norm(p1-p0)
                        if distance < closestDistance:
                            closestDistance = distance
                            closestEdgeIndex = e

                    #now get closest point to the vertex
                    edge = self.edges[closestEdgeIndex]
                    p0 = np.array(edge[0])
                    p1 = np.array(edge[1])
                    p2 = np.array(clickedPoint)
                    #             o p0
                    #             |
                    #             |  d
                    #  new vertex o----o p2
                    #             |   /
                    #           l |  /
                    #             |t/ h
                    #             |/
                    #             o p1
                    d=closestDistance
                    h=np.linalg.norm(p2-p1)
                    t = math.asin(d/h)
                    l=h*math.cos(t)
                    u=(p1-p0)/np.linalg.norm(p1-p0)
                    newVertex = p1 - u*l

                    #delete the old edge
                    self.edges.pop(closestEdgeIndex)
                    #create new edges
                    self.add_edge((p0[0], p0[1]), (newVertex[0], newVertex[1]))
                    self.add_edge((p1[0], p1[1]), (newVertex[0], newVertex[1]))
                    self.re_draw()

                    #print("new Vertex at {}".format(newVertex))
                    #self.drawVertex(newVertex, fill="blue")
                    newPoint = newVertex
                    self.mode = "edge"
            self.previousPoint = newPoint

    def drawLinesOnCanvas(self, lines):
        for l in lines:
            self.w.create_line(l[0], l[1], l[2], l[3], fill='blue')


    def runMainLoop(self):
        self.master.mainloop()
        return


def main():
    mapLabeler = MapEditor()
    mapLabeler.runMainLoop()


if __name__ == '__main__':
    main()

