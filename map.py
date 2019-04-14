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

import pickle
import os
from geometry_tools import *


class Map:
    def __init__(self, width, height, map_name):
        self.width = width
        self.height = height
        pickle_file = os.path.join("maps", map_name + ".p")
        if os.path.exists(pickle_file):
            self.edges = pickle.load(open(pickle_file, "rb"))
        else:
            self.edges = []
        return

    def check_collision_line(self, pose0, pose1):
        for edge in self.edges:
            if lines_intersect(edge[0], edge[1], pose0, pose1):
                return True
        boundary_edges = [((0, 0), (self.width, 0)), ((self.width, 0), (self.width, self.height)),
                          ((self.width, self.height), (0, self.height)), ((0, self.height), (0, 0))]
        for edge in boundary_edges:
            if lines_intersect(edge[0], edge[1], pose0, pose1):
                return True
        return False

    def collision_on_dubins(self, dubins, car_width=0):
        collision_points = []
        if dubins is None:
            return []
        for edge in self.edges:
            collision_points.extend(dubins.path_collision(edge[0], edge[1], car_width=car_width))

        boundary_edges = [((0, 0),(self.width, 0)), ((self.width, 0),(self.width, self.height)),
                          ((self.width, self.height), (0, self.height)), ((0, self.height),(0, 0))]
        for edge in boundary_edges:
            collision_points.extend(dubins.path_collision(edge[0], edge[1], car_width=car_width))
        return collision_points

