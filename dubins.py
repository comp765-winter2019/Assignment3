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
import numpy as np
from geometry_tools import *
import sys

class DubinsPath:
    def __init__(self,pose_i, pose_f, rho):
        self.pose_i = pose_i
        self.pose_f = pose_f
        self.rho = rho

        self.calculate_geometry()
        return

    def calculate_geometry(self):
        self.theta_i = self.pose_i[2]
        self.theta_f = self.pose_f[2]

        self.cr_i = [self.pose_i[0] + self.rho * math.sin(self.pose_i[2]),
                     self.pose_i[1] - self.rho * math.cos(self.pose_i[2])]
        self.cl_i = [self.pose_i[0] - self.rho * math.sin(self.pose_i[2]),
                     self.pose_i[1] + self.rho * math.cos(self.pose_i[2])]
        self.cr_f = [self.pose_f[0] + self.rho * math.sin(self.pose_f[2]),
                     self.pose_f[1] - self.rho * math.cos(self.pose_f[2])]
        self.cl_f = [self.pose_f[0] - self.rho * math.sin(self.pose_f[2]),
                     self.pose_f[1] + self.rho * math.cos(self.pose_f[2])]

        #RSR
        self.rsr_valid = True
        self.dist_cri_crf = math.sqrt((self.cr_f[0] - self.cr_i[0]) ** 2 + (self.cr_f[1] - self.cr_i[1]) ** 2)
        self.rsr_alpha = math.atan2((self.cr_f[1] - self.cr_i[1]), (self.cr_f[0] - self.cr_i[0]))
        self.rsr_beta = pi_2_pi(self.rsr_alpha)
        self.rsr_gamma = pi_2_pi(self.rsr_alpha - 3 * math.pi / 2)
        self.rsr_gamma_ii = pi_2_pi(self.theta_i + math.pi / 2)
        self.rsr_gamma_if = self.rsr_gamma
        self.rsr_gamma_fi = self.rsr_gamma
        self.rsr_gamma_ff = pi_2_pi(self.theta_f + math.pi / 2)
        self.rsr_tang_i = [self.cr_i[0] + self.rho * math.cos(self.rsr_gamma), self.cr_i[1] + self.rho * math.sin(self.rsr_gamma)]
        self.rsr_tang_f = [self.cr_f[0] + self.rho * math.cos(self.rsr_gamma), self.cr_f[1] + self.rho * math.sin(self.rsr_gamma)]
        self.rsr_i_arc_len = arc_length(self.theta_i, self.rsr_beta, self.rho, direction='R')
        self.rsr_f_arc_len = arc_length(self.rsr_beta, self.theta_f, self.rho, direction='R')
        self.rsr_dist = self.dist_cri_crf + self.rsr_i_arc_len + self.rsr_f_arc_len

        #RSL
        self.rsl_valid = True
        self.dist_cri_clf = math.sqrt((self.cl_f[0] - self.cr_i[0]) ** 2 + (self.cl_f[1] - self.cr_i[1]) ** 2)
        self.rsl_alpha = math.atan2((self.cl_f[1] - self.cr_i[1]), (self.cl_f[0] - self.cr_i[0]))
        if 2*self.rho/self.dist_cri_clf > 1:
            self.rsl_valid = False
            self.rsl_omega = 0
        else:
            self.rsl_omega = math.acos(2*self.rho/self.dist_cri_clf)
        self.rsl_gamma = pi_2_pi(self.rsl_omega + self.rsl_alpha - 2*math.pi)
        self.rsl_gamma_ii = pi_2_pi(self.theta_i + math.pi / 2)
        self.rsl_gamma_if = self.rsl_gamma
        self.rsl_gamma_fi = pi_2_pi(self.rsl_gamma + math.pi)
        self.rsl_gamma_ff = pi_2_pi(self.theta_f - math.pi / 2)
        self.rsl_beta = pi_2_pi(self.rsl_gamma - math.pi/2)
        self.rsl_tang_i = [self.cr_i[0] + self.rho * math.cos(self.rsl_gamma), self.cr_i[1] + self.rho * math.sin(self.rsl_gamma)]
        self.rsl_tang_f = [self.cl_f[0] + self.rho * math.cos(self.rsl_gamma + math.pi), self.cl_f[1] + self.rho * math.sin(self.rsl_gamma + math.pi)]
        self.rsl_i_arc_len = arc_length(self.theta_i, self.rsl_beta, self.rho, direction='R')
        self.rsl_f_arc_len = arc_length(self.rsl_beta, self.theta_f, self.rho, direction='L')
        self.rsl_dist = self.dist_cri_clf + self.rsl_i_arc_len + self.rsl_f_arc_len

        # LSL
        self.lsl_valid = True
        self.dist_cli_clf = math.sqrt((self.cl_f[0] - self.cl_i[0]) ** 2 + (self.cl_f[1] - self.cl_i[1]) ** 2)
        self.lsl_alpha = math.atan2((self.cl_f[1] - self.cl_i[1]), (self.cl_f[0] - self.cl_i[0]))
        self.lsl_gamma = pi_2_pi(self.lsl_alpha - math.pi / 2)
        self.lsl_beta = pi_2_pi(self.lsl_alpha)
        self.lsl_gamma_ii = pi_2_pi(self.theta_i - math.pi / 2)
        self.lsl_gamma_if = self.lsl_gamma
        self.lsl_gamma_fi = self.lsl_gamma
        self.lsl_gamma_ff = pi_2_pi(self.theta_f - math.pi / 2)
        self.lsl_tang_i = [self.cl_i[0] + self.rho * math.cos(self.lsl_gamma), self.cl_i[1] + self.rho * math.sin(self.lsl_gamma)]
        self.lsl_tang_f = [self.cl_f[0] + self.rho * math.cos(self.lsl_gamma), self.cl_f[1] + self.rho * math.sin(self.lsl_gamma)]
        self.lsl_i_arc_len = arc_length(self.theta_i, self.lsl_beta, self.rho, direction='L')
        self.lsl_f_arc_len = arc_length(self.lsl_beta, self.theta_f, self.rho, direction='L')
        self.lsl_dist = self.dist_cli_clf + self.lsl_i_arc_len + self.lsl_f_arc_len

        # LSR
        self.lsr_valid = True
        self.dist_cli_crf = math.sqrt((self.cr_f[0] - self.cl_i[0]) ** 2 + (self.cr_f[1] - self.cl_i[1]) ** 2)
        self.lsr_alpha = math.atan2((self.cr_f[1] - self.cl_i[1]), (self.cr_f[0] - self.cl_i[0]))
        if 2 * self.rho / self.dist_cli_crf > 1:
            self.lsr_valid = False
            self.lsr_omega = 0
        else:
            self.lsr_omega = math.acos(2 * self.rho / self.dist_cli_crf)
        self.lsr_gamma = pi_2_pi(self.lsr_alpha - self.lsr_omega)
        self.lsr_beta = pi_2_pi(self.lsr_gamma + math.pi / 2)
        self.lsr_gamma_ii = pi_2_pi(self.theta_i - math.pi / 2)
        self.lsr_gamma_if = self.lsr_gamma
        self.lsr_gamma_fi = pi_2_pi(self.lsr_gamma - math.pi)
        self.lsr_gamma_ff = pi_2_pi(self.theta_f + math.pi / 2)
        self.lsr_tang_i = [self.cl_i[0] + self.rho * math.cos(self.lsr_gamma), self.cl_i[1] + self.rho * math.sin(self.lsr_gamma)]
        self.lsr_tang_f = [self.cr_f[0] + self.rho * math.cos(self.lsr_gamma + math.pi), self.cr_f[1] + self.rho * math.sin(self.lsr_gamma + math.pi)]
        self.lsr_i_arc_len = arc_length(self.theta_i, self.lsr_beta, self.rho, direction='L')
        self.lsr_f_arc_len = arc_length(self.lsr_beta, self.theta_f, self.rho, direction='R')
        self.lsr_dist = self.dist_cli_crf + self.lsr_i_arc_len + self.lsr_f_arc_len

        self.distances = np.array([self.rsr_dist, self.rsl_dist, self.lsl_dist, self.lsr_dist])
        # print(self.distances)
        paths = ['RSR','RSL','LSL','LSR']
        self.valid_paths = np.array([self.rsr_valid, self.rsl_valid, self.lsl_valid, self.lsr_valid])
        self.shortest_path = paths[0]  # paths[np.argmin(self.distances)]
        self.shortest_distance = sys.float_info.max
        for i in range(len(paths)):
            if self.distances[i] < self.shortest_distance and self.valid_paths[i]:
                self.shortest_distance = self.distances[i]
                self.shortest_path = paths[i]

        self.shortest_distance = self.distances[np.argmin(self.distances)]
        # print("max path: {}".format(paths[np.argmin(self.distances)]))

        # some variables to make interpreting the paths easier
        # if self.shortest_path == 'RSR':
        self.dir_i = 'R'
        self.dir_f = 'R'
        self.c_i = self.cr_i
        self.c_f = self.cr_f
        self.gamma_ii = self.rsr_gamma_ii
        self.gamma_if = self.rsr_gamma_if
        self.gamma_fi = self.rsr_gamma_fi
        self.gamma_ff = self.rsr_gamma_ff
        self.tang_i = self.rsr_tang_i
        self.tang_f = self.rsr_tang_f

        if self.shortest_path == 'RSL':
            self.dir_i = 'R'
            self.dir_f = 'L'
            self.c_i = self.cr_i
            self.c_f = self.cl_f
            self.gamma_ii = self.rsl_gamma_ii
            self.gamma_if = self.rsl_gamma_if
            self.gamma_fi = self.rsl_gamma_fi
            self.gamma_ff = self.rsl_gamma_ff
            self.tang_i = self.rsl_tang_i
            self.tang_f = self.rsl_tang_f
        if self.shortest_path == 'LSL':
            self.dir_i = 'L'
            self.dir_f = 'L'
            self.c_i = self.cl_i
            self.c_f = self.cl_f
            self.gamma_ii = self.lsl_gamma_ii
            self.gamma_if = self.lsl_gamma_if
            self.gamma_fi = self.lsl_gamma_fi
            self.gamma_ff = self.lsl_gamma_ff
            self.tang_i = self.lsl_tang_i
            self.tang_f = self.lsl_tang_f
        if self.shortest_path == 'LSR':
            self.dir_i = 'L'
            self.dir_f = 'R'
            self.c_i = self.cl_i
            self.c_f = self.cr_f
            self.gamma_ii = self.lsr_gamma_ii
            self.gamma_if = self.lsr_gamma_if
            self.gamma_fi = self.lsr_gamma_fi
            self.gamma_ff = self.lsr_gamma_ff
            self.tang_i = self.lsr_tang_i
            self.tang_f = self.lsr_tang_f

        self.abs_rotation = rotation_magnitude(self.gamma_ii, self.gamma_if, self.dir_i)
        self.abs_rotation += rotation_magnitude(self.gamma_fi, self.gamma_ff, self.dir_f)
        return

    def path_collision(self, p1, p2, car_width=0):
        offsets = [0]
        if car_width > 0:
            offsets = [car_width / 2.0, -car_width / 2.0]

        collision_points = list()
        for offset in offsets:
            radius = self.rho + offset
            # check if first arc collides
            collision_points.extend(line_intersect_arc(self.c_i, radius, self.gamma_ii, self.gamma_if, self.dir_i, p1, p2))
            # check if second arc collides
            collision_points.extend(line_intersect_arc(self.c_f, radius, self.gamma_fi, self.gamma_ff, self.dir_f, p1, p2))

            # check if strait path collides
            si = 1 if self.dir_i == 'R' else -1
            sf = 1 if self.dir_f == 'R' else -1
            pointi_radius, pointf_radius = (self.rho + si * offset), (self.rho + sf * offset)
            point_i = [self.c_i[0] + pointi_radius * math.cos(self.gamma_if), self.c_i[1] + pointi_radius * math.sin(self.gamma_if)]
            point_f = [self.c_f[0] + pointf_radius * math.cos(self.gamma_fi), self.c_f[1] + pointf_radius * math.sin(self.gamma_fi)]
            if lines_intersect(point_i, point_f, p1, p2):
                collision_points.extend(point_of_line_intersect(point_i, point_f, p1, p2))

        return collision_points
