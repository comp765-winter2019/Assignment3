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

from map import *
from drawer import *
from dubins import *
import numpy as np
import sys
import time


class DubinsNode:
    def __init__(self, pose, min_turn_radius=10, parent_node=None):
        self.pose = pose
        self.parent_node = parent_node
        self.dubins_path = None #path from parent
        self.min_turn_radius = min_turn_radius
        return

    def compute_path(self):
        if self.parent_node is not None:
            self.dubins_path = DubinsPath(self.parent_node.pose, self.pose, self.min_turn_radius)
        return


class DubinsTree:
    def __init__(self, root_node):
        self.root_node = root_node
        # self.vertices = list()
        self.nodes = list()
        self.nodes.append(root_node)
        return

class DubinsRRT:
    def __init__(self, map, pose_start, pose_end):
        self.map = map
        self.pose_start = pose_start
        self.pose_end = pose_end
        self.min_turn = 15  # minimum turning radius of the car
        self.car_width = 8
        self.root_node = DubinsNode(pose_start, self.min_turn)
        self.tree = DubinsTree(self.root_node)
        self.max_translation = 40  # self.min_turn * 6
        self.min_translation = 4  # self.min_turn * 1  # too small the geometry will fail for dubins curve
        self.max_relative_rotation = math.pi / 2
        self.max_abs_rotation = math.pi
        self.turning_cost_scale = 6 * math.pi
        self.goal_reached_threshold = 20.0
        self.goal_distance = self.config_config_distance(pose_start, pose_end)
        self.closest_goal_node = self.root_node
        self.show_path = True
        self.ignore_collisions = False
        self.debug = False  # shows special points and collisions
        return

    def random_configuration(self):
        config = np.random.uniform([0, 0, 0], [self.map.width, self.map.height, 2*math.pi], 3)
        # print("random config: {}".format(config))
        return config

    def config_config_distance(self, pose0, pose1):
        llsq = 0
        h = pose1[0] - pose0[0]
        llsq = llsq + (h * h)
        h = pose1[1] - pose0[1]
        llsq = llsq + (h * h)

        h, dir = small_angle(pose1[2], pose0[2])
        llsq = llsq + self.turning_cost_scale * (h * h)
        return math.sqrt(llsq)

    def added_cost_for_direction_change(self, pose0, pose1):
        added_cost = 0
        p0 = (pose0[0], pose0[1])
        dist_in_front = self.min_turn
        p1 = (pose0[0] + dist_in_front * math.cos(pose0[2]), pose0[1] + dist_in_front * math.sin(pose0[2]))  # a point in front of pose
        p2 = (pose0[0] + 2 * dist_in_front * math.cos(pose0[2]), pose0[1] + 2 * dist_in_front * math.sin(pose0[2]))  # a point way in front of pose

        # two points on a perpendicular line
        perp_line0 = p1
        perp_line1 = (p1[0], -p1[1])

        # opposite sides of line if signs are different
        sign0 = side_of_line(perp_line0, perp_line1, p2)
        sign1 = side_of_line(perp_line0, perp_line1, (pose1[0], pose1[1]))
        side = sign0 * sign1

        opposite_dir = compare_direction(pose0[2], pose1[2])

        if side > 0: # new pose is in front
            if opposite_dir >= 0:  # same direction
                added_cost += 0
            else:
                added_cost += math.pi * self.min_turn
        else:  # is behind
            if opposite_dir >= 0:  # same direction
                added_cost += 2 * math.pi * self.min_turn
            else:
                added_cost += math.pi * self.min_turn
        # if side * opposite_dir < 0:  # -1
        #     return 100 * self.min_turn
        return added_cost

    def find_nearest_node(self, pose):
        min_cost = sys.float_info.max
        # closest_node = self.root_node
        closest_node = None
        temp_node = DubinsNode(pose)

        n=0
        for node in self.tree.nodes:
            # use dubins to find nearest node (expensive!!)
            new_pose = self.new_configuration_from_nearest(node, pose)
            dubins_path = DubinsPath(node.pose, new_pose, self.min_turn)
            dubins_distance = dubins_path.shortest_distance
            config_distance = self.config_config_distance(new_pose, pose)
            cost = config_distance + dubins_distance
            # print("node: {}, from: {}, to: {}, dubins_distance: {}, config_distance: {}".format(n, node.pose, new_pose, dubins_distance, config_distance))
            if cost < min_cost and self.valid_configuration(node.pose, new_pose, dubins_path):
            # if cost < min_cost:
                min_cost = cost
                # print("min cost: {}".format(min_cost))
                closest_node = node

            # use simpler metric
            # dist = self.config_config_distance(node.pose, pose)
            # added = self.added_cost_for_direction_change(node.pose, pose)
            #
            # cost = dist + added
            # # print("node: {}, cost: {}, added: {}".format(n, dist, added))
            # if cost < min_cost:
            #     # print("min cost: {}, added: {}".format(dist, added))
            #     min_cost = cost
            #     closest_node = node
            n += 1
        return closest_node, min_cost

    def new_configuration_from_nearest(self, nearest_node, pose):
        new_pose = list(pose)
        dx = pose[0] - nearest_node.pose[0]
        dy = pose[1] - nearest_node.pose[1]
        dist = math.sqrt(dx**2 + dy**2)
        unit_dir = [dx/dist, dy/dist]
        d_theta, rot_dir = small_angle(nearest_node.pose[2], pose[2])
        # print("{}, {}, d_theta: {}, rot_dir: {}".format(nearest_node.pose[2], pose[2], d_theta, rot_dir))
        if dist > self.max_translation:
            new_pose[0] = nearest_node.pose[0] + self.max_translation * unit_dir[0]
            new_pose[1] = nearest_node.pose[1] + self.max_translation * unit_dir[1]
        if d_theta > self.max_relative_rotation:
            new_pose[2] = nearest_node.pose[2] + rot_dir * self.max_relative_rotation
        return new_pose

    def has_no_collisions(self, dubins_node):
        if not self.ignore_collisions:
            #check for collisions on dubins path (this is more expensive)
            collisions = self.map.collision_on_dubins(dubins_node.dubins_path, car_width=self.car_width)

            # for node in self.tree.nodes:
            #     #check for collisions against other paths
            #     if node.parent_node is not None:
            #         tree_collisions = dubins_node.dubins_path.path_collision(node.parent_node.pose, node.pose)
            #         collisions.extend(tree_collisions)

            # print(collisions)
            if len(collisions) > 0:
                return False
        return True

    def valid_configuration(self, pose0, pose1, dubins_path):
        # make sure there is enough room to do dubins paths
        dx = pose1[0] - pose0[0]
        dy = pose1[1] - pose0[1]
        dist = math.sqrt(dx ** 2 + dy ** 2)
        if dist < self.min_translation:
            return False

        if not self.ignore_collisions:
            # check for SIMPLE collisions between two poses (because this is faster)
            if self.map.check_collision_line(pose0, pose1):
                return False

        #check total rotation
        if dubins_path.abs_rotation > self.max_abs_rotation:
            return False

        # check for collisions on dubins path
        # if len(self.map.collision_on_dubins(dubins_path)) > 0:
        #     return False
        return True


    def expand_tree(self, config=None):
        destination_reached = False
        if config is None:
            config = self.random_configuration()
        nearest_node, cost = self.find_nearest_node(config)
        if nearest_node is not None:
            # print("nearest: {}, random: {}".format(nearest_node.pose, random_configuration))
            new_configuration = self.new_configuration_from_nearest(nearest_node, config)
            temp_node = DubinsNode(new_configuration, self.min_turn, nearest_node)
            temp_node.compute_path()
            if self.valid_configuration(temp_node.parent_node.pose, temp_node.pose, temp_node.dubins_path):
                if self.has_no_collisions(temp_node):
                    # add to tree
                    new_node = temp_node
                    if new_node is not None:
                        self.tree.nodes.append(new_node)

                        # check if near goal
                        goal_distance = self.config_config_distance(new_node.pose, self.pose_end)
                        if goal_distance < self.goal_distance:
                            self.closest_goal_node = new_node
                            self.goal_distance = goal_distance
                        if goal_distance <= self.goal_reached_threshold:
                            destination_reached = True
                            print("Destination Reached: pose: {}, target: {}".format(new_node.pose, self.pose_end))
                        return new_node, destination_reached
        return None, destination_reached



class RRTRunner:
    def __init__(self, rrt, drawer):
        self.rrt = rrt
        self.drawer = drawer

        # self.drawer.canvas.bind('<Double-1>', self.onDoubleClick)
        self.drawer.canvas.bind('<Double-3>', self.on_right_double_click)
        self.drawer.canvas.bind('<ButtonRelease-1>', self.on_left_release)
        self.drawer.canvas.bind('<ButtonPress-1>', self.on_left_press)
        self.drawer.canvas.focus_set()
        self.last_clicked_point = None
        self.draw_line = None
        self.draw_point = None
        return

    def on_left_release(self, event):
        self.drawer.canvas.unbind("<Motion>")
        clickedPoint = (event.x, self.rrt.map.height - event.y)
        angle = pi_2_pi(math.atan2(clickedPoint[1] - self.last_clicked_point[1], clickedPoint[0] - self.last_clicked_point[0]))
        print("new point at: {}".format((clickedPoint[0], clickedPoint[1], angle)))

        new_node, destination_reached  = self.rrt.expand_tree(config=(clickedPoint[0], clickedPoint[1], angle))
        if new_node is not None:
            self.drawer.draw_dubins_path(new_node.dubins_path, car_width=self.rrt.car_width)
            self.draw_debug(new_node)
            self.drawer.draw_arrow(new_node.pose, fill='blue')
            print("added dubins from: {} to {}".format(new_node.parent_node.pose, new_node.pose))
        self.drawer.canvas.delete(self.draw_line)
        self.drawer.canvas.delete(self.draw_point)

        self.drawer.master.update()

        # for edge in self.rrt.map.edges:
        #     intersection = line_intersect_circle(clickedPoint, self.rrt.min_turn, edge[0], edge[1])
        #     if len(intersection) > 0:
        #         print("edge: {}".format(edge))
        #     for point in intersection:
        #         self.drawer.draw_circle(point, 2, fill='red')
        #         self.drawer.master.update()
        #         time.sleep(0.025)
        #
        #         # check if intersections are within gamma_i and gamma_f
        #         angle = math.atan2((point[1] - clickedPoint[1]), (point[0] - clickedPoint[0]))
        #         angle = pi_2_pi(angle)
        #         print("angle: {}".format(angle))
        return

    def mouse_motion(self, event):
        position = (event.x, self.rrt.map.height - event.y)
        # print(position)
        self.drawer.canvas.delete(self.draw_line)
        self.draw_line = self.drawer.draw_line(self.last_clicked_point, position)
        self.drawer.master.update()
        return

    def on_left_press(self, event):
        # print("single clicked x: {}, y: {}".format(event.x, self.rrt.map.height - event.y))
        clicked_point = (event.x, self.rrt.map.height - event.y)
        self.last_clicked_point = clicked_point
        self.draw_point = self.drawer.draw_circle(clicked_point, 3, fill='red')
        self.drawer.master.update()
        self.drawer.canvas.bind("<Motion>", self.mouse_motion)
        return

    def on_right_double_click(self, event):
        print("double clicked x: {}, y: {}".format(event.x, self.rrt.map.height - event.y))
        clicked_point = (event.x, self.rrt.map.height - event.y)
        if self.last_clicked_point is not None:
            for edge in self.rrt.map.edges:
                intersection = point_of_line_intersect(self.last_clicked_point, clicked_point, edge[0], edge[1])
                for point in intersection:
                    print("intersection is: {}".format(point))
                    self.drawer.draw_circle(point, 3, fill='red')
                    self.drawer.master.update()
        return

    def draw_debug(self, new_node):
        # debug code - check for collisions on dubins path
        if self.rrt.debug:
            # self.drawer.draw_circle(new_node.dubins_path.c_i, 3, fill='green')
            # self.drawer.draw_circle(new_node.dubins_path.c_f, 3, fill='green')
            self.drawer.draw_arrow(new_node.pose, fill='blue')

            p1 = (new_node.dubins_path.c_i[0] + new_node.dubins_path.rho * math.cos(new_node.dubins_path.gamma_ii), \
                  new_node.dubins_path.c_i[1] + new_node.dubins_path.rho * math.sin(new_node.dubins_path.gamma_ii))
            self.drawer.draw_circle(p1, 3, fill='purple')
            p2 = (new_node.dubins_path.c_i[0] + new_node.dubins_path.rho * math.cos(new_node.dubins_path.gamma_if), \
                  new_node.dubins_path.c_i[1] + new_node.dubins_path.rho * math.sin(new_node.dubins_path.gamma_if))
            self.drawer.draw_circle(p2, 3, fill='purple')
            p3 = (new_node.dubins_path.c_f[0] + new_node.dubins_path.rho * math.cos(new_node.dubins_path.gamma_fi), \
                  new_node.dubins_path.c_f[1] + new_node.dubins_path.rho * math.sin(new_node.dubins_path.gamma_fi))
            self.drawer.draw_circle(p3, 3, fill='blue')
            p4 = (new_node.dubins_path.c_f[0] + new_node.dubins_path.rho * math.cos(new_node.dubins_path.gamma_ff), \
                  new_node.dubins_path.c_f[1] + new_node.dubins_path.rho * math.sin(new_node.dubins_path.gamma_ff))
            self.drawer.draw_circle(p4, 3, fill='blue')

            text = new_node.dubins_path.shortest_path
            tx, ty = (p2[0] + p3[0]) / 2, (p2[1] + p3[1]) / 2
            self.drawer.canvas.create_text(tx, self.drawer.height - ty, text=text, fill='green')

            # show collisions
            collision_points = []
            for edge in self.rrt.map.edges:
                path_collisions = new_node.dubins_path.path_collision(edge[0], edge[1], car_width=self.rrt.car_width)
                collision_points.extend(path_collisions)
                for point in path_collisions:
                    self.drawer.draw_circle(point, 3, fill='red')
            # check for collisions against other paths
            # for node in self.rrt.tree.nodes:
                # if node.parent_node is not None:
                #     tree_collisions = new_node.dubins_path.path_collision(node.dubins_path.tang_i, node.dubins_path.tang_f, car_width=self.rrt.car_width)
                #     collision_points.extend(tree_collisions)
                #     for point in tree_collisions:
                #         self.drawer.draw_circle(point, 3, fill='red')
        return

    def rrt_search(self, max_iteration=20):
        n = 0
        destination_reached = False
        while n < max_iteration and not destination_reached:
            new_node, destination_reached = self.rrt.expand_tree()
            if new_node is not None:
                n += 1
                # self.drawer.draw_dubins_path(new_node.dubins_path, car_width=self.rrt.car_width)
                self.drawer.draw_dubins_path(new_node.dubins_path)
                if n % 1 == 0:
                    self.drawer.update_text("iteration: {}, distance to goal: {:0.1f}".format(n, self.rrt.goal_distance))
                self.draw_debug(new_node)
                self.drawer.master.update()
                # time.sleep(0.025)

        if n == max_iteration:
            print("Max Iterations Reached: closest node: {}, distance to goal: {:0.1f}".format(self.rrt.closest_goal_node.pose, self.rrt.goal_distance))

        #draw shortest path
        if self.rrt.show_path and len(self.rrt.tree.nodes) > 1:
            node = self.rrt.closest_goal_node
            while node is not None and node.dubins_path is not None:
                self.drawer.draw_dubins_path(node.dubins_path, width=2, color='green', car_width=self.rrt.car_width)
                self.drawer.master.update()
                node = node.parent_node
        self.drawer.runMainLoop()
        return


