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

from rrt import *
from map_editor import *
import matplotlib
import matplotlib.pyplot as plt

""" run map labeler without image"""
# map_name = 'temp'
# map_editor = MapEditor(map_name=map_name, use_image=False, width=1600, height=1000)
# map_editor.runMainLoop()

""" run map labeler with background image"""
# map_name = 'map0_1000_600'
# map_editor = MapEditor(map_name=map_name, use_image=True)
# map_editor.runMainLoop()

""" run dubins rrt"""
map_name = 'map5_1000_600'
width = 1000
height = 600
pose_start = [100, 100, math.pi*0.3]
pose_end = [900, 500, math.pi*0.5]
map = Map(width, height, map_name)
drawer = TKDrawer(map.width, map.height)
drawer.draw_map(map)
drawer.draw_arrow(pose_start, fill='green', length=35, a_width=15, l_width=2)
drawer.draw_arrow(pose_end, fill='red', length=35, a_width=15, l_width=2)
rrt = DubinsRRT(map, pose_start, pose_end)
rrt_runner = RRTRunner(rrt, drawer)
rrt_runner.rrt_search(max_iteration=100000)
