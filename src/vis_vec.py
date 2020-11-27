#!/usr/bin/env python3

import matplotlib.pyplot as plt
import rospy
import numpy as np
from matplotlib.animation import FuncAnimation
from mpl_toolkits.mplot3d import Axes3D
from std_msgs.msg import Float64MultiArray, Float64

class Visualiser:
    def __init__(self):
        self.fig, self.ax = plt.subplots(subplot_kw=dict(projection="3d"))

        self.quiver = self.ax.quiver(0, 0, 0, 0, 0, 50)
        self.x = 100
        self.y = 100
        self.z = 100
        self.theta = 0

    def plot_init(self):
        self.ax.set_xlim3d(-100, 100)
        self.ax.set_ylim3d(-100, 100)
        self.ax.set_zlim3d(-100, 100)

        return self.quiver

    def callback(self, msg):
        self.x = msg.data[0]
        self.y = msg.data[1]
        self.z = msg.data[2]
    
    def theta_callback(self, msg):
        self.theta = msg.data
    
    def update_plot(self, frame):
        self.quiver.remove()
        # self.quiver = self.ax.quiver(0, 0, 0, self.x, self.y, self.z)
        self.quiver = self.ax.quiver([0, 0, 0], [0, 0, 0], [0, 0, 0], [self.x, 0, 0], [self.y, self.y, (30 * np.sin(self.theta))], [self.z, self.z, (30 * np.cos(self.theta))])
        return self.quiver

rospy.init_node('visualize_vec_node')
vis = Visualiser()
vec_sub = rospy.Subscriber('/vec_bg', Float64MultiArray, vis.callback)
com_sub = rospy.Subscriber('/robot/joint2_position_controller/command', Float64, vis.theta_callback)

ani = FuncAnimation(vis.fig, vis.update_plot, init_func=vis.plot_init, interval=40)
plt.show(block=True) 