#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Graph Utilities for plotting longitude and latitude of flight results
"""
from pymavlink import mavutil
import math
import re
import numpy as np
import matplotlib.pyplot as plt


class Location:
    def __init__(self,lat=0.0,lon=0.0):
        """ Create a new point at the origin """
        self.lat = lat
        self.lon = lon

class CoordinateLogger:
    def __init__(self):
        self.lat_array = []
        self.lon_array = []

    def add_data(self,latitude,longitude):
        """

        :rtype: object
        """
        self.lat_array.append([])
        self.lon_array.append([])
        self.lat_array[-1].append(latitude)
        self.lon_array[-1].append(longitude)


##############################################################################################
# Provides Graph Plotting functionality
##############################################################################################
class GraphPlotter:
    def __init__(self,lat1_array,lon1_array,lat2_array,lon2_array,xlabel="",ylabel="",title=""):
        self.lat1_array=lat1_array
        self.lon1_array=lon1_array
        self.lat2_array=lat2_array
        self.lon2_array=lon2_array
        self.xlegend = xlabel
        self.ylegend=ylabel
        self.title=title
	self.markers = []

    def add_marker(self,markerlat,markerlon,color):
	self.markers.append([markerlat, markerlon,color])

    def scatter_plot(self):
        plt.plot(self.lat1_array,self.lon1_array,linewidth=10,color='gray')
        plt.plot(self.lat2_array,self.lon2_array,linewidth=4,color='blue')
        plt.xlabel(self.xlegend)
        plt.ylabel(self.ylegend)
        plt.title(self.title)
	for mark in self.markers:
	    plt.plot([mark[0]], [mark[1]], "{}o".format(mark[2]), markersize=42)

        plt.show()

