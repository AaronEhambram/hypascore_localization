#!/usr/bin/python
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import xml.etree.ElementTree as ET


f = open("/home/ehambram/Data/Stadtmodell_Hannover_CityGML_LoD2/CityGML_LoD2_Nordstadt/nordstadt.map", "w")
building_id_counter = 0

input_files = []
input_files.append("/home/ehambram/Data/Stadtmodell_Hannover_CityGML_LoD2/CityGML_LoD2_Nordstadt/5480_5804.gml")
input_files.append("/home/ehambram/Data/Stadtmodell_Hannover_CityGML_LoD2/CityGML_LoD2_Nordstadt/5490_5804.gml")


for file in input_files:
  tree = ET.parse(file)
  root = tree.getroot()

  for objectMember in root.findall('{http://www.opengis.net/citygml/2.0}cityObjectMember'):
    #fig = plt.figure()
    #ax = Axes3D(fig)
    for bulding in objectMember.findall('{http://www.opengis.net/citygml/building/2.0}Building'):
      f.write("building_id:%i\n" %building_id_counter)
      building_id_counter = building_id_counter + 1
      roof_planes = []
      wall_planes = []
      ground_planes = []
      for boundedBy in bulding.findall('{http://www.opengis.net/citygml/building/2.0}boundedBy'):
        for roof_suf in boundedBy.findall('{http://www.opengis.net/citygml/building/2.0}RoofSurface'):
          poslist = roof_suf.getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0]
          x = [float(i) for i in poslist.text.split()]
          if len(x) > 0:
            i = 0
            surface = []
            while i < len(x): 
              point = [x[i],x[i+1],x[i+2]]
              i = i+3
              surface.append(point)
            roof_planes.append(surface)
        for ground_suf in boundedBy.findall('{http://www.opengis.net/citygml/building/2.0}GroundSurface'):
          poslist = ground_suf.getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0]
          x = [float(i) for i in poslist.text.split()]
          if len(x) > 0:
            i = 0
            surface = []
            while i < len(x): 
              point = [x[i],x[i+1],x[i+2]]
              i = i+3
              surface.append(point)
            ground_planes.append(surface)
        for wall_suf in boundedBy.findall('{http://www.opengis.net/citygml/building/2.0}WallSurface'):
          poslist = wall_suf.getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0].getchildren()[0]
          x = [float(i) for i in poslist.text.split()]
          if len(x) > 0:
            i = 0
            surface = []
            while i < len(x): 
              point = [x[i],x[i+1],x[i+2]]
              i = i+3
              surface.append(point)
            wall_planes.append(surface)

      xr = []
      yr = []
      zr = []
      xg = []
      yg = []
      zg = []
      xw = []
      yw = []
      zw = []
      f.write("roof:\n")
      roof_surface_counter = 0
      for surface in roof_planes:
        xr1 = []
        yr1 = []
        zr1 = []
        f.write("surface:%i\n" %roof_surface_counter)
        for point in surface:
          xr.append(point[0])
          yr.append(point[1])
          zr.append(point[2])
          xr1.append(point[0])
          yr1.append(point[1])
          zr1.append(point[2])
          f.write("%f %f %f\n" % (point[0],point[1],point[2]))
        roof_surface_counter = roof_surface_counter+1
        #ax.plot(xr1, yr1, zr1, color='red', linewidth=6)
      f.write("ground:\n")
      ground_surface_counter = 0
      for surface in ground_planes:
        xg1 = []
        yg1 = []
        zg1 = []
        f.write("surface:%i\n" %ground_surface_counter)
        for point in surface:
          xg.append(point[0])
          yg.append(point[1])
          zg.append(point[2])
          xg1.append(point[0])
          yg1.append(point[1])
          zg1.append(point[2])
          f.write("%f %f %f\n" % (point[0],point[1],point[2]))
        ground_surface_counter = ground_surface_counter+1
        #ax.plot(xg1, yg1, zg1, color='green', linewidth=6)
      f.write("wall:\n")
      wall_surface_counter = 0
      for surface in wall_planes:
        xw1 = []
        yw1 = []
        zw1 = []
        f.write("surface:%i\n" %wall_surface_counter)
        for point in surface:
          xw.append(point[0])
          yw.append(point[1])
          zw.append(point[2])
          xw1.append(point[0])
          yw1.append(point[1])
          zw1.append(point[2])
          f.write("%f %f %f\n" % (point[0],point[1],point[2]))
        wall_surface_counter = wall_surface_counter+1
        #ax.plot(xw1, yw1, zw1, color='blue')
      #ax.set_xlabel('X Label')
      #ax.set_ylabel('Y Label')
      #ax.set_zlabel('Z Label')
      #ax.scatter(xr, yr, zr, color='red', linewidth=6)
      #ax.scatter(xg, yg, zg, color='green', linewidth=6)
      #ax.scatter(xw, yw, zw, color='blue', linewidth=3)
    #plt.show()
    #plt.pause(1)
f.close()