#!/usr/bin/python
import sys
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import xml.etree.ElementTree as ET
import utm

f = open("/home/ehambram/Data/KITTI/0034/0034_map.map", "w")
building_id_counter = 0

input_files = []
input_files.append("/home/ehambram/Data/KITTI/0034/0034_map.osm")

nodes = dict()
building_id_counter = 0
ground_level = 54.5

for file in input_files:
  tree = ET.parse(file)
  root = tree.getroot()

  # save all the nodes (the points that belong to the buildings)
  for objectMember in root.findall('node'):
    utm_coord = utm.from_latlon(float(objectMember.attrib['lat']),float(objectMember.attrib['lon']))
    nodes[objectMember.attrib['id']] = (utm_coord[0],utm_coord[1])

  # get all the ways, that belong to buildings
  for objectMember in root.findall('way'):
    is_building = False
    for tag in objectMember.findall('tag'):
      if tag.attrib['k'] == 'building':
        is_building = True
        break
    if is_building:
      f.write("building_id:%i\n" %building_id_counter)
      building_id_counter = building_id_counter + 1

      # create roof surface
      f.write("roof:\n")
      roof_surface_counter = 0
      f.write("surface:%i\n" %roof_surface_counter)
      height = 5
      for tag in objectMember.findall('tag'):
        if tag.attrib['k'] == 'height':
          s = tag.attrib['v']
          if(s.replace('.','',1).isdigit() and float(tag.attrib['v']) > 0):
            height = float(tag.attrib['v'])
          break
      for nd in objectMember.findall('nd'):
        utm_coord = nodes[nd.attrib['ref']]
        f.write("%f %f %f\n" % (utm_coord[0],utm_coord[1],ground_level+height))
      roof_surface_counter = roof_surface_counter+1

      # create ground surface
      f.write("ground:\n")
      ground_surface_counter = 0
      f.write("surface:%i\n" %ground_surface_counter)
      for nd in objectMember.findall('nd'):
        utm_coord = nodes[nd.attrib['ref']]
        f.write("%f %f %f\n" % (utm_coord[0],utm_coord[1],ground_level))
      ground_surface_counter = ground_surface_counter+1

      # create wall surfaces
      f.write("wall:\n")
      wall_surface_counter = 0
      for i in range(1,len(objectMember.findall('nd'))):
        f.write("surface:%i\n" %wall_surface_counter)
        ref_before = objectMember.findall('nd')[i-1].attrib['ref']
        ref_cur = objectMember.findall('nd')[i].attrib['ref']
        utm_coord_bef = nodes[ref_before]
        utm_coord_cur = nodes[ref_cur]
        f.write("%f %f %f\n" % (utm_coord_bef[0],utm_coord_bef[1],ground_level))
        f.write("%f %f %f\n" % (utm_coord_cur[0],utm_coord_cur[1],ground_level))
        f.write("%f %f %f\n" % (utm_coord_cur[0],utm_coord_cur[1],ground_level+height))
        f.write("%f %f %f\n" % (utm_coord_bef[0],utm_coord_bef[1],ground_level+height))
        f.write("%f %f %f\n" % (utm_coord_bef[0],utm_coord_bef[1],ground_level))
        wall_surface_counter = wall_surface_counter+1

f.close()



