#!/usr/bin/python
import numpy as np
import math
from overall_dict_creator import*

p1 = np.empty([2,1])
p2 = np.empty([2,1])
p3 = np.empty([2,1])
p4 = np.empty([2,1])

p1[0][0] = -375.41132335
p1[1][0] = 113.25454545
p2[0][0] = -375.41132335
p2[1][0] = 116.05176734
p3[0][0] = -366.06594084
p3[1][0] = 116.05176734
p4[0][0] = -366.06594084
p4[1][0] = 113.25454545

poly = []
poly.append(p1)
poly.append(p2)
poly.append(p3)
poly.append(p4)

p_test = np.empty([2,1])
p_test[0][0] = -371.20314895
p_test[1][0] = 113.24983955

print(point_in_polygon_test(poly,p_test))
