#!/usr/bin/python3
import math

L = 0.2
b = 0.127

file = open("UMBmark_output_post2")
odometryData_x = []
odometryData_y = []
globalData_x = []
globalData_y = []
for line in file:
	temp = line.split(" ")
	globalData_x.append(float(temp[0]))
	globalData_y.append(float(temp[1]))
	odometryData_x.append(float(temp[3]))
	odometryData_y.append(float(temp[4]))

ex = [globalData_x[i] - odometryData_x[i] for i in range(10)]
ey = [globalData_y[i] - odometryData_y[i] for i in range(10)]

x_cg_CW = sum(ex[0:4])/5
x_cg_CCW = sum(ex[5:9])/5
y_cg_CW = sum(ey[0:4])/5
y_cg_CCW = sum(ey[5:9])/5

print("Average errors(x CW, x CCW, y CW, y CCW):\n" + "\n".join(str(i) for i in [x_cg_CW, x_cg_CCW, y_cg_CW, y_cg_CCW]))

beta = (x_cg_CW - x_cg_CCW)/(-4*L)
R = L/2/math.sin(beta/2)
Ed_x = (R+b/2)/(R-b/2)

alpha = (x_cg_CW + x_cg_CCW)/(-4*L)
Eb_x = 0.5*math.pi/(0.5*math.pi - alpha)

beta = (y_cg_CW + y_cg_CCW)/(-4*L)
R = L/2/math.sin(beta/2)
Ed_y = (R+b/2)/(R-b/2)

alpha = (y_cg_CW - y_cg_CCW)/(-4*L)
Eb_y = 0.5*math.pi/(0.5*math.pi - alpha)

print("From x:\nE_d:\t" + str(Ed_x) + "\nE_b:\t" + str(Eb_x))
print("From y:\nE_d:\t" + str(Ed_y) + "\nE_b:\t" + str(Eb_y))
print("Average:\nE_d:\t" + str((Ed_x+Ed_y)/2) + "\nE_b:\t" + str((Eb_x+Eb_y)/2))

r_cg_CW = math.sqrt(x_cg_CW**2 + y_cg_CW**2)
r_cg_CCW = math.sqrt(x_cg_CCW**2 + y_cg_CCW**2)
E = max([r_cg_CW, r_cg_CCW])
print("Measure of odometric accuracy for systematic errors: " + str(E))

# pre
# Average errors(x CW, x CCW, y CW, y CCW):
# -0.010718976
# 0.0061535692
# -0.015236762639999998
# -0.00534327936
# From x:
# E_d:	1.013482616389831
# E_b:	1.0036462823697798
# From y:
# E_d:	1.0164694719728073
# E_b:	1.007935459587154
# Average:
# E_d:	1.0149760441813191
# E_b:	1.0057908709784669
# Measure of odometric accuracy for systematic errors: 0.01862942248799666

# post
# Average errors(x CW, x CCW, y CW, y CCW):
# -0.0017289499999999995
# 0.007386696
# -0.0011294320000000003
# -0.005320512
# From x:
# E_d:	1.0072617761779428
# E_b:	0.9955178885576719
# From y:
# E_d:	1.0051327681184452
# E_b:	0.9966759307932112
# Average:
# E_d:	1.006197272148194
# E_b:	0.9960969096754415
# Measure of odometric accuracy for systematic errors: 0.009103357937517343

# post 2
# Average errors(x CW, x CCW, y CW, y CCW):
# 0.00294127
# -0.0008492360000000001
# -0.0015880240000000008
# 0.0026605060000000004
# From x:
# E_d:	0.9969958080500564
# E_b:	0.9983379791508867
# From y:
# E_d:	0.9991490796630533
# E_b:	1.003392341827922
# Average:
# E_d:	0.9980724438565549
# E_b:	1.0008651604894043
# Measure of odometric accuracy for systematic errors: 0.00334258723707789
