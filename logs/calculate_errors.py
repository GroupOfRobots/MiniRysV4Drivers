#!/usr/bin/python3
import math
# import numpy
import matplotlib.pyplot as plt
plt.style.use('seaborn-whitegrid')

L = 0.2
b = 0.127

file = open("UMBmark_output_pre")
# file = open("UMBmark_output_post")
# file = open("UMBmark_output_post2")
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
file.close()

n = int(len(globalData_x))
n2 = int(n/2)

ex = [globalData_x[i] - odometryData_x[i] for i in range(n)]
ey = [globalData_y[i] - odometryData_y[i] for i in range(n)]

x_cg_CW = sum(ex[0:n2])/n2
x_cg_CCW = sum(ex[n2:n])/n2
y_cg_CW = sum(ey[0:n2])/n2
y_cg_CCW = sum(ey[n2:n])/n2

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
print("Center of gravity:\n\tCW:\t" + str(r_cg_CW) + "\n\tCCW:\t" + str(r_cg_CCW))
E = max([r_cg_CW, r_cg_CCW])
print("Measure of odometric accuracy for systematic errors: " + str(E))

fig = plt.figure(figsize=(6, 6))
ax = fig.add_subplot(111)
# print data
ax.plot([ex[0:5]], [ey[0:5]], 'c.', label = r'$(\epsilon_x, \epsilon_y)_{CW}$')
ax.plot([x_cg_CW], [y_cg_CW], 'bo', label = r'$r_{c.g.,CW}$')
ax.plot([ex[5:10]], [ey[5:10]], 'm.', label = r'$(\epsilon_x, \epsilon_y)_{CCW}$')
ax.plot([x_cg_CCW], [y_cg_CCW], 'ro', label = r'$r_{c.g.,CCW}$')

# maksymalna odległość od osi
m = 1.2*max([abs(x) for x in ex+ey])
# szerokość osi
aw = m/400
# szerokość głowy strzałki
hw = m/40
# odległość napisu od osi
td = m/40

# osie x i y
ax.arrow(-m, 0, 2*m, 0, length_includes_head = True, width = aw, head_width = hw, head_length = hw, color='k')
ax.arrow(0, -m, 0, 2*m, length_includes_head = True, width = aw, head_width = hw, head_length = hw, color='k')

# CW
# ax.arrow(x_cg_CW, y_cg_CW, 0, -y_cg_CW, length_includes_head = True, width = aw, head_width = hw, head_length = hw, color='b', label = 'x c.g.,CW')
# plt.text(x_cg_CW+td, y_cg_CW/2, 'x')
# ax.arrow(x_cg_CW, y_cg_CW, -x_cg_CW, 0, length_includes_head = True, width = aw, head_width = hw, head_length = hw, color='b', label = 'y c.g.,CW')
# plt.text(x_cg_CW/2, y_cg_CW+td, 'y')
ax.arrow(x_cg_CW, y_cg_CW, -x_cg_CW, -y_cg_CW, length_includes_head = True, width = aw, head_width = 2*hw, head_length = hw, color='b')
# plt.text(x_cg_CW/2+numpy.sign([x_cg_CW])[0]*td, y_cg_CW/2-numpy.sign([y_cg_CW])[0]*td, 'r CW')

# CCS
# ax.arrow(x_cg_CCW, y_cg_CCW, 0, -y_cg_CCW, length_includes_head = True, width = aw, head_width = hw, head_length = hw, color='r', label = 'x c.g.,CCW')
# plt.text(x_cg_CCW+td, y_cg_CCW/2, 'x')
# ax.arrow(x_cg_CCW, y_cg_CCW, -x_cg_CCW, 0, length_includes_head = True, width = aw, head_width = hw, head_length = hw, color='r', label = 'y c.g.,CCW')
# plt.text(x_cg_CCW/2, y_cg_CCW+td, 'y')
ax.arrow(x_cg_CCW, y_cg_CCW, -x_cg_CCW, -y_cg_CCW, length_includes_head = True, width = aw, head_width = 2*hw, head_length = hw, color='r')
# plt.text(x_cg_CCW/2+numpy.sign([x_cg_CCW])[0]*td, y_cg_CCW/2-numpy.sign([y_cg_CCW])[0]*td, 'r CCW')

plt.axis([-m, m, -m, m])
plt.xlabel('$\epsilon_x[m]$')
plt.ylabel('$\epsilon_y[m]$')
ax.legend()
plt.show()
