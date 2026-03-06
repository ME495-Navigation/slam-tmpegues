import numpy as np
import matplotlib.pyplot as plt


twb_lines = []
with open('./src/repo/turtlelib/twb.txt') as file:
    twb_lines =  [eval(line.strip()) for line in file]

xy, theta = zip(*twb_lines)
x, y = zip(*xy)


world_twist_lines = []
with open('./src/repo/turtlelib/world_twist.txt') as file:
    world_twist_lines =  [eval(line.strip("<")[:-2]) for line in file]

w_om = []
w_x  = []
w_y  = []

for point in world_twist_lines:
    w_om.append(point[0])
    w_x.append(point[1])
    w_y.append(point[2])


body_twist_lines = []
with open('./src/repo/turtlelib/body_twist.txt') as file:
    body_twist_lines =  [eval(line.strip("<")[:-2]) for line in file]

b_om = []
b_x  = []
b_y  = []

for point in body_twist_lines:
    b_om.append(point[0])
    b_x.append(point[1])
    b_y.append(point[2])


fig, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9), (ax10, ax11, ax12)) = plt.subplots(4, 3)
ax1.scatter(x, y)
ax1.set_aspect('equal', 'box')
ax1.set_title('T_wb, dd coordinates in world frame')

ax4.plot(x)
ax4.set_title("T_wb.x")
ax5.plot(y)
ax5.set_title("T_wb.y")
ax6.plot(theta)
ax6.set_title("T_wb.theta")

ax7.plot(b_x)
ax7.set_title("Body Twist.x")
ax8.plot(b_y)
ax8.set_title("Body Twist.y")
ax9.plot(b_om)
ax9.set_title("Body Twist.omega")

ax10.plot(w_x)
ax10.set_title("World Twist.x")
ax11.plot(w_y)
ax11.set_title("World Twist.y")
ax12.plot(w_om)
ax12.set_title("World Twist.omega")
plt.draw()
print("done")
plt.show()
