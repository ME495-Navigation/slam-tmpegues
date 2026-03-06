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

ax5.plot(x)
ax5.set_title("T_wb.x")
ax6.plot(y)
ax6.set_title("T_wb.y")
ax4.plot(theta)
ax4.set_title("T_wb.theta")

ax8.plot(b_x)
ax8.set_title("Body Twist.x")
ax9.plot(b_y)
ax9.set_title("Body Twist.y")
ax7.plot(b_om)
ax7.set_title("Body Twist.omega")

ax11.plot(w_x)
ax11.set_title("World Twist.x")
ax12.plot(w_y)
ax12.set_title("World Twist.y")
ax10.plot(w_om)
ax10.set_title("World Twist.omega")
plt.draw()

# Calculate errors
tester_values = []
with open('./src/repo/turtlelib/tester.txt') as file:
    tester_lines =  [line.strip() for line in file]
tester_lines = [eval(line) for line in tester_lines[4:]]
phidotl, phidotr = tester_lines.pop(0)
x0, y0 = tester_lines.pop(0)
track, wheel_rad, cmdperradsec = tester_lines.pop(0)
time_diff = tester_lines.pop(0)

vel = .5 * cmdperradsec * (phidotl-phidotr) * wheel_rad
rad = (-phidotl*track -phidotr*track)/(2*(phidotl-phidotr))

loop_time = (2*np.pi*(-phidotl*track-phidotr*track))/(cmdperradsec*(phidotl-phidotr)*(phidotl+phidotr)*wheel_rad)
radians_per_sec = 2*np.pi/loop_time

x_cent = x0
y_cent = y0 + ((-phidotl * track - phidotr * track)/(2*(phidotl-phidotr)))

correct_x = []
correct_y = []
for i in range(len(x)+1):
    correct_x.append(x_cent + rad*np.cos(time_diff*(i+1)*radians_per_sec-np.pi/2))
    correct_y.append(y_cent + rad*np.sin(time_diff*(i+1)*radians_per_sec-np.pi/2))



x_err = [cx - actx for cx, actx in zip(correct_x, x)]
y_err = [cy - acty for cy, acty in zip(correct_y, y)]


ax2.plot(x_err)
ax2.set_title("T_wb.x error")
ax3.plot(y_err)
ax3.set_title("T_wb.y error")


plt.show()
