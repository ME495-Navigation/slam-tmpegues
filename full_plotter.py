import numpy as np
import matplotlib.pyplot as plt
import modern_robotics as mr

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


fig1, ((ax1, ax2, ax3), (ax4, ax5, ax6), (ax7, ax8, ax9), (ax10, ax11, ax12)) = plt.subplots(4, 3)
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
radians_per_step = radians_per_sec*time_diff

x_cent = x0
y_cent = y0 + ((-phidotl * track - phidotr * track)/(2*(phidotl-phidotr)))

correct_x = []
correct_y = []
maybe_correct_theta = []
for i in range(len(x)+1):
    i+=1
    correct_x.append(x_cent + rad*np.cos(time_diff*(i)*radians_per_sec-np.pi/2))
    correct_y.append(y_cent + rad*np.sin(time_diff*(i)*radians_per_sec-np.pi/2))
    maybe_correct_theta.append((i) * radians_per_step)

correct_theta = []
for thetas in maybe_correct_theta:
    if thetas < 0:
        thetas -= np.pi
        thetas %= np.pi * 2
        thetas += np.pi
    else:
        thetas += np.pi
        thetas %= np.pi * 2
        thetas -= np.pi
    correct_theta.append(thetas)


x_err = [cx - actx for cx, actx in zip(correct_x, x)]
y_err = [cy - acty for cy, acty in zip(correct_y, y)]
theta_err = [cth - actth for cth, actth in zip(correct_theta, theta)]


# Calculate twist errors, assuming that the body twists are correct

def mrR(th):
    return [[np.cos(th), -np.sin(th),0],[np.sin(th),np.cos(th),0],[0,0,1]]

body_twist = np.array([0,0, b_om[0], b_x[0], b_y[0], 0])
# body_twist_mat = mr.VecTose3(body_twist)
# body_tf = mr.MatrixExp6(body_twist_mat)

correct_twist = []
for cx, cy, cth in zip(correct_x, correct_y, correct_theta):
    Twb = mr.RpToTrans(mrR(cth),(cx, cy, 0))
    # print(mr.Adjoint(Twb))
    world_twist = np.matmul(mr.Adjoint(Twb) ,body_twist)
    correct_twist.append([world_twist[2], world_twist[3], world_twist[4]])


tw_w_error = [cw[0]- actw[0] for cw, actw in zip(correct_twist, world_twist_lines)]
tw_x_error = [cw[1]- actw[1] for cw, actw in zip(correct_twist, world_twist_lines)]
tw_y_error = [cw[2]- actw[2] for cw, actw in zip(correct_twist, world_twist_lines)]




fig2, ((erax1, erax2, erax3), (erax4, erax5, erax6)) = plt.subplots(2, 3)


erax1.plot(theta_err)
erax1.set_title("T_wb.theta error")
erax2.plot(x_err)
erax2.set_title("T_wb.x error")
erax3.plot(y_err)
erax3.set_title("T_wb.y error")

erax4.plot(tw_w_error)
erax4.set_title("World twist omega error")
erax5.plot(tw_x_error)
erax5.set_title("World twist x error")
erax6.plot(tw_y_error)
erax6.set_title("World twist y error")




plt.show()
