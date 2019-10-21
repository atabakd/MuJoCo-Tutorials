#!/usr/bin/env python2
# -*- coding: utf-8 -*-
"""
Created on Mon Aug 20 23:05:56 2018

@author: atabak

Modified from
https://github.com/paulgribble/CompNeuro/blob/master/code/twojointarm_passive.py
"""



from scipy.integrate import odeint
import numpy as np
from matplotlib import pyplot as plt

# two joint arm in a vertical plane, with gravity

# forward dynamics equations of our passive two-joint arm
def twojointarm(state, t, aparams):
  """
  passive two-joint arm in a vertical plane
  X is fwd(+) and back(-)
  Y is up(+) and down(-)
  gravity acts down
  shoulder angle a1 relative to Y vert, +ve counter-clockwise
  elbow angle a2 relative to upper arm, +ve counter-clockwise
  """
  a1, a2, a1d, a2d = state
  l1, l2 = aparams['l1'], aparams['l2']
  m1, m2 = aparams['m1'], aparams['m2']
  i1, i2 = aparams['i1'], aparams['i2']
  r1, r2 = aparams['r1'], aparams['r2']
  g = 9.81
  M11 = i1 + i2 + (m1*r1*r1) + (m2*((l1*l1) + (r2*r2) + (2*l1*r2*np.cos(a2))))
  M12 = i2 + (m2*((r2*r2) + (l1*r2*np.cos(a2))))
  M21 = M12
  M22 = i2 + (m2*r2*r2)
  M = np.matrix([[M11, M12], [M21, M22]])
  C1 = -(m2*l1*a2d*a2d*r2*np.sin(a2)) - (2*m2*l1*a1d*a2d*r2*np.sin(a2))
  C2 = m2*l1*a1d*a1d*r2*np.sin(a2)
  C = np.matrix([[C1], [C2]])
#  G1 = (g*sin(a1)*((m2*l1)+(m1*r1))) + (g*m2*r2*sin(a1+a2))
#  G2 = g*m2*r2*sin(a1+a2)
  G1 = (g*np.cos(a1)*((m2*l1)+(m1*r1))) + (g*m2*r2*np.cos(a1+a2))
  G2 = g*m2*r2*np.cos(a1+a2)
  G = np.matrix([[G1], [G2]])
  u = 242.52*(a1-90*np.pi/180) + 96.33*a2 + 104.59*a1d + 49.05*a2d #  Coefficients comming from Spong
  U = np.matrix([[0.], [u]])
  ACC = np.linalg.inv(M) * (-C-G+U)
  a1dd, a2dd = ACC[0, 0], ACC[1, 0]
  return [a1d, a2d, a1dd, a2dd]

# anthropometric parameters of the arm
aparams = {
#  'l1' : 0.3384, # metres
#  'l2' : 0.4554,
#  'r1' : 0.1692,
#  'r2' : 0.2277,
#  'm1' : 2.10,   # kg
#  'm2' : 1.65,
#  'i1' : 0.025,  # kg*m*m
#  'i2' : 0.075
# Parameters from Spong
  'l1' : 1, # metres
  'l2' : 2,
  'r1' : 0.5,
  'r2' : 1,
  'm1' : 1,   # kg
  'm2' : 1,
  'i1' : 0.083,  # kg*m*m
  'i2' : 0.333
}

# forward kinematics
def joints_to_hand(A, aparams):
  """
  Given joint angles A=(a1, a2) and anthropometric params aparams,
  returns hand position H=(hx, hy) and elbow position E=(ex, ey)
  """
  l1 = aparams['l1']
  l2 = aparams['l2']
  n = np.shape(A)[0]
  E = np.zeros((n, 2))
  H = np.zeros((n, 2))
  for i in range(n):
    E[i, 0] = l1 * np.cos(A[i, 0])
    E[i, 1] = l1 * np.sin(A[i, 0])
    H[i, 0] = E[i, 0] + (l2 * np.cos(A[i, 0]+A[i, 1]))
    H[i, 1] = E[i, 1] + (l2 * np.sin(A[i, 0]+A[i, 1]))
  return H, E

def animatearm(state, t, aparams, step=3):
  """
  animate the twojointarm
  """
  A = state[:, [0, 1]]
  H, E = joints_to_hand(A, aparams)
  l1, l2 = aparams['l1'], aparams['l2']
  plt.figure()
  plt.plot(0, 0, 'b.')
  p1, = plt.plot(E[0, 0], E[0, 1], 'b.')
  p2, = plt.plot(H[0, 0], H[0, 1], 'b.')
  p3, = plt.plot((0, E[0, 0], H[0, 0]), (0, E[0, 1], H[0, 1]), 'b-')
  plt.xlim([-l1-l2, l1+l2])
  plt.ylim([-l1-l2, l1+l2])
  dt = t[1]-t[0]
  tt = plt.title("Click on this plot to continue...")
  plt.ginput(1)
  for i in np.arange(0, np.shape(state)[0]-step, step):
    p1.set_xdata((E[i, 0]))
    p1.set_ydata((E[i, 1]))
    p2.set_xdata((H[i, 0]))
    p2.set_ydata((H[i, 1]))
    p3.set_xdata((0, E[i, 0], H[i, 0]))
    p3.set_ydata((0, E[i, 1], H[i, 1]))
    tt.set_text("%4.2f sec" % (i*dt))
    plt.draw()
    plt.ginput(1)


state0 = [93*np.pi/180, -3*np.pi/180, 0, 0] # initial joint angles and vels
t = np.arange(2001.)/200                # 10 seconds at 200 Hz
state = odeint(twojointarm, state0, t, args=(aparams,))

animatearm(state, t, aparams)
