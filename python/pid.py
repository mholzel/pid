import matplotlib.pyplot as plt
from math import ceil, cos, floor, sin, sqrt


# The system that we want to control
class System:

    def __init__(self, w, x2, x1):
        self.w, self.x2, self.x1 = w, x2, x1

    def step(self, u):
        self.x2, self.x1 = 2 * cos(self.w) * self.x2 - self.x1 + u, self.x2
        return self.x2


class PID:

    def __init__(self, kp, ki, kd):
        self.kp, self.ki, self.kd = kp, ki, kd
        self.previous = 0.0
        self.integral = 0.0

    def update(self, error, dt):
        derivative = (error - self.previous) / dt
        self.previous = error
        self.integral += error * dt
        return -self.kp * error - self.ki * self.integral - self.kd * derivative


def plot(t, y, ax=plt, title=''):
    ax.plot(t, y)
    if ax != plt:
        ax.set_xlabel("time")
        ax.set_title(title)
    else:
        ax.xlabel("time")
        plt.ylabel("output")
        ax.title(title)
        plt.show()


# Create the system
w, x2, x1 = .1, 1, 1
system = System(w, x2, x1)

# Simulate the system with zero inputs
N = 100
dt = .1
t = [dt * i for i in range(N)]
y = [system.step(0) for ti in t]

plot(t, y)

tests = 3

# Simulate the system with various prop gains
k_p = [.1 * i for i in range(1, tests + 1)]
rows = int(floor(sqrt(len(k_p))))
cols = int(ceil(len(k_p) / rows))

fig, axes = plt.subplots(rows, cols)
axes = axes.ravel()
for ax, kp in zip(axes, k_p):
    output = [0.0]
    pid = PID(kp, 0.0, 0.0)
    system = System(w, x2, x1)
    for ti in t:
        error = output[len(output) - 1]
        u = pid.update(error, dt)
        output.append(system.step(u))
    output.pop(0)
    plot(t, output, ax, 'k_p = ' + str(kp))
plt.show()

# Simulate the system with various integral gains
k_i = [.01 * i for i in range(1, tests + 1)]
rows = int(floor(sqrt(len(k_i))))
cols = int(ceil(len(k_i) / rows))

fig, axes = plt.subplots(rows, cols)
axes = axes.ravel()
for ax, ki in zip(axes, k_i):
    output = [0.0]
    pid = PID(0.0, ki, 0.0)
    system = System(w, x2, x1)
    for ti in t:
        error = output[len(output) - 1]
        u = pid.update(error, dt)
        output.append(system.step(u))
    output.pop(0)
    plot(t, output, ax, 'k_i ' + str(ki))
plt.show()

# Simulate the system with various derivative gains
k_d = [.01 * i for i in range(1, tests + 1)]
rows = int(floor(sqrt(len(k_d))))
cols = int(ceil(len(k_d) / rows))

fig, axes = plt.subplots(rows, cols)
axes = axes.ravel()
for ax, kd in zip(axes, k_d):
    output = [0.0]
    pid = PID(0.0, 0.0, kd)
    system = System(w, x2, x1)
    for ti in t:
        error = output[len(output) - 1]
        print(error, dt)
        u = pid.update(error, dt)
        output.append(system.step(u))
    output.pop(0)
    plot(t, output, ax, 'k_d ' + str(kd))
plt.show()

# Simulate the system with various gains to show that all of
# these gains can lead to instability in the discrete-time world
fig, axes = plt.subplots(1, 3)
axes = axes.ravel()
gains = [[10, 0, 0], [0, 1, 0], [0, 0, 1]]
for ax, g in zip(axes, gains):
    output = [0.0]
    pid = PID(g[0], g[1], g[2])
    system = System(w, x2, x1)
    for ti in t:
        error = output[len(output) - 1]
        print(error, dt)
        u = pid.update(error, dt)
        output.append(system.step(u))
    output.pop(0)
    plot(t, output, ax, '[ kp, ki, kd ] = ' + str(g))
plt.show()
