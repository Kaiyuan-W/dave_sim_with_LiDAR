import numpy as np
import math

# Parameters for straight line navigation
duration = 300  # seconds - reduced for testing
sample_time = 0.05  # seconds
v = 0.8  # Reduced velocity for stability

x0 = 10
y0 = 20
z0 = -95

# Define time t
t = np.arange(0, duration, sample_time)
print('t:', t)
print('len(t):', len(t))

traj = np.zeros((int(duration/sample_time+1), 16)) # x y z phi theta psi u v w p q r u1 u2 u3 u4

# Set the initial conditions
traj[:, 0] = x0
traj[:, 1] = y0
traj[:, 2] = z0

# Set the initial orientation
traj[:, 3] = 0
traj[:, 4] = 0
traj[:, 5] = -np.pi  # Face negative x direction initially

# Set the initial velocity
traj[:, 6] = 0
traj[:, 7] = 0
traj[:, 8] = 0

# Set the initial angular velocity
traj[:, 9] = 0
traj[:, 10] = 0
traj[:, 11] = 0

# Set the initial control inputs
traj[:, 12] = 0
traj[:, 13] = 0
traj[:, 14] = 0
traj[:, 15] = 0

# Simple straight line trajectory: move forward along x-axis
for i in range(1, len(t)):
    # Move forward at constant velocity
    traj[i, 0] = x0 + v * t[i]  # Move forward along x-axis
    traj[i, 1] = y0              # Stay on y-axis (straight line)
    traj[i, 2] = z0              # Maintain depth
    traj[i, 5] = -np.pi          # Keep heading straight
    
    # Set velocity to match trajectory
    traj[i, 6] = v               # Forward velocity
    traj[i, 7] = 0               # No lateral velocity
    traj[i, 8] = 0               # No vertical velocity

# Save the trajectory to a txt file
np.savetxt('slam_pool_side.txt', traj, fmt='%f')
print("Straight line trajectory generated successfully!")
print(f"Start: ({x0}, {y0}, {z0})")
print(f"End: ({x0 + v * duration}, {y0}, {z0})")
print(f"Distance: {v * duration} meters")
print(f"Duration: {duration} seconds")

