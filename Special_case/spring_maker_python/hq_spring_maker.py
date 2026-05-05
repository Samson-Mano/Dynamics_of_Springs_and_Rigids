import numpy as np
import matplotlib.pyplot as plt


# 1. Increase the global font size
plt.rcParams.update({'font.size': 16}) 

# 2. Make the lines thicker so they don't disappear on the page
plt.rcParams.update({'lines.linewidth': 5.0})




def draw_spring(x_start, x_end, y_center=0, coils=10,
                         radius=1.5, aspect=0.3, end_ratio=0.0, points=2000):

    L = x_end - x_start
    t = np.linspace(0, 1, points)
    
    theta = 2 * np.pi * coils * t
    
    # Create envelope that only affects X amplitude at ends
    end_pts = int(points * end_ratio)
    envelope = np.ones(points)
    
    # ONLY dampen X oscillation at ends, not Y
    if end_pts > 0:
        # Linear ramp for X amplitude only
        ramp = np.linspace(0, 1, end_pts)
        envelope[:end_pts] = ramp
        envelope[-end_pts:] = ramp[::-1]
    
    # X with amplitude damping, Y with flat ends
    x = x_start + L * t + aspect * radius * envelope * np.cos(theta)
    y = y_center + radius * np.sin(theta)
    
    # Flatten Y at ends
    # y[:end_pts] = y_center
    # y[-end_pts:] = y_center
    
    return x, y




# Example usage
plt.figure(figsize=(10, 3))

# Draw spring
x, y = draw_spring(0, 10)
plt.plot(x, y, linewidth =6.0)

# # Add straight ends (important for realism)
# plt.plot([ -0.5, 0 ], [0, 0], linewidth=3)   # left connector
# plt.plot([10, 10.5], [0, 0], linewidth=3)   # right connector

# # Mass blocks
# plt.scatter([-0.5, 10.5], [0, 0], s=200)

plt.axis('equal')
plt.axis('off')
plt.savefig('spring_pic.png', dpi=300, bbox_inches='tight')
plt.show()