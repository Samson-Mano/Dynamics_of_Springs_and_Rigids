import numpy as np
import matplotlib.pyplot as plt
from scipy.special import jv

def bessel_function(m, x):
    return np.cos(m * x) - (1/m) * np.sin(m * x)

delta_x = 0.01
x_maximum = 10.0
a = np.arange(0.0, x_maximum + delta_x, delta_x)
m = np.arange(0, 2, 1)

# Compute Bessel function values
bessel_values = np.zeros((len(m), len(a)))
for i, n in enumerate(m):
    bessel_values[i] = jv(n, a)

# Plot the Bessel function values
for i, n in enumerate(m):
    plt.plot(a, bessel_values[i], label=f'm={n}')

plt.xlabel('a')
plt.ylabel('J(m, a)')
plt.title('Bessel Function')
plt.legend()
plt.grid(True)
plt.show()