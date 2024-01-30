import numpy as np
import matplotlib.pyplot as plt
from scipy.special import jv
import math

def bessel_function_m(m, x):
    # Returns the value of bessel function at x of order m
    # Calculates using midpoint rule
    n = 32
    inv_n = 1.0 / float(n)
    j_n = 0.0

    neg_factor = 1.0
    if( (m - 2)%4 == 0):
        neg_factor = -1.0

    if(m%2 == 0):
        for k in range(0, n):  # sequence of decreasing values of h
            rad1 = (math.pi /(2.0*float(n))) *(k + 0.5)
            rad2 = ((math.pi * m)/(2.0*float(n))) *(k + 0.5)
            cos_val1 = math.cos(rad1)

            j_n = j_n + (neg_factor) * (math.cos(x*cos_val1) * math.cos(rad2))

    else:
        for k in range(0, n):  # sequence of decreasing values of h
            rad1 = (math.pi /(2.0*float(n))) *(k + 0.5)
            rad2 = ((math.pi * m)/(2.0*float(n))) *(k + 0.5)
            sin_val1 = math.sin(rad1)

            j_n = j_n + (math.sin(x*sin_val1) * math.sin(rad2))
       
    return inv_n * j_n

def bessel_function(m, x):
    return np.cos(m * x) - (1/m) * np.sin(m * x)

delta_x = 0.01
x_maximum = 100.0
a = np.arange(0.0, x_maximum + delta_x, delta_x)
order = 40
m = np.arange(order, order + 1, 1)

# Compute Bessel function values
bessel_values = np.zeros((len(m), len(a)))
lib_bessel_values = np.zeros((len(m), len(a)))

for i, n in enumerate(m):
    j =0
    soln = np.zeros(len(a))
    for x_val in a:
        soln[j] = bessel_function_m(n, x_val)
        j = j+1

    bessel_values[i] = soln
    lib_bessel_values[i] = jv(n,a)

# Plot the Bessel function values
for i, n in enumerate(m):
    plt.plot(a, bessel_values[i], label=f'm={n}')
    plt.plot(a, lib_bessel_values[i], label=f'm={n}')

plt.xlabel('a')
plt.ylabel('J(m, a)')
plt.title('Bessel Function')
plt.legend()
plt.grid(True)
plt.show()