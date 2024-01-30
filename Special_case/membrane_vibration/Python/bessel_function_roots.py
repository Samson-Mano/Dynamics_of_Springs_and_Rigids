from scipy.special import jn_zeros


def bessel_function_roots(m, n):
    # Returns the first n roots of Bessel function of order m
    return jn_zeros(m, n)

# Example usage:
order = 3
num_roots = 8
roots = bessel_function_roots(order, num_roots)
print("Roots of Bessel function of order", order, ":", roots)