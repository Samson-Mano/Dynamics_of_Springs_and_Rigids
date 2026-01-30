import numpy as np
from scipy.linalg import eigh

class NDofSystem:
    def __init__(self, masses, stiffnesses, damp_z):
        """
        General N-DOF system (Chain of masses).
        masses: array-like of length N [m1, m2, ..., mN]
        stiffnesses: array-like of length N+1 [k1, k2, ..., k_N+1] 
        where k1 is ground-to-m1 and k_N+1 is mN-to-ground.
        damp_z: Modal damping ratio (float)
        """
        self.M_vec = np.array(masses)
        self.K_vec = np.array(stiffnesses)
        self.n = len(self.M_vec)
        self.damp_z = damp_z

        # 1. Create N x N Mass Matrix
        self.M = np.diag(self.M_vec)

        # 2. Create N x N Stiffness Matrix (Tridiagonal)
        self.K = np.zeros((self.n, self.n))
        for i in range(self.n):
            # Diagonal elements
            self.K[i, i] = self.K_vec[i] + self.K_vec[i+1]
            # Off-diagonal elements (coupling)
            if i > 0:
                self.K[i, i-1] = -self.K_vec[i]
            if i < self.n - 1:
                self.K[i, i+1] = -self.K_vec[i+1]

        self.perform_eigen_analysis()
        self.perform_modal_superposition()

    def perform_eigen_analysis(self):
        eigenvalues, eigenvectors = eigh(self.K, self.M)
        self.omega_n = np.sqrt(eigenvalues) 
        self.freq_hz = self.omega_n / (2 * np.pi) 
        self.Phi = eigenvectors     # Mass-normalized eigenvectors

    def perform_modal_superposition(self):
        # With mass-normalized Phi, these become simple diagonal matrices
        self.modal_mass = np.ones(self.n) 
        self.modal_stiff = self.omega_n**2
        self.modal_damp = 2 * self.damp_z * self.omega_n

        # Physical Damping Matrix (C) for reference
        self.C = self.M @ self.Phi @ np.diag(self.modal_damp) @ self.Phi.T @ self.M

    def get_shm_solution_at_t(self, time_t, omega_n, i_displ, i_velo):
        """Analytical solution for a damped SDOF modal coordinate."""
        # Handle zero frequency if rigid body mode exists
        if omega_n < 1e-9:
            return i_displ + i_velo * time_t, i_velo, 0

        omega_d = omega_n * np.sqrt(1.0 - self.damp_z**2)
        alpha = -self.damp_z * omega_n

        A = i_displ
        B = (i_velo - alpha * i_displ) / omega_d

        exp_t = np.exp(alpha * time_t)
        cos_t = np.cos(omega_d * time_t)
        sin_t = np.sin(omega_d * time_t)

        q = exp_t * (A * cos_t + B * sin_t)
        dq = (alpha * q) + exp_t * (omega_d * (-A * sin_t + B * cos_t))
        d2q = -2.0 * self.damp_z * omega_n * dq - (omega_n**2 * q)

        return q, dq, d2q

    def solve(self, t_span, x0, v0):
        """
        x0, v0: initial state vectors of length N
        """
        # Transform to Modal Initial Conditions
        modal_x0 = self.Phi.T @ self.M @ x0
        modal_v0 = self.Phi.T @ self.M @ v0

        # Pre-allocate output arrays (Time x Nodes)
        displ = np.zeros((len(t_span), self.n))
        velo = np.zeros((len(t_span), self.n))
        accl = np.zeros((len(t_span), self.n))

        # Vectorized modal calculation for each time step
        for j, t in enumerate(t_span):
            q_vec = np.zeros(self.n)
            dq_vec = np.zeros(self.n)
            d2q_vec = np.zeros(self.n)

            for i in range(self.n):
                q, dq, d2q = self.get_shm_solution_at_t(
                    t, self.omega_n[i], modal_x0[i], modal_v0[i]
                    )
                q_vec[i], dq_vec[i], d2q_vec[i] = q, dq, d2q

            # Project back to Physical Coordinates
            displ[j, :] = self.Phi @ q_vec
            velo[j, :] = self.Phi @ dq_vec
            accl[j, :] = self.Phi @ d2q_vec

        return displ, velo, accl

