import numpy as np
import multi_dof_system_soln as multi_dof_system_soln
import matplotlib.pyplot as plt
from matplotlib.collections import LineCollection



def plot_phase_value(t_span, displ_data, velo_data, accl_data, target_nodes):
    """
    Plots phase and kinematics for specified nodes.
    target_nodes: List of 0-based indices (e.g., [4, 6] for nodes 5 and 7).
                  If None, plots all available nodes.
    """
    # 1. Determine which nodes to plot
    all_nodes = list(range(velo_data.shape[1]))
    nodes_to_plot = target_nodes if target_nodes is not None else all_nodes
    num_subplots = len(nodes_to_plot)

    # 2. Create subplots dynamically
    fig, axes = plt.subplots(num_subplots, 1, figsize=(12, 4 * num_subplots), 
                             sharex=True, constrained_layout=True)
    
    # Standardize axes to a list for iteration
    if num_subplots == 1:
        axes = [axes]

    ref_lines = {0: '0', np.pi/2: r'$\pi/2$', np.pi: r'$\pi$'}

    for idx, node_idx in enumerate(nodes_to_plot):
        ax = axes[idx]
        
        # 3. Calculate Phase and Normalized Data for the chosen node
        p1 = np.abs(np.arctan2(velo_data[:, node_idx], accl_data[:, node_idx]))
        p2 = np.abs(np.arctan2(accl_data[:, node_idx], velo_data[:, node_idx]))
        
        v_norm = velo_data[:, node_idx] / (np.max(np.abs(velo_data[:, node_idx])) + 1e-12)
        a_norm = accl_data[:, node_idx] / (np.max(np.abs(accl_data[:, node_idx])) + 1e-12)

        # 4. Draw Reference Lines
        for val, label in ref_lines.items():
            ax.axhline(y=val, color='gray', linestyle='--', alpha=0.3, linewidth=0.8)
            ax.text(t_span.max() * 1.005, val, label, va='center', color='gray')

        # 5. Plot Data (Adjusting labels to show human-readable Node IDs)
        ax.plot(t_span, p1, label=f'Node {node_idx+1}: $|atan2(v, a)|$', color='#1f77b4', lw=2)
        ax.plot(t_span, p2, label=f'Node {node_idx+1}: $|atan2(a, v)|$', color='#073d63', lw=1.5, ls='--')
        ax.plot(t_span, v_norm, label='Norm Velocity', color='#aec7e8', alpha=0.5)
        ax.plot(t_span, a_norm, label='Norm Acceleration', color='#ffbb78', alpha=0.5)

        # Formatting
        ax.set_title(f'Phase and Kinematic Analysis: Node {node_idx+1}', fontsize=14)
        ax.set_ylabel('Rad / Amplitude', fontsize=11)
        ax.legend(loc='upper left', bbox_to_anchor=(1.05, 1), fontsize='small')

    axes[-1].set_xlabel('Time (s)', fontsize=12)
    plt.show()



def plot_phase_magnitude_nodes(velo_data, accl_data, target_nodes):
    """
    Plots Magnitude vs Phase for both nodes.
    x-axis: Phase
    y-axis: Magnitude (also used for line color)
    """

    # 1. Determine which nodes to plot
    all_nodes = list(range(velo_data.shape[1]))
    nodes_to_plot = target_nodes if target_nodes is not None else all_nodes
    num_subplots = len(nodes_to_plot)

    fig, axes = plt.subplots(1, num_subplots, figsize=(12, 5), constrained_layout=True)
    
    # Handle single node case if input is (N, 1)
    if num_subplots == 1:
        axes = [axes]

    for idx, node_idx in enumerate(nodes_to_plot):
        ax = axes[idx]

        # Calculate Phase and Magnitude for current node
        # Phase on x, Magnitude on y
        wrapped_phase  = np.arctan2(velo_data[:, node_idx], accl_data[:, node_idx])
        magnitude = np.sqrt(velo_data[:, node_idx]**2 + accl_data[:, node_idx]**2)

        # cos(arctan(x)) = 1/sqrt(1 + x^2)
        # sin(arctan(x)) = x/sqrt(1 + x^2)

        x_val = magnitude * np.cos(wrapped_phase ) # Acceleration
        y_val = magnitude * np.sin(wrapped_phase ) # Velocity


        # Create segments for LineCollection
        points = np.array([x_val, y_val]).T.reshape(-1, 1, 2)
        segments = np.concatenate([points[:-1], points[1:]], axis=1)

        # Create the collection
        lc = LineCollection(segments, cmap='plasma', norm=plt.Normalize(y_val.min(), y_val.max()))
        lc.set_array(y_val)
        lc.set_linewidth(2)

        # Plotting
        line = ax.add_collection(lc)
        ax.set_xlim(x_val.min() - 0.1, x_val.max() + 0.1)
        ax.set_ylim(y_val.min() - 0.1, y_val.max() + 0.1)
        
        ax.set_title(f'Node {node_idx+1} Phase-Magnitude Ellipse')
        ax.set_xlabel('Acceleration')
        ax.set_ylabel('Velocity')
        
        fig.colorbar(line, ax=ax, label='Magnitude Intensity')

    plt.show()




mass_m1 = 10.0
mass_m2 = 2.0 
mass_m3 = 12.0

# masses: array-like of length N [m1, m2, ..., mN]
mass_m = [mass_m1, mass_m2, mass_m3]


stiff_k1 = 10.0
stiff_k2 = 100.0
stiff_k3 = 120.0
stiff_k4 = 20.0

# stiffnesses: array-like of length N+1 [k1, k2, ..., k_N+1] 
stiff_k = [stiff_k1, stiff_k2, stiff_k3, stiff_k4]

damp_z = 0.015

t_final = 10.0 #4.6850 # seconds

n_dof_soln = multi_dof_system_soln.NDofSystem(mass_m, stiff_k, damp_z)

# Extract parameters from your solver instance
omega = n_dof_soln.omega_n  # Natural circular frequencies (rad/s)
Phi = n_dof_soln.Phi        # Mass-normalized mode shapes

idispl_ampltiude = 10.0
q = idispl_ampltiude / omega

t_span = np.linspace(0.0, t_final, 4000)

# initial conditions
# Mid of mode 1 and 2  Phi[:,0] * q[0] + Phi[:,1] * q[1]  
idispl = np.array([11.0, 10.0, -10.0]) # 11.0, 10.0
ivelo = np.array([0.0, 0.0,0.0])

displ_modal_soln, velo_modal_soln, accl_modal_soln = n_dof_soln.solve(t_span, idispl, ivelo)

# Plot the results to compare
# plot_modal_solution(t_span, displ_modal_soln, velo_modal_soln, accl_modal_soln)
target_nodes = [0, 1, 2]

plot_phase_value(t_span, displ_modal_soln, velo_modal_soln, accl_modal_soln, target_nodes)
plot_phase_magnitude_nodes(velo_modal_soln, accl_modal_soln, target_nodes)







