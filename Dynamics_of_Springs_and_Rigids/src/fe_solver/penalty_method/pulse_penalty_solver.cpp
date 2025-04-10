#include "pulse_penalty_solver.h"

pulse_penalty_solver::pulse_penalty_solver()
{
	// Empty constructor
}


pulse_penalty_solver::~pulse_penalty_solver()
{
	// Empty destructor
}


void pulse_penalty_solver::clear_results()
{
	// Clear the analysis results
	is_pulse_analysis_complete = false;
	time_step_count = 0;
	time_interval = 0.0;
	total_simulation_time = 0.0;

}


void pulse_penalty_solver::pulse_analysis_penaltymethod_start(const nodes_list_store& model_nodes, 
	const elementline_list_store& model_lineelements, 
	const nodeconstraint_list_store& node_constraints, 
	const nodeload_list_store& node_loads,
	const nodepointmass_list_store& node_ptmass, 
	const nodeinlcond_list_store& node_inlcond, 
	const std::unordered_map<int, material_data>& material_list, 
	const modal_penalty_solver& modal_solver, 
	const double total_simulation_time, 
	const double time_interval, 
	const double damping_ratio, 
	const int mode_range_startid, 
	const int mode_range_endid, 
	const int selected_pulse_option, 
	pulse_node_list_store& pulse_result_nodes, 
	pulse_elementline_list_store& pulse_result_lineelements)
{



}


void pulse_penalty_solver::get_reduced_global_matrix(Eigen::MatrixXd& reducedglobalMatrix, 
	const Eigen::MatrixXd& globalMatrix, 
	const Eigen::VectorXi& globalDOFMatrix, 
	const int& numDOF, 
	const int& reducedDOF)
{


}


void pulse_penalty_solver::create_initial_condition_matrices(Eigen::VectorXd& modal_reducedInitialDisplacementMatrix, 
	Eigen::VectorXd& modal_reducedInitialVelocityMatrix, 
	const nodeinlcond_list_store& node_inlcond, 
	const Eigen::VectorXi& globalDOFMatrix, 
	const Eigen::MatrixXd& globalSupportInclinationMatrix, 
	const Eigen::MatrixXd& reduced_eigenVectorsMatrix, 
	const int& numDOF, 
	const int& reducedDOF, 
	std::ofstream& output_file)
{


}


void pulse_penalty_solver::get_reduced_global_vector(Eigen::VectorXd& reducedglobalVector, 
	const Eigen::VectorXd& globalVector, 
	const Eigen::VectorXi& globalDOFMatrix, 
	const int& numDOF, 
	const int& reducedDOF)
{


}


void pulse_penalty_solver::get_global_resp_vector(Eigen::VectorXd& globalVector, 
	const Eigen::VectorXd& reducedglobalVector, 
	const Eigen::VectorXi& globalDOFMatrix, 
	const int& numDOF, 
	const int& reducedDOF)
{



}


void pulse_penalty_solver::create_pulse_load_matrices(pulse_load_data& pulse_loads, 
	const load_data& ld, 
	const nodes_list_store& model_nodes, 
	const Eigen::VectorXi& globalDOFMatrix, 
	const Eigen::MatrixXd& globalSupportInclinationMatrix, 
	const Eigen::MatrixXd& reduced_eigenVectorsMatrix_transpose, 
	const int& numDOF, 
	const int& reducedDOF, 
	std::ofstream& output_file)
{



}


void pulse_penalty_solver::get_steady_state_initial_condition_soln(double& steady_state_displ_resp, 
	const double& time_t, 
	const double& modal_mass, 
	const double& modal_stiff, 
	const double& modal_initial_displacement, 
	const double& modal_initial_velocity)
{
	// Return the steady state solution for the intial displacment and velocity
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n

	steady_state_displ_resp = (modal_initial_displacement * std::cos(modal_omega_n * time_t)) +
		((modal_initial_velocity / modal_omega_n) * std::sin(modal_omega_n * time_t));

}


void pulse_penalty_solver::get_steady_state_half_sine_pulse_soln(double& steady_state_displ_resp, 
	const double& time_t, 
	const double& modal_mass, 
	const double& modal_stiff, 
	const double& modal_force_ampl, 
	const double& modal_force_starttime, 
	const double& modal_force_endtime)
{
	// Return the steady state solution for the half sine pulse
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n
	double modal_omega_f = m_pi / (modal_force_endtime - modal_force_starttime);

	// natural time period
	double T_n = (2.0 * m_pi) / modal_omega_n;
	// Force period
	double t_d = (modal_force_endtime - modal_force_starttime);
	// time at
	double t_at = 0.0;

	steady_state_displ_resp = 0.0;

	// Check whether the current time is greater than the force start time
	if (time_t >= modal_force_starttime)
	{
		t_at = time_t - modal_force_starttime;
		if (time_t <= modal_force_endtime)
		{
			// current time is within the force period
			if (std::abs((t_d / T_n) - 0.5) < 0.000001)
			{
				// Resonance case
				double k_fact = (modal_force_ampl / (2.0 * modal_stiff));
				steady_state_displ_resp = k_fact * (std::sin(modal_omega_n * t_at) - (modal_omega_n * t_at * std::cos(modal_omega_n * t_at)));
			}
			else
			{
				// Normal case
				double const1 = m_pi / (modal_omega_n * t_d);
				double const2 = 1.0 - std::pow(const1, 2);
				double k_fact = (modal_force_ampl / modal_stiff) * (1 / const2);
				steady_state_displ_resp = k_fact * (std::sin((m_pi / t_d) * t_at) - (const1 * std::sin(modal_omega_n * t_at)));
			}
		}
		else if (time_t > modal_force_endtime)
		{
			// current time is over the force period
			if (std::abs((t_d / T_n) - 0.5) < 0.000001)
			{
				// Resonance case
				double k_fact = ((modal_force_ampl * m_pi) / (2.0 * modal_stiff));
				steady_state_displ_resp = k_fact * std::cos((modal_omega_n * t_at) - m_pi);
			}
			else
			{
				// Normal case
				double const1 = m_pi / (modal_omega_n * t_d);
				double const2 = std::pow(const1, 2) - 1.0;
				double k_fact = (modal_force_ampl / modal_stiff) * ((2 * const1) / const2) * std::cos(modal_omega_n * t_d * 0.5);
				steady_state_displ_resp = k_fact * std::sin(modal_omega_n * (t_at - (t_d * 0.5)));
			}
		}
	}

}


void pulse_penalty_solver::get_steady_state_rectangular_pulse_soln(double& steady_state_displ_resp, 
	const double& time_t, 
	const double& modal_mass, 
	const double& modal_stiff, 
	const double& modal_force_ampl, 
	const double& modal_force_starttime, 
	const double& modal_force_endtime)
{
	// Return the steady state solution for the Rectangular pulse
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n
	double modal_omega_f = m_pi / (modal_force_endtime - modal_force_starttime);

	// natural time period
	double T_n = (2.0 * m_pi) / modal_omega_n;
	// Force period
	double t_d = (modal_force_endtime - modal_force_starttime);
	// time at
	double t_at = 0.0;

	steady_state_displ_resp = 0.0;

	// Check whether the current time is greater than the force start time
	if (time_t >= modal_force_starttime)
	{
		t_at = time_t - modal_force_starttime;
		double k_fact = (modal_force_ampl / modal_stiff);

		if (time_t <= modal_force_endtime)
		{

			// current time is within the force period
			steady_state_displ_resp = k_fact * (1.0 - std::cos(modal_omega_n * t_at));

		}
		else if (time_t > modal_force_endtime)
		{
			// current time is over the force period
			steady_state_displ_resp = k_fact * (std::cos(modal_omega_n * (t_at - t_d)) - std::cos(modal_omega_n * t_at));

		}
	}

}


void pulse_penalty_solver::get_steady_state_triangular_pulse_soln(double& steady_state_displ_resp, 
	const double& time_t, 
	const double& modal_mass, 
	const double& modal_stiff, 
	const double& modal_force_ampl, 
	const double& modal_force_starttime, 
	const double& modal_force_endtime)
{
	// Return the steady state solution for the Triangular pulse
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n
	double modal_omega_f = m_pi / (modal_force_endtime - modal_force_starttime);

	// natural time period
	double T_n = (2.0 * m_pi) / modal_omega_n;
	// Force period
	double t_d = (modal_force_endtime - modal_force_starttime);
	// time at
	double t_at = 0.0;

	steady_state_displ_resp = 0.0;

	// Check whether the current time is greater than the force start time
	if (time_t >= modal_force_starttime)
	{
		t_at = time_t - modal_force_starttime;

		if (time_t <= modal_force_endtime)
		{
			// current time is within the force period
			if (t_at < (t_d / 2.0))
			{
				double k_fact = ((2.0 * modal_force_ampl) / modal_stiff);

				steady_state_displ_resp = k_fact * ((t_at / t_d) - (std::sin(modal_omega_n * t_at) / (t_d * modal_omega_n)));
			}
			else
			{
				double k_fact = ((2.0 * modal_force_ampl) / modal_stiff);
				double factor1 = (1 / (t_d * modal_omega_n)) * ((2.0 * std::sin(modal_omega_n * (t_at - (0.5 * t_d)))) -
					std::sin(modal_omega_n * t_at));

				steady_state_displ_resp = k_fact * (1.0 - (t_at / t_d) + factor1);
			}

		}
		else if (time_t > modal_force_endtime)
		{
			// current time is over the force period

			double k_fact = ((2.0 * modal_force_ampl) / modal_stiff);
			double m_factor = (1 / (t_d * modal_omega_n));
			double factor1 = 2.0 * std::sin(modal_omega_n * (t_at - (0.5 * t_d)));
			double factor2 = std::sin(modal_omega_n * (t_at - t_d));

			steady_state_displ_resp = k_fact * (m_factor * (factor1 - factor2 - std::sin(modal_omega_n * t_at)));

		}
	}

}


void pulse_penalty_solver::get_steady_state_stepforce_finiterise_soln(double& steady_state_displ_resp, 
	const double& time_t, 
	const double& modal_mass, 
	const double& modal_stiff, 
	const double& modal_force_ampl, 
	const double& modal_force_starttime, 
	const double& modal_force_endtime)
{
	// Return the steady state solution for the Step Force with Finite Rise
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n
	double modal_omega_f = m_pi / (modal_force_endtime - modal_force_starttime);

	// natural time period
	double T_n = (2.0 * m_pi) / modal_omega_n;
	// Force period
	double t_d = (modal_force_endtime - modal_force_starttime);
	// time at
	double t_at = 0.0;

	steady_state_displ_resp = 0.0;

	// Check whether the current time is greater than the force start time
	if (time_t >= modal_force_starttime)
	{
		t_at = time_t - modal_force_starttime;
		double k_fact = (modal_force_ampl / modal_stiff);

		if (time_t <= modal_force_endtime)
		{

			// current time is within the force period
			steady_state_displ_resp = k_fact * ((t_at / t_d) -
				(std::sin(modal_omega_n * t_at) / (t_d * modal_omega_n)));

		}
		else if (time_t > modal_force_endtime)
		{
			// current time is over the force period
			double factor1 = std::sin(modal_omega_n * (t_at - t_d)) - std::sin(modal_omega_n * t_at);

			steady_state_displ_resp = k_fact * (1.0 + (1 / (modal_omega_n * t_d)) * factor1);

		}
	}

}


void pulse_penalty_solver::get_total_harmonic_soln(double& steady_state_displ_resp, 
	const double& time_t, 
	const double& modal_mass, 
	const double& modal_stiff, 
	const double& modal_force_ampl, 
	const double& modal_force_starttime, 
	const double& modal_force_endtime)
{
	// Return the Total solution (Transient + steady state) for the Harmonic excitation
		// Return the steady state solution for the Step Force with Finite Rise
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n
	double modal_omega_f = m_pi / (2.0 * (modal_force_endtime - modal_force_starttime));

	double transient_displ_resp;
	steady_state_displ_resp = 0.0;

	if (std::abs(modal_omega_f - modal_omega_n) < epsilon)
	{
		// Resonance case
		transient_displ_resp = 0.0;

		double force_factor = (modal_force_ampl / (2.0 * modal_stiff));

		steady_state_displ_resp = force_factor * ((modal_omega_n * time_t * std::cos(modal_omega_n * time_t)) - std::sin(modal_omega_n * time_t));
	}
	else
	{
		// Regular case
		double force_factor = (modal_force_ampl / modal_stiff);
		double freq_ratio = modal_omega_f / modal_omega_n;
		double freq_factor = (1.0 - (freq_ratio * freq_ratio));


		transient_displ_resp = -1.0 * force_factor * (freq_ratio / freq_factor) * std::sin(modal_omega_n * time_t);

		steady_state_displ_resp = force_factor * (1.0 / freq_factor) * std::sin(modal_omega_f * time_t);

	}

}


void pulse_penalty_solver::map_pulse_analysis_results(pulse_node_list_store& pulse_result_nodes, 
	pulse_elementline_list_store& pulse_result_lineelements, 
	const int& number_of_time_steps, 
	const nodes_list_store& model_nodes, 
	const elementline_list_store& model_lineelements, 
	const std::unordered_map<int, pulse_node_result>& node_results)
{


}
