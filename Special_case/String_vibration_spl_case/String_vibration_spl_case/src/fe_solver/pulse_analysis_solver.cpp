#include "pulse_analysis_solver.h"

pulse_analysis_solver::pulse_analysis_solver()
{
	// Empty constructor
}

pulse_analysis_solver::~pulse_analysis_solver()
{
	// Empty destructor
}

void pulse_analysis_solver::clear_results()
{
	// Clear the analysis results
	is_pulse_analysis_complete = false;
	time_step_count = 0;
	time_interval = 0.0;
	total_simulation_time = 0.0;

}

void pulse_analysis_solver::pulse_analysis_start(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const nodeconstraint_list_store& node_constraints,
	const nodeload_list_store& node_loads,
	const nodeinlcond_list_store& node_inldispl,
	const nodeinlcond_list_store& node_inlvelo,
	const material_data& mat_data,
	const modal_analysis_solver& modal_solver,
	const double total_simulation_time,
	const double time_interval,
	const double damping_ratio,
	const int selected_pulse_option,
	pulse_node_list_store& pulse_result_nodes,
	pulse_elementline_list_store& pulse_result_lineelements)
{
	// Main solver call
	this->is_pulse_analysis_complete = false;

	// Check the model
	// Number of loads, initial condition (Exit if no load and no initial condition is present)
	if (node_loads.load_count == 0 &&
		static_cast<int>(node_inldispl.inlcondMap.size()) == 0 &&
		static_cast<int>(node_inlvelo.inlcondMap.size()) == 0)
	{
		return;
	}

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);
	std::cout << "Pulse analysis started" << std::endl;

	// Retrive the Eigen values and Eigen vectors data from Modan Analysis solver
	this->model_type = modal_solver.model_type;
	this->numDOF = modal_solver.node_count;
	this->reducedDOF = modal_solver.matrix_size;
	this->eigen_values_vector = modal_solver.eigen_values_vector;
	this->eigen_vectors_matrix = modal_solver.eigen_vectors_matrix;
	this->eigen_vectors_matrix_inverse = modal_solver.eigen_vectors_matrix_inverse;


	//____________________________________________________________________________________________________________________
	// Step: 1 Create the initial displacement matrix (Modal Transformed initial displacement matrix)
	Eigen::VectorXd modal_reducedInitialDisplacementMatrix(reducedDOF);
	Eigen::VectorXd modal_reducedInitialVelocityMatrix(reducedDOF);


	create_initial_condition_matrices(modal_reducedInitialDisplacementMatrix,
		modal_reducedInitialVelocityMatrix,
		model_nodes,
		node_inldispl,
		node_inlvelo,
		this->eigen_vectors_matrix_inverse);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Intial condition matrices created at" << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Step: 2 Create the pulse load matrix (Modal Transformed pulse loads)

	this->pulse_loads.clear();

	for (auto& ld_m : node_loads.loadMap)
	{
		// get the loads
		load_data ld = ld_m.second;

		// temporary value to store the pulse load
		pulse_load_data pulse_ld;

		create_pulse_load_matrices(pulse_ld,
			ld,
			model_nodes,
			this->eigen_vectors_matrix_inverse);

		// Set the time data to the pulse loads
		pulse_ld.load_id = ld.load_setid;
		pulse_ld.load_start_time = ld.load_start_time;
		pulse_ld.load_end_time = ld.load_end_time;

		// Add to the pulse load list
		this->pulse_loads.push_back(pulse_ld);
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Pulse Load matrices created at" << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Step: 3 Find the pulse response
	std::unordered_map<int, pulse_node_result> node_results;

	// Time step count
	this->time_step_count = 0;

	for (double time_t = 0.0; time_t <= total_simulation_time; time_t = time_t + time_interval)
	{
		// Displacement amplitude matrix
		Eigen::VectorXd modal_displ_ampl_respMatrix(reducedDOF);
		modal_displ_ampl_respMatrix.setZero();

		// 1D results for modal transformed Simple Harmonic Motion
		for (int i = 0; i < reducedDOF; i++)
		{
			double modal_mass = 1.0;
			double modal_stiff = eigen_values_vector.coeff(i);

			// Displacement response due to initial condition
			double displ_resp_initial = 0.0;

			// Get the steady state displacemetn response for the initial condition
			displ_resp_initial = get_steady_state_initial_condition_soln(time_t,
				modal_mass,
				modal_stiff,
				modal_reducedInitialDisplacementMatrix.coeff(i),
				modal_reducedInitialVelocityMatrix.coeff(i));

			//_______________________________________________________________________
			// Displacement response due to pulse force
			double displ_resp_force = 0.0;

			// Cycle through all the loads
			for (auto& pulse_load : pulse_loads)
			{


				if (std::abs(pulse_load.modal_LoadamplMatrix(i)) > epsilon)
				{
					// Go through all the force
					double at_force_displ_resp = 0.0;

					// Load amplitude at index not equal to zero

					if (selected_pulse_option == 0)
					{
						// Half sine pulse

						at_force_displ_resp = get_steady_state_half_sine_pulse_soln(time_t,
							modal_mass,
							modal_stiff,
							pulse_load.modal_LoadamplMatrix(i),
							pulse_load.load_start_time,
							pulse_load.load_end_time);

					}
					else if (selected_pulse_option == 1)
					{
						// Rectangular pulse

						at_force_displ_resp = get_steady_state_rectangular_pulse_soln(time_t,
							modal_mass,
							modal_stiff,
							pulse_load.modal_LoadamplMatrix(i),
							pulse_load.load_start_time,
							pulse_load.load_end_time);

					}
					else if (selected_pulse_option == 2)
					{
						// Triangular pulse

						at_force_displ_resp = get_steady_state_triangular_pulse_soln(time_t,
							modal_mass,
							modal_stiff,
							pulse_load.modal_LoadamplMatrix(i),
							pulse_load.load_start_time,
							pulse_load.load_end_time);

					}
					else if (selected_pulse_option == 3)
					{
						// Step force with finite rise

						at_force_displ_resp = get_steady_state_stepforce_finiterise_soln(time_t,
							modal_mass,
							modal_stiff,
							pulse_load.modal_LoadamplMatrix(i),
							pulse_load.load_start_time,
							pulse_load.load_end_time);

					}
					else if (selected_pulse_option == 4)
					{
						// Harmonic Excitation

						at_force_displ_resp = get_total_harmonic_soln(time_t,
							modal_mass,
							modal_stiff,
							pulse_load.modal_LoadamplMatrix(i),
							pulse_load.load_start_time,
							pulse_load.load_end_time);

					}

					displ_resp_force = displ_resp_force + at_force_displ_resp;
				}
			}

			// Store the displacement result
			modal_displ_ampl_respMatrix.coeffRef(i) = displ_resp_initial + displ_resp_force;

		}


		// Transform the modal displacement to the global displacement
		Eigen::VectorXd global_displ_ampl_respMatrix(reducedDOF);

		global_displ_ampl_respMatrix = this->eigen_vectors_matrix * modal_displ_ampl_respMatrix;

		// Store the results to node results
		for (auto& nd_m : model_nodes.nodeMap)
		{
			// get the node id
			int node_id = nd_m.second.node_id;

			// Node displacement response
			glm::vec2 node_displ = glm::vec2(0);

			if (this->model_type == 0)
			{
				// Fixed  - Fixed Condition so skip the first and last node
				if (node_id != 0 && node_id != (numDOF - 1))
				{
					node_displ = glm::vec2(0.0, global_displ_ampl_respMatrix.coeff(node_id - 1));
				}
			}
			else if (this->model_type == 1)
			{
				// Fixed - Free Condition so skip the first node
				if (node_id != 0)
				{
					node_displ = glm::vec2(0.0, global_displ_ampl_respMatrix.coeff(node_id - 1));
				}
			}
			else if (this->model_type == 3)
			{
				// Circular string Free - Free
				glm::vec2 normal_dir = glm::normalize(nd_m.second.node_pt);

				// get the appropriate modal displacement of this particular node
				node_displ = static_cast<float>(global_displ_ampl_respMatrix.coeff(node_id)) * glm::vec2(normal_dir.x, -1.0 * normal_dir.y);

			}

			// Node displacement magnitude
			double displ_magnitude = static_cast<float>(glm::length(node_displ));
			glm::vec2 normalized_node_displ = glm::vec2(0.0, -1.0);

			if (displ_magnitude > epsilon)
			{
				normalized_node_displ = glm::normalize(node_displ);
			}

			// Add the index
			node_results[node_id].index.push_back(this->time_step_count);
			// Add the time val
			node_results[node_id].time_val.push_back(time_t);
			// Add the displacement magnitude
			node_results[node_id].displ_magnitude.push_back(displ_magnitude);
			// Add the Normalized displacement
			node_results[node_id].normalized_displ.push_back(normalized_node_displ);
		}

		// iterate the time step count
		this->time_step_count++;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Pulse analysis solved at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Step: 4 Map the results

	map_pulse_analysis_results(pulse_result_nodes,
		pulse_result_lineelements,
		this->time_step_count,
		model_nodes,
		model_lineelements,
		node_results);



	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Save the Pulse analysis settings
	this->time_interval = time_interval;
	this->total_simulation_time = total_simulation_time;

	if (pulse_result_lineelements.max_line_displ == 0)
	{
		// Analysis failed 
		stopwatch_elapsed_str.str("");
		stopwatch_elapsed_str << stopwatch.elapsed();
		std::cout << "Analysis failed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		return;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results storage completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	std::cout << "Pulse analysis complete " << std::endl;

	// Analysis complete
	this->is_pulse_analysis_complete = true;

}

void pulse_analysis_solver::create_initial_condition_matrices(Eigen::VectorXd& modal_reducedInitialDisplacementMatrix,
	Eigen::VectorXd& modal_reducedInitialVelocityMatrix,
	const nodes_list_store& model_nodes,
	const nodeinlcond_list_store& node_inldispl,
	const nodeinlcond_list_store& node_inlvelo,
	const Eigen::MatrixXd& eigen_vectors_matrix_inverse)
{
	// Modal reduction of global initial displacement
	// Modal vector transformation 
	modal_reducedInitialDisplacementMatrix.setZero();
	modal_reducedInitialVelocityMatrix.setZero();

	if (this->model_type == 0)
	{
		// Fixed  - Fixed Condition so skip the first and last node
		Eigen::VectorXd global_reducedInitialDisplacementMatrix(reducedDOF);
		Eigen::VectorXd global_reducedInitialVelocityMatrix(reducedDOF);

		int q = 0;
		for (int i = 1; i < this->numDOF - 1; i++)
		{
			global_reducedInitialDisplacementMatrix.coeffRef(q) = -1.0 * node_inldispl.inlcondMap.at(i).y_val;
			global_reducedInitialVelocityMatrix.coeffRef(q) = -1.0 * node_inlvelo.inlcondMap.at(i).y_val;

			q++;
		}

		// Apply modal Transformation
		modal_reducedInitialDisplacementMatrix = eigen_vectors_matrix_inverse * global_reducedInitialDisplacementMatrix;
		modal_reducedInitialVelocityMatrix = eigen_vectors_matrix_inverse * global_reducedInitialVelocityMatrix;

	}
	else if (this->model_type == 1)
	{
		// Fixed - Free Condition so skip the first node
		Eigen::VectorXd global_reducedInitialDisplacementMatrix(reducedDOF);
		Eigen::VectorXd global_reducedInitialVelocityMatrix(reducedDOF);

		int q = 0;
		for (int i = 1; i < this->numDOF; i++)
		{
			global_reducedInitialDisplacementMatrix.coeffRef(q) = -1.0 * node_inldispl.inlcondMap.at(i).y_val;
			global_reducedInitialVelocityMatrix.coeffRef(q) = -1.0 * node_inlvelo.inlcondMap.at(i).y_val;

			q++;
		}

		// Apply modal Transformation
		modal_reducedInitialDisplacementMatrix = eigen_vectors_matrix_inverse * global_reducedInitialDisplacementMatrix;
		modal_reducedInitialVelocityMatrix = eigen_vectors_matrix_inverse * global_reducedInitialVelocityMatrix;

	}
	else if (this->model_type == 3)
	{
		// Circular (No DOF) Free - Free

		Eigen::VectorXd global_reducedInitialDisplacementMatrix(reducedDOF);
		Eigen::VectorXd global_reducedInitialVelocityMatrix(reducedDOF);

		int q = 0;
		for (int i = 0; i < this->numDOF; i++)
		{
			global_reducedInitialDisplacementMatrix.coeffRef(q) = node_inldispl.inlcondMap.at(i).y_val;
			global_reducedInitialVelocityMatrix.coeffRef(q) = node_inlvelo.inlcondMap.at(i).y_val;

			q++;
		}

		// Apply modal Transformation
		modal_reducedInitialDisplacementMatrix = eigen_vectors_matrix_inverse * global_reducedInitialDisplacementMatrix;
		modal_reducedInitialVelocityMatrix = eigen_vectors_matrix_inverse * global_reducedInitialVelocityMatrix;

	}

}


void pulse_analysis_solver::create_pulse_load_matrices(pulse_load_data& pulse_ld,
	const load_data& ld,
	const nodes_list_store& model_nodes,
	const Eigen::MatrixXd& eigen_vectors_matrix_inverse)
{
	// Create Pulse load matrix
	// Modal vector transformation


	if (this->model_type == 0)
	{
		// Fixed  - Fixed Condition so skip the first and last node
		Eigen::VectorXd global_reducedLoadMatrix(reducedDOF);

		int q = 0;
		for (int i = 1; i < this->numDOF - 1; i++)
		{
			global_reducedLoadMatrix.coeffRef(q) = 0.0;

			// Apply the loads which are applied to the nodes (based on the node id)
			if (i == ld.node_id)
			{
				global_reducedLoadMatrix.coeffRef(q) = ld.load_value;
			}
			q++;
		}

		// Apply modal Transformation
		Eigen::VectorXd modal_reducedLoadMatrix(reducedDOF);

		modal_reducedLoadMatrix = eigen_vectors_matrix.transpose() * global_reducedLoadMatrix;

		// Store the modal amplitude matrix
		pulse_ld.modal_LoadamplMatrix = modal_reducedLoadMatrix;

	}
	else if (this->model_type == 1)
	{
		// Fixed - Free Condition so skip the first node
		Eigen::VectorXd global_reducedLoadMatrix(reducedDOF);

		int q = 0;
		for (int i = 1; i < this->numDOF; i++)
		{
			global_reducedLoadMatrix.coeffRef(q) = 0.0;

			// Apply the loads which are applied to the nodes (based on the node id)
			if (i == ld.node_id)
			{
				global_reducedLoadMatrix.coeffRef(q) = ld.load_value;
			}
			q++;
		}
		// Apply modal Transformation
		Eigen::VectorXd modal_reducedLoadMatrix(reducedDOF);

		modal_reducedLoadMatrix = eigen_vectors_matrix.transpose() * global_reducedLoadMatrix;

		// Store the modal amplitude matrix
		pulse_ld.modal_LoadamplMatrix = modal_reducedLoadMatrix;

	}
	else if (this->model_type == 3)
	{
		// Circular (No DOF)
		// Fixed - Free Condition so skip the first node
		Eigen::VectorXd global_reducedLoadMatrix(reducedDOF);

		int q = 0;
		for (int i = 0; i < this->numDOF; i++)
		{
			global_reducedLoadMatrix.coeffRef(q) = 0.0;

			// Apply the loads which are applied to the nodes (based on the node id)
			if (i == ld.node_id)
			{
				global_reducedLoadMatrix.coeffRef(q) = -1.0 * ld.load_value;
			}
			q++;
		}
		// Apply modal Transformation
		Eigen::VectorXd modal_reducedLoadMatrix(reducedDOF);

		modal_reducedLoadMatrix = eigen_vectors_matrix.transpose() * global_reducedLoadMatrix;

		// Store the modal amplitude matrix
		pulse_ld.modal_LoadamplMatrix = modal_reducedLoadMatrix;

	}


}


double pulse_analysis_solver::get_steady_state_initial_condition_soln(const double& time_t,
	const double& modal_mass,
	const double& modal_stiff,
	const double& modal_initial_displacement,
	const double& modal_initial_velocity)
{
	// Return the steady state solution for the intial displacment and velocity
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n

	double steady_state_displ_resp = (modal_initial_displacement * std::cos(modal_omega_n * time_t)) +
		((modal_initial_velocity / modal_omega_n) * std::sin(modal_omega_n * time_t));

	return steady_state_displ_resp;
}


double pulse_analysis_solver::get_steady_state_half_sine_pulse_soln(const double& time_t,
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

	double steady_state_displ_resp = 0.0;

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


	return steady_state_displ_resp;
}


double pulse_analysis_solver::get_steady_state_rectangular_pulse_soln(const double& time_t,
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

	double steady_state_displ_resp = 0.0;

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

	return steady_state_displ_resp;
}


double pulse_analysis_solver::get_steady_state_triangular_pulse_soln(const double& time_t,
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

	double	steady_state_displ_resp = 0.0;

	// Check whether the current time is greater than the force start time
	if (time_t >= modal_force_starttime)
	{
		t_at = time_t - modal_force_starttime;

		if (time_t <= modal_force_endtime)
		{
			// current time is within the force period
			if (t_at < (t_d * 0.5))
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

	return steady_state_displ_resp;
}



double pulse_analysis_solver::get_steady_state_stepforce_finiterise_soln(const double& time_t,
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

	double	steady_state_displ_resp = 0.0;

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

	return steady_state_displ_resp;
}


double pulse_analysis_solver::get_total_harmonic_soln(const double& time_t,
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
	double steady_state_displ_resp;

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

	return (transient_displ_resp + steady_state_displ_resp);
}

void pulse_analysis_solver::map_pulse_analysis_results(pulse_node_list_store& pulse_result_nodes,
	pulse_elementline_list_store& pulse_result_lineelements,
	const int& number_of_time_steps,
	const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const std::unordered_map<int, pulse_node_result>& node_results)
{
	// Map the pulse analysis results
		// map the node results
	pulse_result_nodes.clear_data();

	for (auto& nd_m : model_nodes.nodeMap)
	{
		// Extract the model node
		node_store nd = nd_m.second;

		// Add to the pulse node results store
		pulse_result_nodes.add_result_node(nd.node_id, nd.node_pt, node_results.at(nd.node_id), number_of_time_steps);
	}

	// map the line results
	pulse_result_lineelements.clear_data();

	for (auto& ln_m : model_lineelements.elementlineMap)
	{
		// Extract the model lines
		elementline_store ln = ln_m.second;

		// Extract the pulse node store -> start node and end node
		pulse_node_store* startNode = &pulse_result_nodes.pulse_nodeMap[ln.startNode->node_id];
		pulse_node_store* endNode = &pulse_result_nodes.pulse_nodeMap[ln.endNode->node_id];

		// Add to the pulse element results store
		pulse_result_lineelements.add_pulse_elementline(ln.line_id, startNode, endNode);
	}

	//_________________________________________________________________________________________________________________
	double maximum_displacement = 0.0;

	for (auto& ln_m : pulse_result_lineelements.pulse_elementlineMap)
	{
		pulse_elementline_store ln = ln_m.second;

		//get all the two points
		// Point 1 displacement
		for (auto& startpt_displ : ln.startpt_displ_magnitude)
		{
			maximum_displacement = std::max(maximum_displacement, startpt_displ);
		}

		// Point 2 displacement
		for (auto& endpt_displ : ln.endpt_displ_magnitude)
		{
			maximum_displacement = std::max(maximum_displacement, endpt_displ);
		}
	}

	// Set the maximim displacement
	pulse_result_nodes.max_node_displ = maximum_displacement;
	pulse_result_lineelements.max_line_displ = maximum_displacement;


}