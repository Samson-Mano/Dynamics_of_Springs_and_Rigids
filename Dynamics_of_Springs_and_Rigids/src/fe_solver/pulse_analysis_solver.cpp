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
	const nodepointmass_list_store& node_ptmass, 
	const nodeinlcond_list_store& node_inlcond, 
	const std::unordered_map<int, material_data>& material_list, 
	const modal_analysis_solver& modal_solver, 
	const double total_simulation_time, 
	const double time_interval, 
	const double damping_ratio, 
	const int mode_range_startid,
	const int mode_range_endid,
	const int selected_pulse_option,
	pulse_node_list_store& pulse_result_nodes, 
	pulse_elementline_list_store& pulse_result_lineelements)
{
	// Main solver call
	this->is_pulse_analysis_complete = false;

	// Check the model
	// Number of loads, initial condition (Exit if no load and no initial condition is present)
	if (node_loads.load_count == 0 && node_inlcond.inlcond_count == 0)
	{
		return;
	}

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);
	std::cout << "Pulse analysis started" << std::endl;

	// Assign the node id map
	this->nodeid_map = modal_solver.nodeid_map;

	//___________________________________________________________________________________
	// Create a file to keep track of frequency response matrices
	std::ofstream output_file;
	output_file.open("pulse_analysis_results.txt");

	//___________________________________________________________________________________
	// Get the modal vectors (within the range)
	int numDOF = modal_solver.numDOF;
	int reducedDOF = modal_solver.reducedDOF;
	int k = 0;

	Eigen::VectorXi globalDOFMatrix = modal_solver.globalDOFMatrix; // retrive the global DOF matrix from the modal solver
	Eigen::MatrixXd globalSupportInclinationMatrix = modal_solver.globalSupportInclinationMatrix; // retrive the global Support Inclination matrix from the modal solver
	Eigen::MatrixXd global_eigenVectorsMatrix = modal_solver.global_eigenvectors_transformed; // Retrive the global EigenVectors matrix 
	Eigen::MatrixXd reduced_eigenVectorsMatrix = modal_solver.reduced_eigenvectors; // Retrive the reduced EigenVectors matrix 

	if (print_matrix == true)
	{
		// Print the Reduced eigen vectors 
		output_file << "Reduced Eigen vectors matrix" << std::endl;
		output_file << reduced_eigenVectorsMatrix << std::endl;
		output_file << std::endl;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Reduced Eigen vectors retrived at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//--------------------------------------------------------------------------------------------------------------------
	// Create modal reduced intial condition matrices
	Eigen::VectorXd modal_reducedInitialDisplacementMatrix(reducedDOF);
	Eigen::VectorXd modal_reducedInitialVelocityMatrix(reducedDOF);

	create_initial_condition_matrices(modal_reducedInitialDisplacementMatrix,
		modal_reducedInitialVelocityMatrix,
		node_inlcond,
		globalDOFMatrix,
		globalSupportInclinationMatrix,
		reduced_eigenVectorsMatrix,
		numDOF,
		reducedDOF,
		output_file);

	//____________________________________________________________________________________________________________________
	// Create the Pulse force data for all the individual 
	std::vector<pulse_load_data> pulse_loads(node_loads.load_count);
	k = 0;

	for (auto& ld_m : node_loads.loadMap)
	{
		load_data ld = ld_m.second; // get the load data

		pulse_loads[k].load_id = k;
		create_pulse_load_matrices(pulse_loads[k],
			ld,
			model_nodes,
			globalDOFMatrix,
			globalSupportInclinationMatrix,
			reduced_eigenVectorsMatrix.transpose(),
			numDOF,
			reducedDOF,
			output_file);

		k++; // iterate load id
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Pulse loads matrices created at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Pulse Response
	std::unordered_map<int, pulse_node_result> node_results;
	int r_id = 0;

	for (double time_t = 0.0; time_t <= total_simulation_time; time_t = time_t + time_interval)
	{

		Eigen::VectorXd displ_ampl_RespMatrix_reduced_b4eig_trans(reducedDOF);
		displ_ampl_RespMatrix_reduced_b4eig_trans.setZero();

		for (int i = mode_range_startid; i <= mode_range_endid; i++)
		{
			double displ_resp_initial = 0.0; // Displacement response due to initial condition

			get_steady_state_initial_condition_soln(displ_resp_initial,
				time_t,
				modal_solver.reduced_modalMass(i),
				modal_solver.reduced_modalStiff(i),
				modal_reducedInitialDisplacementMatrix.coeff(i),
				modal_reducedInitialVelocityMatrix.coeff(i));

			//_______________________________________________________________________
			double displ_resp_force = 0.0; // Displacement response due to pulse force

			// get all the loads
			for (auto& pulse_load : pulse_loads)
			{
				// Go through all the force
				double at_force_displ_resp = 0.0;

				if (selected_pulse_option == 0)
				{
					// Half sine pulse

					get_steady_state_half_sine_pulse_soln(at_force_displ_resp,
						time_t,
						modal_solver.reduced_modalMass(i),
						modal_solver.reduced_modalStiff(i),
						pulse_load.modal_reducedLoadamplMatrix(i),
						pulse_load.load_start_time,
						pulse_load.load_end_time);

				}
				else if (selected_pulse_option == 1)
				{
					// Rectangular pulse

					get_steady_state_rectangular_pulse_soln(at_force_displ_resp,
						time_t,
						modal_solver.reduced_modalMass(i),
						modal_solver.reduced_modalStiff(i),
						pulse_load.modal_reducedLoadamplMatrix(i),
						pulse_load.load_start_time,
						pulse_load.load_end_time);

				}
				else if (selected_pulse_option == 2)
				{
					// Triangular pulse

					get_steady_state_triangular_pulse_soln(at_force_displ_resp,
						time_t,
						modal_solver.reduced_modalMass(i),
						modal_solver.reduced_modalStiff(i),
						pulse_load.modal_reducedLoadamplMatrix(i),
						pulse_load.load_start_time,
						pulse_load.load_end_time);

				}
				else if (selected_pulse_option == 3)
				{
					// Step force with finite rise

					get_steady_state_stepforce_finiterise_soln(at_force_displ_resp,
						time_t,
						modal_solver.reduced_modalMass(i),
						modal_solver.reduced_modalStiff(i),
						pulse_load.modal_reducedLoadamplMatrix(i),
						pulse_load.load_start_time,
						pulse_load.load_end_time);

				}
				else if (selected_pulse_option == 4)
				{
					// Harmonic Excitation

					get_total_harmonic_soln(at_force_displ_resp,
						time_t,
						modal_solver.reduced_modalMass(i),
						modal_solver.reduced_modalStiff(i),
						pulse_load.modal_reducedLoadamplMatrix(i),
						pulse_load.load_start_time,
						pulse_load.load_end_time);

				}

				displ_resp_force = displ_resp_force + at_force_displ_resp;
			}

			// Add to the modal displ matrix
			displ_ampl_RespMatrix_reduced_b4eig_trans.coeffRef(i) = displ_resp_initial + displ_resp_force;
		}

		// Apply modal de-transformation
		Eigen::VectorXd displ_ampl_RespMatrix_reduced(reducedDOF);
		displ_ampl_RespMatrix_reduced.setZero();

		displ_ampl_RespMatrix_reduced = reduced_eigenVectorsMatrix * displ_ampl_RespMatrix_reduced_b4eig_trans;

		// Extend reduced modal displ matrix to global modal displ matrix
		Eigen::VectorXd displ_ampl_RespMatrix_b4supp_trans(numDOF);
		displ_ampl_RespMatrix_b4supp_trans.setZero();

		get_global_resp_vector(displ_ampl_RespMatrix_b4supp_trans,
			displ_ampl_RespMatrix_reduced,
			globalDOFMatrix,
			numDOF,
			reducedDOF);

		// Apply support transformation
		Eigen::VectorXd displ_ampl_RespMatrix(numDOF);
		displ_ampl_RespMatrix.setZero();

		displ_ampl_RespMatrix = globalSupportInclinationMatrix * displ_ampl_RespMatrix_b4supp_trans;


		// Store the results to node results
		for (auto& nd_m : model_nodes.nodeMap)
		{
			// get the node id
			int nd_id = nd_m.second.node_id;
			int nd_index = nodeid_map[nd_id];

			// Node displacement response
			glm::vec2 node_displ = glm::vec2(displ_ampl_RespMatrix.coeff((nd_index * 2) + 0),
				displ_ampl_RespMatrix.coeff((nd_index * 2) + 1));

			double displ_magnitude = static_cast<float>(glm::length(node_displ));
			glm::vec2 normalized_node_displ = glm::vec2(1.0, 0.0);

			if (displ_magnitude > 0.0)
			{
				normalized_node_displ = glm::normalize(node_displ);
			}

			// Add the index
			node_results[nd_id].index.push_back(r_id);
			// Add the time val
			node_results[nd_id].time_val.push_back(time_t);
			// Add the displacement magnitude
			node_results[nd_id].displ_magnitude.push_back(displ_magnitude);
			// Add the Normalized displacement
			node_results[nd_id].normalized_displ.push_back(normalized_node_displ);
		}

		r_id++;
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Pulse analysis solved at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Map the results
	map_pulse_analysis_results(pulse_result_nodes,
		pulse_result_lineelements,
		r_id,
		model_nodes,
		model_lineelements,
		node_results);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Save the Pulse analysis settings
	this->time_step_count = r_id;
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

	//____________________________________________________________________________________________________________________
	output_file.close();

}


void pulse_analysis_solver::get_reduced_global_matrix(Eigen::MatrixXd& reducedglobalMatrix,
	const Eigen::MatrixXd& globalMatrix,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF)
{
	// Curtailment of Global Eigen Vector matrix based on DOF
	// Get the reduced global  matrix
	int r = 0;
	int s = 0;

	// Loop throug the Degree of freedom of indices
	for (int i = 0; i < numDOF; i++)
	{
		if (globalDOFMatrix.coeff(i) == 0)
		{
			// constrained row index, so skip
			continue;
		}
		else
		{
			s = 0;
			for (int j = 0; j < numDOF; j++)
			{
				if (globalDOFMatrix.coeff(j) == 0)
				{
					// constrained column index, so skip
					continue;
				}
				else
				{
					// Get the reduced matrices
					reducedglobalMatrix.coeffRef(r, s) = globalMatrix.coeff(i, j);
					s++;
				}
			}
			r++;
		}
	}

}


void pulse_analysis_solver::create_initial_condition_matrices(Eigen::VectorXd& modal_reducedInitialDisplacementMatrix,
	Eigen::VectorXd& modal_reducedInitialVelocityMatrix,
	const nodeinlcond_list_store& node_inlcond,
	const Eigen::VectorXi& globalDOFMatrix,
	const Eigen::MatrixXd& globalSupportInclinationMatrix,
	const Eigen::MatrixXd& reduced_eigenVectorsMatrix,
	const int& numDOF,
	const int& reducedDOF,
	std::ofstream& output_file)
{
	// Create a global initial condition matrix
	Eigen::VectorXd globalInitialDisplacementMatrix(numDOF);
	Eigen::VectorXd globalInitialVelocityMatrix(numDOF);

	globalInitialDisplacementMatrix.setZero();
	globalInitialVelocityMatrix.setZero();

	for (auto& inlc_m : node_inlcond.inlcondMap)
	{
		nodeinl_condition_data inlc = inlc_m.second;

		// get the matrix id
		int n_id = nodeid_map[inlc.node_id]; // get the ordered map of the start node ID

		// Create a node initial displacement matrix
		Eigen::MatrixXd nodeinitialDisplacementMatrix(2, 1);
		nodeinitialDisplacementMatrix.setZero();

		nodeinitialDisplacementMatrix.coeffRef(0, 0) = inlc.inl_displacement_x;
		nodeinitialDisplacementMatrix.coeffRef(1, 0) = (-1.0) * inlc.inl_displacement_y;

		// global initial displacement matrix
		globalInitialDisplacementMatrix.block<2, 1>(n_id * 2, 0) += nodeinitialDisplacementMatrix.block<2, 1>(0, 0);

		// Create a node initial velocity matrix
		Eigen::MatrixXd nodeinitialVelocityMatrix(2, 1);
		nodeinitialVelocityMatrix.setZero();

		nodeinitialVelocityMatrix.coeffRef(0, 0) = inlc.inl_velocity_x;
		nodeinitialVelocityMatrix.coeffRef(1, 0) = (-1.0) * inlc.inl_velocity_y;

		// global initial velocity matrix
		globalInitialVelocityMatrix.block<2, 1>(n_id * 2, 0) += nodeinitialVelocityMatrix.block<2, 1>(0, 0);
	}

	// Apply support inclination transformation to Global initial displacement and velocity matrices
	globalInitialDisplacementMatrix = globalSupportInclinationMatrix.transpose() * globalInitialDisplacementMatrix;
	globalInitialVelocityMatrix = globalSupportInclinationMatrix.transpose() * globalInitialVelocityMatrix;



	// Reduce the intial condition matrix with the degree of freedom 
	Eigen::VectorXd reducedInitialDisplacementMatrix(reducedDOF);
	Eigen::VectorXd reducedInitialVelocityMatrix(reducedDOF);

	reducedInitialDisplacementMatrix.setZero();
	reducedInitialVelocityMatrix.setZero();

	// reduced initial displacement matrix
	get_reduced_global_vector(reducedInitialDisplacementMatrix,
		globalInitialDisplacementMatrix,
		globalDOFMatrix,
		numDOF,
		reducedDOF);

	// reduced initial velocity matrix
	get_reduced_global_vector(reducedInitialVelocityMatrix,
		globalInitialVelocityMatrix,
		globalDOFMatrix,
		numDOF,
		reducedDOF);

	// Inverse the reduced eigen vectors (Can be inversed because its a square matrix)
	Eigen::MatrixXd reduced_eigenVectorsMatrix_inv_matrix = reduced_eigenVectorsMatrix.inverse();

	// apply modal decomposition to the initial displacements & intial velocities
	modal_reducedInitialDisplacementMatrix = reduced_eigenVectorsMatrix_inv_matrix * reducedInitialDisplacementMatrix;
	modal_reducedInitialVelocityMatrix = reduced_eigenVectorsMatrix_inv_matrix * reducedInitialVelocityMatrix;


	if (print_matrix == true)
	{
		// Print the Reduced eigen vectors inverse
		output_file << "Reduced Eigen vectors matrix inverse" << std::endl;
		output_file << reduced_eigenVectorsMatrix_inv_matrix << std::endl;
		output_file << std::endl;

		// Print the Modal Reduced Initial Displacement matrix
		output_file << "Modal Reduced Initial Displacement Matrix" << std::endl;
		output_file << modal_reducedInitialDisplacementMatrix << std::endl;
		output_file << std::endl;

		// Print the Modal Reduced Initial Velocity matrix
		output_file << "Modal Reduced Initial Velocity Matrix" << std::endl;
		output_file << modal_reducedInitialVelocityMatrix << std::endl;
		output_file << std::endl;
	}
}


void pulse_analysis_solver::get_reduced_global_vector(Eigen::VectorXd& reducedglobalVector,
	const Eigen::VectorXd& globalVector,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF)
{
	// Get the reduced global vector with the Degree of freedom
	int r = 0;

	// Loop throug the Degree of freedom of indices
	for (int i = 0; i < numDOF; i++)
	{
		if (globalDOFMatrix(i) == 0)
		{
			// constrained row index, so skip
			continue;
		}
		else
		{
			// Get the reduced vectors
			reducedglobalVector.coeffRef(r) = globalVector.coeff(i);
			r++;
		}
	}
}


void pulse_analysis_solver::get_global_resp_vector(Eigen::VectorXd& globalVector,
	const Eigen::VectorXd& reducedglobalVector,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF)
{
	// Get global response matrix from the reduced matrices
	// Loop throug the Degree of freedom of indices
	int r = 0;

	// Loop throug the Degree of freedom of indices
	for (int i = 0; i < numDOF; i++)
	{
		if (globalDOFMatrix.coeff(i) == 0)
		{
			// constrained row index, so skip
			continue;
		}
		else
		{
			// Get the reduced matrices
			globalVector.coeffRef(i) = reducedglobalVector(r);
			r++;
		}
	}

}



void pulse_analysis_solver::create_pulse_load_matrices(pulse_load_data& pulse_loads,
	const load_data& ld,
	const nodes_list_store& model_nodes,
	const Eigen::VectorXi& globalDOFMatrix,
	const Eigen::MatrixXd& globalSupportInclinationMatrix,
	const Eigen::MatrixXd& reduced_eigenVectorsMatrix_transpose,
	const int& numDOF,
	const int& reducedDOF,
	std::ofstream& output_file)
{
	// Create the global load amplitude matrix
	// Extract the line in which the load is applied
	node_store nd = model_nodes.nodeMap.at(ld.node_id);

	// Get the Matrix row ID
	int n_id = nodeid_map[nd.node_id]; // get the ordered map of the start node ID

	// Create a matrix for element load matrix
	double load_val = ld.load_value; // Load value
	double load_angle_rad = ld.load_angle * (m_pi / 180.0f);

	double f_x = load_val * std::cos(load_angle_rad);
	double f_y = load_val * std::sin(load_angle_rad);

	// global load matrix
	Eigen::VectorXd globalLoadamplMatrix(numDOF);
	globalLoadamplMatrix.setZero();

	globalLoadamplMatrix.coeffRef((n_id * 2) + 0) += f_x;
	globalLoadamplMatrix.coeffRef((n_id * 2) + 1) += f_y;

	//______________________________________________________________________________________________
	// Apply support inclination transformation to Global load matrices
	globalLoadamplMatrix = globalSupportInclinationMatrix.transpose() * globalLoadamplMatrix;

	//______________________________________________________________________________________________
	// Reduce the global load matrix with DOF
	Eigen::VectorXd reducedLoadamplMatrix(reducedDOF);
	reducedLoadamplMatrix.setZero();

	// reduced global load matrix matrix
	get_reduced_global_vector(reducedLoadamplMatrix,
		globalLoadamplMatrix,
		globalDOFMatrix,
		numDOF,
		reducedDOF);

	//______________________________________________________________________________________________
	// Apply modal transformation to the reduced load ampl matrix

	Eigen::VectorXd modal_reducedLoadamplMatrix(reducedDOF);
	modal_reducedLoadamplMatrix.setZero();


	modal_reducedLoadamplMatrix = reduced_eigenVectorsMatrix_transpose * reducedLoadamplMatrix;


	//______________________________________________________________________________________________
	// Copy the data to pulse load data variable
	pulse_loads.load_id = ld.load_id;
	pulse_loads.load_start_time = ld.load_start_time;
	pulse_loads.load_end_time = ld.load_end_time;
	pulse_loads.globalLoadamplMatrix = globalLoadamplMatrix; // Global load matrix for this load
	pulse_loads.reducedLoadamplMatrix = reducedLoadamplMatrix; // Reduced load matrix with constraint matrix
	pulse_loads.modal_reducedLoadamplMatrix = modal_reducedLoadamplMatrix; // Modal reduction applied to load matrix


	if (print_matrix == true)
	{
		// Print the Modal Reduced Load Amplitude matrix
		output_file << "Modal Reduced Load Amplitude Matrix " << std::endl;
		output_file << modal_reducedLoadamplMatrix << std::endl;
		output_file << std::endl;
	}
}


void pulse_analysis_solver::get_steady_state_initial_condition_soln(double& steady_state_displ_resp,
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


void pulse_analysis_solver::get_steady_state_half_sine_pulse_soln(double& steady_state_displ_resp,
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


void pulse_analysis_solver::get_steady_state_rectangular_pulse_soln(double& steady_state_displ_resp,
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


void pulse_analysis_solver::get_steady_state_triangular_pulse_soln(double& steady_state_displ_resp,
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
				double k_fact = ((2.0*modal_force_ampl) /  modal_stiff);

				steady_state_displ_resp = k_fact * ((t_at / t_d)-(std::sin(modal_omega_n * t_at) / (t_d * modal_omega_n)));
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
				double factor2 = std::sin(modal_omega_n * (t_at -  t_d));

				steady_state_displ_resp = k_fact *(m_factor * (factor1 - factor2 - std::sin(modal_omega_n * t_at)));
			
		}
	}

}



void pulse_analysis_solver::get_steady_state_stepforce_finiterise_soln(double& steady_state_displ_resp,
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
				(std::sin(modal_omega_n * t_at)/(t_d * modal_omega_n)));

		}
		else if (time_t > modal_force_endtime)
		{
			// current time is over the force period
			double factor1 = std::sin(modal_omega_n * (t_at - t_d)) - std::sin(modal_omega_n * t_at);

			steady_state_displ_resp = k_fact * (1.0 + (1/(modal_omega_n * t_d)) * factor1);

		}
	}

}


void pulse_analysis_solver::get_total_harmonic_soln(double& steady_state_displ_resp,
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
		bool is_rigid = false;

		if (ln.material_id == 0)
		{
			is_rigid = true;
		}

		pulse_result_lineelements.add_pulse_elementline(ln.line_id, startNode, endNode,is_rigid);
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