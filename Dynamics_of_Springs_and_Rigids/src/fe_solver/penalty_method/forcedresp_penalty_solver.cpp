#include "forcedresp_penalty_solver.h"

forcedresp_penalty_solver::forcedresp_penalty_solver()
{
	// Empty constructor
}

forcedresp_penalty_solver::~forcedresp_penalty_solver()
{
	// Empty destructor
}


void forcedresp_penalty_solver::clear_results()
{
	// Clear the analysis results

}


void forcedresp_penalty_solver::forcedresp_analysis_penaltymethod_start(std::vector<frequency_reponse_data>& frf_data, 
	std::vector<chart_setting_data>& frf_chart_setting, 
	const nodes_list_store& model_nodes, 
	const elementline_list_store& model_lineelements, 
	const nodeconstraint_list_store& node_constraints, 
	const nodeload_list_store& node_loads, 
	const nodepointmass_list_store& node_ptmass, 
	const std::unordered_map<int, material_data>& material_list, 
	const modal_penalty_solver& modal_penalty_s, 
	const std::vector<int> selected_nodes, 
	const double start_frequency, 
	const double end_frequency, 
	const double frequency_interval, 
	const double damping_ratio, 
	const int mode_range_startid, 
	const int mode_range_endid,
	bool& is_forcedresp_analysis_complete)
{
	// Main solver call
	is_forcedresp_analysis_complete = false;

	// Check the model
	// Number of loads (Exit if no load is present)
	if (node_loads.load_count == 0)
	{
		return;
	}

	// No nodes selected
	if (static_cast<int>(selected_nodes.size()) == 0)
	{
		return;
	}

	// Assign the node id map
	this->nodeid_map = modal_penalty_s.nodeid_map;

	//___________________________________________________________________________________
	// Create a file to keep track of frequency response matrices
	std::ofstream output_file;
	output_file.open("forcedresp_analysis_results.txt");


	if (print_matrix == true)
	{
		output_file << "Forced Response Analysis using Penalty method" << std::endl;
		output_file << "___________________________________________________" << std::endl;

	}



	//___________________________________________________________________________________
	// Get the modal vectors (within the range)
	int numDOF = modal_penalty_s.numDOF;
	int k = 0;

	Eigen::MatrixXd global_eigenVectorsMatrix = modal_penalty_s.global_eigenvectors; // Retrive the global EigenVectors matrix 


	if (print_matrix == true)
	{
		// Print the Global eigen vectors 
		output_file << "Global Eigen vectors matrix" << std::endl;
		output_file << global_eigenVectorsMatrix << std::endl;
		output_file << std::endl;
	}


	//____________________________________________________________________________________________________________________
	// Create the Pulse force data for all the individual 
	std::vector<forceresp_load_penalty_data> forcedresp_loads(node_loads.load_count);
	k = 0;

	for (auto& ld_m : node_loads.loadMap)
	{
		load_data ld = ld_m.second; // get the load data

		forcedresp_loads[k].load_id = k;
		forcedresp_loads[k].load_phase = ld.load_phase;
		create_forcedresp_load_matrices(forcedresp_loads[k],
			ld,
			model_nodes,
			global_eigenVectorsMatrix,
			numDOF,
			output_file);

		k++; // iterate load id
	}


	//____________________________________________________________________________________________________________________
	// Forced Response Analysis
	// Empty value for node response data
	frf_data.clear();
	frf_data.resize(static_cast<int>(selected_nodes.size()));
	int j = 0;
	for (const int& nd_id : selected_nodes)
	{
		frf_data[j].node_id = nd_id;
		j++;
	}


	// Fill the frequency data
	double start_frequency_d = start_frequency;

	if (start_frequency == 0.0)
	{
		start_frequency_d += frequency_interval;
	}

	for (double freq_x = start_frequency_d; freq_x <= end_frequency; freq_x += frequency_interval)
	{
		// Phase Response matrix
		Eigen::VectorXd phase_RespMatrix(numDOF);
		phase_RespMatrix.setZero();

		// Displacement amplitude Response Matrix Reduced befor EigenVector Transformation
		Eigen::VectorXd displ_ampl_RespMatrix_b4eig_trans(numDOF);
		displ_ampl_RespMatrix_b4eig_trans.setZero();

		for (int i = mode_range_startid; i < mode_range_endid; i++)
		{
			//_______________________________________________________________________
			double displ_resp_force = 0.0; // Displacement response due to Excitation Force
			double phase_resp = 0.0; // Phase response due to Excitation Force

			// get all the force loads
			for (auto& force_load : forcedresp_loads)
			{
				// Go through all the force
				double at_force_displ_resp = 0.0;
				double at_force_phase = 0.0;

				get_steady_state_harmonic_periodic_soln(at_force_displ_resp,
					at_force_phase,
					modal_penalty_s.modalMass(i),
					modal_penalty_s.modalStiff(i),
					damping_ratio,
					force_load.modal_globalLoadamplMatrix(i),
					freq_x);

				displ_resp_force = displ_resp_force + at_force_displ_resp;
				phase_resp = phase_resp + at_force_phase;
			}

			// Add to the modal displ matrix
			displ_ampl_RespMatrix_b4eig_trans.coeffRef(i) = displ_resp_force;
			phase_RespMatrix.coeffRef(i) = phase_resp;
		}


		// Apply modal de-transformation
		Eigen::VectorXd displ_ampl_RespMatrix(numDOF);
		displ_ampl_RespMatrix.setZero();

		displ_ampl_RespMatrix = global_eigenVectorsMatrix * displ_ampl_RespMatrix_b4eig_trans;


		// Store the results to node results
		j = 0;
		for (const int& nd_id : selected_nodes)
		{
			// get the node index from node id
			int nd_index = nodeid_map[nd_id];

			// Node displacement response
			// X displacement & X Phase
			double displ_x = displ_ampl_RespMatrix.coeff((nd_index * 2) + 0);
			double phase_x = phase_RespMatrix.coeff((nd_index * 2) + 0);

			// Y displacement
			double displ_y = displ_ampl_RespMatrix.coeff((nd_index * 2) + 1);
			double phase_y = phase_RespMatrix.coeff((nd_index * 2) + 1);

			// Displacement magnitude
			double displ_magnitude = std::sqrt((displ_x * displ_x) + (displ_y * displ_y));
			double phase_magnitude = std::atan(displ_y / displ_x);

			// Add to the list
			frf_data[j].frequency_values.push_back(freq_x);
			// X response
			frf_data[j].displ_x.push_back(displ_x);
			frf_data[j].phase_x.push_back(phase_x);

			// Y response
			frf_data[j].displ_y.push_back(displ_y);
			frf_data[j].phase_y.push_back(phase_y);

			// Magnitude response
			frf_data[j].displ_magnitude.push_back(displ_magnitude);
			frf_data[j].phase_magnitude.push_back(phase_magnitude);


			j++; // iterate

		}
	}

	frf_chart_setting.resize(6);

	// Set the chart data
	frf_chart_setting[0].chart_x_min = static_cast<float>(start_frequency_d);
	frf_chart_setting[0].chart_x_max = static_cast<float>(end_frequency + frequency_interval);

	double max_displ_magnitude = DBL_MIN;
	double min_displ_magnitude = DBL_MAX;

	j = 0;
	for (const int& nd_id : selected_nodes)
	{
		//// X response
		//frf_data[j].displ_x;
		//frf_data[j].phase_x;

		//// Y response
		//frf_data[j].displ_y;
		//frf_data[j].phase_y;

		// Magnitude response
		for (const auto& displ_mag : frf_data[j].displ_magnitude)
		{
			max_displ_magnitude = std::max(max_displ_magnitude, displ_mag);
			min_displ_magnitude = std::min(min_displ_magnitude, displ_mag);
		}

		// frf_data[j].phase_magnitude;


		j++; // iterate

	}

	frf_chart_setting[0].chart_y_min = static_cast<float>(min_displ_magnitude);
	frf_chart_setting[0].chart_y_max = static_cast<float>(max_displ_magnitude);

	frf_chart_setting[0].data_pt_count = static_cast<int>(frf_data[0].displ_magnitude.size());


	is_forcedresp_analysis_complete = true;

	//____________________________________________________________________________________________________________________


}


void forcedresp_penalty_solver::create_forcedresp_load_matrices(forceresp_load_penalty_data& forcedresp_loads, 
	const load_data& ld, 
	const nodes_list_store& model_nodes, 
	const Eigen::MatrixXd& global_eigenVectorsMatrix, 
	const int& numDOF, 
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
	// Apply modal transformation to the reduced load ampl matrix
	Eigen::VectorXd modal_globalLoadamplMatrix(numDOF);
	modal_globalLoadamplMatrix.setZero();


	modal_globalLoadamplMatrix = global_eigenVectorsMatrix.transpose() * globalLoadamplMatrix;


	//______________________________________________________________________________________________
	// Copy the data to pulse load data variable
	forcedresp_loads.load_id = ld.load_id;
	forcedresp_loads.modal_globalLoadamplMatrix = modal_globalLoadamplMatrix; // Global load matrix for this load


	if (print_matrix == true)
	{
		// Print the Modal Reduced Load Amplitude matrix
		output_file << "Modal Global Load Amplitude Matrix " << std::endl;
		output_file << modal_globalLoadamplMatrix << std::endl;
		output_file << std::endl;
	}



}


void forcedresp_penalty_solver::get_steady_state_harmonic_periodic_soln(double& steady_state_displ_resp, 
	double& phase_resp, 
	const double& modal_mass, 
	const double& modal_stiff, 
	const double& damping_ratio, 
	const double& modal_force_ampl, 
	const double& forcing_freq)
{
	// Return the steady state response
	double modal_omega_n = std::sqrt(modal_stiff / modal_mass); // Modal omega n
	double forcing_omega = 2.0 * m_pi * forcing_freq;
	double freq_ratio = forcing_omega / modal_omega_n;
	double freq_ratio_sq = std::pow(freq_ratio, 2);

	// Steady state amplitude
	double steady_state_ampl = modal_force_ampl / modal_stiff; // P_0/k

	// Dynamic amplification factor (Response Amplitude Operator)
	double daf = 1.0 / std::sqrt(std::pow((1.0 - freq_ratio_sq), 2) + std::pow((2.0 * damping_ratio * freq_ratio), 2));

	// Phase 
	phase_resp = std::atan((2 * damping_ratio * freq_ratio) / (1.0 - freq_ratio_sq));
	steady_state_displ_resp = steady_state_ampl * daf;

}


