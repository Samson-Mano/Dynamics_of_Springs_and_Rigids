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


