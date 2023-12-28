#include "forcedresp_analysis_solver.h"

forcedresp_analysis_solver::forcedresp_analysis_solver()
{
	// Empty constructor
}

forcedresp_analysis_solver::~forcedresp_analysis_solver()
{
	// Empty destructor
}

void forcedresp_analysis_solver::clear_results()
{
	// Clear the analysis results
	is_forcedresp_analysis_complete = false;
}

void forcedresp_analysis_solver::forcedresp_analysis_start(std::vector<frequency_reponse_data> frf_data,
	const nodes_list_store& model_nodes, 
	const elementline_list_store& model_lineelements, 
	const nodeconstraint_list_store& node_constraints, 
	const nodeload_list_store& node_loads, 
	const nodepointmass_list_store& node_ptmass, 
	const std::unordered_map<int, material_data>& material_list, 
	const modal_analysis_solver& modal_solver, 
	const std::vector<int> selected_nodes,
	const double start_frequency, 
	const double end_frequency, 
	const double frequency_interval, 
	const double damping_ratio, 
	const int mode_range_startid, 
	const int mode_range_endid)
{
	// Main solver call
	this->is_forcedresp_analysis_complete = false;

	// Check the model
	// Number of loads (Exit if no load is present)
	if (node_loads.load_count == 0)
	{
		return;
	}

	// Assign the node id map
	this->nodeid_map = modal_solver.nodeid_map;

	//___________________________________________________________________________________
	// Create a file to keep track of frequency response matrices
	std::ofstream output_file;
	output_file.open("forcedresp_analysis_results.txt");

	//___________________________________________________________________________________
	// Get the modal vectors (within the range)
	int numDOF = modal_solver.numDOF;
	int reducedDOF = modal_solver.reducedDOF;
	int k = 0;

	Eigen::VectorXi globalDOFMatrix = modal_solver.globalDOFMatrix; // retrive the global DOF matrix from the modal solver
	Eigen::MatrixXd globalSupportInclinationMatrix = modal_solver.globalSupportInclinationMatrix; // retrive the global Support Inclination matrix from the modal solver
	Eigen::MatrixXd global_eigenVectorsMatrix = modal_solver.global_eigenvectors_transformed; // Retrive the global EigenVectors matrix 
	Eigen::MatrixXd reduced_eigenVectorsMatrix = modal_solver.reduced_eigenvectors_transformed; // Retrive the reduced EigenVectors matrix 

	if (print_matrix == true)
	{
		// Print the Reduced eigen vectors 
		output_file << "Reduced Eigen vectors matrix" << std::endl;
		output_file << reduced_eigenVectorsMatrix << std::endl;
		output_file << std::endl;
	}

	//____________________________________________________________________________________________________________________
	// Create the Pulse force data for all the individual 
	std::vector<forceresp_load_data> forcedresp_loads(node_loads.load_count);
	k = 0;

	for (auto& ld_m : node_loads.loadMap)
	{
		load_data ld = ld_m.second; // get the load data

		forcedresp_loads[k].load_id = k;
		forcedresp_loads[k].load_phase = ld.load_phase;
		create_forcedresp_load_matrices(forcedresp_loads[k],
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

	//____________________________________________________________________________________________________________________
	// Forced Response Analysis

	// Fill the frequency data
	std::vector<double> forcing_frequency_data;
	double start_frequency_d = start_frequency;

	if (start_frequency == 0.0)
	{
		start_frequency_d += frequency_interval;
	}

	for (double freq_x = start_frequency_d; freq_x <= end_frequency; freq_x += frequency_interval)
	{
		forcing_frequency_data.push_back(freq_x);
	}

	//____________________________________________________________________________________________________________________

	for (const double freq_x : forcing_frequency_data)
	{


	}


}



void forcedresp_analysis_solver::create_forcedresp_load_matrices(forceresp_load_data& forcedresp_loads,
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
	forcedresp_loads.load_id = ld.load_id;
	forcedresp_loads.globalLoadamplMatrix = globalLoadamplMatrix; // Global load matrix for this load
	forcedresp_loads.reducedLoadamplMatrix = reducedLoadamplMatrix; // Reduced load matrix with constraint matrix
	forcedresp_loads.modal_reducedLoadamplMatrix = modal_reducedLoadamplMatrix; // Modal reduction applied to load matrix


	if (print_matrix == true)
	{
		// Print the Modal Reduced Load Amplitude matrix
		output_file << "Modal Reduced Load Amplitude Matrix " << std::endl;
		output_file << modal_reducedLoadamplMatrix << std::endl;
		output_file << std::endl;
	}


}


void forcedresp_analysis_solver::get_reduced_global_vector(Eigen::VectorXd& reducedglobalVector,
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