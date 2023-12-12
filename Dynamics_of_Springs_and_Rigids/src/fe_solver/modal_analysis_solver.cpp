#include "modal_analysis_solver.h"

modal_analysis_solver::modal_analysis_solver()
{
	// Empty constructor
}

modal_analysis_solver::~modal_analysis_solver()
{
	// Empty destructor
}


void modal_analysis_solver::clear_results()
{
	// Clear the eigen values and eigen vectors
	number_of_modes = 0;
	nodeid_map.clear();
	eigen_values.clear();
	eigen_vectors.clear();
	eigen_vectors_reduced.clear();
	mode_result_str.clear();
	is_modal_analysis_complete = false;
}

void modal_analysis_solver::modal_analysis_start(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const nodeconstraint_list_store& model_constarints,
	const nodepointmass_list_store& model_ptmass,
	const std::unordered_map<int, material_data>& material_list,
	result_node_list_store& modal_result_nodes,
	result_elementline_list_store& modal_result_lineelements,
	bool& is_modal_analysis_complete)
{
	// Main solver call
	is_modal_analysis_complete = false;

	// Check the model
	// Number of nodes
	if (model_nodes.node_count == 0)
	{
		return;
	}

	// Number of elements
	if (model_lineelements.elementline_count == 0)
	{
		return;
	}

	// Number of point mass
	if (model_ptmass.ptmass_count == 0)
	{
		return;
	}

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Modal analysis started" << std::endl;

	// Create a node ID map (to create a nodes as ordered and numbered from 0,1,2...n)
	int i = 0;
	for (auto& nd : model_nodes.nodeMap)
	{
		nodeid_map[nd.first] = i;
		i++;
	}

	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Node maping completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// set the penalty stiffness for rigid element
	double max_stiffness = 0.0;
	for (const auto& mat_m : material_list)
	{
		if (mat_m.first == 0)
		{
			// First material is rigid with inf stiffness (so skip)
			continue;
		}

		material_data mat = mat_m.second;

		// Find the maximum material stiffness
		if (max_stiffness < mat.material_stiffness)
		{
			max_stiffness = mat.material_stiffness;
			w_penalty = 1000.0 * max_stiffness; // penalty stiffness = 1000 * max stiffness
		}
	}

	// Create a file to keep track of matrices
	std::ofstream output_file;
	output_file.open("modal_analysis_results.txt");

	//____________________________________________________________________________________________________________________
	numDOF = model_nodes.node_count * 2; // Number of degrees of freedom (2 DOFs per node (2 translation))

	// Global Stiffness Matrix
	globalStiffnessMatrix.resize(numDOF, numDOF);
	globalStiffnessMatrix.setZero();

	get_global_stiffness_matrix(globalStiffnessMatrix,
		model_lineelements,
		material_list,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Global Point Mass Matrix
	Eigen::MatrixXd globalPointMassMatrix(numDOF, numDOF);
	globalPointMassMatrix.setZero();

	get_global_pointmass_matrix(globalPointMassMatrix,
		model_nodes,
		model_ptmass,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global point mass matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Global DOF Mass Matrix
	globalDOFMatrix.resize(numDOF, 1);
	globalDOFMatrix.setZero();

	// Determine the size of the reduced stiffness matrix based on the number of unconstrained degrees of freedom
	reducedDOF = 0;

	get_global_dof_matrix(globalDOFMatrix,
		model_nodes,
		model_constarints,
		reducedDOF,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global DOF matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________



}


void modal_analysis_solver::get_global_stiffness_matrix(Eigen::MatrixXd& globalStiffnessMatrix,
	const elementline_list_store& model_lineelements,
	const std::unordered_map<int, material_data>& material_list,
	std::ofstream& output_file)
{
	// Create global stiffness matrix
	for (auto& ln_m : model_lineelements.elementlineMap)
	{
		// Create the element stiffness matrix
		elementline_store ln = ln_m.second;
		material_data elementline_material = material_list.at(ln.material_id);

		// Create a matrix for element stiffness matrix
		Eigen::MatrixXd elementStiffnessMatrix(4, 4);
		elementStiffnessMatrix.setZero();

		get_element_stiffness_matrix(elementStiffnessMatrix, ln, elementline_material, output_file);

		// Get the Node ID
		int sn_id = nodeid_map[ln.startNode->node_id]; // get the ordered map of the start node ID
		int en_id = nodeid_map[ln.endNode->node_id]; // get the ordered map of the end node ID

		globalStiffnessMatrix.block<2, 2>(sn_id * 2, sn_id * 2) += elementStiffnessMatrix.block<2, 2>(0, 0);
		globalStiffnessMatrix.block<2, 2>(sn_id * 2, en_id * 2) += elementStiffnessMatrix.block<2, 2>(0, 2);
		globalStiffnessMatrix.block<2, 2>(en_id * 2, sn_id * 2) += elementStiffnessMatrix.block<2, 2>(2, 0);
		globalStiffnessMatrix.block<2, 2>(en_id * 2, en_id * 2) += elementStiffnessMatrix.block<2, 2>(2, 2);
	}

	if (print_matrix == true)
	{
		// Print the Global Stiffness matrix
		output_file << "Global Stiffness Matrix" << std::endl;
		output_file << globalStiffnessMatrix << std::endl;
		output_file << std::endl;
	}

}


void modal_analysis_solver::get_element_stiffness_matrix(Eigen::MatrixXd& elementStiffnessMatrix,
	const elementline_store& ln,
	const material_data& elementline_material,
	std::ofstream& output_file)
{
	// Get element stiffness matrix
	// Compute the differences in x and y coordinates
	double dx = ln.endNode->node_pt.x - ln.startNode->node_pt.x;
	double dy = -1.0 * (ln.endNode->node_pt.y - ln.startNode->node_pt.y);

	// Compute the length of the truss element
	double eLength = std::sqrt((dx * dx) + (dy * dy));

	// Compute the direction cosines
	double Lcos = (dx / eLength);
	double Msin = (dy / eLength);

	//_________________________________________________________
	// Local -> Global transformation matrix
	Eigen::MatrixXd L_transformation_matrix(2, 4);
	L_transformation_matrix.setZero();

	L_transformation_matrix.row(0) = Eigen::RowVectorXd({ {Lcos, Msin, 0.0, 0.0 } });
	L_transformation_matrix.row(1) = Eigen::RowVectorXd({ {0.0, 0.0,  Lcos, Msin} });

	//_________________________________________________________
	// Local element stiffness matrix
	Eigen::MatrixXd local_element_stiffness_matrix(2, 2);
	local_element_stiffness_matrix.setZero();

	double k1 = 0.0;
	if (elementline_material.material_id == 0)
	{
		// Line is rigid
		k1 = w_penalty; // penalty 
	}
	else
	{
		k1 = elementline_material.material_stiffness;
	}
	
	local_element_stiffness_matrix.row(0) = Eigen::RowVectorXd({ {		 k1, -1.0 * k1 } });
	local_element_stiffness_matrix.row(1) = Eigen::RowVectorXd({ {-1.0 * k1,		k1 } });

	//_________________________________________________________
	// Transformed element stiffness matrix
	elementStiffnessMatrix.setZero();

	elementStiffnessMatrix = L_transformation_matrix.transpose() * local_element_stiffness_matrix * L_transformation_matrix;

}


void modal_analysis_solver::get_global_pointmass_matrix(Eigen::MatrixXd& globalPointMassMatrix,
	const nodes_list_store& model_nodes,
	const nodepointmass_list_store& model_ptmass,
	std::ofstream& output_file)
{
	// Create a global point mass matrix
	for (auto& nd_m : model_nodes.nodeMap)
	{
		// Get the node data
		node_store nd = nd_m.second;
		int nd_map = nodeid_map[nd.node_id]; // get the ordered map of the node ID

		if (model_ptmass.ptmassMap.find(nd.node_id) != model_ptmass.ptmassMap.end())
		{
			// Nodes have point mass
			nodepointmass_data ptm = model_ptmass.ptmassMap.at(nd.node_id);

			globalPointMassMatrix((nd_map * 2) + 0, (nd_map * 2) + 0) = ptm.ptmass_x;
			globalPointMassMatrix((nd_map * 2) + 1, (nd_map * 2) + 1) = ptm.ptmass_y;
		}
		else
		{
			// Nodes doesnt have point mass
			globalPointMassMatrix((nd_map * 2) + 0, (nd_map * 2) + 0) = 0.0;
			globalPointMassMatrix((nd_map * 2) + 1, (nd_map * 2) + 1) = 0.0;
		}
	}

	if (print_matrix == true)
	{
		// Print the Global Force matrix
		output_file << "Global Point Mass Matrix" << std::endl;
		output_file << globalPointMassMatrix << std::endl;
		output_file << std::endl;
	}
}


void modal_analysis_solver::get_global_dof_matrix(Eigen::MatrixXd& globalDOFMatrix,
	const nodes_list_store& model_nodes,
	const nodeconstraint_list_store& model_constarints,
	int& reducedDOF,
	std::ofstream& output_file)
{
	// Create global DOF Matrix
	for (auto& nd_m : model_nodes.nodeMap)
	{
		// Get the node data
		node_store nd = nd_m.second;
		int nd_map = nodeid_map[nd.node_id]; // get the ordered map of the node ID

		if (model_constarints.constraintMap.find(nd.node_id) != model_constarints.constraintMap.end())
		{
			// Nodes have point mass
			constraint_data cd = model_constarints.constraintMap.at(nd.node_id);

			if (cd.constraint_type == 0)
			{
				// Pin End

				globalDOFMatrix.coeffRef((nd_map * 2) + 0, 0) = 0.0;
				globalDOFMatrix.coeffRef((nd_map * 2) + 1, 0) = 0.0;
			}
			else if (cd.constraint_type == 1)
			{
				// Pin Roller end

				globalDOFMatrix.coeffRef((nd_map * 2) + 0, 0) = 1.0; // X is free to move
				globalDOFMatrix.coeffRef((nd_map * 2) + 1, 0) = 0.0;

				reducedDOF = reducedDOF + 1;
			}
		}
		else
		{
			// Nodes doesnt have Constraint
			globalDOFMatrix.coeffRef((nd_map * 2) + 0, 0) = 1.0;
			globalDOFMatrix.coeffRef((nd_map * 2) + 1, 0) = 1.0;

			reducedDOF = reducedDOF + 2;
		}
	}

	if (print_matrix == true)
	{
		// Print the Global Force matrix
		output_file << "Global DOF Matrix" << std::endl;
		output_file << globalDOFMatrix << std::endl;
		output_file << std::endl;
	}

}
