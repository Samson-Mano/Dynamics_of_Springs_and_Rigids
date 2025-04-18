#include "modal_elim_solver.h"

modal_elim_solver::modal_elim_solver()
{
	// Empty constructor
}

modal_elim_solver::~modal_elim_solver()
{
	// Empty destructor
}


void modal_elim_solver::clear_results()
{
	// Clear the eigen values and eigen vectors
	number_of_modes = 0;
	nodeid_map.clear();
	m_eigenvalues.clear();
	m_eigenvectors.clear();
	numDOF = 0;
	reducedDOF = 0;
	mode_result_str.clear();
}


void modal_elim_solver::modal_analysis_elimmethod_start(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const nodeconstraint_list_store& node_constraints,
	const nodepointmass_list_store& node_ptmass,
	const std::unordered_map<int, material_data>& material_list,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements,
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
	if (node_ptmass.ptmass_count == 0)
	{
		return;
	}

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Modal analysis - Elimination method started" << std::endl;

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
	this->max_stiffness = 0.0;
	for (const auto& mat_m : material_list)
	{
		if (mat_m.first == 0)
		{
			// First material is rigid with inf stiffness (so skip)
			continue;
		}

		material_data mat = mat_m.second;

		// Find the maximum material stiffness
		if (this->max_stiffness < mat.material_stiffness)
		{
			this->max_stiffness = mat.material_stiffness;
		}
	}



	//____________________________________________________________________________________________________________________
	// set the penalty mass for zero mass node
	double min_pointmass = DBL_MAX;
	for (const auto& pt_m : node_ptmass.ptmassMap)
	{
		nodepointmass_data pt_mass = pt_m.second;

		// Find the minimum point mass
		if (min_pointmass > pt_mass.ptmass_x && pt_mass.ptmass_x !=0)
		{
			min_pointmass = pt_mass.ptmass_x;
			zero_ptmass = min_pointmass * (1.0 / penalty_scale_factor);
		}

		if (min_pointmass > pt_mass.ptmass_y && pt_mass.ptmass_y != 0)
		{
			min_pointmass = pt_mass.ptmass_y;
			zero_ptmass = min_pointmass * (1.0 / penalty_scale_factor);
		}

	}



	// Create a file to keep track of matrices
	std::ofstream output_file;
	output_file.open("modal_analysis_results.txt");

	if (print_matrix == true)
	{
		output_file << "Modal Analysis using Elimination method" << std::endl;
		output_file << "____________________________________________" << std::endl;

	}


	//____________________________________________________________________________________________________________________
	numDOF = model_nodes.node_count * 2; // Number of degrees of freedom (2 DOFs per node (2 translation))

	// Global Stiffness Matrix
	Eigen::MatrixXd	globalStiffnessMatrix_R(numDOF, numDOF);
	globalStiffnessMatrix_R.setZero();

	get_global_stiffness_matrix(globalStiffnessMatrix_R,
		model_lineelements,
		node_constraints,
		material_list,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//_____________________________________________________________________________________________
	// Create global support inclination matrix

	globalSupportInclinationMatrix.resize(numDOF, numDOF);
	globalSupportInclinationMatrix.setZero();

	get_globalSupportInclinationMatrix(globalSupportInclinationMatrix,
		model_nodes,
		node_constraints,
		numDOF,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global support inclination matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//_____________________________________________________________________________________________
	// Transform the global stiffness matrix with the global support inclination matrix

	Eigen::MatrixXd globalStiffnessMatrix = globalSupportInclinationMatrix.transpose() * globalStiffnessMatrix_R * globalSupportInclinationMatrix;


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness matrix transformed for support inclination at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	if (print_matrix == true)
	{
		// Print the Global Stiffness Matrix transformed
		output_file << "Global Stiffness Matrix transformed for support inclination" << std::endl;
		output_file << globalStiffnessMatrix << std::endl;
		output_file << std::endl;
	}


	//____________________________________________________________________________________________________________________
	// Global Point Mass Matrix
	Eigen::MatrixXd	globalPointMassMatrix_R(numDOF, numDOF);
	globalPointMassMatrix_R.setZero();

	get_global_pointmass_matrix(globalPointMassMatrix_R,
		model_nodes,
		node_ptmass,
		output_file);


	//_____________________________________________________________________________________________
	// Transform the global point mass matrix with the global support inclination matrix

	Eigen::MatrixXd globalPointMassMatrix = globalSupportInclinationMatrix.transpose() * globalPointMassMatrix_R * globalSupportInclinationMatrix;

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global point mass matrix transformed for support inclination at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	if (print_matrix == true)
	{
		// Print the Global Point Mass Matrix transformed
		output_file << "Global Point Mass Matrix transformed for support inclination" << std::endl;
		output_file << globalPointMassMatrix << std::endl;
		output_file << std::endl;
	}

	//____________________________________________________________________________________________________________________
	// Global DOF Mass Matrix
	globalDOFMatrix.resize(numDOF);
	globalDOFMatrix.setZero();

	// Determine the size of the reduced stiffness matrix based on the number of unconstrained degrees of freedom
	reducedDOF = 0;

	get_global_dof_matrix(globalDOFMatrix,
		globalPointMassMatrix,
		model_nodes,
		node_constraints,
		numDOF,
		reducedDOF,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global DOF matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Create Reduced Global Mass and stiffness matrix
	// Reduced Global stiffness matrix
	Eigen::MatrixXd reduced_globalStiffnessMatrix(reducedDOF, reducedDOF);
	reduced_globalStiffnessMatrix.setZero();

	// Reduced Global Mass matrix
	Eigen::MatrixXd reduced_globalPointMassMatrix(reducedDOF, reducedDOF);
	reduced_globalPointMassMatrix.setZero();

	//// Reduced Global Support Inclination matrix
	//Eigen::MatrixXd reduced_globalSupportInclinationMatrix(reducedDOF, reducedDOF);
	//reduced_globalSupportInclinationMatrix.setZero();


	get_reduced_global_matrices(reduced_globalStiffnessMatrix,
		reduced_globalPointMassMatrix,
		globalStiffnessMatrix,
		globalPointMassMatrix,
		globalSupportInclinationMatrix,
		globalDOFMatrix,
		numDOF,
		reducedDOF,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness, Global Point mass matrices are reduced at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//___________________________________________________________________________________________________________________
	// Convert generalized eigen value problem -> standard eigen value problem
	// Find the inverse square root of diagonal mass matrix (which is the inverse of cholesky decomposition L-matrix)

	Eigen::MatrixXd reduced_invsqrt_globalPointMassMatrix(reducedDOF, reducedDOF);
	reduced_invsqrt_globalPointMassMatrix.setZero();

	get_invsqrt_PointMassMatrix(reduced_invsqrt_globalPointMassMatrix,
		reduced_globalPointMassMatrix,
		reducedDOF,
		output_file);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Inverse Squareroot Point mass matrices completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//___________________________________________________________________________________________________________________
	// Standard eigen value problem

	Eigen::MatrixXd Z_matrix(reducedDOF, reducedDOF);
	Z_matrix.setZero();

	Z_matrix = reduced_invsqrt_globalPointMassMatrix * reduced_globalStiffnessMatrix * reduced_invsqrt_globalPointMassMatrix.transpose();


	if (print_matrix == true)
	{
		// Print the Standard Eigen Value problem
		output_file << "Standard Eigen Value Z - matrix:" << std::endl;
		output_file << Z_matrix << std::endl;
		output_file << std::endl;

	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Standard Eigenvalue problem Z_matrix created at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//___________________________________________________________________________________________________________________
	// Solve the Eigen value problem: Compute the eigenvalues and eigenvectors
	Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigenSolver(Z_matrix);

	if (eigenSolver.info() != Eigen::Success) {
		// Eigenvalue problem failed to converge
		std::cout << "Eigenvalue problem failed to converge !!!!! " << std::endl;
		output_file << "Eigenvalue problem failed to converge !!!!! " << std::endl;
		output_file.close();
		return;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen value problem solved at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Get the eigenvalues and eigenvectors
	Eigen::VectorXd eigenvalues = eigenSolver.eigenvalues(); // Eigenvalues
	Eigen::MatrixXd eigenvectors_reduced = reduced_invsqrt_globalPointMassMatrix.transpose() * eigenSolver.eigenvectors(); // Eigenvectors

	// sort the eigen value and eigen vector (ascending)
	sort_eigen_values_vectors(eigenvalues, eigenvectors_reduced, reducedDOF);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen values and Eigen vectors are sorted at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	// Normailize eigen vectors
	normalize_eigen_vectors(eigenvectors_reduced, reducedDOF);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen vectors are normalized at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	if (print_matrix == true)
	{
		// Eigenvalue problem solve successful
		output_file << "Modal Analysis Success !!!!! " << std::endl;

		// Print the Eigen values
		output_file << "Eigen Values " << std::endl;
		output_file << eigenvalues << std::endl;
		output_file << std::endl;

		// Print the Eigen vectors
		output_file << "Eigen Vectors of the Reduced matrix " << std::endl;
		output_file << eigenvectors_reduced << std::endl;
		output_file << std::endl;
	}

	////____________________________________________________________________________________________________________________
	//// Transform the eigenvalues with the support inclination matrix
	//reduced_eigenvectors_transformed.resize(reducedDOF, reducedDOF);
	//reduced_eigenvectors_transformed.setZero();

	//reduced_eigenvectors_transformed = eigenvectors_reduced; // reduced_globalSupportInclinationMatrix * eigenvectors_reduced;

	reduced_eigenvectors = eigenvectors_reduced;


	//____________________________________________________________________________________________________________________
	// Convert the reduced transformed eigenvectors to eigen vectors for the whole model (including the nodes with supports)
	global_eigenvectors.resize(numDOF, reducedDOF);
	global_eigenvectors.setZero();

	get_globalized_eigen_vector_matrix(global_eigenvectors, eigenvectors_reduced, globalDOFMatrix, numDOF,reducedDOF, output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen vectors globalized at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	global_eigenvectors_transformed = globalSupportInclinationMatrix * global_eigenvectors;


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen vectors transformed with global support inclination matrix " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	//_____________________________________________________________________________________________
	// Calculate the effective mass participation factor & Cummulative effective mass participation factor
	// effective mass participation factor = percentage of the system mass that participates in a particular mode

	Eigen::VectorXd participation_factor(reducedDOF);
	participation_factor.setZero();

	get_modal_participation_factor(participation_factor,
		globalPointMassMatrix,
		global_eigenvectors_transformed,
		numDOF,
		reducedDOF,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal Mass of Participation Factor completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Store the results

	// Clear the modal results
	number_of_modes = 0; // Number of modes
	mode_result_str.clear(); // Result string list
	m_eigenvalues.clear(); // Eigen values
	m_eigenvectors.clear(); // Eigen vectors

	// Add the eigen values and eigen vectors
	for (int i = 0; i < reducedDOF; i++)
	{
		std::vector<double> eigen_vec; // Eigen vectors of all nodes (including constrainded)

		for (int j = 0; j < numDOF; j++)
		{
			eigen_vec.push_back(global_eigenvectors_transformed.coeff(j, i));
		}

		// Add to the Eigen values storage
		m_eigenvalues.insert({ i, eigenvalues.coeff(i) });

		// Add to the Eigen vectors storage 
		m_eigenvectors.insert({ i, eigen_vec });

		// Frequency
		double nat_freq = std::sqrt(eigenvalues.coeff(i)) / (2.0 * m_pi);

		// Modal results
		std::stringstream ss, mf;
		ss << std::fixed << std::setprecision(3) << nat_freq;
		mf << std::fixed << std::setprecision(3) << participation_factor.coeff(i);

		// Add to the string list
		mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz , Modal mass = " + mf.str());
	}

	number_of_modes = reducedDOF; // total number of modes


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen values/ vectors stored at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	is_modal_analysis_complete = true;

	//_____________________________________________________________________________________________

	// Add the modal analysis results to node & element
	// Clear the modal node and modal element results
	modal_result_nodes.clear_data();
	modal_result_lineelements.clear_data();

	map_modal_analysis_results(model_nodes,
		model_lineelements,
		modal_result_nodes,
		modal_result_lineelements,
		output_file);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal analysis results maped to nodes and elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	//____________________________________________________________________________________________________________________
	// Modal Decomposition
	// Create modal matrices
	reduced_modalMass.resize(reducedDOF);
	reduced_modalStiff.resize(reducedDOF);

	get_modal_matrices(reduced_modalMass,
		reduced_modalStiff,
		eigenvectors_reduced,
		reduced_globalPointMassMatrix,
		reduced_globalStiffnessMatrix,
		reducedDOF,
		output_file);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal mass and stiffness storage completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	std::cout << "Modal analysis complete " << std::endl;

	//____________________________________________________________________________________________________________________
	stopwatch.stop();

	output_file.close();

}


void modal_elim_solver::get_global_stiffness_matrix(Eigen::MatrixXd& globalStiffnessMatrix,
	const elementline_list_store& model_lineelements,
	const nodeconstraint_list_store& node_constraints,
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

		get_element_stiffness_matrix(elementStiffnessMatrix, ln, node_constraints, elementline_material, output_file);

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


void modal_elim_solver::get_element_stiffness_matrix(Eigen::MatrixXd& elementStiffnessMatrix,
	const elementline_store& ln,
	const nodeconstraint_list_store& node_constraints,
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
		k1 = penalty_scale_factor * max_stiffness; // Rigid stiffness
	}
	else
	{
		k1 = elementline_material.material_stiffness;
	}

	local_element_stiffness_matrix.row(0) = Eigen::RowVectorXd({ {		 k1, -1.0 * k1 } });
	local_element_stiffness_matrix.row(1) = Eigen::RowVectorXd({ {-1.0 * k1,		k1 } });

	//_________________________________________________________
	// Transformed element stiffness matrix
	elementStiffnessMatrix = L_transformation_matrix.transpose() * local_element_stiffness_matrix * L_transformation_matrix;


}



void modal_elim_solver::get_globalSupportInclinationMatrix(Eigen::MatrixXd& globalSupportInclinationMatrix,
	const nodes_list_store& model_nodes,
	const nodeconstraint_list_store& node_constraints,
	const int& numDOF,
	std::ofstream& output_file)
{
	//globalSupportInclinationMatrix.setIdentity();

	//return;

	// Create the global support inclination matrix
	int node_id = 0;

	// Transform the Nodal results with support inclination
	double constraint_angle_rad = 0.0;
	double support_Lcos = 0.0;
	double support_Msin = 0.0;

	for (auto& nd_m : model_nodes.nodeMap)
	{
		node_id = nd_m.first;
		int matrix_index = nodeid_map[node_id];


		if (node_constraints.constraintMap.find(node_id) == node_constraints.constraintMap.end())
		{
			// No constraint is in this node
			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 0, (matrix_index * 2) + 0) = 1.0;
			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 0, (matrix_index * 2) + 1) = 0.0;

			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 1, (matrix_index * 2) + 0) = 0.0;
			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 1, (matrix_index * 2) + 1) = 1.0;
		}
		else
		{
			// Constraint present in this node
			constraint_angle_rad = (node_constraints.constraintMap.at(node_id).constraint_angle - 90.0f) * (m_pi / 180.0f); // Constrint angle in radians
			support_Lcos = std::cos(constraint_angle_rad); // cosine of support inclination
			support_Msin = std::sin(constraint_angle_rad); // sine of support inclination

			// Pin or Roller Support
			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 0, (matrix_index * 2) + 0) = support_Lcos;
			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 0, (matrix_index * 2) + 1) = -1.0 * support_Msin;

			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 1, (matrix_index * 2) + 0) = support_Msin;
			globalSupportInclinationMatrix.coeffRef((matrix_index * 2) + 1, (matrix_index * 2) + 1) = support_Lcos;
		}
	}

	if (print_matrix == true)
	{
		// Print the Global support inclination matrix
		output_file << "Global Support Inclination Matrix" << std::endl;
		output_file << globalSupportInclinationMatrix << std::endl;
		output_file << std::endl;
	}

}


void modal_elim_solver::get_global_pointmass_matrix(Eigen::MatrixXd& globalPointMassMatrix,
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
			globalPointMassMatrix((nd_map * 2) + 0, (nd_map * 2) + 0) = zero_ptmass;
			globalPointMassMatrix((nd_map * 2) + 1, (nd_map * 2) + 1) = zero_ptmass;
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


void modal_elim_solver::get_global_dof_matrix(Eigen::VectorXi& globalDOFMatrix,
	const Eigen::MatrixXd& globalPointMassMatrix,
	const nodes_list_store& model_nodes,
	const nodeconstraint_list_store& model_constarints,
	const int& numDOF,
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
			// Nodes have constraints
			constraint_data cd = model_constarints.constraintMap.at(nd.node_id);

			if (cd.constraint_type == 0)
			{
				// Pin End (0.0 = Fixed)

				globalDOFMatrix.coeffRef((nd_map * 2) + 0) = 0;
				globalDOFMatrix.coeffRef((nd_map * 2) + 1) = 0;
			}
			else if (cd.constraint_type == 1)
			{
				// Pin Roller end

				globalDOFMatrix.coeffRef((nd_map * 2) + 0) = 1; // X is free to move
				globalDOFMatrix.coeffRef((nd_map * 2) + 1) = 0;

				reducedDOF = reducedDOF + 1;
			}
		}
		else
		{
			// Nodes doesnt have Constraint (1.0 = Free)
			globalDOFMatrix.coeffRef((nd_map * 2) + 0) = 1;
			globalDOFMatrix.coeffRef((nd_map * 2) + 1) = 1;

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


void modal_elim_solver::get_global_augmentation_matrix(Eigen::MatrixXd& globalAGMatrix,
	const Eigen::MatrixXd& globalSupportInclinationMatrix,
	const nodeconstraint_list_store& model_constarints,
	const elementline_list_store& model_lineelements,
	const std::unordered_map<int, material_data>& material_list,
	const int& numDOF,
	int& agDOF,
	std::ofstream& output_file)
{
	// Create global Augmentation matrix
	Eigen::VectorXd row_augmentation_matrix(numDOF);

	// Rigid element Augmentation
	for (auto& ln_m : model_lineelements.elementlineMap)
	{
		// Create the element stiffness matrix
		elementline_store ln = ln_m.second;
		material_data elementline_material = material_list.at(ln.material_id);

		if (elementline_material.material_id == 0)
		{
			// Rigid element
			row_augmentation_matrix.setZero(); //set zero

			// Compute the differences in x and y coordinates
			double dx = ln.endNode->node_pt.x - ln.startNode->node_pt.x;
			double dy = -1.0 * (ln.endNode->node_pt.y - ln.startNode->node_pt.y);

			// Compute the length of the truss element
			double eLength = std::sqrt((dx * dx) + (dy * dy));

			// Compute the direction cosines
			double Lcos = (dx / eLength);
			double Msin = (dy / eLength);

			// Get the Node ID's of the Rigid element
			int sn_id = nodeid_map[ln.startNode->node_id]; // get the ordered map of the start node ID
			int en_id = nodeid_map[ln.endNode->node_id]; // get the ordered map of the end node ID

			// Rigid element's (First node)
			row_augmentation_matrix.coeffRef((sn_id * 2) + 0) = Lcos;
			row_augmentation_matrix.coeffRef((sn_id * 2) + 1) = Msin;

			// Rigid element's (Second node)
			row_augmentation_matrix.coeffRef((en_id * 2) + 0) = -1.0 * Lcos;
			row_augmentation_matrix.coeffRef((en_id * 2) + 1) = -1.0 * Msin;

			// Append to the global matrix
			globalAGMatrix.conservativeResize(globalAGMatrix.rows() + 1, Eigen::NoChange);
			globalAGMatrix.row(globalAGMatrix.rows() - 1) = row_augmentation_matrix;

			// Add to the AG DOF
			agDOF = agDOF + 1;
		}
	}

	globalAGMatrix = globalAGMatrix * globalSupportInclinationMatrix;

	if (print_matrix == true)
	{
		// Print the Global Augmentation matrix
		output_file << "Global Augmentation Matrix" << std::endl;
		output_file << globalAGMatrix << std::endl;
		output_file << std::endl;
	}

}


void modal_elim_solver::get_reduced_global_matrices(Eigen::MatrixXd& reduced_globalStiffnessMatrix,
	Eigen::MatrixXd& reduced_globalPointMassMatrix,
	const Eigen::MatrixXd& globalStiffnessMatrix,
	const Eigen::MatrixXd& globalPointMassMatrix,
	const Eigen::MatrixXd& globalSupportInclinationMatrix,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF,
	std::ofstream& output_file)
{
	// Curtailment of Global stiffness and Global Point mass matrix based on DOF
	// Get the reduced global stiffness matrix
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
					reduced_globalStiffnessMatrix.coeffRef(r, s) = globalStiffnessMatrix.coeff(i, j);
					reduced_globalPointMassMatrix.coeffRef(r, s) = globalPointMassMatrix.coeff(i, j);
					// reduced_globalSupportInclinationMatrix.coeffRef(r, s) = globalSupportInclinationMatrix.coeff(i, j);
					s++;
				}
			}
			r++;
		}
	}

	if (print_matrix == true)
	{
		// Print the Reduced Global Stiffness and Reduced Global Mass matrix
		output_file << "Reduced Global Stiffness Matrix" << std::endl;
		output_file << reduced_globalStiffnessMatrix << std::endl;
		output_file << std::endl;

		output_file << "Reduced Global Mass Matrix" << std::endl;
		output_file << reduced_globalPointMassMatrix << std::endl;
		output_file << std::endl;

	}

}

void modal_elim_solver::get_reduced_global_augmentation_matrices(Eigen::MatrixXd& reduced_globalAGMatrix,
	const Eigen::MatrixXd& globalAGMatrix,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& agDOF,
	std::ofstream& output_file)
{
	int r = 0;

	// Reduce the augmentation matrix
	for (int i = 0; i < numDOF; i++)
	{
		if (globalDOFMatrix.coeff(i) == 1)
		{
			for (int j = 0; j < agDOF; j++)
			{
				reduced_globalAGMatrix.coeffRef(j, r) = globalAGMatrix.coeff(j, i);
			}
			r++;
		}
	}

	if (print_matrix == true)
	{
		output_file << "Reduced Global Augmentation Matrix" << std::endl;
		output_file << reduced_globalAGMatrix << std::endl;
		output_file << std::endl;
	}

}


void modal_elim_solver::get_invsqrt_PointMassMatrix(Eigen::MatrixXd& reduced_invsqrt_globalPointMassMatrix,
	const Eigen::MatrixXd& reduced_globalPointMassMatrix,
	const int& reducedDOF,
	std::ofstream& output_file)
{
	// Return the inverse square root of Point Mass matrix
	for (int i = 0; i < reducedDOF; i++)
	{
		reduced_invsqrt_globalPointMassMatrix.coeffRef(i, i) = (1.0 / sqrt(reduced_globalPointMassMatrix.coeff(i, i)));
	}


	if (print_matrix == true)
	{
		// Print the Reduced Inverse Square Root Point Mass Matrix
		output_file << "Reduced Inverse Square Root Point Mass Matrix" << std::endl;
		output_file << reduced_invsqrt_globalPointMassMatrix << std::endl;
		output_file << std::endl;

	}

}

void modal_elim_solver::get_standardEigenValueProblem(Eigen::MatrixXd& Z_matrix,
	Eigen::MatrixXd& conversion_PointMassMatrix,
	const Eigen::MatrixXd& reduced_globalStiffnessMatrix,
	const Eigen::MatrixXd& reduced_globalAGMatrix,
	const Eigen::MatrixXd& reduced_invsqrt_globalPointMassMatrix,
	const int& reducedDOF,
	const int& agDOF,
	std::ofstream& output_file)
{
	// Create a Large value matrix
	Eigen::MatrixXd large_value_matrix(reducedDOF, reducedDOF);
	large_value_matrix.setIdentity();
	large_value_matrix *= largeValue;

	// Get the Converted Stiffness Matrix
	Eigen::MatrixXd Stiffness_topleftCorner = reduced_invsqrt_globalPointMassMatrix * reduced_globalStiffnessMatrix * reduced_invsqrt_globalPointMassMatrix;


	// Get the Converted Constraint matrix
	Eigen::MatrixXd Constraint_bottomleftCorner = reduced_globalAGMatrix * (large_value_matrix * reduced_invsqrt_globalPointMassMatrix);


	// Get the Converted Constraint matrix transpose
	Eigen::MatrixXd Constraint_toprightCorner = (large_value_matrix * reduced_invsqrt_globalPointMassMatrix) * reduced_globalAGMatrix.transpose();


	//___________________________________________________________________________________________
	// Create the Standard Eigenvalue problem

	Z_matrix.topLeftCorner(reducedDOF, reducedDOF) = Stiffness_topleftCorner;
	Z_matrix.bottomLeftCorner(agDOF, reducedDOF) = Constraint_bottomleftCorner;
	Z_matrix.topRightCorner(reducedDOF, agDOF) = Constraint_toprightCorner;
	Z_matrix.bottomRightCorner(agDOF, agDOF).setIdentity();

	Z_matrix.bottomRightCorner(agDOF, agDOF) *= (smallValue * smallValue);

	//___________________________________________________________________________________________
	// Create the Conversion Point Mass Matrix
	conversion_PointMassMatrix.topLeftCorner(reducedDOF, reducedDOF) = reduced_invsqrt_globalPointMassMatrix;
	conversion_PointMassMatrix.bottomRightCorner(agDOF, agDOF).setIdentity();
	conversion_PointMassMatrix.bottomRightCorner(agDOF, agDOF) *= largeValue;

	if (print_matrix == true)
	{
		// Print the Standard Eigen Value Problem A - Matrix
		output_file << "Standard Eigen Value Problem A - Matrix " << std::endl;
		output_file << Z_matrix << std::endl;
		output_file << std::endl;

		// Print the Conversion Point Mass Matrix
		output_file << "Conversion inverse Point Mass Matrix " << std::endl;
		output_file << conversion_PointMassMatrix << std::endl;
		output_file << std::endl;
	}


}

void modal_elim_solver::get_augmented_Z_matrix(Eigen::MatrixXd& augmented_Z_Matrix,
	const Eigen::MatrixXd& Z_Matrix,
	const Eigen::MatrixXd& reduced_globalAGMatrix,
	const int& reducedDOF,
	const int& agDOF,
	std::ofstream& output_file)
{
	// Augment the global Z - Matrix

	// Copy globalStiffnessMatrix to the top-left block of augmented Z-matrix
	augmented_Z_Matrix.topLeftCorner(reducedDOF, reducedDOF) = Z_Matrix;

	// Copy globalAGMatrix to the bottom-left block of augmented Z-matrix
	augmented_Z_Matrix.bottomLeftCorner(agDOF, reducedDOF) = reduced_globalAGMatrix;

	// Copy the transpose of globalAGMatrix to the top-right block of augmented Z-matrix
	augmented_Z_Matrix.topRightCorner(reducedDOF, agDOF) = reduced_globalAGMatrix.transpose();

	// Fill the bottom-right block of augmented Z-matrix with zeros
	augmented_Z_Matrix.bottomRightCorner(agDOF, agDOF).setZero();


	if (print_matrix == true)
	{
		// Print the Augmented Global Z-matrix
		output_file << "Augmented Global Z Matrix" << std::endl;
		output_file << augmented_Z_Matrix << std::endl;
		output_file << std::endl;
	}

}

void modal_elim_solver::remove_augmentation(Eigen::VectorXd& eigenvalues,
	Eigen::MatrixXd& eigenvectors,
	const Eigen::VectorXd& eigenvalues_SEVP,
	const Eigen::MatrixXd& eigenvectors_SEVP,
	const int& reducedDOF,
	const int& agDOF,
	bool& augmentation_removal_failed,
	std::ofstream& output_file)
{
	// Remove eigenvalues of augmented matrix
	int r = 0;
	for (int i = agDOF; i < reducedDOF + agDOF; i++)
	{
		// Eigen values are negative (and at the first two rows)
		eigenvalues.coeffRef(r) = eigenvalues_SEVP.coeff(i);
		r++;

		if (eigenvalues_SEVP.coeff(i) < 0.0)
		{
			augmentation_removal_failed = true;
		}
	}

	// Remove eigenvectors of augmented matrix
	r = 0;
	for (int i = agDOF; i < reducedDOF + agDOF; i++)
	{
		for (int j = 0; j < reducedDOF; j++)
		{
			eigenvectors.coeffRef(j, r) = eigenvectors_SEVP.coeff(j, i);
		}
		r++;
	}

	if (print_matrix == true)
	{
		// Print the Augmented removed Eigen Values
		output_file << "Augmentation removed Eigen Values" << std::endl;
		output_file << eigenvalues << std::endl;
		output_file << std::endl;

		// Print the Augmented removed Eigen Vectors
		output_file << "Augmentation removed Eigen Vectors" << std::endl;
		output_file << eigenvectors << std::endl;
		output_file << std::endl;
	}


}



void modal_elim_solver::sort_eigen_values_vectors(Eigen::VectorXd& eigenvalues,
	Eigen::MatrixXd& eigenvectors,
	const int& m_size)
{
	int p = 0;
	int q = 0;
	int i = 0;

	double swap_temp = 0.0;

	// sort the eigen value and eigen vector (ascending)
	for (p = 0; p < m_size; p++)
	{
		for (q = p + 1; q < m_size; q++)
		{
			if (eigenvalues(p) > eigenvalues(q))
			{
				swap_temp = eigenvalues(p);
				eigenvalues(p) = eigenvalues(q);
				eigenvalues(q) = swap_temp;

				for (i = 0; i < m_size; i++)
				{
					swap_temp = eigenvectors(i, p);
					eigenvectors(i, p) = eigenvectors(i, q);
					eigenvectors(i, q) = swap_temp;
				}
			}
		}
	}
}


void modal_elim_solver::normalize_eigen_vectors(Eigen::MatrixXd& eigenvectors,
	const int& m_size)
{
	// Normalize eigen vectors
	int p = 0;
	int q = 0;

	// loop throught each column
	for (p = 0; p < m_size; p++)
	{
		double max_modal_vector = 0.0;

		// Loop through each row
		for (q = 0; q < m_size; q++)
		{
			if (std::abs(eigenvectors(q, p)) > max_modal_vector)
			{
				// Max modal vector in the column (for particular mode)
				max_modal_vector = std::abs(eigenvectors(q, p));
			}
		}

		// Normalize the column using maximum modal vector
		for (q = 0; q < m_size; q++)
		{
			eigenvectors(q, p) = eigenvectors(q, p) / max_modal_vector;

			// Round the eigen vectors to 6 digit precision after normalizing
			eigenvectors(q, p) = std::round(eigenvectors(q, p) * 1000000) / 1000000;
		}
	}


}



void modal_elim_solver::get_globalized_eigen_vector_matrix(Eigen::MatrixXd& global_eigenvectors,
	const Eigen::MatrixXd& reduced_eigenvectors,
	const Eigen::VectorXi& globalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF,
	std::ofstream& output_file)
{
	// Global eigen vector Matrix
	// Loop throug the Degree of freedom of indices

	// J loops through number of modes (along the column)
	for (int j = 0; j < reducedDOF; j++)
	{
		int s = 0;
		// i loops through the number of nodes (along the row)
		for (int i = 0; i < numDOF; i++)
		{
			if (globalDOFMatrix.coeff(i) == 0)
			{
				// constrained row index, so Displacement is Zero
				global_eigenvectors.coeffRef(i, j) = 0;
			}
			else
			{
				// Un constrained row index, so Displacement is Zero
				global_eigenvectors.coeffRef(i, j) = reduced_eigenvectors.coeff(s, j);
				s++;
			}
		}
	}

	if (print_matrix == true)
	{
		// Print the Global Displacement matrix
		output_file << "Global Eigen Vector Matrix" << std::endl;
		output_file << global_eigenvectors << std::endl;
		output_file << std::endl;
	}

}

void modal_elim_solver::get_modal_participation_factor(Eigen::VectorXd& participation_factor,
	const Eigen::MatrixXd& globalPointMassMatrix,
	const Eigen::MatrixXd& global_eigenvectors_transformed,
	const int& numDOF,
	const int& reducedDOF,
	std::ofstream& output_file)
{
	// Global Participation Factor

	// Influence vector
	Eigen::VectorXd influence_vector(numDOF);
	influence_vector.setOnes();


	double temp_modal_mass = 0.0;
	double temp_distribution_factor = 0.0;
	double temp_participation_factor = 0.0;
	double total_participation_factor = 0.0;

	double total_mass = influence_vector.transpose() * globalPointMassMatrix * influence_vector;

	for (int i = 0; i < reducedDOF; i++)
	{
		// Get the nth Mode (Column)
		Eigen::VectorXd eigen_vector_i = global_eigenvectors_transformed.col(i);

		// Modal Mass
		temp_modal_mass = eigen_vector_i.transpose() * globalPointMassMatrix * eigen_vector_i;

		// Force distribution factor
		temp_distribution_factor = eigen_vector_i.transpose() * globalPointMassMatrix * influence_vector;

		// Add to the Participation factor
		temp_participation_factor = std::pow(temp_distribution_factor, 2) / temp_modal_mass;

		participation_factor.coeffRef(i) = temp_participation_factor;

		// Cummulative participation factor
		total_participation_factor += temp_participation_factor;

	}

	if (print_matrix == true)
	{
		// Print the Global ParticipationFactors
		output_file << "Global Participation Factors" << std::endl;
		output_file << participation_factor << std::endl;
		output_file << std::endl;

		output_file << "Cumulative Participation Factors" << std::endl;
		output_file << total_participation_factor << std::endl;
		output_file << std::endl;
	}

}

void modal_elim_solver::map_modal_analysis_results(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements,
	std::ofstream& output_file)
{
	// Map the results to modal_result_nodes and modal_result_lineelements

	for (auto& nd_m : model_nodes.nodeMap)
	{
		int node_id = nd_m.first;
		int matrix_index = nodeid_map[node_id];

		// Modal analysis results
		std::unordered_map<int, glm::vec2> node_modal_displ;

		for (int i = 0; i < number_of_modes; i++)
		{
			// Get the mode result list
			std::vector<double> globalEigenVector = m_eigenvectors[i];

			// get the appropriate modal displacement of this particular node
			glm::vec2 modal_displ = glm::vec2(globalEigenVector[(matrix_index * 2) + 0],
				globalEigenVector[(matrix_index * 2) + 1]);

			// add to modal result of this node
			node_modal_displ.insert({ i,modal_displ });
		}

		// Create the modal analysis result node
		glm::vec2 node_pt = nd_m.second.node_pt;
		modal_result_nodes.add_result_node(node_id, node_pt, node_modal_displ);
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model nodes at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________

		// Add the modal line element result
	for (auto& ln_m : model_lineelements.elementlineMap)
	{
		elementline_store ln = ln_m.second;
		bool is_rigid = false;

		if (ln.material_id == 0)
		{
			// Check whether the material is rigid
			is_rigid = true;
		}

		modal_result_lineelements.add_modal_elementline(ln.line_id,
			&modal_result_nodes.modal_nodeMap[ln.startNode->node_id],
			&modal_result_nodes.modal_nodeMap[ln.endNode->node_id], is_rigid);
	}



	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________



}


void modal_elim_solver::get_modal_matrices(Eigen::VectorXd& reduced_modalMass,
	Eigen::VectorXd& reduced_modalStiff,
	const Eigen::MatrixXd& reduced_eigenvectors,
	const Eigen::MatrixXd& reduced_globalPointMassMatrix,
	const Eigen::MatrixXd& reduced_globalStiffnessMatrix,
	const int& reducedDOF,
	std::ofstream& output_file)
{
	// Get the modal matrices
	Eigen::MatrixXd modalMassMatrix(reducedDOF, reducedDOF);
	modalMassMatrix = reduced_eigenvectors.transpose() * reduced_globalPointMassMatrix * reduced_eigenvectors;

	// Create modal stiffness matrix
	Eigen::MatrixXd modalStiffMatrix(reducedDOF, reducedDOF);
	modalStiffMatrix = reduced_eigenvectors.transpose() * reduced_globalStiffnessMatrix * reduced_eigenvectors;

	// Create the modal vectors
	reduced_modalMass = modalMassMatrix.diagonal();
	reduced_modalStiff = modalStiffMatrix.diagonal();

	if (print_matrix == true)
	{
		// Print the Modal Mass Matrix
		output_file << "Reduced Modal Mass Matrix" << std::endl;
		output_file << reduced_modalMass << std::endl;
		output_file << std::endl;

		// Print the Modal Stiffness matrix
		output_file << "Reduced Modal Stiffness Matrix" << std::endl;
		output_file << reduced_modalStiff << std::endl;
		output_file << std::endl;
	}
}
