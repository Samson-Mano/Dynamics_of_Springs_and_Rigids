#include "modal_lagrange_solver.h"

modal_lagrange_solver::modal_lagrange_solver()
{
	// Empty constructor
}




modal_lagrange_solver::~modal_lagrange_solver()
{
	// Empty destructor
}



void modal_lagrange_solver::clear_results()
{
	// Clear the eigen values and eigen vectors
	number_of_modes = 0;
	nodeid_map.clear();
	m_eigenvalues.clear();
	m_eigenvectors.clear();
	numDOF = 0;
	mode_result_str.clear();

}



void modal_lagrange_solver::modal_analysis_lagrangemethod_start(const nodes_list_store& model_nodes,
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
	// set the maximum stiffness

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
	// set the mass for zero mass node (node without any pt mass assignment)
	double min_pointmass = DBL_MAX;
	for (const auto& pt_m : node_ptmass.ptmassMap)
	{
		nodepointmass_data pt_mass = pt_m.second;

		// Find the minimum point mass
		if (min_pointmass > pt_mass.ptmass_x && pt_mass.ptmass_x != 0)
		{
			min_pointmass = pt_mass.ptmass_x;
			this->zero_ptmass = min_pointmass * (1.0 / penalty_scale_factor);
		}

		if (min_pointmass > pt_mass.ptmass_y && pt_mass.ptmass_y != 0)
		{
			min_pointmass = pt_mass.ptmass_y;
			this->zero_ptmass = min_pointmass * (1.0 / penalty_scale_factor);
		}

	}



	// Create a file to keep track of matrices
	std::ofstream output_file;
	output_file.open("modal_analysis_results.txt");

	if (print_matrix == true)
	{
		output_file << "Modal Analysis using Lagrange method" << std::endl;
		output_file << "____________________________________________" << std::endl;

	}



	//____________________________________________________________________________________________________________________
	numDOF = model_nodes.node_count * 2; // Number of degrees of freedom (2 DOFs per node (2 translation))


	//____________________________________________________________________________________________________________________
	// Global Stiffness Matrix
	Eigen::MatrixXd	globalStiffnessMatrix(numDOF, numDOF);
	globalStiffnessMatrix.setZero();

	get_global_stiffness_matrix(globalStiffnessMatrix,
		model_lineelements,
		node_constraints,
		material_list,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;



	//____________________________________________________________________________________________________________________
	// Global Point Mass Matrix
	Eigen::MatrixXd	globalPointMassMatrix(numDOF, numDOF);
	globalPointMassMatrix.setZero();

	get_global_pointmass_matrix(globalPointMassMatrix,
		model_nodes,
		node_ptmass,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global point mass matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	//____________________________________________________________________________________________________________________
	// Global Constraint A matrix
	Eigen::MatrixXd globalConstraint_MPC_AMatrix; // resize inside the function
	Eigen::MatrixXd globalConstraint_SPC_AMatrix; // resize inside the function


	get_global_constraint_A_matrix(globalConstraint_SPC_AMatrix,
		globalConstraint_MPC_AMatrix,
		model_nodes,
		model_lineelements,
		node_constraints,
		material_list,
		output_file);


	Eigen::MatrixXd globalConstraint_AMatrix; // resize inside the function

	// Create the global constraint A matrix
	int MPCconstraintEqnSize = globalConstraint_MPC_AMatrix.rows();
	int SPCconstraintEqnSize = globalConstraint_SPC_AMatrix.rows();

	int constraintEqnSize = SPCconstraintEqnSize + MPCconstraintEqnSize;

	int LagrageAugmentedMatrixSize = numDOF + constraintEqnSize;

	globalConstraint_AMatrix.conservativeResize(MPCconstraintEqnSize + SPCconstraintEqnSize, numDOF); // where the row size is SPC A matrix + MPC A matrix

	if (MPCconstraintEqnSize > 0)
	{
		globalConstraint_AMatrix.topRows(MPCconstraintEqnSize) = globalConstraint_MPC_AMatrix;
	}

	if (SPCconstraintEqnSize > 0)
	{
		globalConstraint_AMatrix.bottomRows(SPCconstraintEqnSize) = globalConstraint_SPC_AMatrix;
	}


	if (print_matrix == true)
	{
		// Print the Global Constraint A matrix
		output_file << "Global Constraint A Matrix" << std::endl;
		output_file << std::fixed << std::setprecision(6) << globalConstraint_AMatrix << std::endl;  // Set decimal precision to 6 
		output_file << std::endl;
	}


	//___________________________________________________________________________________________________________________
	// Augmentation of Stiffness matrix with Constraint A Matrix

	Eigen::MatrixXd AugmentedStiffnessMatrix(LagrageAugmentedMatrixSize, LagrageAugmentedMatrixSize);
	AugmentedStiffnessMatrix.setZero();

	// Top-left block: K
	AugmentedStiffnessMatrix.topLeftCorner(numDOF, numDOF) = globalStiffnessMatrix;

	// Top-right block: A^T
	AugmentedStiffnessMatrix.topRightCorner(numDOF, constraintEqnSize) = globalConstraint_AMatrix.transpose();

	// Bottom-left block: A
	AugmentedStiffnessMatrix.bottomLeftCorner(constraintEqnSize, numDOF) = globalConstraint_AMatrix;

	// Bottom-right block: zero matrix (already zero-initialized)



	if (print_matrix == true)
	{
		// Print the Global Constraint A matrix
		output_file << "Augmented Stiffness matrix with global Constraint A Matrix" << std::endl;
		output_file << std::fixed << std::setprecision(6) << AugmentedStiffnessMatrix << std::endl;  // Set decimal precision to 6 
		output_file << std::endl;
	}


	//___________________________________________________________________________________________________________________
	// Augmentation of Mass matrix with Zero matrix

	Eigen::MatrixXd AugmentedPointMatrix(LagrageAugmentedMatrixSize, LagrageAugmentedMatrixSize);
	AugmentedPointMatrix.setZero();

	// Top-left block: M
	AugmentedPointMatrix.topLeftCorner(numDOF, numDOF) = globalPointMassMatrix;

	// Top-right block: zero matrix (already zero-initialized)


	// Bottom-left block: zero matrix (already zero-initialized)


	// Bottom-right block: zero matrix: Very small value to keep it positive definite
	AugmentedPointMatrix.bottomLeftCorner(constraintEqnSize, constraintEqnSize) = 1E-8 * Eigen::MatrixXd::Identity(constraintEqnSize, constraintEqnSize);


	//___________________________________________________________________________________________________________________
	// Solve the generalized eigenvalue problem: [K] * [phi] = lambda * [M] * [phi]
	Eigen::GeneralizedEigenSolver<Eigen::MatrixXd> eigenSolver;
	eigenSolver.compute(AugmentedStiffnessMatrix, AugmentedPointMatrix);


	if (eigenSolver.info() != Eigen::Success)
	{
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
	Eigen::VectorXd eigenvalues = eigenSolver.eigenvalues().real(); // Eigenvalues
	Eigen::MatrixXd eigenvectors = eigenSolver.eigenvectors().real().topRows(numDOF); // Eigenvectors

	// Filter the eigen values and its corresponding eigen vectors
	filter_eigenvalues_eigenvectors(eigenvalues, eigenvectors);

	// Normailize eigen vectors
	normalize_eigen_vectors(eigenvectors);


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
		output_file << "Eigen Vectors " << std::endl;
		output_file << eigenvectors << std::endl;
		output_file << std::endl;
	}



	//_____________________________________________________________________________________________
	// Calculate the effective mass participation factor & Cummulative effective mass participation factor
	// effective mass participation factor = percentage of the system mass that participates in a particular mode


	Eigen::VectorXd participation_factor(numDOF);
	participation_factor.setZero();

	get_modal_participation_factor(participation_factor,
		globalPointMassMatrix,
		eigenvectors,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal Mass of Participation Factor completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;



	//____________________________________________________________________________________________________________________
	// Store the results

	// Clear the modal results
	this->number_of_modes = eigenvalues.rows(); // Number of modes
	this->mode_result_str.clear(); // Result string list
	this->m_eigenvalues.clear(); // Eigen values
	this->m_eigenvectors.clear(); // Eigen vectors

	// Add the eigen values and eigen vectors
	for (int i = 0; i < this->number_of_modes; i++)
	{
		std::vector<double> eigen_vec; // Eigen vectors of all nodes (including constrainded)

		for (int j = 0; j < numDOF; j++)
		{
			eigen_vec.push_back(eigenvectors.coeff(j, i));
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
	this->modalMass.resize(numDOF);
	this->modalStiff.resize(numDOF);

	get_modal_matrices(modalMass,
		modalStiff,
		eigenvectors,
		globalPointMassMatrix,
		globalStiffnessMatrix,
		output_file);

	// resize the results with number of mode
	this->modalMass.conservativeResize(this->number_of_modes);
	this->modalStiff.conservativeResize(this->number_of_modes);

	int n_count = 0;

	// Remove the eigen values (of constraints)
	for (auto it = m_eigenvalues.begin(); it != m_eigenvalues.end(); )
	{
		if (n_count++ >= this->number_of_modes)
		{
			it = m_eigenvalues.erase(it);
		}
		else
		{
			++it;
		}
	}

	// Remove the eigen vectors (of constraints)
	n_count = 0;

	for (auto it = m_eigenvectors.begin(); it != m_eigenvectors.end(); )
	{
		if (n_count++ >= this->number_of_modes)
		{
			it = m_eigenvectors.erase(it);
		}
		else
		{
			++it;
		}
	}

	this->global_eigenvectors = eigenvectors;

	// resize the modal string
	mode_result_str.resize(this->number_of_modes);



	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal mass and stiffness storage completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	std::cout << "Modal analysis complete " << std::endl;



	//____________________________________________________________________________________________________________________
	stopwatch.stop();

	output_file.close();



}



void modal_lagrange_solver::get_global_stiffness_matrix(Eigen::MatrixXd& globalStiffnessMatrix,
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




void modal_lagrange_solver::get_element_stiffness_matrix(Eigen::MatrixXd& elementStiffnessMatrix,
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
	// Compute the stiffness factor

	double k1 = 0.0;
	if (elementline_material.material_id == 0)
	{
		// Line is rigid
		k1 = 0.0; // penalty_scale_factor * max_stiffness; // Rigid stiffness
	}
	else
	{
		k1 = elementline_material.material_stiffness;
	}


	//Stiffness matrix components
	double v1 = k1 * std::pow(Lcos, 2);
	double v2 = k1 * std::pow(Msin, 2);
	double v3 = k1 * (Lcos * Msin);


	// Create the Element stiffness matrix
	elementStiffnessMatrix.row(0) = Eigen::RowVector4d(v1, v3, -v1, -v3);
	elementStiffnessMatrix.row(1) = Eigen::RowVector4d(v3, v2, -v3, -v2);
	elementStiffnessMatrix.row(2) = Eigen::RowVector4d(-v1, -v3, v1, v3);
	elementStiffnessMatrix.row(3) = Eigen::RowVector4d(-v3, -v2, v3, v2);


	if (print_matrix == true)
	{
		// Print the Element Stiffness matrix
		output_file << "Member (" << ln.startNode->node_id << " -> " << ln.endNode->node_id << ")" << std::endl;
		output_file << elementStiffnessMatrix << std::endl;
		output_file << std::endl;
	}

}




void modal_lagrange_solver::get_global_pointmass_matrix(Eigen::MatrixXd& globalPointMassMatrix,
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

			// Point mass y
			if (ptm.ptmass_x != 0)
			{
				globalPointMassMatrix((nd_map * 2) + 0, (nd_map * 2) + 0) = ptm.ptmass_x;
			}
			else
			{
				globalPointMassMatrix((nd_map * 2) + 0, (nd_map * 2) + 0) = zero_ptmass;
			}

			// Point mass x
			if (ptm.ptmass_y != 0)
			{
				globalPointMassMatrix((nd_map * 2) + 1, (nd_map * 2) + 1) = ptm.ptmass_y;
			}
			else
			{
				globalPointMassMatrix((nd_map * 2) + 1, (nd_map * 2) + 1) = zero_ptmass;
			}

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




void modal_lagrange_solver::get_global_constraint_A_matrix(Eigen::MatrixXd& globalConstraint_SPC_AMatrix,
	Eigen::MatrixXd& globalConstraint_MPC_AMatrix,
	const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const nodeconstraint_list_store& node_constraints,
	const std::unordered_map<int, material_data>& material_list,
	std::ofstream& output_file)
{

	// Apply boundary condition using Lagrange method
	// Single point constraint (Pinned or Roller boundary condition)

	globalConstraint_SPC_AMatrix.resize(0, numDOF); // Start with zero rows

	int currentRows = 0;

	for (auto& nd_m : model_nodes.nodeMap)
	{
		// Get the node data
		node_store nd = nd_m.second;
		int nd_map = nodeid_map[nd.node_id]; // get the ordered map of the node ID

		if (node_constraints.constraintMap.find(nd.node_id) != node_constraints.constraintMap.end())
		{
			// Nodes have constraints
			constraint_data cd = node_constraints.constraintMap.at(nd.node_id);

			int constraint_type = cd.constraint_type; // constraint type
			double constraint_angle_rad = (cd.constraint_angle - 90.0f) * (m_pi / 180.0f); // constraint angle
			double support_Lcos = std::cos(constraint_angle_rad); // cosine of support inclination
			double support_Msin = std::sin(constraint_angle_rad); // sine of support inclination

			// Single point constraint A vector
			Eigen::VectorXd SPC_AVector(numDOF);
			SPC_AVector.setZero();


			if (constraint_type == 0)
			{
				// Pinned support
				SPC_AVector[(nd_map * 2) + 0] = 1.0;
				SPC_AVector[(nd_map * 2) + 1] = 0.0;

				// **Expand A_matrix by adding a new row**
				currentRows = globalConstraint_SPC_AMatrix.rows();
				globalConstraint_SPC_AMatrix.conservativeResize(currentRows + 1, numDOF); // Add one row
				globalConstraint_SPC_AMatrix.row(currentRows) = SPC_AVector;       // Insert the new vector (X fixed)



				SPC_AVector[(nd_map * 2) + 0] = 0.0;
				SPC_AVector[(nd_map * 2) + 1] = 1.0;

				// **Expand A_matrix by adding a new row**
				currentRows = globalConstraint_SPC_AMatrix.rows();
				globalConstraint_SPC_AMatrix.conservativeResize(currentRows + 1, numDOF); // Add one row
				globalConstraint_SPC_AMatrix.row(currentRows) = SPC_AVector;       // Insert the new vector (Y fixed)

			}
			else if (constraint_type == 1)
			{
				// Roller support
				SPC_AVector[(nd_map * 2) + 0] = -support_Msin;
				SPC_AVector[(nd_map * 2) + 1] = support_Lcos;

				// **Expand A_matrix by adding a new row**
				currentRows = globalConstraint_SPC_AMatrix.rows();
				globalConstraint_SPC_AMatrix.conservativeResize(currentRows + 1, numDOF); // Add one row
				globalConstraint_SPC_AMatrix.row(currentRows) = SPC_AVector;       // Insert the new vector

			}

		}

	}


	// Multi point constraint (Rigid link)

	globalConstraint_MPC_AMatrix.resize(0, numDOF); // Start with zero rows


	for (auto& ln_m : model_lineelements.elementlineMap)
	{
		// Get the element data
		elementline_store ln = ln_m.second;
		material_data elementline_material = material_list.at(ln.material_id);

		if (elementline_material.material_id == 0)
		{
			// Rigid link
			// 
			// Get the Node ID
			int sn_id = nodeid_map[ln.startNode->node_id]; // get the ordered map of the start node ID
			int en_id = nodeid_map[ln.endNode->node_id]; // get the ordered map of the end node ID

			// Compute the differences in x and y coordinates
			double dx = ln.endNode->node_pt.x - ln.startNode->node_pt.x;
			double dy = -1.0 * (ln.endNode->node_pt.y - ln.startNode->node_pt.y);

			// Compute the length of the truss element
			double eLength = std::sqrt((dx * dx) + (dy * dy));

			// Compute the direction cosines
			double Lcos = (dx / eLength);
			double Msin = (dy / eLength);

			// Multi point constraint A vector
			Eigen::VectorXd MPC_AVector(numDOF);
			MPC_AVector.setZero();

			// start node transformation
			MPC_AVector[(sn_id * 2) + 0] = Lcos;
			MPC_AVector[(sn_id * 2) + 1] = Msin;

			// end node transformation
			MPC_AVector[(en_id * 2) + 0] = -Lcos;
			MPC_AVector[(en_id * 2) + 1] = -Msin;

			// **Expand A_matrix by adding a new row**
			currentRows = globalConstraint_MPC_AMatrix.rows();
			globalConstraint_MPC_AMatrix.conservativeResize(currentRows + 1, numDOF); // Add one row
			globalConstraint_MPC_AMatrix.row(currentRows) = MPC_AVector;       // Insert the new vector

		}

	}

	if (print_matrix == true)
	{
		// Print the SPC A matrix
		output_file << "Global SPC A Matrix" << std::endl;
		output_file << std::fixed << std::setprecision(6) << globalConstraint_SPC_AMatrix << std::endl;  // Set decimal precision to 6 
		output_file << std::endl;

		// Print the MPC A matrix
		output_file << "Global MPC A Matrix" << std::endl;
		output_file << std::fixed << std::setprecision(6) << globalConstraint_MPC_AMatrix << std::endl;  // Set decimal precision to 6 
		output_file << std::endl;

	}


}




void modal_lagrange_solver::filter_eigenvalues_eigenvectors(Eigen::VectorXd& eigenvalues,
	Eigen::MatrixXd& eigenvectors)
{
	// Store the indices of valid eigenvalues
	std::vector<int> valid_indices;

	for (int i = 0; i < eigenvalues.size(); ++i)
	{
		double val = eigenvalues(i);
		if (std::isfinite(val) == true && val > -1.0)  // excludes NaN, +Inf, -Inf
		{
			valid_indices.push_back(i);
		}
	}

	// Create filtered eigenvalues and eigenvectors
	Eigen::VectorXd filtered_eigenvalues(valid_indices.size());
	Eigen::MatrixXd filtered_eigenvectors(eigenvectors.rows(), valid_indices.size());

	for (size_t i = 0; i < valid_indices.size(); ++i)
	{
		int idx = valid_indices[i];
		filtered_eigenvalues(i) = eigenvalues(idx);
		filtered_eigenvectors.col(i) = eigenvectors.col(idx);
	}

	//_____________________________________________________________________________________________
	int n = filtered_eigenvalues.size();

	// Create index vector [0, 1, ..., n-1]
	std::vector<int> indices(n);

	for (int i = 0; i < n; ++i)
	{
		indices[i] = i;
	}


	// Sort the indices based on eigenvalues
	std::sort(indices.begin(), indices.end(),
		[&](int i1, int i2) { return filtered_eigenvalues(i1) < filtered_eigenvalues(i2); });

	// Create sorted eigenvalues and eigenvectors
	Eigen::VectorXd sorted_eigenvalues(n);
	Eigen::MatrixXd sorted_eigenvectors(eigenvectors.rows(), n);

	for (int i = 0; i < n; ++i)
	{
		sorted_eigenvalues(i) = filtered_eigenvalues(indices[i]);
		sorted_eigenvectors.col(i) = filtered_eigenvectors.col(indices[i]);
	}


	// Replace input with filtered versions
	eigenvalues = sorted_eigenvalues;
	eigenvectors = sorted_eigenvectors;

}




void modal_lagrange_solver::normalize_eigen_vectors(Eigen::MatrixXd& eigenvectors)
{
	// Normalize eigen vectors
	int col_size = eigenvectors.cols();
	int row_size = eigenvectors.rows();

	// loop throught each column
	for (int p = 0; p < col_size; p++)
	{
		double max_modal_vector = 0.0;

		// Loop through each row
		for (int q = 0; q < row_size; q++)
		{
			if (std::abs(eigenvectors(q, p)) > max_modal_vector)
			{
				// Max modal vector in the column (for particular mode)
				max_modal_vector = std::abs(eigenvectors(q, p));
			}
		}

		// Normalize the column using maximum modal vector
		for (int q = 0; q < row_size; q++)
		{
			eigenvectors(q, p) = eigenvectors(q, p) / max_modal_vector;

			// Round the eigen vectors to 6 digit precision after normalizing
			eigenvectors(q, p) = std::round(eigenvectors(q, p) * 1000000) / 1000000;
		}
	}


}




void modal_lagrange_solver::get_modal_participation_factor(Eigen::VectorXd& participation_factor,
	const Eigen::MatrixXd& globalPointMassMatrix,
	const Eigen::MatrixXd& eigenvectors,
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

	for (int i = 0; i < eigenvectors.cols(); i++)
	{
		// Get the nth Mode (Column)
		Eigen::VectorXd eigen_vector_i = eigenvectors.col(i);

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



void modal_lagrange_solver::map_modal_analysis_results(const nodes_list_store& model_nodes,
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



void modal_lagrange_solver::get_modal_matrices(Eigen::VectorXd& modalMass,
	Eigen::VectorXd& modalStiff,
	const Eigen::MatrixXd& eigenvectors,
	const Eigen::MatrixXd& globalPointMassMatrix,
	const Eigen::MatrixXd& globalStiffnessMatrix,
	std::ofstream& output_file)
{
	// Get the modal matrices
	Eigen::MatrixXd modalMassMatrix(numDOF, numDOF);
	modalMassMatrix = eigenvectors.transpose() * globalPointMassMatrix * eigenvectors;

	// Create modal stiffness matrix
	Eigen::MatrixXd modalStiffMatrix(numDOF, numDOF);
	modalStiffMatrix = eigenvectors.transpose() * globalStiffnessMatrix * eigenvectors;

	// Create the modal vectors
	this->modalMass = modalMassMatrix.diagonal();
	this->modalStiff = modalStiffMatrix.diagonal();

	if (print_matrix == true)
	{
		// Print the Modal Mass Matrix
		output_file << "Reduced Modal Mass Matrix" << std::endl;
		output_file << this->modalMass << std::endl;
		output_file << std::endl;

		// Print the Modal Stiffness matrix
		output_file << "Reduced Modal Stiffness Matrix" << std::endl;
		output_file << this->modalStiff << std::endl;
		output_file << std::endl;
	}


}









