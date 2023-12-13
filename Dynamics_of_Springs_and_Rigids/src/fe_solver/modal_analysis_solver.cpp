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
	globalPointMassMatrix.resize(numDOF, numDOF);
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
	globalDOFMatrix.resize(numDOF);
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
	// Gloabl Augmentation matrix
	agDOF = 0.0; // size of the augmentation matrix

	globalAGMatrix.resize(0, numDOF);
	globalAGMatrix.setZero();


	get_global_augmentation_matrix(globalAGMatrix,
		model_constarints,
		model_lineelements,
		material_list,
		agDOF,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Augmentation matrix creation completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Augmented Global Stiffness Matrix
	agglobalStiffnessMatrix.resize(numDOF + agDOF, numDOF + agDOF);
	agglobalStiffnessMatrix.setZero();


	get_augmented_global_stiffness_matrix(agglobalStiffnessMatrix,
		globalStiffnessMatrix,
		globalAGMatrix,
		output_file);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Augmented Global stiffness matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Augmented Global Point mass Matrix
	agglobalPointMassMatrix.resize(numDOF + agDOF, numDOF + agDOF);
	agglobalPointMassMatrix.setZero();

	// Augmented Global DOF Matrix
	agglobalDOFMatrix.resize(numDOF + agDOF);
	agglobalDOFMatrix.setZero();

	get_augmented_global_ptmass_matrix(agglobalPointMassMatrix,
		agglobalDOFMatrix,
		globalPointMassMatrix,
		globalDOFMatrix,
		output_file);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Augmented Global point mass matrix completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Create Reduced Global Mass and stiffness matrix
	reduced_agglobalStiffnessMatrix(reducedDOF + agDOF, reducedDOF + agDOF);
	reduced_agglobalStiffnessMatrix.setZero();

	// Reduced Global Mass matrix
	reduced_agglobalPointMassMatrix(reducedDOF + agDOF, reducedDOF + agDOF);
	reduced_agglobalPointMassMatrix.setZero();

	get_reduced_global_matrices(reduced_agglobalStiffnessMatrix,
		reduced_agglobalPointMassMatrix,
		agglobalStiffnessMatrix,
		agglobalPointMassMatrix,
		agglobalDOFMatrix,
		numDOF,
		reducedDOF,
		agDOF,
		output_file);


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Global stiffness/ point mass matrces are reduced at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Modal Analysis - main solve
	// Solve the generalized Eigenvalue problem k * phi = lamda * m * phi
	// using GeneralizedSelfAdjointEigenSolver
	
	// Convert dense matrices to sparse matrices
	Eigen::SparseMatrix<double> sparseStiffnessMatrix = reduced_agglobalStiffnessMatrix.sparseView();
	Eigen::SparseMatrix<double> sparsePointMassMatrix = reduced_agglobalPointMassMatrix.sparseView();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Sprase matrix conversion completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	// Solve the generalized eigenvalue problem
	Eigen::GeneralizedSelfAdjointEigenSolver<Eigen::SparseMatrix<double>> solver(sparseStiffnessMatrix, sparsePointMassMatrix);

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str.clear();
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Eigen value problem solved at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//____________________________________________________________________________________________________________________
	if (solver.info() == Eigen::Success) 
	{
		// Eigenvalues
		Eigen::VectorXd eigenvalues = solver.eigenvalues();

		// Eigenvectors
		Eigen::MatrixXd  eigenvectors = solver.eigenvectors();

		// Process eigenvalues and eigenvectors
		// sort the eigen value and eigen vector (ascending)
		sort_eigen_values_vectors(eigenvalues, eigenvectors, reducedDOF + agDOF);

		stopwatch_elapsed_str.str("");
		stopwatch_elapsed_str.clear();
		stopwatch_elapsed_str << stopwatch.elapsed();
		std::cout << "Eigen values and Eigen vectors are sorted at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

		//____________________________________________________________________________________________________________________
		// Normailize eigen vectors
		normalize_eigen_vectors(eigenvectors, reducedDOF);

		stopwatch_elapsed_str.str("");
		stopwatch_elapsed_str.clear();
		stopwatch_elapsed_str << stopwatch.elapsed();
		std::cout << "Eigen vectors are normalized at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

		//____________________________________________________________________________________________________________________




	}
	else 
	{
		// Handle the case where the solver failed
		std::cerr << "Eigenvalue solver failed!" << std::endl;
		output_file.close();
		return;
	}





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


void modal_analysis_solver::get_global_dof_matrix(Eigen::VectorXd& globalDOFMatrix,
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
				// Pin End (1.0 = Fixed)

				globalDOFMatrix.coeffRef((nd_map * 2) + 0) = 1.0;
				globalDOFMatrix.coeffRef((nd_map * 2) + 1) = 1.0;
			}
			else if (cd.constraint_type == 1)
			{
				// Pin Roller end
				// Added as augmentation matrix
				//globalDOFMatrix.coeffRef((nd_map * 2) + 0) = 0.0; // X is free to move
				//globalDOFMatrix.coeffRef((nd_map * 2) + 1) = 1.0;

				//reducedDOF = reducedDOF + 1;
			}
		}
		else
		{
			// Nodes doesnt have Constraint (0.0 = Free)
			globalDOFMatrix.coeffRef((nd_map * 2) + 0) = 0.0;
			globalDOFMatrix.coeffRef((nd_map * 2) + 1) = 0.0;

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


void modal_analysis_solver::get_global_augmentation_matrix(Eigen::MatrixXd& globalAGMatrix,
	const nodeconstraint_list_store& model_constarints,
	const elementline_list_store& model_lineelements,
	const std::unordered_map<int, material_data>& material_list,
	int& agDOF,
	std::ofstream& output_file)
{
	// Create global Augmentation matrix
	// Contraint (Roller) Augmentation
	Eigen::VectorXd row_augmentation_matrix(numDOF);
	

	for (const auto& cnst_m : model_constarints.constraintMap)
	{
		constraint_data cnst = cnst_m.second;

		if (cnst.constraint_type == 1)
		{
			int nd_map = nodeid_map[cnst.node_id]; // get the ordered map of the node ID

			// Roller support
			row_augmentation_matrix.setZero(); //set zero

			double constraint_angle_rad = (180.0f - cnst.constraint_angle) * (m_pi / 180.0f); // constarint angle radian

			row_augmentation_matrix.coeffRef((nd_map * 2) + 0) = std::cos(constraint_angle_rad);
			row_augmentation_matrix.coeffRef((nd_map * 2) + 1) = std::sin(constraint_angle_rad);

			// Append to the global matrix
			globalAGMatrix.conservativeResize(globalAGMatrix.rows() + 1, Eigen::NoChange);
			globalAGMatrix.row(globalAGMatrix.rows() - 1) = row_augmentation_matrix;

			// Add to the AG DOF
			agDOF = agDOF + 1;
		}

	}


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


	if (print_matrix == true)
	{
		// Print the Global Augmentation matrix
		output_file << "Global Augmentation Matrix" << std::endl;
		output_file << globalAGMatrix << std::endl;
		output_file << std::endl;
	}

}


void modal_analysis_solver::get_augmented_global_stiffness_matrix(Eigen::MatrixXd& agglobalStiffnessMatrix,
	const Eigen::MatrixXd& globalStiffnessMatrix,
	const Eigen::MatrixXd& globalAGMatrix,
	std::ofstream& output_file)
{
	// Augment the global stiffness matrix

	// Copy globalStiffnessMatrix to the top-left block of agglobalStiffnessMatrix
	agglobalStiffnessMatrix.topLeftCorner(numDOF, numDOF) = globalStiffnessMatrix;

	// Copy globalAGMatrix to the bottom-left block of agglobalStiffnessMatrix
	agglobalStiffnessMatrix.bottomLeftCorner(agDOF, numDOF) = globalAGMatrix;

	// Copy the transpose of globalAGMatrix to the top-right block of agglobalStiffnessMatrix
	agglobalStiffnessMatrix.topRightCorner(numDOF, agDOF) = globalAGMatrix.transpose();

	// Fill the bottom-right block of agglobalStiffnessMatrix with zeros
	agglobalStiffnessMatrix.bottomRightCorner(agDOF, agDOF).setZero();

	if (print_matrix == true)
	{
		// Print the Augmented Global stiffness matrix
		output_file << "Augmented Global stiffness Matrix" << std::endl;
		output_file << agglobalStiffnessMatrix << std::endl;
		output_file << std::endl;
	}

}


void modal_analysis_solver::get_augmented_global_ptmass_matrix(Eigen::MatrixXd& agglobalPointMassMatrix,
	Eigen::VectorXd& agglobalDOFMatrix,
	const Eigen::MatrixXd& globalPointMassMatrix,
	const Eigen::VectorXd& globalDOFMatrix,
	std::ofstream& output_file)
{
	// Augment the global Point Mass matrix

	// Copy globalPointMassMatrix to the top-left block of agglobalPointMassMatrix
	agglobalPointMassMatrix.topLeftCorner(numDOF, numDOF) = globalPointMassMatrix;

	// Fill the bottom-left block of agglobalPointMassMatrix with zeros
	agglobalPointMassMatrix.bottomLeftCorner(agDOF, numDOF).setZero();

	// Fill the top-right block of agglobalPointMassMatrix with zeros
	agglobalPointMassMatrix.topRightCorner(numDOF, agDOF).setZero();

	// Fill the bottom-right block of agglobalPointMassMatrix with zeros
	agglobalPointMassMatrix.bottomRightCorner(agDOF, agDOF).setZero();

	//_______________________________________________________________________________
	// Augment the global DOF matrix

	// Copy globalDOFMatrix to the top of agglobalDOFMatrix
	agglobalDOFMatrix.head(numDOF) = globalDOFMatrix;

	// Fill the bottom of agglobalDOFMatrix with zeros
	agglobalDOFMatrix.tail(agDOF).setZero();

	if (print_matrix == true)
	{
		// Print the Augmented Global point mass matrix
		output_file << "Augmented Global point mass Matrix" << std::endl;
		output_file << agglobalPointMassMatrix << std::endl;
		output_file << std::endl;

		// Print the Augmented Global DOF matrix
		output_file << "Augmented Global DOF Matrix" << std::endl;
		output_file << agglobalDOFMatrix << std::endl;
		output_file << std::endl;
	}
}


void modal_analysis_solver::get_reduced_global_matrices(Eigen::MatrixXd& reduced_agglobalStiffnessMatrix,
	Eigen::MatrixXd& reduced_agglobalPointMassMatrix,
	const Eigen::MatrixXd& agglobalStiffnessMatrix,
	const Eigen::MatrixXd& agglobalPointMassMatrix,
	const Eigen::VectorXd& agglobalDOFMatrix,
	const int& numDOF,
	const int& reducedDOF,
	const int& agDOF,
	std::ofstream& output_file)
{
	// Curtailment of Global stiffness and Global force matrix based on DOF
	// Get the reduced global stiffness matrix
	int r = 0;
	int s = 0;

	// Loop throug the Degree of freedom of indices
	for (int i = 0; i < (numDOF + agDOF); i++)
	{
		if (agglobalDOFMatrix.coeff(i) == 1)
		{
			// constrained row index, so skip
			continue;
		}
		else
		{
			s = 0;
			for (int j = 0; j < (numDOF + agDOF); j++)
			{
				if (agglobalDOFMatrix.coeff(j) == 1)
				{
					// constrained column index, so skip
					continue;
				}
				else
				{
					// Get the reduced matrices
					reduced_agglobalStiffnessMatrix.coeffRef(r, s) = agglobalPointMassMatrix.coeff(i, j);
					reduced_agglobalPointMassMatrix.coeffRef(r, s) = agglobalStiffnessMatrix.coeff(i, j);
					s++;
				}
			}
			r++;
		}
	}

	if (print_matrix == true)
	{
		// Print the Reduced Augmented Global Stiffness and Reduced Augmented Global Mass matrix
		output_file << "Reduced Augmented Global Stiffness Matrix" << std::endl;
		output_file << reduced_agglobalStiffnessMatrix << std::endl;
		output_file << std::endl;

		output_file << "Reduced Augmented Global Mass Matrix" << std::endl;
		output_file << reduced_agglobalPointMassMatrix << std::endl;
		output_file << std::endl;
	}

}


void modal_analysis_solver::sort_eigen_values_vectors(Eigen::VectorXd& eigenvalues,
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


void modal_analysis_solver::normalize_eigen_vectors(Eigen::MatrixXd& eigenvectors,
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