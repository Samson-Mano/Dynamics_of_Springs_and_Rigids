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
	nodeid_map.clear(); // Node ID map
	number_of_modes = 0;
	node_count = 0;

	mode_result_str.clear();
	is_modal_analysis_complete = false;
}


void modal_analysis_solver::modal_analysis_start(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const elementquad_list_store& model_quadelements,
	const material_data& mat_data,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements,
	modal_elementquad_list_store& modal_result_quadelements)
{
	// Main solver call
	this->is_modal_analysis_complete = false;

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

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Modal analysis - started" << std::endl;

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


	this->node_count = model_nodes.node_count;
	this->model_type = mat_data.model_type;

	this->matrix_size = 0;

	if (this->model_type == 0)
	{
		// Circular Membrane
		// Number of fixed nodes = 128 or 256 
		// Radius = 100
		this->matrix_size = node_count - 128;
	}
	else if (this->model_type == 1)
	{
		// Rectangular 1:1
		// Number of fixed nodes = 200

		this->matrix_size = node_count - 200;
	}
	else if (this->model_type == 2)
	{
		// Rectangular 1:2
		// Number of fixed nodes = 300
		this->matrix_size = node_count - 300;
	}
	else if (this->model_type == 3)
	{
		// Rectangular 1:3
		// Number of fixed nodes = 400
		this->matrix_size = node_count - 400;
	}


	// Displacement vectors matrix
	displ_vectors_matrix.resize(node_count, this->matrix_size); // Number of nodes as row & Mode count as column
	displ_vectors_matrix.setZero();

	// Clear the angular frequency matrix
	angular_freq_vector.resize(matrix_size);
	angular_freq_vector.setZero();

	// Clear the eigen values
	eigen_values_vector.resize(matrix_size);
	eigen_values_vector.setZero();

	// Clear the eigen vectors
	eigen_vectors_matrix.resize(matrix_size, matrix_size);
	eigen_vectors_matrix.setZero();

	// Clear the eigen vector inverse
	eigen_vectors_matrix_inverse.resize(matrix_size, matrix_size);
	eigen_vectors_matrix_inverse.setZero();

	// Re-initialize the variables
	modal_result_nodes.clear_data();
	modal_result_lineelements.clear_data();
	modal_result_quadelements.clear_data();

	// Mode results as string
	mode_result_str.clear();

	if (mat_data.model_type == 0)
	{
		// Circular membrane
		modal_analysis_model_circular1(model_nodes,
			mat_data);

		// Map the results
		map_modal_analysis_circular_results(model_nodes,
			model_lineelements,
			model_quadelements,
			modal_result_nodes,
			modal_result_lineelements,
			modal_result_quadelements);

		this->is_modal_analysis_complete = true;

	}
	else if (mat_data.model_type == 1)
	{
		// Rectangular 1:1
		modal_analysis_model_rectangular1(model_nodes,
			model_lineelements,
			mat_data);

		// Map the results
		map_modal_analysis_rectangular_results(model_nodes,
			model_lineelements,
			modal_result_nodes,
			modal_result_lineelements);

		this->is_modal_analysis_complete = true;

	}
	else if (mat_data.model_type == 2)
	{
		// Rectangular 1:2
		modal_analysis_model_rectangular2(model_nodes,
			model_lineelements,
			mat_data);

		// Map the results
		map_modal_analysis_rectangular_results(model_nodes,
			model_lineelements,
			modal_result_nodes,
			modal_result_lineelements);

		this->is_modal_analysis_complete = true;

	}
	else if (mat_data.model_type == 3)
	{
		// Rectangular 1:3
		modal_analysis_model_rectangular3(model_nodes,
			model_lineelements,
			mat_data);

		// Map the results
		map_modal_analysis_rectangular_results(model_nodes,
			model_lineelements,
			modal_result_nodes,
			modal_result_lineelements);

		this->is_modal_analysis_complete = true;

	}

	if (print_matrix == true)
	{
		// Create a file to keep track of matrices
		std::ofstream output_file;
		output_file.open("modal_analysis_results.txt");

		output_file << "Eigen vectors:" << std::endl;
		output_file << eigen_vectors_matrix << std::endl;
		output_file << std::endl;

		// eigen_vectors_matrix_inverse = eigen_vectors_matrix.inverse();

		output_file << "Eigen vectors Inverse" << std::endl;
		output_file << eigen_vectors_matrix_inverse << std::endl;
		output_file << std::endl;

		Eigen::MatrixXd eigen_check = eigen_vectors_matrix_inverse * eigen_vectors_matrix;

		output_file << "Eigen Check" << std::endl;
		output_file << eigen_check << std::endl;
		output_file << std::endl;

		output_file.close();
	}


}

void modal_analysis_solver::modal_analysis_model_circular1(const nodes_list_store& model_nodes,
	const material_data& mat_data)
{
	// Circular string
	int node_id = 0;
	double c_param = std::sqrt(mat_data.line_tension / mat_data.material_density);
	double c_radius = 100.0;
	this->number_of_modes = 0;

	// Find the modal frequency
	int modal_m_count = std::ceilf(std::sqrt(this->matrix_size));

	// Bessel function Frequency
	std::vector<bessel_function_Frequency> t_bessel_roots;

	for (int m = 0; m < modal_m_count; m++)
	{
		for (int n = 0; n < modal_m_count; n++)
		{
			double bessel_value = boost::math::cyl_bessel_j_zero((float)m, (n+1),ignore_all_policy());
	
			// Temp 
			bessel_function_Frequency temp_bessel_roots;
			temp_bessel_roots.m = m;
			temp_bessel_roots.n = n;
			temp_bessel_roots.root_value = bessel_value;

			// Add to the list
			t_bessel_roots.push_back(temp_bessel_roots);
		}
	}

	// Sort the vector based on root_value
	std::sort(t_bessel_roots.begin(), t_bessel_roots.end(), compareRootValues);

	// Final Bessel function Frequency
	bessel_roots.clear();
	int temp_mode_number = 0;

	// std::ofstream outFile("bessel_roots.txt"); // Open a file for writing

	for (auto& b_root : t_bessel_roots)
	{
		// Temp 
		bessel_function_Frequency temp_bessel_roots;
		temp_bessel_roots.mode_number = temp_mode_number;
		temp_bessel_roots.m = b_root.m;
		temp_bessel_roots.n = b_root.n;
		temp_bessel_roots.root_value = b_root.root_value;

		// Add to the list
		bessel_roots.push_back(temp_bessel_roots);

		// outFile << temp_mode_number << ", " << b_root.m << ", "<< b_root.n << ", "<<b_root.root_value << std::endl; // Write to the file

		temp_mode_number++;
	}

	// outFile.close(); // Close the file

	// Create the eigen vector node map
	int j = 0;
	this->eigen_vec_nodeid_map.clear();

	for (auto& nd_m : model_nodes.nodeMap)
	{
		node_id = nd_m.first;
		glm::vec3 node_pt = nd_m.second.node_pt;
		double nd_radius = glm::length(node_pt);

		if (nd_radius >= (c_radius- 0.1) )
		{
			// Zero for fixed nodes
			continue;
		}

		this->eigen_vec_nodeid_map[node_id] = j;
		j++;
	}


	// Create the mode shapes
	this->number_of_modes = 0;


	for (int i =0; i<this->matrix_size; i++)
	{

		double t_eigen = (bessel_roots[i].root_value / c_radius) * c_param;

		// Angular frequency wn
		angular_freq_vector.coeffRef(i) = t_eigen;

		// Eigen value
		eigen_values_vector.coeffRef(i) = (t_eigen * t_eigen);

		this->number_of_modes++;

		// Frequency
		double nat_freq = t_eigen / (2.0 * m_pi);

		if (this->number_of_modes < paint_mode_count)
		{
			// Modal results
			std::stringstream ss;
			ss << std::fixed << std::setprecision(3) << nat_freq;

			// Add to the string list
			mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz (m ="
				+ std::to_string(bessel_roots[i].m) + ", n=" + std::to_string(bessel_roots[i].n) + ")");
		}

		for (auto& nd_m : model_nodes.nodeMap)
		{
			node_id = nd_m.first;
			glm::vec3 node_pt = nd_m.second.node_pt;
			int displ_matrix_index = nodeid_map[node_id];
			int eigvec_matrix_index = eigen_vec_nodeid_map[node_id];

			// Ignore the boundary nodes
			// Check for boundary nodes
			double nd_radius = glm::length(node_pt);
			double nd_theta = std::atan2(node_pt.y, node_pt.x);

			if (nd_radius  >= (c_radius - 0.1))
			{
				// Zero for fixed nodes
				displ_vectors_matrix.coeffRef(displ_matrix_index, i) = 0.0;
				continue;
			}


			// Get the radius ratio
			double radius_ratio = nd_radius / c_radius;

			// Eigen vectors
			double t_eigen_vec = std::cyl_bessel_j(bessel_roots[i].m,bessel_roots[i].root_value * radius_ratio) * std::cos(bessel_roots[i].m * nd_theta);


			displ_vectors_matrix.coeffRef(displ_matrix_index, i) = t_eigen_vec;

			// Add the eigen vectors
			eigen_vectors_matrix.coeffRef(eigvec_matrix_index, i) = t_eigen_vec;
		}

		// Normalize the eigen vector matrix
		normalizeColumn(displ_vectors_matrix, i);
		normalizeColumn(eigen_vectors_matrix, i);
	}

	double inv_factor = 2.0 / static_cast<float>(matrix_size);

	// Create the eigen vectors inverse matrix

	eigen_vectors_matrix_inverse = inv_factor * eigen_vectors_matrix.transpose();


}


void modal_analysis_solver::modal_analysis_model_rectangular1(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const material_data& mat_data)
{
	// Fixed - Fixed Line 

	double line_length = mat_data.line_length;
	double c_param = std::sqrt(mat_data.line_tension / mat_data.material_density);
	this->number_of_modes = 0;

	for (int i = 0; i < node_count; i++)
	{
		// Eigen values
		int mode_number = i + 1;

		if (i < matrix_size)
		{
			double t_eigen = (mode_number * m_pi * c_param) / line_length;

			// Angular frequency wn
			angular_freq_vector.coeffRef(i) = t_eigen;

			// Eigen value
			eigen_values_vector.coeffRef(i) = (t_eigen * t_eigen);

			this->number_of_modes++;

			// Frequency
			double nat_freq = t_eigen / (2.0 * m_pi);

			// Modal results
			std::stringstream ss;
			ss << std::fixed << std::setprecision(3) << nat_freq;

			// Add to the string list
			mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz");

		}

		// First node is fixed
		displ_vectors_matrix.coeffRef(0, i) = 0.0;

		for (int j = 1; j < node_count; j++)
		{
			double length_ratio = (static_cast<float>(j) / static_cast<float>(node_count - 1));
			double t_eigen_vec = std::sin(mode_number * m_pi * length_ratio);

			displ_vectors_matrix.coeffRef(j, i) = t_eigen_vec;

			if (i < matrix_size && j < (matrix_size + 1))
			{
				eigen_vectors_matrix.coeffRef(j - 1, i) = t_eigen_vec;
			}
		}

		// Last node is fixed
		displ_vectors_matrix.coeffRef(node_count - 1, i) = 0.0;

	}

	// Create the eigen vectors inverse matrix
	double inv_factor = 2.0 / static_cast<float>(matrix_size + 1.0);

	//for (int i = 0; i < matrix_size; i++)
	//{
	//	for (int j = 0; j < matrix_size; j++)
	//	{
	//		double length_ratio = (static_cast<float>(j + 1) / static_cast<float>(node_count - 1));
	//		double t_eigen_vec = std::sin((i + 1) * m_pi * length_ratio);

	//		eigen_vectors_matrix_inverse.coeffRef(j, i) = inv_factor * t_eigen_vec;

	//	}
	//}

	// Test of not using Eigen vectors inverse
	eigen_vectors_matrix_inverse = inv_factor * eigen_vectors_matrix.transpose();

}


void modal_analysis_solver::modal_analysis_model_rectangular2(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const material_data& mat_data)
{
	// Fixed - Free Line 

	double line_length = mat_data.line_length;
	double c_param = std::sqrt(mat_data.line_tension / mat_data.material_density);
	this->number_of_modes = 0;

	for (int i = 0; i < node_count; i++)
	{
		// Eigen values

		int mode_number = (2 * (i + 1)) - 1;

		if (i < matrix_size)
		{
			double t_eigen = (mode_number * m_pi * c_param) / (2.0 * line_length);

			// Angular frequency wn
			angular_freq_vector.coeffRef(i) = t_eigen;

			// Eigen value
			eigen_values_vector.coeffRef(i) = (t_eigen * t_eigen);

			this->number_of_modes++;

			// Frequency
			double nat_freq = t_eigen / (2.0 * m_pi);

			// Modal results
			std::stringstream ss;
			ss << std::fixed << std::setprecision(3) << nat_freq;

			// Add to the string list
			mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz");

		}

		// First node is fixed
		displ_vectors_matrix.coeffRef(0, i) = 0.0;

		for (int j = 1; j < node_count; j++)
		{
			double length_ratio = (static_cast<float>(j) / static_cast<float>(node_count - 1));
			double t_eigen_vec = std::sin(mode_number * m_pi * (length_ratio / 2.0));

			displ_vectors_matrix.coeffRef(j, i) = t_eigen_vec;

			if (i < matrix_size && j < (matrix_size + 1))
			{
				eigen_vectors_matrix.coeffRef(j - 1, i) = t_eigen_vec;
			}
		}

		// Last node is free
	}


	// Create the eigen vectors inverse matrix
	double inv_factor = 2.0 / static_cast<float>(matrix_size + 0);

	//for (int i = 0; i < matrix_size; i++)
	//{
	//	if (i == (matrix_size - 1))
	//	{
	//		inv_factor = inv_factor * 0.5;
	//	}

	//	for (int j = 0; j < matrix_size; j++)
	//	{
	//		double length_ratio = (static_cast<float>((j * 2)+1) / static_cast<float>(node_count - 1));
	//		double t_eigen_vec = std::sin((i+1) * m_pi * length_ratio * 0.5);

	//		eigen_vectors_matrix_inverse.coeffRef(j, i) = inv_factor * t_eigen_vec;

	//	}
	//}

	eigen_vectors_matrix_inverse = inv_factor * eigen_vectors_matrix.transpose();

	// Edit the last column
	for (int j = 0; j < matrix_size; j++)
	{
		eigen_vectors_matrix_inverse.coeffRef(j, (matrix_size - 1)) = 0.5 * eigen_vectors_matrix_inverse.coeffRef(j, (matrix_size - 1));

	}

}


void modal_analysis_solver::modal_analysis_model_rectangular3(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const material_data& mat_data)
{
	// Free - Free Line 

	double line_length = mat_data.line_length;
	double c_param = std::sqrt(mat_data.line_tension / mat_data.material_density);
	this->number_of_modes = 0;

	for (int i = 0; i < node_count; i++)
	{
		// Eigen values
		double mode_number = (i + 2);

		double t_eigen = (mode_number * m_pi * c_param) / line_length;

		// Angular frequency wn
		angular_freq_vector.coeffRef(i) = t_eigen;

		// Eigen value
		eigen_values_vector.coeffRef(i) = (t_eigen * t_eigen);

		this->number_of_modes++;

		// Frequency
		double nat_freq = t_eigen / (2.0 * m_pi);

		// Modal results
		std::stringstream ss;
		ss << std::fixed << std::setprecision(3) << nat_freq;

		// Add to the string list
		if (this->number_of_modes < paint_mode_count)
		{
			mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz");
		}

		// First node is free

		for (int j = 0; j < node_count; j++)
		{
			double length_ratio = (static_cast<float>(j) / static_cast<float>(node_count - 1));
			double t_eigen_vec = std::sin((mode_number * m_pi * length_ratio) + (m_pi * 0.5));

			displ_vectors_matrix.coeffRef(j, i) = t_eigen_vec;

			eigen_vectors_matrix.coeffRef(j, i) = t_eigen_vec;
		}

		// Last node is free
	}

}


void modal_analysis_solver::map_modal_analysis_circular_results(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const elementquad_list_store& model_quadelements,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements,
	modal_elementquad_list_store& modal_result_quadelements)
{
	for (auto& nd_m : model_nodes.nodeMap)
	{
		int node_id = nd_m.first;
		glm::vec3 node_pt = nd_m.second.node_pt;
		int matrix_index = nodeid_map[node_id];

		// Modal analysis results
		std::unordered_map<int, glm::vec3> node_modal_displ;

		for (int i = 0; i < paint_mode_count; i++)
		{
			double displ_magnitude = static_cast<float>(displ_vectors_matrix.coeff(matrix_index, i));
			// get the appropriate modal displacement of this particular node
			glm::vec3 modal_displ = glm::vec3(0.0, 0.0, displ_magnitude);

			// add to modal result of this node
			node_modal_displ.insert({ i,modal_displ });
		}

		// Create the modal analysis result node
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

		modal_result_lineelements.add_modal_elementline(ln.line_id,
			&modal_result_nodes.modal_nodeMap[ln.startNode->node_id],
			&modal_result_nodes.modal_nodeMap[ln.endNode->node_id]);
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model Line Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________


	// Add the modal quad element result
	for (auto& quad_m : model_quadelements.elementquadMap)
	{
		elementquad_store quad = quad_m.second;

		modal_result_quadelements.add_modal_elementquadrilateral(quad.quad_id,
			&modal_result_nodes.modal_nodeMap[quad.nd1->node_id],
			&modal_result_nodes.modal_nodeMap[quad.nd2->node_id],
			&modal_result_nodes.modal_nodeMap[quad.nd3->node_id],
			&modal_result_nodes.modal_nodeMap[quad.nd4->node_id]);
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model Quad Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________


}


void modal_analysis_solver::map_modal_analysis_rectangular_results(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements)
{
	// Map the results to modal_result_nodes and modal_result_lineelements

	for (auto& nd_m : model_nodes.nodeMap)
	{
		int node_id = nd_m.first;

		// Modal analysis results
		std::unordered_map<int, glm::vec3> node_modal_displ;

		for (int i = 0; i < number_of_modes; i++)
		{
			// get the appropriate modal displacement of this particular node
			glm::vec3 modal_displ = glm::vec3(0.0,0.0, displ_vectors_matrix.coeff(node_id, i));

			// add to modal result of this node
			node_modal_displ.insert({ i,modal_displ });
		}

		// Create the modal analysis result node
		glm::vec3 node_pt = nd_m.second.node_pt;
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

		modal_result_lineelements.add_modal_elementline(ln.line_id,
			&modal_result_nodes.modal_nodeMap[ln.startNode->node_id],
			&modal_result_nodes.modal_nodeMap[ln.endNode->node_id]);
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________
}


void modal_analysis_solver::normalizeColumn(Eigen::MatrixXd& matrix, int columnIndex)
{
	// Find the maximum of elements in the specified column
	double max = matrix.col(columnIndex).maxCoeff();

	// Normalize each element in the column
	matrix.col(columnIndex) /= max;
}