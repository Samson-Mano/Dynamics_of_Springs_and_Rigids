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
	node_count = 0;

	mode_result_str.clear();
	is_modal_analysis_complete = false;
}


void modal_analysis_solver::modal_analysis_start(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const nodeconstraint_list_store& node_constraints,
	const material_data& mat_data,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements)
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


	this->node_count = model_nodes.node_count;
	this->model_type = mat_data.model_type;

	// Displacement vectors matrix
	displ_vectors_matrix.resize(node_count, node_count);
	displ_vectors_matrix.setZero();

	this->matrix_size = 0;

	if (this->model_type == 0)
	{
		// Fixed - Fixed
		this->matrix_size = node_count - 2;
	}
	else if (this->model_type == 1)
	{
		// Fixed - Free
		this->matrix_size = node_count - 1;

	}
	else if (this->model_type == 3)
	{
		// Circular (Free - Free)
		this->matrix_size = node_count;
	}

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

	// Mode results as string
	mode_result_str.clear();

	if (mat_data.model_type == 0)
	{
		// Line (Fixed - Fixed)
		modal_analysis_model_linear1(model_nodes,
			model_lineelements,
			mat_data);

		// Map the results
		map_modal_analysis_linear_results(model_nodes,
			model_lineelements,
			modal_result_nodes,
			modal_result_lineelements);

		this->is_modal_analysis_complete = true;

	}
	else if (mat_data.model_type == 1)
	{
		// Line (Fixed - Free)
		modal_analysis_model_linear2(model_nodes,
			model_lineelements,
			mat_data);

		// Map the results
		map_modal_analysis_linear_results(model_nodes,
			model_lineelements,
			modal_result_nodes,
			modal_result_lineelements);

		this->is_modal_analysis_complete = true;

	}
	else if (mat_data.model_type == 2)
	{
		// Line (Free - Free)
		modal_analysis_model_linear3(model_nodes,
			model_lineelements,
			mat_data);

		// Map the results
		map_modal_analysis_linear_results(model_nodes,
			model_lineelements,
			modal_result_nodes,
			modal_result_lineelements);

		this->is_modal_analysis_complete = true;

	}
	else if (mat_data.model_type == 3)
	{
		// Circular (Free - Free)

		modal_analysis_model_circular1(model_nodes,
			model_lineelements,
			mat_data);

		// Map the results
		map_modal_analysis_circular_results(model_nodes,
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

void modal_analysis_solver::modal_analysis_model_linear1(const nodes_list_store& model_nodes,
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


void modal_analysis_solver::modal_analysis_model_linear2(const nodes_list_store& model_nodes,
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


void modal_analysis_solver::modal_analysis_model_linear3(const nodes_list_store& model_nodes,
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
		mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz");

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

void modal_analysis_solver::modal_analysis_model_circular1(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	const material_data& mat_data)
{
	// Circular string

	double line_length = mat_data.line_length;
	double c_param = std::sqrt(mat_data.line_tension / mat_data.material_density);
	this->number_of_modes = 0;
	int q = 1;

	for (int i = 0; i < node_count; i++)
	{
		//if (i < node_count - 1)
		//{
		double t_eigen = (q * m_pi * c_param) / line_length;

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

		// }

		int is_odd = i % 2;

		if (is_odd == 0)
		{
			// Cosine Mode
			// Even number 0,2,4,6 .....

			for (int j = 0; j < node_count; j++)
			{
				double angle_ratio = (static_cast<float>(j) / static_cast<float>(node_count)) * 2.0 * m_pi;
				double t_eigen_vec = std::cos(q * angle_ratio);

				displ_vectors_matrix.coeffRef(j, i) = t_eigen_vec;

				// if ((i < node_count - 1) && j < (node_count - 1))
				// {
				eigen_vectors_matrix.coeffRef(j, i) = t_eigen_vec;
				// }
			}
		}
		else
		{
			// Sine Mode
			// Odd number 1,3,5,7 .....

			for (int j = 0; j < node_count; j++)
			{
				double angle_ratio = (static_cast<float>(j) / static_cast<float>(node_count)) * 2.0 * m_pi;
				double t_eigen_vec = std::sin(q * angle_ratio);

				displ_vectors_matrix.coeffRef(j, i) = t_eigen_vec;

				// if ((i < node_count - 1) && j < (node_count - 1))
				// {
				eigen_vectors_matrix.coeffRef(j, i) = t_eigen_vec;
				// }
			}

			// Itereate q which is used to calculate the modal frequency 1,1,2,2,3,3,4..
			q++;
		}
	}

	// Modify the last column (Because all the values are zero )
	for (int j = 0; j < node_count; j++)
	{
		int is_odd = j % 2;
		if (is_odd == 0)
		{
			eigen_vectors_matrix.coeffRef(j, node_count - 1) = 0.0;
		}
		else
		{
			eigen_vectors_matrix.coeffRef(j, node_count - 1) = 0.0;
		}
	
	}


	//// Modify the last column (Because all the values are zero )
	//for (int j = 0; j < node_count; j++)
	//{
	//	int is_odd = (node_count - 1) % 2;
	//	double val = 0.0;
	//	if (is_odd == 0)
	//	{
	//		// Cosine Mode
	//		double angle_ratio = (static_cast<float>(j) / static_cast<float>(node_count)) * 2.0 * m_pi;
	//		val = std::cos(q * angle_ratio);
	//	}
	//	else
	//	{
	//		// Sine Mode
	//		double angle_ratio = (static_cast<float>(j) / static_cast<float>(node_count)) * 2.0 * m_pi;
	//		val = std::sin(q * angle_ratio);

	//	}
	//	eigen_vectors_matrix.coeffRef(j, node_count - 1) = val;
	//	displ_vectors_matrix.coeffRef(j, node_count - 1) = val;
	//}

	double inv_factor = 2.0 / static_cast<float>(matrix_size);

	// Create the eigen vectors inverse matrix

	eigen_vectors_matrix_inverse = inv_factor * eigen_vectors_matrix.transpose();


}


void modal_analysis_solver::map_modal_analysis_linear_results(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements)
{
	// Map the results to modal_result_nodes and modal_result_lineelements

	for (auto& nd_m : model_nodes.nodeMap)
	{
		int node_id = nd_m.first;

		// Modal analysis results
		std::unordered_map<int, glm::vec2> node_modal_displ;

		for (int i = 0; i < number_of_modes; i++)
		{
			// get the appropriate modal displacement of this particular node
			glm::vec2 modal_displ = glm::vec2(0.0, displ_vectors_matrix.coeff(node_id, i));

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

		modal_result_lineelements.add_modal_elementline(ln.line_id,
			&modal_result_nodes.modal_nodeMap[ln.startNode->node_id],
			&modal_result_nodes.modal_nodeMap[ln.endNode->node_id]);
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________
}


void modal_analysis_solver::map_modal_analysis_circular_results(const nodes_list_store& model_nodes,
	const elementline_list_store& model_lineelements,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements)
{
	for (auto& nd_m : model_nodes.nodeMap)
	{
		int node_id = nd_m.first;
		glm::vec2 node_pt = nd_m.second.node_pt;

		// Modal analysis results
		std::unordered_map<int, glm::vec2> node_modal_displ;

		for (int i = 0; i < number_of_modes; i++)
		{

			glm::vec2 normal_dir = glm::normalize(node_pt);

			// get the appropriate modal displacement of this particular node
			glm::vec2 modal_displ = static_cast<float>(displ_vectors_matrix.coeff(node_id, i)) * glm::vec2(normal_dir.x, -1.0 * normal_dir.y);

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
	std::cout << "Results mapped to model Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________


}
