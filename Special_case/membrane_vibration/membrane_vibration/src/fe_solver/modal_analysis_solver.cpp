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
			model_lineelements,
			model_quadelements,
			mat_data,
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
	const elementline_list_store& model_lineelements,
	const elementquad_list_store& model_quadelements,
	const material_data& mat_data,
	modal_nodes_list_store& modal_result_nodes,
	modal_elementline_list_store& modal_result_lineelements,
	modal_elementquad_list_store& modal_result_quadelements)
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

	std::unordered_map<int, quad_midnode_eigenvector_store> quad_midnode;

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

		if (this->number_of_modes <= paint_mode_count)
		{
			// Modal results
			std::stringstream ss;
			ss << std::fixed << std::setprecision(3) << nat_freq;

			// Add to the string list
			mode_result_str.push_back("Mode " + std::to_string(i + 1) + " = " + ss.str() + " Hz (m ="
				+ std::to_string(bessel_roots[i].m) + ", n=" + std::to_string(bessel_roots[i].n) + ")");
		}

		// Node
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
		double column_max = displ_vectors_matrix.col(i).maxCoeff();

		displ_vectors_matrix.col(i) = displ_vectors_matrix.col(i) / column_max;
		eigen_vectors_matrix.col(i) = eigen_vectors_matrix.col(i) / column_max;


		// Quad
		if (this->number_of_modes <= paint_mode_count)
		{
			for (auto& quad_m : model_quadelements.elementquadMap)
			{
				elementquad_store quad = quad_m.second;

				// Get the mid point of all the quad edges
				// Edge 13
				glm::vec3 edpt13_025 = geom_parameters::linear_interpolation3d(quad.nd1->node_pt,
					quad.nd3->node_pt, 0.25); // Mid of edge 13 0.25
				glm::vec3 edpt13_050 = geom_parameters::linear_interpolation3d(quad.nd1->node_pt,
					quad.nd3->node_pt, 0.50); // Mid of edge 13 0.50
				glm::vec3 edpt13_075 = geom_parameters::linear_interpolation3d(quad.nd1->node_pt,
					quad.nd3->node_pt, 0.75); // Mid of edge 13 0.75

				// Edge 32
				glm::vec3 edpt32_025 = geom_parameters::linear_interpolation3d(quad.nd3->node_pt,
					quad.nd2->node_pt, 0.25); // Mid of edge 32 0.25
				glm::vec3 edpt32_050 = geom_parameters::linear_interpolation3d(quad.nd3->node_pt,
					quad.nd2->node_pt, 0.50); // Mid of edge 32 0.50
				glm::vec3 edpt32_075 = geom_parameters::linear_interpolation3d(quad.nd3->node_pt,
					quad.nd2->node_pt, 0.75); // Mid of edge 32 0.75

				// Edge 21
				glm::vec3 edpt21_025 = geom_parameters::linear_interpolation3d(quad.nd2->node_pt,
					quad.nd1->node_pt, 0.25); // Mid of edge 21 0.25
				glm::vec3 edpt21_050 = geom_parameters::linear_interpolation3d(quad.nd2->node_pt,
					quad.nd1->node_pt, 0.50); // Mid of edge 21 0.50
				glm::vec3 edpt21_075 = geom_parameters::linear_interpolation3d(quad.nd2->node_pt,
					quad.nd1->node_pt, 0.75); // Mid of edge 21 0.75

				// Edge 14
				glm::vec3 edpt14_025 = geom_parameters::linear_interpolation3d(quad.nd1->node_pt,
					quad.nd4->node_pt, 0.25); // Mid of edge 14 0.25
				glm::vec3 edpt14_050 = geom_parameters::linear_interpolation3d(quad.nd1->node_pt,
					quad.nd4->node_pt, 0.50); // Mid of edge 14 0.50
				glm::vec3 edpt14_075 = geom_parameters::linear_interpolation3d(quad.nd1->node_pt,
					quad.nd4->node_pt, 0.75); // Mid of edge 14 0.75

				// Edge 43
				glm::vec3 edpt43_025 = geom_parameters::linear_interpolation3d(quad.nd4->node_pt,
					quad.nd3->node_pt, 0.25); // Mid of edge 43 0.25
				glm::vec3 edpt43_050 = geom_parameters::linear_interpolation3d(quad.nd4->node_pt,
					quad.nd3->node_pt, 0.50); // Mid of edge 43 0.50
				glm::vec3 edpt43_075 = geom_parameters::linear_interpolation3d(quad.nd4->node_pt,
					quad.nd3->node_pt, 0.75); // Mid of edge 43 0.75


				//_________________________________________________________________________________________________
				glm::vec3 edgeval13_025 = glm::vec3(0.0,0.0,
					bessel_eigen_vec(bessel_roots[i], edpt13_025, c_radius)/ column_max); // eigen vector at edge 13 0.25
				glm::vec3 edgeval13_050 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt13_050, c_radius) / column_max); // eigen vector at edge 13 0.50
				glm::vec3 edgeval13_075 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt13_075, c_radius) / column_max); // eigen vector at edge 13 0.75

				glm::vec3 edgeval32_025 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt32_025, c_radius) / column_max); // eigen vector at edge 32 0.25
				glm::vec3 edgeval32_050 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt32_050, c_radius) / column_max); // eigen vector at edge 32 0.50
				glm::vec3 edgeval32_075 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt32_075, c_radius) / column_max); // eigen vector at edge 32 0.75

				glm::vec3 edgeval21_025 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt21_025, c_radius) / column_max); // eigen vector at edge 21 0.25
				glm::vec3 edgeval21_050 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt21_050, c_radius) / column_max); // eigen vector at edge 21 0.50
				glm::vec3 edgeval21_075 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt21_075, c_radius) / column_max); // eigen vector at edge 21 0.75

				glm::vec3 edgeval14_025 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt14_025, c_radius) / column_max); // eigen vector at edge 14 0.25
				glm::vec3 edgeval14_050 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt14_050, c_radius) / column_max); // eigen vector at edge 14 0.50
				glm::vec3 edgeval14_075 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt14_075, c_radius) / column_max); // eigen vector at edge 14 0.75

				glm::vec3 edgeval43_025 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt43_025, c_radius) / column_max); // eigen vector at edge 43 0.25
				glm::vec3 edgeval43_050 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt43_050, c_radius) / column_max); // eigen vector at edge 43 0.50
				glm::vec3 edgeval43_075 = glm::vec3(0.0, 0.0, 
					bessel_eigen_vec(bessel_roots[i], edpt43_075, c_radius) / column_max); // eigen vector at edge 43 0.75

				
				// Add to the quad mid node data
				int quad_id = quad.quad_id;
				quad_midnode[quad_id].quad_id = quad_id;
				quad_midnode[quad_id].edge13_025.push_back(edgeval13_025); // eigen vector at edge 13 0.25
				quad_midnode[quad_id].edge13_050.push_back(edgeval13_050); // eigen vector at edge 13 0.50
				quad_midnode[quad_id].edge13_075.push_back(edgeval13_075); // eigen vector at edge 13 0.75

				quad_midnode[quad_id].edge32_025.push_back(edgeval32_025); // eigen vector at edge 32 0.25
				quad_midnode[quad_id].edge32_050.push_back(edgeval32_050); // eigen vector at edge 32 0.50
				quad_midnode[quad_id].edge32_075.push_back(edgeval32_075); // eigen vector at edge 32 0.75

				quad_midnode[quad_id].edge21_025.push_back(edgeval21_025); // eigen vector at edge 21 0.25
				quad_midnode[quad_id].edge21_050.push_back(edgeval21_050); // eigen vector at edge 21 0.50
				quad_midnode[quad_id].edge21_075.push_back(edgeval21_075); // eigen vector at edge 21 0.75

				quad_midnode[quad_id].edge14_025.push_back(edgeval14_025); // eigen vector at edge 14 0.25
				quad_midnode[quad_id].edge14_050.push_back(edgeval14_050); // eigen vector at edge 14 0.50
				quad_midnode[quad_id].edge14_075.push_back(edgeval14_075); // eigen vector at edge 14 0.75

				quad_midnode[quad_id].edge43_025.push_back(edgeval43_025); // eigen vector at edge 43 0.25
				quad_midnode[quad_id].edge43_050.push_back(edgeval43_050); // eigen vector at edge 43 0.50
				quad_midnode[quad_id].edge43_075.push_back(edgeval43_075); // eigen vector at edge 43 0.75

			}
		}
		
	}

	double inv_factor = 2.0 / static_cast<float>(matrix_size);

	// Create the eigen vectors inverse matrix

	eigen_vectors_matrix_inverse = inv_factor * eigen_vectors_matrix.transpose();

	//_____________________________________________________________________________________
	// Map the results

	for (auto& nd_m : model_nodes.nodeMap)
	{
		int node_id = nd_m.first;
		glm::vec3 node_pt = nd_m.second.node_pt;
		int matrix_index = nodeid_map[node_id];

		// Modal analysis results
		std::vector<glm::vec3> node_modal_displ;

		for (int i = 0; i < paint_mode_count; i++)
		{
			double displ_magnitude = static_cast<float>(displ_vectors_matrix.coeff(matrix_index, i));
			// get the appropriate modal displacement of this particular node
			glm::vec3 modal_displ = glm::vec3(0.0, 0.0, displ_magnitude);

			// add to modal result of this node
			node_modal_displ.push_back(modal_displ);
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
		int quad_id = quad.quad_id;

		modal_result_quadelements.add_modal_elementquadrilateral(quad_id,
			&modal_result_nodes.modal_nodeMap[quad.nd1->node_id],
			&modal_result_nodes.modal_nodeMap[quad.nd2->node_id],
			&modal_result_nodes.modal_nodeMap[quad.nd3->node_id],
			&modal_result_nodes.modal_nodeMap[quad.nd4->node_id],
			quad_midnode[quad_id].edge13_025,
			quad_midnode[quad_id].edge13_050,
			quad_midnode[quad_id].edge13_075,
			quad_midnode[quad_id].edge32_025,
			quad_midnode[quad_id].edge32_050,
			quad_midnode[quad_id].edge32_075,
			quad_midnode[quad_id].edge21_025,
			quad_midnode[quad_id].edge21_050,
			quad_midnode[quad_id].edge21_075,
			quad_midnode[quad_id].edge14_025,
			quad_midnode[quad_id].edge14_050,
			quad_midnode[quad_id].edge14_075,
			quad_midnode[quad_id].edge43_025,
			quad_midnode[quad_id].edge43_050,
			quad_midnode[quad_id].edge43_075);
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model Quad Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________



}


double modal_analysis_solver::bessel_eigen_vec(const bessel_function_Frequency& bessel_root_i, const glm::vec3& edgpt, const double& c_radius)
{
	double nd_radius = glm::length(edgpt); // get the radius (origin is 0,0)
	double nd_theta = std::atan2(edgpt.y, edgpt.x); // get the theta
	double radius_ratio = nd_radius / c_radius; // Radius ratio
	double t_eigen_vec = 0.0;

	// Eigen vector Mid of edge 1-2
	if (nd_radius < (c_radius - 0.1))
	{
		t_eigen_vec = std::cyl_bessel_j(bessel_root_i.m, bessel_root_i.root_value * radius_ratio) * std::cos(bessel_root_i.m * nd_theta);

	}

	return t_eigen_vec;
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
		std::vector<glm::vec3> node_modal_displ;

		for (int i = 0; i < number_of_modes; i++)
		{
			// get the appropriate modal displacement of this particular node
			glm::vec3 modal_displ = glm::vec3(0.0,0.0, displ_vectors_matrix.coeff(node_id, i));

			// add to modal result of this node
			node_modal_displ.push_back(modal_displ);
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

