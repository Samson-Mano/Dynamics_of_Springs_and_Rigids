#include "modal_analysis_solver.h"

modal_analysis_solver::modal_analysis_solver()
{
	// Empty constructor
}


void modal_analysis_solver::clear_results()
{
	// Clear the eigen values and eigen vectors
	constrained_node_map.clear(); // Constrained Node  map
	nodeid_map.clear(); // Node ID map
	number_of_modes = 0;
	node_count = 0;

	mode_result_str.clear();
	is_modal_analysis_complete = false;
}



void modal_analysis_solver::modal_analysis_start(const model_mesh_store& model_mesh,
	const material_data& mat_data,
	rslt_modalmesh_store& rslt_modalmesh)
{
	// Main solver call
	this->is_modal_analysis_complete = false;

	// Check the model
	// Number of nodes
	if (static_cast<int>(model_mesh.nodes.size()) == 0)
	{
		return;
	}

	// Number of lines
	if (static_cast<int>(model_mesh.wireframe.size()) == 0)
	{
		return;
	}

	// Number of quads/ tris
	if (static_cast<int>(model_mesh.tris.size() + model_mesh.quads.size()) == 0)
	{
		return;
	}

	//____________________________________________
	Eigen::initParallel();  // Initialize Eigen's thread pool

	stopwatch.start();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Modal analysis - started" << std::endl;
	//____________________________________________________________________________________________________________________


	this->node_count = static_cast<int>(model_mesh.nodes.size());
	this->model_type = mat_data.model_type;

	this->matrix_size = 0;

	if (this->model_type == 0)
	{
		// Circular Membrane
		// Number of fixed nodes = 128 or 256 test = 16
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
	else if(this->model_type == 4)
	{
		// Circular Membrane Triangles
		// Number of fixed nodes = 64
		// Radius = 100
		this->matrix_size = node_count - 64;
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

	// Re-initialize the variables
	rslt_modalmesh.clear_mesh();

	// modal_result_nodes.clear_data();
	// modal_result_lineelements.clear_data();
	// modal_result_quadelements.clear_data();

	// Mode results as string
	mode_result_str.clear();

	if (paint_mode_count > matrix_size)
	{
		paint_mode_count = matrix_size;
	}


	if (mat_data.model_type == 0 || mat_data.model_type == 4)
	{
		// Circular membrane
		double c_radius = 100.0;

		modal_analysis_model_circular(model_mesh,
			mat_data,
			c_radius,
			rslt_modalmesh);

		this->is_modal_analysis_complete = true;

	}
	else if (mat_data.model_type == 1)
	{
		

	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Modal analysis complete at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


}

void modal_analysis_solver::modal_analysis_model_circular(const model_mesh_store& model_mesh,
	const material_data& mat_data,
	const double& c_radius,
	rslt_modalmesh_store& rslt_modalmesh)
{
	// Circular string
	int node_id = 0;
	double c_param = std::sqrt(mat_data.line_tension / mat_data.material_density);
	
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
	eigen_freq.clear();
	int temp_mode_number = 0;

	// std::ofstream outFile("bessel_roots.txt"); // Open a file for writing

	for (auto& b_root : t_bessel_roots)
	{
		if (b_root.m == 0)
		{
			// m = 0 has no phase change
			bessel_function_Frequency temp_bessel_roots;
			temp_bessel_roots.mode_number = temp_mode_number;
			temp_bessel_roots.m = b_root.m;
			temp_bessel_roots.n = b_root.n;
			temp_bessel_roots.phase_pi = 0.0; // Phase = 0 when m = 0
			temp_bessel_roots.root_value = b_root.root_value;

			// Add to the list
			eigen_freq.push_back(temp_bessel_roots);

			// outFile << temp_mode_number << ", " << b_root.m << ", "<< b_root.n << ", "<<b_root.root_value << std::endl; // Write to the file

			temp_mode_number++;
		}
		else
		{
			// m not equal to zero // so add the anti mode 
			bessel_function_Frequency temp_bessel_roots_p0;
			temp_bessel_roots_p0.mode_number = temp_mode_number;
			temp_bessel_roots_p0.m = b_root.m;
			temp_bessel_roots_p0.n = b_root.n;
			temp_bessel_roots_p0.phase_pi = 0.0; // Phase = 0 when m = 0
			temp_bessel_roots_p0.root_value = b_root.root_value;

			// Add to the list
			eigen_freq.push_back(temp_bessel_roots_p0);

			// outFile << temp_mode_number << ", " << b_root.m << ", "<< b_root.n << ", "<<b_root.root_value << std::endl; // Write to the file

			temp_mode_number++;

			// Antimode with pi/2 * m phase
			bessel_function_Frequency temp_bessel_roots_p1;
			temp_bessel_roots_p1.mode_number = temp_mode_number;
			temp_bessel_roots_p1.m = b_root.m;
			temp_bessel_roots_p1.n = b_root.n;
			temp_bessel_roots_p1.phase_pi = (1.0 / static_cast<float>(b_root.m)) * (m_pi * 0.5); // Phase = (pi/2) * (1/m) when m not equal to 0
			temp_bessel_roots_p1.root_value = b_root.root_value;

			// Add to the list
			eigen_freq.push_back(temp_bessel_roots_p1);

			// outFile << temp_mode_number << ", " << b_root.m << ", "<< b_root.n << ", "<<b_root.root_value << std::endl; // Write to the file

			temp_mode_number++;

		}
		
	}

	// Resize the eigen freq size (to be inline with matrix size)
	eigen_freq.resize(this->matrix_size);


	// outFile.close(); // Close the file

	// Create the eigen vector node map
	int j = 0;
	this->constrained_node_map.clear();
	this->nodeid_map.clear();

	std::unordered_map<int, glm::vec3> nodept_map;

	for (auto& nd : model_mesh.nodes)
	{
		node_id = nd.node_id;
		glm::vec3 node_pt = nd.node_pt;
		double nd_radius = glm::length(node_pt);

		// Create the node point 
		nodept_map[node_id] = node_pt;


		if (nd_radius >= (c_radius- 0.1) )
		{
			// Zero for fixed nodes
			this->constrained_node_map[node_id] = true; // is constrained
			continue;
		}

		this->constrained_node_map[node_id] = false; // Node is not fixed
		this->nodeid_map[node_id] = j;
		j++;
	}

	//________________________________________________________________________________________________________________

	// Create the mode shapes
	this->number_of_modes = 0;

	std::unordered_map<int, quad_midnode_eigenvector_store> quad_midnode;

	// Create the quad mid nodes
	for (auto& quad : model_mesh.quads)
	{

		// Get the mid point of the quad
		std::vector<glm::vec3> quad_corner_pts;

		quad_corner_pts.push_back(nodept_map[quad.nd1_id]); // quad point 1
		quad_corner_pts.push_back(nodept_map[quad.nd2_id]); // quad point 2
		quad_corner_pts.push_back(nodept_map[quad.nd3_id]); // quad point 3
		quad_corner_pts.push_back(nodept_map[quad.nd4_id]); // quad point 4

		// Quad mid point
		glm::vec3 quad_midpt = geom_parameters::findGeometricCenter(quad_corner_pts);

		quad_midnode[quad.quad_id].mid_pt = quad_midpt;
	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Node maping completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	//________________________________________________________________________________________________________________
	int num_intervals = this->matrix_size;
	double progress_interval = num_intervals / 10.0;
	int last_printed_progress = -1; // Initialize with an invalid value

	for (int i =0; i<this->matrix_size; i++)
	{

		double t_eigen = (eigen_freq[i].root_value / c_radius) * c_param;

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
				+ std::to_string(eigen_freq[i].m) + ", n=" + std::to_string(eigen_freq[i].n) + ")");
		}

		// Node
		for (auto& nd : model_mesh.nodes)
		{
			// Get the node ID
			node_id = nd.node_id;

			if (constrained_node_map[node_id] == false)
			{
				// Node is not constrained, so get the co-ordinate
				glm::vec3 node_pt = nd.node_pt;

				int matrix_index = nodeid_map[node_id];

				// Eigen vectors
				double t_eigen_vec = bessel_eigen_vec(eigen_freq[i], node_pt, c_radius);

				// Add the eigen vectors
				eigen_vectors_matrix.coeffRef(matrix_index, i) = t_eigen_vec;
			}
		}

		// Normalize the eigen vector matrix
		double column_max = eigen_vectors_matrix.col(i).maxCoeff();

		eigen_vectors_matrix.col(i) = eigen_vectors_matrix.col(i) / column_max;

		// Quad
		// Quad Eigen vector calculation
		if (this->number_of_modes <= paint_mode_count)
		{
			for (auto& quad : model_mesh.quads)
			{

				int quad_id = quad.quad_id;

				//_________________________________________________________________________________________________
				double t_eigen_vec = bessel_eigen_vec(eigen_freq[i], quad_midnode[quad_id].mid_pt, c_radius) / column_max;

				glm::vec3 quad_midpt_eigvec = glm::vec3(0.0, 0.0,t_eigen_vec); // eigen vector at quad mid pt
				
				// Add to the quad mid node eigenvector list
				quad_midnode[quad_id].midpt_displ.push_back(quad_midpt_eigvec);
				quad_midnode[quad_id].midpt_displ_mag.push_back(std::abs(t_eigen_vec));

			}
		}

		// Calculate percentage progress
		int progress_percentage = static_cast<int>((i / static_cast<float>(this->matrix_size)) * 100);
		// Check if it's a new 10% interval
		if ((progress_percentage / 10) > last_printed_progress)
		{
			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << progress_percentage << "% modal analysis completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
			last_printed_progress = (progress_percentage / 10);
		}
	}


	//_____________________________________________________________________________________
	// Map the results

	// Add the modal quad element result
	int next_node_id = 0;
	int next_tri_id = 0;

	std::vector<rslt_modalnode_store> rsltnodes;
	std::vector<elementline_store> rsltwireframes;
	std::vector<elementtri_store> rslttris;
	std::unordered_map<int, int> added_nodes;

	// Lambda to create node lambda (captures all needed data)
	auto create_node = [&](int node_id) -> int {
		return get_or_create_node(node_id, added_nodes, rsltnodes,
			nodept_map, constrained_node_map, nodeid_map,
			eigen_vectors_matrix, paint_mode_count, next_node_id);
		};



	// Add the modal tri element result
	for (auto& tri : model_mesh.tris)
	{
		// Get or create corner nodes
		int tri_node_id1 = create_node(tri.nd1_id);
		int tri_node_id2 = create_node(tri.nd2_id);
		int tri_node_id3 = create_node(tri.nd3_id);

		// Create 1 Triangle
		rslttris.emplace_back(next_tri_id++, tri_node_id1, tri_node_id2, tri_node_id3);

	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model Tri Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	for (auto& quad : model_mesh.quads)
	{
		// Get or create corner nodes
		int quad_node_id1 = create_node(quad.nd1_id);
		int quad_node_id2 = create_node(quad.nd2_id);
		int quad_node_id3 = create_node(quad.nd3_id);
		int quad_node_id4 = create_node(quad.nd4_id);

		// Create mid node (unique to this quad, not shared)
		int quad_node_idmid = next_node_id++;
		rsltnodes.emplace_back(quad_node_idmid,
			quad_midnode[quad.quad_id].mid_pt,
			quad_midnode[quad.quad_id].midpt_displ,
			quad_midnode[quad.quad_id].midpt_displ_mag);

		// Create 4 triangles
		rslttris.emplace_back(next_tri_id++, quad_node_id1, quad_node_id2, quad_node_idmid);
		rslttris.emplace_back(next_tri_id++, quad_node_id2, quad_node_id3, quad_node_idmid);
		rslttris.emplace_back(next_tri_id++, quad_node_id3, quad_node_id4, quad_node_idmid);
		rslttris.emplace_back(next_tri_id++, quad_node_id4, quad_node_id1, quad_node_idmid);

	}

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Results mapped to model Quad Elements at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
	//____________________________________________________________________________________________________________________


	// Add the wire frame element result
	for (auto& ln : model_mesh.wireframe)
	{
		int line_id = ln.line_id;
		int ln_startnd_id = added_nodes.at(ln.startnd_id);
		int ln_endnd_id = added_nodes.at(ln.endnd_id);

		rsltwireframes.emplace_back(line_id, ln_startnd_id, ln_endnd_id);

	}


	// Add to the result mesh
	rslt_modalmesh.add_result_mesh(rsltnodes, rsltwireframes, rslttris);


}


double modal_analysis_solver::bessel_eigen_vec(const bessel_function_Frequency& bessel_root_i, const glm::vec3& nodept, const double& c_radius)
{
	double nd_radius = glm::length(nodept); // get the radius (origin is 0,0)
	double nd_theta = std::atan2(nodept.y, nodept.x); // get the theta
	double radius_ratio = nd_radius / c_radius; // Radius ratio
	double t_eigen_vec = 0.0;

	// Eigen vector at the node point
	if (nd_radius < (c_radius - 0.1))
	{
		t_eigen_vec = std::cyl_bessel_j(bessel_root_i.m, bessel_root_i.root_value * radius_ratio) * std::cos(bessel_root_i.m *( nd_theta + bessel_root_i.phase_pi));

	}

	return t_eigen_vec;
}




int modal_analysis_solver::get_or_create_node(int original_node_id,
	std::unordered_map<int, int>& added_nodes,
	std::vector<rslt_modalnode_store>& rsltnodes,
	const std::unordered_map<int, glm::vec3>& nodept_map,
	const std::unordered_map<int, bool>& constrained_node_map,
	const std::unordered_map<int, int>& nodeid_map,
	const Eigen::MatrixXd& eigen_vectors_matrix,
	int paint_mode_count,
	int& next_node_id)
{
	auto it = added_nodes.find(original_node_id);
	if (it != added_nodes.end()) 
	{
		return it->second;  // Node already exists
	}

	// Create new result node
	int new_node_id = next_node_id++;
	added_nodes[original_node_id] = new_node_id;

	glm::vec3 node_pt = nodept_map.at(original_node_id);

	std::vector<glm::vec3> node_modal_displ;
	std::vector<double> node_modal_displ_magnitude;

	// Check if node is constrained
	if (!constrained_node_map.at(original_node_id)) {
		int matrix_index = nodeid_map.at(original_node_id);
		node_modal_displ.reserve(paint_mode_count);
		node_modal_displ_magnitude.reserve(paint_mode_count);

		for (int i = 0; i < paint_mode_count; i++) {
			double displ_magnitude = static_cast<float>(eigen_vectors_matrix.coeff(matrix_index, i));

			// Assuming Z-direction displacement (modify if needed)
			glm::vec3 modal_displ(0.0, 0.0, displ_magnitude);

			node_modal_displ.push_back(modal_displ);
			node_modal_displ_magnitude.push_back(std::abs(displ_magnitude));
		}
	}
	else {
		// Constrained node - all zeros
		node_modal_displ.assign(paint_mode_count, glm::vec3(0.0f));
		node_modal_displ_magnitude.assign(paint_mode_count, 0.0);
	}

	rsltnodes.emplace_back(new_node_id, node_pt, node_modal_displ, node_modal_displ_magnitude);
	return new_node_id;

}



