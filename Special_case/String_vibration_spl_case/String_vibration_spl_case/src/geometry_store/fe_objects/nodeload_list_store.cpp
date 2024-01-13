#include "nodeload_list_store.h"

nodeload_list_store::nodeload_list_store()
{
	// Empty constructor
}

nodeload_list_store::~nodeload_list_store()
{
	// Empty destructor
}

void nodeload_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	load_value_labels.init(geom_param_ptr);

	// Create the shader and Texture for the drawing the constraints
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	load_shader.create_shader((shadersPath.string() + "/resources/shaders/load_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/load_frag_shader.frag").c_str());

	// Clear the loads
	load_count = 0;
	load_max = 0.0;
	loadMap.clear();
	all_load_ids.clear();
	all_load_setids.clear();
}

void nodeload_list_store::set_zero_condition(double& model_total_length,const int& number_of_nodes, const int& model_type)
{
	this->number_of_nodes = number_of_nodes; // Number of nodes
	this->model_total_length = model_total_length; // Total length
	this->model_type = model_type; // Model type 0,1 Line, 2,3 Circular
}

void nodeload_list_store::add_loads(nodes_list_store& model_nodes, double& load_start_time, double& load_end_time,
	double& load_value, int& node_start_id, int& node_end_id, int& interpolation_type)
{

	// Check 1 node start is less than node end
	if (node_start_id >= node_end_id)
		return;

	// Check 2 before adding (Node start is within the node range or not)
	if (node_start_id<0 || node_start_id >(number_of_nodes - 1))
		return;

	if (interpolation_type != 4)
	{
		// Ignore single node application (Only start node matters)
		// Check 3 before adding (Node end is within the node range or not)
		if (node_end_id<0 || node_end_id >(number_of_nodes - 1))
			return;
	}

	// Check 4 if there is any load value or not
	if (std::abs(load_value) < epsilon)
		return;

	// Declare 4 points for quadratic interpolation
	glm::vec2 pt1 = glm::vec2(0); // end point 1
	glm::vec2 pt2 = glm::vec2(0); // slope point 1
	glm::vec2 pt3 = glm::vec2(0); // end point 2
	glm::vec2 pt4 = glm::vec2(0); // slope point 2
	glm::vec2 pt_t = glm::vec2(0); // interpolation point
	double t_val = 0.0; // interpolation parameter t

	// temporary initial condition
	std::vector<load_data> temp_loadMap;

	//_____________________________________________________ Input is valid
	double load_spread_length = (static_cast<float>(node_end_id - node_start_id) / static_cast<float>(number_of_nodes));

	if (interpolation_type == 0)
	{
		// Linear interpolation
		int mid_node_id = ((node_start_id + node_end_id) / 2);

		// Positive slope interpolation end points
		pt1 = glm::vec2(0, 0);
		pt2 = glm::vec2(load_spread_length / 2.0f, load_value);

		// Go through start to mid node (Positive slope)
		for (int i = node_start_id; i < mid_node_id; i++)
		{
			t_val = static_cast<float>(i - node_start_id) / static_cast<float>(mid_node_id - node_start_id);

			pt_t = linear_interpolation(pt1, pt2, t_val);

			// Dont add zero loads
			if (std::abs(pt_t.y) < epsilon)
			{
				continue;
			}

			// Create a temporary load data
			load_data temp_load;
			temp_load.load_id = 0; //NULL Load id
			temp_load.node_id = i; // id of the line its applied to
			temp_load.load_loc = model_nodes.nodeMap[i].node_pt; // Load location
			temp_load.load_start_time = load_start_time; // Load start time
			temp_load.load_end_time = load_end_time; // Load end time
			temp_load.load_value = pt_t.y; // Load value
			temp_load.load_angle = get_load_angle(model_nodes.nodeMap[i].node_pt); // Load angle
			temp_load.show_load_label = false;

			// Add to the vector
			temp_loadMap.push_back(temp_load);
		}

		// Negative slope interpolation end points
		pt1 = glm::vec2(load_spread_length / 2.0f, load_value);
		pt2 = glm::vec2(load_spread_length, 0);

		// Go through mid to end node (Negative slope)
		for (int i = mid_node_id; i <= node_end_id; i++)
		{
			t_val = static_cast<float>(i - mid_node_id) / static_cast<float>(node_end_id - mid_node_id);

			pt_t = linear_interpolation(pt1, pt2, t_val);

			// Dont add zero loads
			if (std::abs(pt_t.y) < epsilon)
			{
				continue;
			}

			// Create a temporary load data
			load_data temp_load;
			temp_load.load_id = 0; //NULL Load id
			temp_load.node_id = i; // id of the line its applied to
			temp_load.load_loc = model_nodes.nodeMap[i].node_pt; // Load location
			temp_load.load_start_time = load_start_time; // Load start time
			temp_load.load_end_time = load_end_time; // Load end time
			temp_load.load_value = pt_t.y; // Load value
			temp_load.load_angle = get_load_angle(model_nodes.nodeMap[i].node_pt); // Load angle
			temp_load.show_load_label = false;

			if (pt_t.y == load_value)
			{
				temp_load.show_load_label = true;
			}

			// Add to the vector
			temp_loadMap.push_back(temp_load);
		}
	}
	else if (interpolation_type == 1)
	{
		// Cubic bezier interpolation
		// Linear interpolation
		int mid_node_id = ((node_start_id + node_end_id) / 2);

		// Positive slope interpolation end points
		pt1 = glm::vec2(0, 0);
		pt2 = glm::vec2(load_spread_length / 4.0f, 0);
		pt3 = glm::vec2(load_spread_length / 4.0f, load_value);
		pt4 = glm::vec2(load_spread_length / 2.0f, load_value);

		// Go through start to mid node (Positive slope)
		for (int i = node_start_id; i < mid_node_id; i++)
		{
			t_val = static_cast<float>(i - node_start_id) / static_cast<float>(mid_node_id - node_start_id);

			pt_t = cubic_bezier_interpolation(pt1, pt2, pt3, pt4, t_val);

			// Dont add zero loads
			if (std::abs(pt_t.y) < epsilon)
			{
				continue;
			}

			// Create a temporary load data
			load_data temp_load;
			temp_load.load_id = 0; //NULL Load id
			temp_load.node_id = i; // id of the line its applied to
			temp_load.load_loc = model_nodes.nodeMap[i].node_pt; // Load location
			temp_load.load_start_time = load_start_time; // Load start time
			temp_load.load_end_time = load_end_time; // Load end time
			temp_load.load_value = pt_t.y; // Load value
			temp_load.load_angle = get_load_angle(model_nodes.nodeMap[i].node_pt); // Load angle
			temp_load.show_load_label = false;

			// Add to the vector
			temp_loadMap.push_back(temp_load);
		}

		// Negative slope interpolation end points
		pt1 = glm::vec2(load_spread_length / 2.0f, load_value);
		pt2 = glm::vec2(load_spread_length * (3.0f / 4.0f), load_value);
		pt3 = glm::vec2(load_spread_length * (3.0f / 4.0f), 0);
		pt4 = glm::vec2(load_spread_length, 0);

		// Go through mid to end node (Negative slope)
		for (int i = mid_node_id; i <= node_end_id; i++)
		{
			t_val = static_cast<float>(i - mid_node_id) / static_cast<float>(node_end_id - mid_node_id);

			pt_t = cubic_bezier_interpolation(pt1, pt2, pt3, pt4, t_val);

			// Dont add zero loads
			if (std::abs(pt_t.y) < epsilon)
			{
				continue;
			}

			// Create a temporary load data
			load_data temp_load;
			temp_load.load_id = 0; //NULL Load id
			temp_load.node_id = i; // id of the line its applied to
			temp_load.load_loc = model_nodes.nodeMap[i].node_pt; // Load location
			temp_load.load_start_time = load_start_time; // Load start time
			temp_load.load_end_time = load_end_time; // Load end time
			temp_load.load_value = pt_t.y; // Load value
			temp_load.load_angle = get_load_angle(model_nodes.nodeMap[i].node_pt); // Load angle
			temp_load.show_load_label = false;

			if (pt_t.y == load_value)
			{
				temp_load.show_load_label = true;
			}

			// Add to the vector
			temp_loadMap.push_back(temp_load);
		}
	}
	else if (interpolation_type == 2)
	{
		// Sine interpolation
		pt1 = glm::vec2(0, 0);
		pt2 = glm::vec2(load_spread_length / 2.0f, load_value);
		pt3 = glm::vec2(load_spread_length, 0);

		int mid_node_id = ((node_start_id + node_end_id) / 2);

		// Go through start to end node 
		for (int i = node_start_id; i <= node_end_id; i++)
		{
			t_val = static_cast<float>(i - node_start_id) / static_cast<float>(node_end_id - node_start_id);

			pt_t = half_sine_interpolation(pt1, pt2, pt3, t_val);

			// Dont add zero loads
			if (std::abs(pt_t.y) < epsilon)
			{
				continue;
			}

			// Create a temporary load data
			load_data temp_load;
			temp_load.load_id = 0; //NULL Load id
			temp_load.node_id = i; // id of the line its applied to
			temp_load.load_loc = model_nodes.nodeMap[i].node_pt; // Load location
			temp_load.load_start_time = load_start_time; // Load start time
			temp_load.load_end_time = load_end_time; // Load end time
			temp_load.load_value = pt_t.y; // Load value
			temp_load.load_angle = get_load_angle(model_nodes.nodeMap[i].node_pt); // Load angle
			temp_load.show_load_label = false;

			if (i == mid_node_id)
			{
				temp_load.show_load_label = true;
			}

			// Add to the vector
			temp_loadMap.push_back(temp_load);
		}

	}
	else if (interpolation_type == 3)
	{
		// Rectangular interpolation
		int mid_node_id = ((node_start_id + node_end_id) / 2);

		// Go through start to end node 
		for (int i = node_start_id; i <= node_end_id; i++)
		{
			// Create a temporary load data
			load_data temp_load;
			temp_load.load_id = 0; //NULL Load id
			temp_load.node_id = i; // id of the line its applied to
			temp_load.load_loc = model_nodes.nodeMap[i].node_pt; // Load location
			temp_load.load_start_time = load_start_time; // Load start time
			temp_load.load_end_time = load_end_time; // Load end time
			temp_load.load_value = load_value; // Load value
			temp_load.load_angle = get_load_angle(model_nodes.nodeMap[i].node_pt); // Load angle
			temp_load.show_load_label = false;

			if (i == mid_node_id)
			{
				temp_load.show_load_label = true;
			}

			// Add to the vector
			temp_loadMap.push_back(temp_load);
		}

	}
	else if (interpolation_type == 4)
	{
		// Create a temporary load data
		load_data temp_load;
		temp_load.load_id = 0; //NULL Load id
		temp_load.node_id = node_start_id; // id of the line its applied to
		temp_load.load_loc = model_nodes.nodeMap[node_start_id].node_pt; // Load location
		temp_load.load_start_time = load_start_time; // Load start time
		temp_load.load_end_time = load_end_time; // Load end time
		temp_load.load_value = load_value; // Load value
		temp_load.load_angle = get_load_angle(model_nodes.nodeMap[node_start_id].node_pt); // Load angle
		temp_load.show_load_label = true;

		// Add to the vector
		temp_loadMap.push_back(temp_load);

	}
	

	int temp_load_setids = get_unique_load_id(all_load_setids);

	for (auto& temp_load : temp_loadMap)
	{
		// Insert the load to line
		int temp_load_id = get_unique_load_id(all_load_ids);
		temp_load.load_id = temp_load_id;
		temp_load.load_setid = temp_load_setids;
		loadMap.insert({ temp_load_id, temp_load });
		all_load_ids.push_back(temp_load_id); // Add to the id vector
		load_count++;
	}


	all_load_setids.push_back(temp_load_setids);

	set_buffer();
}

void nodeload_list_store::delete_all_loads()
{
	// Delete all the loads in the node
	load_count = 0;
	load_max = 0.0;
	loadMap.clear();
	all_load_ids.clear();
	all_load_setids.clear();

	set_buffer();
}

void nodeload_list_store::set_buffer()
{
	// Set the buffer for Loads
	if (load_count == 0)
	{
		// No load to paint
		// Set the load lables
		load_value_labels.clear_labels();
		return;
	}

	// Set the load max
	// Load Max
	load_max = 0.0;
	// Set the load lables
	load_value_labels.clear_labels();

	// Find the load maximum
	for (auto& loadx : loadMap)
	{
		load_data load = loadx.second;

		if (load_max < std::abs(load.load_value))
		{
			load_max = std::abs(load.load_value);
		}
		//__________________________________________________________________________

		if (load.show_load_label == true)
		{
			std::stringstream ss;
			ss << std::fixed << std::setprecision(geom_param_ptr->load_precision) << std::abs(load.load_value);

			glm::vec3 temp_color = geom_param_ptr->geom_colors.load_color;
			std::string	temp_str = "(" + std::to_string(load.node_id) + ") " + ss.str();
			double load_angle = load.load_angle;
			double load_angle_rad = ((90 - load_angle) * 3.14159365f) / 180.0f;

			bool is_load_val_above = false;
			if (load.load_value < 0)
			{
				is_load_val_above = true;
			}

			load_value_labels.add_text(temp_str, load.load_loc, glm::vec2(0), temp_color, load_angle_rad, is_load_val_above, false);
		}
	}

	load_value_labels.set_buffer();

	//__________________________________________________________________________

	unsigned int load_vertex_count = 4 * 7 * load_count;
	float* load_vertices = new float[load_vertex_count];

	unsigned int load_indices_count = 6 * load_count;
	unsigned int* load_indices = new unsigned int[load_indices_count];

	unsigned int load_v_index = 0;
	unsigned int load_i_index = 0;

	for (auto& ldx : loadMap)
	{
		load_data ld = ldx.second;

		// Add the load buffer
		get_load_buffer(ld, load_vertices, load_v_index, load_indices, load_i_index);
	}

	VertexBufferLayout load_layout;
	load_layout.AddFloat(2);  // Position
	load_layout.AddFloat(2);  // Center
	load_layout.AddFloat(3);  // Color

	unsigned int load_vertex_size = load_vertex_count * sizeof(float);

	// Create the Constraint buffers
	load_buffer.CreateBuffers(load_vertices, load_vertex_size,
		load_indices, load_indices_count, load_layout);

	// Delete the Dynamic arrays
	delete[] load_vertices;
	delete[] load_indices;
}

void nodeload_list_store::paint_loads()
{
	// Paint the loads
	load_shader.Bind();
	load_buffer.Bind();
	glDrawElements(GL_LINES, 6 * load_count, GL_UNSIGNED_INT, 0);
	load_buffer.UnBind();
	load_shader.UnBind();
}

void nodeload_list_store::paint_load_labels()
{
	// Paint load labels
	load_value_labels.paint_text();
}

void nodeload_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	// Update the load value label uniforms
	load_value_labels.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);

	if (set_modelmatrix == true)
	{
		// set the model matrix
		load_shader.setUniform("geom_scale", static_cast<float>(geom_param_ptr->geom_scale));
		load_shader.setUniform("transparency", 1.0f);

		load_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);
	}

	if (set_pantranslation == true)
	{
		// set the pan translation
		load_shader.setUniform("panTranslation", geom_param_ptr->panTranslation, false);
	}

	if (set_zoomtranslation == true)
	{
		// set the zoom translation
		load_shader.setUniform("zoomscale", static_cast<float>(geom_param_ptr->zoom_scale));
	}

	if (set_transparency == true)
	{
		// set the alpha transparency
		load_shader.setUniform("transparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

	if (set_deflscale == true)
	{
		// set the deflection scale
		// load_shader.setUniform("deflscale", static_cast<float>(geom_param_ptr->defl_scale));
	}
}

void nodeload_list_store::get_load_buffer(load_data& ld, float* load_vertices, unsigned int& load_v_index, unsigned int* load_indices, unsigned int& load_i_index)
{
	int load_sign = ld.load_value > 0 ? 1 : -1;

	glm::vec2 load_loc = ld.load_loc;
	glm::vec3 load_color = geom_param_ptr->geom_colors.load_color;

	// Rotate the corner points
	glm::vec2 load_arrow_startpt = glm::vec2(0, -1.0f * load_sign * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale))); // 0
	glm::vec2 load_arrow_endpt = glm::vec2(0, -20.0f * (ld.load_value / load_max) * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale))); // 1
	glm::vec2 load_arrow_pt1 = glm::vec2(0, -5.0f * load_sign * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale))); // 2
	glm::vec2 load_arrow_pt2 = glm::vec2(0, -5.0f * load_sign * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale))); // 3

	// Load angle
	double radians = ((ld.load_angle + 90.0) * 3.14159365) / 180.0; // convert degrees to radians
	double cos_theta = cos(radians);
	double sin_theta = sin(radians);

	glm::vec2 rotated_load_arrow_startpt = glm::vec2((load_arrow_startpt.x * cos_theta) + (load_arrow_startpt.y * sin_theta),
		-(load_arrow_startpt.x * sin_theta) + (load_arrow_startpt.y * cos_theta)); // 0
	glm::vec2 rotated_load_arrow_endpt = glm::vec2((load_arrow_endpt.x * cos_theta) + (load_arrow_endpt.y * sin_theta),
		-(load_arrow_endpt.x * sin_theta) + (load_arrow_endpt.y * cos_theta)); // 1


	// Load arrow point 1
	radians = ((ld.load_angle + 90.0 + 20.0) * 3.14159365) / 180.0; // convert degrees to radians
	cos_theta = cos(radians);
	sin_theta = sin(radians);

	glm::vec2 rotated_load_arrow_pt1 = glm::vec2((load_arrow_pt1.x * cos_theta) + (load_arrow_pt1.y * sin_theta),
		-(load_arrow_pt1.x * sin_theta) + (load_arrow_pt1.y * cos_theta)); // 2


	// Load arrow point 2
	radians = ((ld.load_angle + 90.0 - 20.0) * 3.14159365) / 180.0; // convert degrees to radians
	cos_theta = cos(radians);
	sin_theta = sin(radians);

	glm::vec2 rotated_load_arrow_pt2 = glm::vec2((load_arrow_pt2.x * cos_theta) + (load_arrow_pt2.y * sin_theta),
		-(load_arrow_pt2.x * sin_theta) + (load_arrow_pt2.y * cos_theta)); // 3

	//__________________________________________________________________________________________________________

	// Load 0th point
	// Position
	load_vertices[load_v_index + 0] = load_loc.x + rotated_load_arrow_startpt.x;
	load_vertices[load_v_index + 1] = load_loc.y + rotated_load_arrow_startpt.y;

	// Load location center
	load_vertices[load_v_index + 2] = load_loc.x;
	load_vertices[load_v_index + 3] = load_loc.y;

	// Load color
	load_vertices[load_v_index + 4] = load_color.x;
	load_vertices[load_v_index + 5] = load_color.y;
	load_vertices[load_v_index + 6] = load_color.z;

	load_v_index = load_v_index + 7;

	// Load 1th point
	// Position
	load_vertices[load_v_index + 0] = load_loc.x + rotated_load_arrow_endpt.x;
	load_vertices[load_v_index + 1] = load_loc.y + rotated_load_arrow_endpt.y;

	// Load location center
	load_vertices[load_v_index + 2] = load_loc.x;
	load_vertices[load_v_index + 3] = load_loc.y;

	// Load color
	load_vertices[load_v_index + 4] = load_color.x;
	load_vertices[load_v_index + 5] = load_color.y;
	load_vertices[load_v_index + 6] = load_color.z;

	load_v_index = load_v_index + 7;

	// Load 2th point
	// Position
	load_vertices[load_v_index + 0] = load_loc.x + rotated_load_arrow_pt1.x;
	load_vertices[load_v_index + 1] = load_loc.y + rotated_load_arrow_pt1.y;

	// Load location center
	load_vertices[load_v_index + 2] = load_loc.x;
	load_vertices[load_v_index + 3] = load_loc.y;

	// Load color
	load_vertices[load_v_index + 4] = load_color.x;
	load_vertices[load_v_index + 5] = load_color.y;
	load_vertices[load_v_index + 6] = load_color.z;

	load_v_index = load_v_index + 7;

	// Load 3th point
	// Position
	load_vertices[load_v_index + 0] = load_loc.x + rotated_load_arrow_pt2.x;
	load_vertices[load_v_index + 1] = load_loc.y + rotated_load_arrow_pt2.y;

	// Load location center
	load_vertices[load_v_index + 2] = load_loc.x;
	load_vertices[load_v_index + 3] = load_loc.y;

	// Load color
	load_vertices[load_v_index + 4] = load_color.x;
	load_vertices[load_v_index + 5] = load_color.y;
	load_vertices[load_v_index + 6] = load_color.z;

	load_v_index = load_v_index + 7;

	//______________________________________________________________________
	// 
	// Set the Load indices
	unsigned int t_id = ((load_i_index / 6) * 4);

	// Line 0,1
	load_indices[load_i_index + 0] = t_id + 0;
	load_indices[load_i_index + 1] = t_id + 1;

	// Line 0,2
	load_indices[load_i_index + 2] = t_id + 0;
	load_indices[load_i_index + 3] = t_id + 2;

	// Line 0,3
	load_indices[load_i_index + 4] = t_id + 0;
	load_indices[load_i_index + 5] = t_id + 3;

	// Increment
	load_i_index = load_i_index + 6;
}

int nodeload_list_store::get_unique_load_id(std::vector<int>& all_ids)
{
	// Return the unique Load id
	if (all_ids.size() != 0)
	{
		int i;
		std::sort(all_ids.begin(), all_ids.end());

		// Find if any of the nodes are missing in an ordered int
		for (i = 0; i < all_ids.size(); i++)
		{
			if (all_ids[i] != i)
			{
				return i;
			}
		}

		// no node id is missing in an ordered list so add to the end
		return static_cast<int>(all_ids.size());
	}
	return 0;
}



glm::vec2 nodeload_list_store::linear_interpolation(glm::vec2 pt1, glm::vec2 pt2, double t_val)
{
	double x = ((1 - t_val) * pt1.x) + (t_val * pt2.x);

	double y = ((1 - t_val) * pt1.y) + (t_val * pt2.y);

	return (glm::vec2(x, y));
}

glm::vec2 nodeload_list_store::cubic_bezier_interpolation(glm::vec2 pt1, glm::vec2 pt2, glm::vec2 pt3, glm::vec2 pt4, double t_val)
{
	double x = (std::pow((1 - t_val), 3) * pt1.x) +
		(3 * std::pow((1 - t_val), 2) * t_val * pt2.x) +
		(3 * (1 - t_val) * std::pow(t_val, 2) * pt3.x) +
		(std::pow(t_val, 3) * pt4.x);

	double y = (std::pow((1 - t_val), 3) * pt1.y) +
		(3 * std::pow((1 - t_val), 2) * t_val * pt2.y) +
		(3 * (1 - t_val) * std::pow(t_val, 2) * pt3.y) +
		(std::pow(t_val, 3) * pt4.y);

	return (glm::vec2(x, y));
}

glm::vec2 nodeload_list_store::half_sine_interpolation(glm::vec2 pt1, glm::vec2 pt2, glm::vec2 pt3, double t_val)
{
	// Calculate the half-sine weight for the y component
	double weightY = sin(t_val * glm::pi<double>());

	// Linearly interpolate the x component between pt1 and pt3
	double x = ((1.0 - t_val) * pt1.x) + (t_val * pt3.x);

	// Interpolate the y component between pt1.y and pt2.y using the weight
	double y = pt1.y + weightY * (pt2.y - pt1.y);

	return glm::vec2(x, y);
}

double nodeload_list_store::get_load_angle(const glm::vec2& node_pt)
{
	// Get Load angle
	if (model_type == 0 || model_type == 1 || model_type == 2)
	{
		// Line
		return 90.0;
	}
	else if (model_type == 3)
	{
		// Circular
		return (180.0 - (std::atan2(node_pt.y, node_pt.x) * (180.0 / 3.14159265358979323846)));
	}

}