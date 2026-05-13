#include "nodeinlcond_list_store.h"

nodeinlcond_list_store::nodeinlcond_list_store()
{
	// Empty constructor
}


void nodeinlcond_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	inlcond_value_labels.init(geom_param_ptr);

	auto shaderSrc = ShaderLibrary::Get(ShaderLibrary::ShaderType::LoadViewShader);

	inlcond_shader.createShader(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

	inlcondMap.clear();
	inlcond_count = 0;
}

void nodeinlcond_list_store::set_zero_condition(int inlcond_type, const int& model_type)
{
	this->inlcond_type = inlcond_type; // Initial condition type 0 - Displacement, 1 - Velocity
	this->model_type = model_type; // Model type 0, 1 Line, 2,3 Circle
}


void nodeinlcond_list_store::add_inlcondition(int& node_id, glm::vec3& inlcond_loc, double& inl_amplitude_z)
{
	// Add the initial condition to the particular node
	nodeinl_condition_data temp_inl_condition_data;
	temp_inl_condition_data.node_id = node_id;
	temp_inl_condition_data.inlcond_loc = inlcond_loc;
	temp_inl_condition_data.inl_amplitude_z = inl_amplitude_z;

	// Insert the inital condition data to unordered map
	// Searching for node_id
	if (inlcondMap.find(node_id) != inlcondMap.end())
	{
		// Node is already have constraint
		// so remove the constraint
		inlcondMap[node_id] = temp_inl_condition_data;

		return;
	}

	// Insert the constraint to nodes
	inlcondMap.insert({ node_id, temp_inl_condition_data });
	inlcond_count++;

}


void nodeinlcond_list_store::delete_inlcondition(int& node_id)
{
	// Delete the initial condition in this node
	if (inlcond_count != 0)
	{
		// Remove the intial condition data to unordered map
		// Searching for node_id
		// Check there is already a initial conditon in the found node
		if (inlcondMap.find(node_id) != inlcondMap.end())
		{
			// Node is already have initial condition
			// so remove the intial condition
			inlcondMap.erase(node_id);

			// Update the buffer
			set_buffer();

			// adjust the initial condition count
			inlcond_count--;
		}
	}
}

void nodeinlcond_list_store::set_buffer()
{
	// Set the buffer for initial condition
	if (inlcond_count == 0)
	{
		// No initial condition to paint
		// Clear the initial condition lables
		inlcond_value_labels.clear_labels();
		return;
	}


	// Reset the points based on the addition of new inl condition points
	// Set the initial condition amplitude max
	// initial condition amplitude Max
	inlcond_max = 0.0;
	// Clear the initial condition lables
	inlcond_value_labels.clear_labels();


	// Find the initial condition amplitude maximum
	for (auto& inlcond_m : inlcondMap)
	{
		nodeinl_condition_data inlcond = inlcond_m.second;

		if (inlcond_max < std::abs(inlcond.inl_amplitude_z))
		{
			inlcond_max = std::abs(inlcond.inl_amplitude_z);
		}
	}

	if (inlcond_max == 0)
	{
		// No initial condition value found
		return;
	}

	// Only show label for the single initial condition value
	std::unordered_set<double> inlcond_val_set;


	for (auto& inlcond_m : inlcondMap)
	{
		nodeinl_condition_data inlcond = inlcond_m.second;

		// Check whether the initial condition value exists or not
		if (inlcond_val_set.find(inlcond.inl_amplitude_z) == inlcond_val_set.end())
		{
			inlcond_val_set.insert(inlcond.inl_amplitude_z);

			std::stringstream ss;
			ss << std::fixed << std::setprecision(geom_param_ptr->load_precision) << std::abs(inlcond.inl_amplitude_z);

			glm::vec3 temp_color;

			if (inlcond_type == 0)
			{
				// Initial Displacement
				temp_color = geom_param_ptr->geom_colors.inlcond_displ_color;
			}
			else if (inlcond_type == 1)
			{
				// Initial Velocity
				temp_color = geom_param_ptr->geom_colors.inlcond_velo_color;
			}

			std::string	temp_str = "(" + std::to_string(inlcond.node_id) + ") " + ss.str();

			glm::vec3 inlcond_label_loc = inlcond.inlcond_loc;

			bool is_inlcond_val_above = false;
			if (inlcond.inl_amplitude_z < 0)
			{
				is_inlcond_val_above = true;
			}

			inlcond_value_labels.add_text(temp_str, inlcond_label_loc, is_inlcond_val_above, true);

		}
	}

	inlcond_value_labels.set_buffer();



	//__________________________________________________________________________

	unsigned int inlcond_vertex_count = 6 * 6 * inlcond_count; // 6 points to draw initial condition (3 position, 3 origin)
	float* inlcond_vertices = new float[inlcond_vertex_count];

	unsigned int inlcond_indices_count = 10 * inlcond_count;
	unsigned int* inlcond_indices = new unsigned int[inlcond_indices_count];

	unsigned int inlcond_v_index = 0;
	unsigned int inlcond_i_index = 0;


	for (auto& inlcond_m : inlcondMap)
	{
		nodeinl_condition_data inlcond = inlcond_m.second;

		// Add the initial condition buffer
		get_inlcond_buffer(inlcond, inlcond_vertices, inlcond_v_index, inlcond_indices, inlcond_i_index);

	}

	VertexBufferLayout inlcond_layout;
	inlcond_layout.AddFloat(3);  // Position
	inlcond_layout.AddFloat(3);  // Center

	unsigned int inlcond_vertex_size = inlcond_vertex_count * sizeof(float);

	// Create the Initial condition buffers
	inlcond_buffer.CreateBuffers(inlcond_vertices, inlcond_vertex_size,
		inlcond_indices, inlcond_indices_count, inlcond_layout);

	// Delete the Dynamic arrays
	delete[] inlcond_vertices;
	delete[] inlcond_indices;

}



void nodeinlcond_list_store::paint_inlcond()
{
	// Skip if no Initial condition
	if (inlcond_count == 0)
	{
		return;
	}

	// Paint the initial displacement points
	inlcond_shader.Bind();
	inlcond_buffer.Bind();
	glDrawElements(GL_LINES, 10 * inlcond_count, GL_UNSIGNED_INT, 0);
	inlcond_buffer.UnBind();
	inlcond_shader.UnBind();

}

void nodeinlcond_list_store::paint_inlcond_label()
{
	// Paint the initial condition labels
	inlcond_value_labels.paint_text();
}

void nodeinlcond_list_store::update_openGLuniforms()
{
	// Update the load value label uniforms
	inlcond_value_labels.update_openGLuniforms();


	// Update model openGL uniforms
	// Update the shader uniforms for the load shader
	float zoomScale = static_cast<float>(geom_param_ptr->zoom_scale);
	glm::mat4 scalingMatrix = glm::scale(glm::mat4(1.0f),
		glm::vec3(zoomScale, zoomScale, zoomScale));

	glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * scalingMatrix;

	// Compute MVP matrix
	glm::mat4 mvp = geom_param_ptr->projectionMatrix *
		viewMatrix *
		geom_param_ptr->rotateTranslation *
		geom_param_ptr->modelMatrix;

	inlcond_shader.Bind();
	inlcond_shader.setUniform("uMVP", mvp, false);
	inlcond_shader.setUniform("uZoomScale", zoomScale);


	if (inlcond_type == 0)
	{
		// Initial Displacement
		glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.inlcond_displ_color, geom_param_ptr->geom_transparency);

		inlcond_shader.setUniform("uVertexColor", vertexColor);
		inlcond_shader.UnBind();

		// Label color
		inlcond_value_labels.update_labelcolor(geom_param_ptr->geom_colors.inlcond_displ_color);
	}
	else if (inlcond_type == 1)
	{
		// Initial Velocity
		glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.inlcond_velo_color, geom_param_ptr->geom_transparency);

		inlcond_shader.setUniform("uVertexColor", vertexColor);
		inlcond_shader.UnBind();

		// Label color
		inlcond_value_labels.update_labelcolor(geom_param_ptr->geom_colors.inlcond_velo_color);
	}

	//
}



void nodeinlcond_list_store::get_inlcond_buffer(nodeinl_condition_data& inlcond, float* inlcond_vertices, unsigned int& inlcond_v_index,
	unsigned int* inlcond_indices, unsigned int& inlcond_i_index)
{

	// Slightly offset the start point
	int inlcond_amplitude_sign = inlcond.inl_amplitude_z > 0 ? 1 : -1;

	glm::vec2 inlcond_loc = glm::vec2(inlcond.inlcond_loc.x, inlcond.inlcond_loc.y);

	float inlcond_visualization_factor = inlcond_amplitude_sign *
		(geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale));
	float inlcond_scale = (inlcond.inl_amplitude_z / inlcond_max) *
		(geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale));

	glm::vec2 inlcond_dir;

	if (glm::length(inlcond_loc) > 0.001f)
	{
		// Direction from initial condition location towards origin (0,0)
		inlcond_dir = glm::normalize(-inlcond_loc);  // Simpler: direction from point to origin
		// Or explicitly: glm::normalize(glm::vec2(0.0f) - inlcond_loc);
	}
	else
	{
		// Initial condition is already at center, default direction (e.g., upward)
		inlcond_dir = glm::vec2(0.0f, 1.0f);
	}

	float arrowLength = -20.0f * inlcond_scale;
	float arrowheadSize = -5.0f * inlcond_visualization_factor;
	float arrowheadAngle = 10.0f;


	glm::vec2 inlcond_arrow_startpt = inlcond_loc - inlcond_dir * (0.2f * inlcond_visualization_factor);  // Start at load location with slight offset
	glm::vec2 inlcond_arrow_endpt = inlcond_loc + inlcond_dir * arrowLength;

	// Arrowhead wings
	glm::vec2 backward = -arrowheadSize * inlcond_dir;
	float angleRad = glm::radians(arrowheadAngle);

	glm::vec2 load_startarrow_pt1 = inlcond_arrow_startpt - glm::rotate(backward, angleRad);
	glm::vec2 load_startarrow_pt2 = inlcond_arrow_startpt - glm::rotate(backward, -angleRad);

	glm::vec2 load_endarrow_pt1 = inlcond_arrow_endpt + glm::rotate(backward, angleRad);
	glm::vec2 load_endarrow_pt2 = inlcond_arrow_endpt + glm::rotate(backward, -angleRad);



	//__________________________________________________________________________________________________________
	// Intial condition Line Start Point
	inlcond_vertices[inlcond_v_index + 0] = inlcond_arrow_startpt.x;
	inlcond_vertices[inlcond_v_index + 1] = inlcond_arrow_startpt.y;
	inlcond_vertices[inlcond_v_index + 2] = 0.0;

	// Load location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = 0.0;

	inlcond_v_index = inlcond_v_index + 6;

	//__________________________________________________________________________________________________________
	// Intial condition Line End Point
	inlcond_vertices[inlcond_v_index + 0] = inlcond_arrow_endpt.x;
	inlcond_vertices[inlcond_v_index + 1] = inlcond_arrow_endpt.y;
	inlcond_vertices[inlcond_v_index + 2] = 0.0;

	// initial condition location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = 0.0;

	inlcond_v_index = inlcond_v_index + 6;

	//__________________________________________________________________________________________________________
	// Intial condition Line Start Point - Arrow pt1
	inlcond_vertices[inlcond_v_index + 0] = load_startarrow_pt1.x;
	inlcond_vertices[inlcond_v_index + 1] = load_startarrow_pt1.y;
	inlcond_vertices[inlcond_v_index + 2] = 0.0;

	// initial condition location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = 0.0;

	inlcond_v_index = inlcond_v_index + 6;

	//__________________________________________________________________________________________________________
	// Intial condition Line Start Point - Arrow pt2
	inlcond_vertices[inlcond_v_index + 0] = load_startarrow_pt2.x;
	inlcond_vertices[inlcond_v_index + 1] = load_startarrow_pt2.y;
	inlcond_vertices[inlcond_v_index + 2] = 0.0;

	// initial condition location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = 0.0;

	inlcond_v_index = inlcond_v_index + 6;

	//__________________________________________________________________________________________________________
	// Intial condition Line End Point - Arrow pt1
	inlcond_vertices[inlcond_v_index + 0] = load_endarrow_pt1.x;
	inlcond_vertices[inlcond_v_index + 1] = load_endarrow_pt1.y;
	inlcond_vertices[inlcond_v_index + 2] = 0.0;

	// initial condition location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = 0.0;

	inlcond_v_index = inlcond_v_index + 6;

	//__________________________________________________________________________________________________________
	// Intial condition Line End Point - Arrow pt2
	inlcond_vertices[inlcond_v_index + 0] = load_endarrow_pt2.x;
	inlcond_vertices[inlcond_v_index + 1] = load_endarrow_pt2.y;
	inlcond_vertices[inlcond_v_index + 2] = 0.0;

	// initial condition location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = 0.0;

	inlcond_v_index = inlcond_v_index + 6;




	//______________________________________________________________________
	// Set the Initial condition line indices

	unsigned int t_id = ((inlcond_i_index / 10) * 6);

	// Line from start to end
	inlcond_indices[inlcond_i_index + 0] = t_id + 0;
	inlcond_indices[inlcond_i_index + 1] = t_id + 1;

	// Line from start to arrow head pt
	inlcond_indices[inlcond_i_index + 2] = t_id + 0;
	inlcond_indices[inlcond_i_index + 3] = t_id + 2;

	// Line from start to arrow head pt
	inlcond_indices[inlcond_i_index + 4] = t_id + 0;
	inlcond_indices[inlcond_i_index + 5] = t_id + 3;

	// Line from end to arrow head pt
	inlcond_indices[inlcond_i_index + 6] = t_id + 1;
	inlcond_indices[inlcond_i_index + 7] = t_id + 4;

	// Line from end to arrow head pt
	inlcond_indices[inlcond_i_index + 8] = t_id + 1;
	inlcond_indices[inlcond_i_index + 9] = t_id + 5;

	inlcond_i_index += 10;

}