#include "nodeinlcond_list_store.h"

nodeinlcond_list_store::nodeinlcond_list_store()
{
	// Empty constructor
}


void nodeinlcond_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	auto shaderSrc = ShaderLibrary::Get(ShaderLibrary::ShaderType::LoadViewShader);

	inlcond_shader.create_shader_data(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

	inlcondMap.clear();
	inlcond_count = 0;
}

void nodeinlcond_list_store::set_zero_condition(int inlcond_type,const int& model_type)
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

	// Reset the points based on the addition of new inl condition points
	// Set the initial condition amplitude max
	// initial condition amplitude Max
	inlcond_max = 0.0;

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


	//__________________________________________________________________________

	unsigned int inlcond_vertex_count = 2 * 6 * inlcond_count; // 2 points to draw load (3 position, 3 origin)
	float* inlcond_vertices = new float[inlcond_vertex_count];

	unsigned int inlcond_indices_count = 2 * inlcond_count;
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
	// Paint the initial displacement points
	inlcond_shader.Bind();
	inlcond_buffer.Bind();
	glDrawElements(GL_LINES, 2 * inlcond_count, GL_UNSIGNED_INT, 0);
	inlcond_buffer.UnBind();
	inlcond_shader.UnBind();

}

void nodeinlcond_list_store::paint_inlcond_label()
{
	// Paint the peak displacement label
	// inl_condition_labels.paint_text();
}

void nodeinlcond_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
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


	inlcond_shader.setUniform("uMVP", mvp, false);
	inlcond_shader.setUniform("uZoomScale", zoomScale);


	if (inlcond_type == 0)
	{
		// Initial Displacement
		glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.inlcond_displ_color, geom_param_ptr->geom_transparency);

		inlcond_shader.setUniform("uVertexColor", vertexColor);
	}
	else if (inlcond_type == 1)
	{
		// Initial Velocity
		glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.inlcond_velo_color, geom_param_ptr->geom_transparency);

		inlcond_shader.setUniform("uVertexColor", vertexColor);
	}

	//
}



void nodeinlcond_list_store::get_inlcond_buffer(nodeinl_condition_data& inlcond, float* inlcond_vertices, unsigned int& inlcond_v_index,
	unsigned int* inlcond_indices, unsigned int& inlcond_i_index)
{

	// initial condition point amplitude
	double pt_amplitude = -10.0f * (inlcond.inl_amplitude_z / inlcond_max) * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale));

	// initial condition point
	glm::vec3 inlcond_loc = inlcond.inlcond_loc;
	glm::vec3 inlcond_endpt = glm::vec3(0.0, 0.0, pt_amplitude);


	// Slightly offset the start point
	int inlcond_amplitude_sign = pt_amplitude > 0 ? 1 : -1;

	glm::vec3 inlcond_startpt = glm::vec3(0.0, 0.0,
		-1.0f * inlcond_amplitude_sign * (geom_param_ptr->node_circle_radii / static_cast<float>(geom_param_ptr->geom_scale))); // 0



	//__________________________________________________________________________________________________________
	// Intial condition Line Start Point
	inlcond_vertices[inlcond_v_index + 0] = inlcond_loc.x + inlcond_startpt.x;
	inlcond_vertices[inlcond_v_index + 1] = inlcond_loc.y + inlcond_startpt.y;
	inlcond_vertices[inlcond_v_index + 2] = inlcond_loc.z + inlcond_startpt.z;

	// Load location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = inlcond_loc.z;

	inlcond_v_index = inlcond_v_index + 6;

	//__________________________________________________________________________________________________________
	// Intial condition Line End Point
	inlcond_vertices[inlcond_v_index + 0] = inlcond_loc.x + inlcond_endpt.x;
	inlcond_vertices[inlcond_v_index + 1] = inlcond_loc.y + inlcond_endpt.y;
	inlcond_vertices[inlcond_v_index + 2] = inlcond_loc.z + inlcond_endpt.z;

	// initial condition location center
	inlcond_vertices[inlcond_v_index + 3] = inlcond_loc.x;
	inlcond_vertices[inlcond_v_index + 4] = inlcond_loc.y;
	inlcond_vertices[inlcond_v_index + 5] = inlcond_loc.z;

	inlcond_v_index = inlcond_v_index + 6;


	//______________________________________________________________________
	// Set the Initial condition line indices

	inlcond_indices[inlcond_i_index + 0] = inlcond_i_index + 0;
	inlcond_indices[inlcond_i_index + 1] = inlcond_i_index + 1;

	inlcond_i_index += 2;

}