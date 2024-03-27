#include "quad_list_store.h"

quad_list_store::quad_list_store()
{
	// Empty constructor
}

quad_list_store::~quad_list_store()
{
	// Empty destructor
}

void quad_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the point shader
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	quad_shader.create_shader((shadersPath.string() + "/resources/shaders/point_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/point_frag_shader.frag").c_str());

	// Delete all the triangles
	quad_count = 0;
	quadMap.clear();
}

void quad_list_store::add_quad(int& quad_id, const glm::vec3& quadpt1_loc, const glm::vec3& quadpt2_loc,
	const glm::vec3& quadpt3_loc, const glm::vec3& quadpt4_loc,
	glm::vec3& quadpt1_color, glm::vec3& quadpt2_color,
	glm::vec3& quadpt3_color, glm::vec3& quadpt4_color)
{
	// Create a temporary quads
	quad_store temp_quad;
	temp_quad.quad_id = quad_id;

	// Boundary Node points
	temp_quad.quadpt1_loc = quadpt1_loc;
	temp_quad.quadpt2_loc = quadpt2_loc;
	temp_quad.quadpt3_loc = quadpt3_loc;
	temp_quad.quadpt4_loc = quadpt4_loc;

	// Boundary Node Color
	temp_quad.quadpt1_color = quadpt1_color;
	temp_quad.quadpt2_color = quadpt2_color;
	temp_quad.quadpt3_color = quadpt3_color;
	temp_quad.quadpt4_color = quadpt4_color;

	// Add to the list
	quadMap.push_back(temp_quad);

	// Iterate the point count
	quad_count++;

}

void quad_list_store::set_buffer()
{
	// Define the quad vertices of the model for a node (3 position, 3 color) 4 points
	const unsigned int quad_vertex_count = 6 * 4 * quad_count;
	float* quad_vertices = new float[quad_vertex_count];

	unsigned int quad_indices_count = 6 * quad_count; // 6 indices to form a quadrilateral ( 3 + 3 triangles)
	unsigned int* quad_vertex_indices = new unsigned int[quad_indices_count];

	unsigned int quad_v_index = 0;
	unsigned int quad_i_index = 0;

	// Set the quad vertices
	for (auto& quad : quadMap)
	{
		// Add quadrilateral buffers
		get_quad_buffer(quad, quad_vertices, quad_v_index, quad_vertex_indices, quad_i_index);
	}

	VertexBufferLayout quad_pt_layout;
	quad_pt_layout.AddFloat(3);  // Node center
	quad_pt_layout.AddFloat(3);  // Node Color

	unsigned int quad_vertex_size = quad_vertex_count * sizeof(float); // Size of the node_vertex

	// Create the quadrilateral buffers
	quad_buffer.CreateBuffers(quad_vertices, quad_vertex_size, quad_vertex_indices, quad_indices_count, quad_pt_layout);

	// Delete the dynamic array
	delete[] quad_vertices;
	delete[] quad_vertex_indices;
}

void quad_list_store::paint_quadrilaterals()
{
	// Paint all the quadrilaterals
	quad_shader.Bind();
	quad_buffer.Bind();
	glDrawElements(GL_TRIANGLES, (6 * quad_count), GL_UNSIGNED_INT, 0);
	quad_buffer.UnBind();
	quad_shader.UnBind();
}

void quad_list_store::clear_quadrilaterals()
{
	// Delete all the quadrilaterals
	quad_count = 0;
	quadMap.clear();
}

void quad_list_store::update_opengl_uniforms(bool set_modelmatrix, bool set_pantranslation, 
	bool set_rotatetranslation, bool set_zoomtranslation, 
	bool set_transparency, bool set_deflscale)
{

	if (set_modelmatrix == true)
	{
		// set the model matrix
		quad_shader.setUniform("geom_scale", static_cast<float>(geom_param_ptr->geom_scale));
		quad_shader.setUniform("transparency", 0.8f);

		quad_shader.setUniform("projectionMatrix", geom_param_ptr->projectionMatrix, false);
		quad_shader.setUniform("viewMatrix", geom_param_ptr->viewMatrix, false);
		quad_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);
	}

	if (set_pantranslation == true)
	{
		// set the pan translation
		quad_shader.setUniform("panTranslation", geom_param_ptr->panTranslation, false);
	}

	if (set_rotatetranslation == true)
	{
		// set the rotate translation
		quad_shader.setUniform("rotateTranslation", geom_param_ptr->rotateTranslation, false);
	}

	if (set_zoomtranslation == true)
	{
		// set the zoom translation
		quad_shader.setUniform("zoomscale", static_cast<float>(geom_param_ptr->zoom_scale));
	}

	if (set_transparency == true)
	{
		// set the alpha transparency
		quad_shader.setUniform("transparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

	if (set_deflscale == true)
	{
		// set the deflection scale
		quad_shader.setUniform("normalized_deflscale", static_cast<float>(geom_param_ptr->normalized_defl_scale));
		quad_shader.setUniform("deflscale", static_cast<float>(geom_param_ptr->defl_scale));
	}
}

void quad_list_store::get_quad_buffer(quad_store& quad, float* quad_vertices, unsigned int& quad_v_index, 
	unsigned int* quad_vertex_indices, unsigned int& quad_i_index)
{
	// Get the three node buffer for the shader
	// Point 1
	// Point location
	quad_vertices[quad_v_index + 0] = quad.quadpt1_loc.x;
	quad_vertices[quad_v_index + 1] = quad.quadpt1_loc.y;
	quad_vertices[quad_v_index + 2] = quad.quadpt1_loc.z;

	// Point color
	quad_vertices[quad_v_index + 3] = quad.quadpt1_color.x;
	quad_vertices[quad_v_index + 4] = quad.quadpt1_color.y;
	quad_vertices[quad_v_index + 5] = quad.quadpt1_color.z;

	// Iterate
	quad_v_index = quad_v_index + 6;

	// Point 2
	// Point location
	quad_vertices[quad_v_index + 0] = quad.quadpt2_loc.x;
	quad_vertices[quad_v_index + 1] = quad.quadpt2_loc.y;
	quad_vertices[quad_v_index + 2] = quad.quadpt2_loc.z;

	// Point color
	quad_vertices[quad_v_index + 3] = quad.quadpt2_color.x;
	quad_vertices[quad_v_index + 4] = quad.quadpt2_color.y;
	quad_vertices[quad_v_index + 5] = quad.quadpt2_color.z;

	// Iterate
	quad_v_index = quad_v_index + 6;

	// Point 3
	// Point location
	quad_vertices[quad_v_index + 0] = quad.quadpt3_loc.x;
	quad_vertices[quad_v_index + 1] = quad.quadpt3_loc.y;
	quad_vertices[quad_v_index + 2] = quad.quadpt3_loc.z;

	// Point color
	quad_vertices[quad_v_index + 3] = quad.quadpt3_color.x;
	quad_vertices[quad_v_index + 4] = quad.quadpt3_color.y;
	quad_vertices[quad_v_index + 5] = quad.quadpt3_color.z;

	// Iterate
	quad_v_index = quad_v_index + 6;


	// Point 4
	// Point location
	quad_vertices[quad_v_index + 0] = quad.quadpt4_loc.x;
	quad_vertices[quad_v_index + 1] = quad.quadpt4_loc.y;
	quad_vertices[quad_v_index + 2] = quad.quadpt4_loc.z;

	// Point color
	quad_vertices[quad_v_index + 3] = quad.quadpt4_color.x;
	quad_vertices[quad_v_index + 4] = quad.quadpt4_color.y;
	quad_vertices[quad_v_index + 5] = quad.quadpt4_color.z;

	// Iterate
	quad_v_index = quad_v_index + 6;


	//__________________________________________________________________________
	// Add the indices
	// Index 0 1 2
	quad_vertex_indices[quad_i_index + 0] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 0;
	quad_vertex_indices[quad_i_index + 1] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 1;
	quad_vertex_indices[quad_i_index + 2] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 2;

	// Index 2 3 0
	quad_vertex_indices[quad_i_index + 3] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 2;
	quad_vertex_indices[quad_i_index + 4] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 3;
	quad_vertex_indices[quad_i_index + 5] = static_cast<int>((quad_i_index / 6.0) * 4.0) + 0;

	
	quad_i_index = quad_i_index + 6;

}
