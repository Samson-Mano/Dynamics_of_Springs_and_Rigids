#include "dynamic_line_list_store.h"

dynamic_line_list_store::dynamic_line_list_store()
{
	// Empty constructor
}

dynamic_line_list_store::~dynamic_line_list_store()
{
	// Empty destructor
}

void dynamic_line_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the point shader
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	dyn_line_shader.create_shader((shadersPath.string() + "/resources/shaders/dyntripoint_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/dyntripoint_frag_shader.frag").c_str());

	// Delete all the lines
	dyn_line_count = 0;
	dyn_lineMap.clear();
}


void dynamic_line_list_store::add_line(int& line_id, 
	const glm::vec3& line_startpt_loc, const glm::vec3& line_endpt_loc,
	const std::vector<glm::vec3>& line_startpt_offset, const std::vector<glm::vec3>& line_endpt_offset,
	const std::vector<double>& line_startpt_offset_mag, const std::vector<double>& line_endpt_offset_mag)
{
	// Create a temporary lines
	dynamic_line_store dyn_temp_ln;
	dyn_temp_ln.line_id = line_id;

	// Line points
	dyn_temp_ln.line_startpt_loc = line_startpt_loc;
	dyn_temp_ln.line_endpt_loc = line_endpt_loc;

	// Line offsets
	dyn_temp_ln.line_startpt_offset = line_startpt_offset;
	dyn_temp_ln.line_endpt_offset = line_endpt_offset;

	// Line offset values
	dyn_temp_ln.line_startpt_offset_val = line_startpt_offset_mag;
	dyn_temp_ln.line_endpt_offset_val = line_endpt_offset_mag;

	//___________________________________________________________________
	// Add to the list
	dyn_lineMap.push_back(dyn_temp_ln);

	// Iterate the line count
	dyn_line_count++;
}

void dynamic_line_list_store::set_buffer()
{
	// Set the buffer for index
	unsigned int line_indices_count = 2 * dyn_line_count; // 2 indices to form a line
	unsigned int* line_vertex_indices = new unsigned int[line_indices_count];

	unsigned int line_i_index = 0;

	// Set the node vertices
	for (auto& ln : dyn_lineMap)
	{
		// Add line buffers
		get_line_index_buffer(line_vertex_indices, line_i_index);
	}

	VertexBufferLayout line_pt_layout;
	line_pt_layout.AddFloat(3);  // Node center
	line_pt_layout.AddFloat(3);  // Node offset
	line_pt_layout.AddFloat(1);  // Node Normalized deflection

	// Define the node vertices of the model for a node (3 position, 3 defl, 1 normalized defl length) 
	const unsigned int line_vertex_count = 7 * 2 * dyn_line_count;
	unsigned int line_vertex_size = line_vertex_count * sizeof(float); // Size of the node_vertex

	// Create the Node Deflection buffers
	dyn_line_buffer.CreateDynamicBuffers(line_vertex_size, line_vertex_indices, line_indices_count, line_pt_layout);

	// Delete the dynamic array
	delete[] line_vertex_indices;
}

void dynamic_line_list_store::paint_lines(const int& dyn_index)
{
	// Paint all the lines
	dyn_line_shader.Bind();
	dyn_line_buffer.Bind();

	// Update the line buffer data for dynamic drawing
	update_buffer(dyn_index);

	glDrawElements(GL_LINES, (2 * dyn_line_count), GL_UNSIGNED_INT, 0);
	dyn_line_buffer.UnBind();
	dyn_line_shader.UnBind();
}

void dynamic_line_list_store::paint_lines()
{
	// Paint all the lines
	dyn_line_shader.Bind();
	dyn_line_buffer.Bind();

	// line buffer updated seperately for dynamic drawing (To draw cyclic data ex. modal vectors)


	glDrawElements(GL_LINES, (2 * dyn_line_count), GL_UNSIGNED_INT, 0);
	dyn_line_buffer.UnBind();
	dyn_line_shader.UnBind();
}

void dynamic_line_list_store::update_buffer(const int& dyn_index)
{
	// Define the node vertices of the model for a node (3 position, 3 defl, 1 normalized defl length) 
	const unsigned int line_vertex_count = 7 * 2 * dyn_line_count;
	float* line_vertices = new float[line_vertex_count];

	unsigned int line_v_index = 0;

	// Set the line point vertices
	for (auto& ln : dyn_lineMap)
	{
		// Add points buffers
		get_line_vertex_buffer(ln, dyn_index, line_vertices, line_v_index);
	}

	unsigned int line_vertex_size = line_vertex_count * sizeof(float); // Size of the line point vertex

	// Update the buffer
	dyn_line_buffer.UpdateDynamicVertexBuffer(line_vertices, line_vertex_size);

	// Delete the dynamic array
	delete[] line_vertices;
}

void dynamic_line_list_store::clear_lines()
{
	// Delete all the lines
	dyn_line_count = 0;
	dyn_lineMap.clear();
}

void dynamic_line_list_store::update_opengl_uniforms(bool set_modelmatrix, 	bool set_pantranslation, bool set_rotatetranslation,
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	if (set_modelmatrix == true)
	{
		// set the model matrix
		dyn_line_shader.setUniform("geom_scale", static_cast<float>(geom_param_ptr->geom_scale));
		dyn_line_shader.setUniform("transparency", 1.0f);

		dyn_line_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);
	}

	if (set_pantranslation == true)
	{
		// set the pan translation
		dyn_line_shader.setUniform("panTranslation", geom_param_ptr->panTranslation, false);
	}

	if (set_rotatetranslation == true)
	{
		// set the rotate translation
		dyn_line_shader.setUniform("rotateTranslation", geom_param_ptr->rotateTranslation, false);
	}

	if (set_zoomtranslation == true)
	{
		// set the zoom translation
		dyn_line_shader.setUniform("zoomscale", static_cast<float>(geom_param_ptr->zoom_scale));
	}

	if (set_transparency == true)
	{
		// set the alpha transparency
		dyn_line_shader.setUniform("transparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

	if (set_deflscale == true)
	{
		// set the deflection scale
		dyn_line_shader.setUniform("normalized_deflscale", static_cast<float>(geom_param_ptr->normalized_defl_scale));
		dyn_line_shader.setUniform("deflscale", static_cast<float>(geom_param_ptr->defl_scale));
	}
}

void dynamic_line_list_store::get_line_vertex_buffer(dynamic_line_store& ln, const int& dyn_index,
	float* dyn_line_vertices, unsigned int& dyn_line_v_index)
{
	// Get the node buffer for the shader
	// Start Point
	// Point location
	dyn_line_vertices[dyn_line_v_index + 0] = ln.line_startpt_loc.x;
	dyn_line_vertices[dyn_line_v_index + 1] = ln.line_startpt_loc.y;
	dyn_line_vertices[dyn_line_v_index + 2] = ln.line_startpt_loc.z;

	// Point offset
	dyn_line_vertices[dyn_line_v_index + 3] = ln.line_startpt_offset[dyn_index].x;
	dyn_line_vertices[dyn_line_v_index + 4] = ln.line_startpt_offset[dyn_index].y;
	dyn_line_vertices[dyn_line_v_index + 5] = ln.line_startpt_offset[dyn_index].z;

	// Normalized offset length
	dyn_line_vertices[dyn_line_v_index + 6] = ln.line_startpt_offset_val[dyn_index];

	// Iterate
	dyn_line_v_index = dyn_line_v_index + 7;

	// End Point
	// Point location
	dyn_line_vertices[dyn_line_v_index + 0] = ln.line_endpt_loc.x;
	dyn_line_vertices[dyn_line_v_index + 1] = ln.line_endpt_loc.y;
	dyn_line_vertices[dyn_line_v_index + 2] = ln.line_endpt_loc.z;

	// Point offset
	dyn_line_vertices[dyn_line_v_index + 3] = ln.line_endpt_offset[dyn_index].x;
	dyn_line_vertices[dyn_line_v_index + 4] = ln.line_endpt_offset[dyn_index].y;
	dyn_line_vertices[dyn_line_v_index + 5] = ln.line_endpt_offset[dyn_index].z;

	// Normalized offset length
	dyn_line_vertices[dyn_line_v_index + 6] = ln.line_endpt_offset_val[dyn_index];

	// Iterate
	dyn_line_v_index = dyn_line_v_index + 7;
}

void dynamic_line_list_store::get_line_index_buffer(unsigned int* dyn_line_vertex_indices, unsigned int& dyn_line_i_index)
{
	//__________________________________________________________________________
	// Add the indices
	// Index 1
	dyn_line_vertex_indices[dyn_line_i_index] = dyn_line_i_index;

	dyn_line_i_index = dyn_line_i_index + 1;

	// Index 2
	dyn_line_vertex_indices[dyn_line_i_index] = dyn_line_i_index;

	dyn_line_i_index = dyn_line_i_index + 1;
}
