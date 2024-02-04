#include "dynamic_tri_list_store.h"


dynamic_tri_list_store::dynamic_tri_list_store()
{
	// Empty constructor
}

dynamic_tri_list_store::~dynamic_tri_list_store()
{
	// Empty destructor
}

void dynamic_tri_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the point shader
	std::filesystem::path shadersPath = geom_param_ptr->resourcePath;

	dyn_tri_shader.create_shader((shadersPath.string() + "/resources/shaders/dyntripoint_vert_shader.vert").c_str(),
		(shadersPath.string() + "/resources/shaders/dyntripoint_frag_shader.frag").c_str());

	// Delete all the triangles
	dyn_tri_count = 0;
	dyn_triMap.clear();

}


void dynamic_tri_list_store::add_tri(int& tri_id, glm::vec3& tript1_loc, glm::vec3& tript2_loc, glm::vec3& tript3_loc,
	std::vector<glm::vec3>& tript1_offset, std::vector<glm::vec3>& tript2_offset, std::vector<glm::vec3>& tript3_offset,
	std::vector<glm::vec3>& tript1_color, std::vector<glm::vec3>& tript2_color, std::vector<glm::vec3>& tript3_color,
	std::vector<glm::vec3>& edge12_025color, std::vector<glm::vec3>& edge12_050color, std::vector<glm::vec3>& edge12_075color,
	std::vector<glm::vec3>& edge23_025color, std::vector<glm::vec3>& edge23_050color, std::vector<glm::vec3>& edge23_075color,
	std::vector<glm::vec3>& edge31_025color, std::vector<glm::vec3>& edge31_050color, std::vector<glm::vec3>& edge31_075color)
{
	// Create a temporary lines
	dynamic_tri_store dyn_temp_tri;
	dyn_temp_tri.tri_id = tri_id;

	// Tri points
	dyn_temp_tri.tript1_loc = tript1_loc;
	dyn_temp_tri.tript2_loc = tript2_loc;
	dyn_temp_tri.tript3_loc = tript3_loc;

	// Tri offsets
	dyn_temp_tri.tript1_offset = tript1_offset;
	dyn_temp_tri.tript2_offset = tript2_offset;
	dyn_temp_tri.tript3_offset = tript3_offset;

	// Tri vertex Color
	dyn_temp_tri.tript1_color = tript1_color;
	dyn_temp_tri.tript2_color = tript2_color;
	dyn_temp_tri.tript3_color = tript3_color;

	// Tri Edge color
	// Edge 12
	dyn_temp_tri.edge12_025color = edge12_025color;
	dyn_temp_tri.edge12_050color = edge12_050color;
	dyn_temp_tri.edge12_075color = edge12_075color;

	// Edge 23
	dyn_temp_tri.edge23_025color = edge23_025color;
	dyn_temp_tri.edge23_050color = edge23_050color;
	dyn_temp_tri.edge23_075color = edge23_075color;

	// Edge 31
	dyn_temp_tri.edge31_025color = edge31_025color;
	dyn_temp_tri.edge31_050color = edge31_050color;
	dyn_temp_tri.edge31_075color = edge31_075color;


	// Reserve space for the new element
	dyn_triMap.reserve(dyn_triMap.size() + 1);

	// Add to the list
	dyn_triMap.push_back(dyn_temp_tri);

	// Iterate the tri count
	dyn_tri_count++;

}


void dynamic_tri_list_store::set_buffer()
{
	// Set the buffer for index
	unsigned int tri_indices_count = 3 * dyn_tri_count; // 3 indices to form a triangle
	unsigned int* tri_vertex_indices = new unsigned int[tri_indices_count];

	unsigned int tri_i_index = 0;

	// Set the node vertices
	for (auto& tri : dyn_triMap)
	{
		// Add tri buffers
		get_tri_index_buffer(tri_vertex_indices, tri_i_index);
	}

	VertexBufferLayout tri_pt_layout;
	tri_pt_layout.AddFloat(3);  // Node center
	tri_pt_layout.AddFloat(3);  // Node offset
	tri_pt_layout.AddFloat(3);  // vertex Color
	tri_pt_layout.AddFloat(3);  // edge 025 Color
	tri_pt_layout.AddFloat(3);  // edge 050 Color
	tri_pt_layout.AddFloat(3);  // edge 075 Color

	// Define the node vertices of the model for a node (3 position, 3 defl, 3 color, 3 Edge 025, 3 Edge 050, 3 Edge 075 ) 
	const unsigned int tri_vertex_count = 18 * 3 * dyn_tri_count;
	unsigned int tri_vertex_size = tri_vertex_count * sizeof(float); // Size of the node_vertex

	// Create the Node Deflection buffers
	dyn_tri_buffer.CreateDynamicBuffers(tri_vertex_size, tri_vertex_indices, tri_indices_count, tri_pt_layout);

	// Delete the dynamic array
	delete[] tri_vertex_indices;
}


void dynamic_tri_list_store::paint_triangles(const int& dyn_index)
{
	// Paint all the triangles
	dyn_tri_shader.Bind();
	dyn_tri_buffer.Bind();

	// Update the tri buffer data for dynamic drawing
	update_buffer(dyn_index);

	glDrawElements(GL_TRIANGLES, (3 * dyn_tri_count), GL_UNSIGNED_INT, 0);
	dyn_tri_buffer.UnBind();
	dyn_tri_shader.UnBind();

}



void dynamic_tri_list_store::paint_triangles()
{
	// Paint all the triangles
	dyn_tri_shader.Bind();
	dyn_tri_buffer.Bind();

	// tri buffer updated seperately for dynamic drawing (To draw cyclic data ex. modal vectors)

	glDrawElements(GL_TRIANGLES, (3 * dyn_tri_count), GL_UNSIGNED_INT, 0);
	dyn_tri_buffer.UnBind();
	dyn_tri_shader.UnBind();

}

void dynamic_tri_list_store::update_buffer(const int& dyn_index)
{
	// Define the node vertices of the model for a node (3 position, 3 defl, 3 color, 3 Edge 025, 3 Edge 050, 3 Edge 075 ) 
	const unsigned int tri_vertex_count = 18 * 3 * dyn_tri_count;
	float* tri_vertices = new float[tri_vertex_count];

	unsigned int tri_v_index = 0;

	// Set the tri point vertices
	for (auto& tri : dyn_triMap)
	{
		// Add points buffers
		get_tri_vertex_buffer(tri, dyn_index, tri_vertices, tri_v_index);
	}

	unsigned int tri_vertex_size = tri_vertex_count * sizeof(float); // Size of the line point vertex

	// Update the buffer
	dyn_tri_buffer.UpdateDynamicVertexBuffer(tri_vertices, tri_vertex_size);

	// Delete the dynamic array
	delete[] tri_vertices;
}


void dynamic_tri_list_store::clear_triangles()
{
	// Delete all the triangles
	dyn_tri_count = 0;
	dyn_triMap.clear();

}


void dynamic_tri_list_store::update_opengl_uniforms(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	if (set_modelmatrix == true)
	{
		// set the model matrix
		dyn_tri_shader.setUniform("geom_scale", static_cast<float>(geom_param_ptr->geom_scale));
		dyn_tri_shader.setUniform("transparency", 0.6f);

		dyn_tri_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);
	}

	if (set_pantranslation == true)
	{
		// set the pan translation
		dyn_tri_shader.setUniform("panTranslation", geom_param_ptr->panTranslation, false);
	}

	if (set_rotatetranslation == true)
	{
		// set the rotate translation
		dyn_tri_shader.setUniform("rotateTranslation", geom_param_ptr->rotateTranslation, false);
	}

	if (set_zoomtranslation == true)
	{
		// set the zoom translation
		dyn_tri_shader.setUniform("zoomscale", static_cast<float>(geom_param_ptr->zoom_scale));
	}

	if (set_transparency == true)
	{
		// set the alpha transparency
		dyn_tri_shader.setUniform("transparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

	if (set_deflscale == true)
	{
		// set the deflection scale
		dyn_tri_shader.setUniform("normalized_deflscale", static_cast<float>(geom_param_ptr->normalized_defl_scale));
		dyn_tri_shader.setUniform("deflscale", static_cast<float>(geom_param_ptr->defl_scale));
	}

}

void dynamic_tri_list_store::get_tri_vertex_buffer(dynamic_tri_store& tri, const int& dyn_index,
	float* dyn_tri_vertices, unsigned int& dyn_tri_v_index)
{
	// Get the node buffer for the shader
	// Tri Point 1
	// Point location
	dyn_tri_vertices[dyn_tri_v_index + 0] = tri.tript1_loc.x;
	dyn_tri_vertices[dyn_tri_v_index + 1] = tri.tript1_loc.y;
	dyn_tri_vertices[dyn_tri_v_index + 2] = tri.tript1_loc.z;

	// Point offset
	dyn_tri_vertices[dyn_tri_v_index + 3] = tri.tript1_offset[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 4] = tri.tript1_offset[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 5] = tri.tript1_offset[dyn_index].z;

	// Point color
	dyn_tri_vertices[dyn_tri_v_index + 6] = tri.tript1_color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 7] = tri.tript1_color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 8] = tri.tript1_color[dyn_index].z;

	// Edge 12 025
	dyn_tri_vertices[dyn_tri_v_index + 9] = tri.edge12_025color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 10] = tri.edge12_025color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 11] = tri.edge12_025color[dyn_index].z;

	// Edge 12 050
	dyn_tri_vertices[dyn_tri_v_index + 12] = tri.edge12_050color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 13] = tri.edge12_050color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 14] = tri.edge12_050color[dyn_index].z;

	// Edge 12 075
	dyn_tri_vertices[dyn_tri_v_index + 15] = tri.edge12_075color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 16] = tri.edge12_075color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 17] = tri.edge12_075color[dyn_index].z;

	// Iterate
	dyn_tri_v_index = dyn_tri_v_index + 18;

	// Tri Point 2
	// Point location
	dyn_tri_vertices[dyn_tri_v_index + 0] = tri.tript2_loc.x;
	dyn_tri_vertices[dyn_tri_v_index + 1] = tri.tript2_loc.y;
	dyn_tri_vertices[dyn_tri_v_index + 2] = tri.tript2_loc.z;

	// Point offset
	dyn_tri_vertices[dyn_tri_v_index + 3] = tri.tript2_offset[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 4] = tri.tript2_offset[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 5] = tri.tript2_offset[dyn_index].z;

	// Point color
	dyn_tri_vertices[dyn_tri_v_index + 6] = tri.tript2_color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 7] = tri.tript2_color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 8] = tri.tript2_color[dyn_index].z;

	// Edge 23 025
	dyn_tri_vertices[dyn_tri_v_index + 9] = tri.edge23_025color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 10] = tri.edge23_025color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 11] = tri.edge23_025color[dyn_index].z;

	// Edge 23 050
	dyn_tri_vertices[dyn_tri_v_index + 12] = tri.edge23_050color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 13] = tri.edge23_050color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 14] = tri.edge23_050color[dyn_index].z;

	// Edge 23 075
	dyn_tri_vertices[dyn_tri_v_index + 15] = tri.edge23_075color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 16] = tri.edge23_075color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 17] = tri.edge23_075color[dyn_index].z;

	// Iterate
	dyn_tri_v_index = dyn_tri_v_index + 18;


	// Tri Point 3
	// Point location
	dyn_tri_vertices[dyn_tri_v_index + 0] = tri.tript3_loc.x;
	dyn_tri_vertices[dyn_tri_v_index + 1] = tri.tript3_loc.y;
	dyn_tri_vertices[dyn_tri_v_index + 2] = tri.tript3_loc.z;

	// Point offset
	dyn_tri_vertices[dyn_tri_v_index + 3] = tri.tript3_offset[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 4] = tri.tript3_offset[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 5] = tri.tript3_offset[dyn_index].z;

	// Point color
	dyn_tri_vertices[dyn_tri_v_index + 6] = tri.tript3_color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 7] = tri.tript3_color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 8] = tri.tript3_color[dyn_index].z;

	// Edge 31 025
	dyn_tri_vertices[dyn_tri_v_index + 9] = tri.edge31_025color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 10] = tri.edge31_025color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 11] = tri.edge31_025color[dyn_index].z;

	// Edge 31 050
	dyn_tri_vertices[dyn_tri_v_index + 12] = tri.edge31_050color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 13] = tri.edge31_050color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 14] = tri.edge31_050color[dyn_index].z;

	// Edge 31 075
	dyn_tri_vertices[dyn_tri_v_index + 15] = tri.edge31_075color[dyn_index].x;
	dyn_tri_vertices[dyn_tri_v_index + 16] = tri.edge31_075color[dyn_index].y;
	dyn_tri_vertices[dyn_tri_v_index + 17] = tri.edge31_075color[dyn_index].z;

	// Iterate
	dyn_tri_v_index = dyn_tri_v_index + 18;
}


void dynamic_tri_list_store::get_tri_index_buffer(unsigned int* dyn_tri_vertex_indices, unsigned int& dyn_tri_i_index)
{
	// Add the indices
	// Index 1
	dyn_tri_vertex_indices[dyn_tri_i_index] = dyn_tri_i_index;

	dyn_tri_i_index = dyn_tri_i_index + 1;

	// Index 2
	dyn_tri_vertex_indices[dyn_tri_i_index] = dyn_tri_i_index;

	dyn_tri_i_index = dyn_tri_i_index + 1;

	// Index 3
	dyn_tri_vertex_indices[dyn_tri_i_index] = dyn_tri_i_index;

	dyn_tri_i_index = dyn_tri_i_index + 1;

}