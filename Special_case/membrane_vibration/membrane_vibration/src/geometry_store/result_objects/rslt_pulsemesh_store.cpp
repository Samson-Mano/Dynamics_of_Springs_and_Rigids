#include "rslt_pulsemesh_store.h"


rslt_pulsemesh_store::rslt_pulsemesh_store()
{
	// Empty constructor
}


void rslt_pulsemesh_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the mesh shader
	auto shaderSrc = ShaderLibrary::Get(ShaderLibrary::ShaderType::PulseRsltShader);

	rsltmesh_shader.create_shader_data(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

	// Clear mesh
	clear_mesh();
}

void rslt_pulsemesh_store::add_result_mesh(std::vector<rslt_pulsenode_store> rsltnodes,
	std::vector<elementline_store> wireframe,
	std::vector<elementtri_store> tris)
{
	// Clear the existing mesh
	clear_mesh();

	// Move instead of copy (O(1) pointer exchange)
	this->rsltnodes = std::move(rsltnodes);
	this->wireframe = std::move(wireframe);
	this->tris = std::move(tris);


	// Create the openGL objects
	create_buffer_data();

}

void rslt_pulsemesh_store::clear_mesh()
{
	prev_time_step = 0;
	number_of_timesteps = 0;

	// Clear mesh
	this->rsltnodes.clear();
	this->wireframe.clear();
	this->tris.clear();

	// Clear the OpenGL variables
	this->pointIdToIndex.clear();

	this->pointIndexData.clear();
	this->wireframeIndexData.clear();
	this->triangleIndexData.clear();
	//
}

void rslt_pulsemesh_store::create_buffer()
{
	// Create the point vertices
	// Define the node vertices of the model for a node (3 position, 3 Modal displacement value, 1 amplitude value) 
	int rsltnode_count = static_cast<int>(this->rsltnodes.size());
	const unsigned int point_vertex_count = 7 * rsltnode_count;


	//1.  Create the Dynamic vertex buffer (VBO) for the points
	unsigned int point_vertex_size = point_vertex_count * sizeof(float);
	// this->point_vbo.createVertexBuffer(pointVertices.data(), point_vertex_size);
	this->point_vbo.createDynamicVertexBuffer(point_vertex_count * sizeof(float));

	// this->point_vbo.updateVertexBuffer(pointVertices.data(), point_vertex_count * sizeof(float));


	//2. Create and add to the buffer layout
	VertexBufferLayout point_BufferLayout;
	point_BufferLayout.AddFloat(3); // Vertex layout
	point_BufferLayout.AddFloat(3); // Modal Displacement data
	point_BufferLayout.AddFloat(1); // Amplitude value

	//3. Create the vertex Array VAO (Add vertexBuffer binds both the vertexbuffer and vertexarray)
	this->point_vao.createVertexArray();
	this->point_vao.AddBuffer(this->point_vbo, point_BufferLayout);

	// Unbind VAO before creating IBOs!
	this->point_vao.UnBind();

	// 4. Create the index buffer object (IBO) for the points 
	// 4A. Create the point index data 
	this->point_ibo.createIndexBuffer(this->pointIndexData.data(),
		static_cast<unsigned int>(this->pointIndexData.size()));

	// 4B. Create the line index data 
	this->wireframe_ibo.createIndexBuffer(this->wireframeIndexData.data(),
		static_cast<unsigned int>(this->wireframeIndexData.size()));

	// 4C. Create the triangle index data
	this->triangle_ibo.createIndexBuffer(this->triangleIndexData.data(),
		static_cast<unsigned int>(this->triangleIndexData.size()));

}


void rslt_pulsemesh_store::update_buffer(int time_step)
{
	if (prev_time_step == time_step) return;

	if (time_step > number_of_timesteps) return;


	prev_time_step = time_step;

	// this->point_vbo.Bind();

	// Map buffer for direct writing
	float* vertexPtr = this->point_vbo.mapBuffer();

	if (!vertexPtr)
	{
		// Update the buffer
		// Update the point vertices VBO
		// Define the node vertices of the model for a node (3 position, 3 Modal displacement value, 1 amplitude value) 
		int rsltnode_count = static_cast<int>(this->rsltnodes.size());
		const unsigned int point_vertex_count = 7 * rsltnode_count;

		std::vector<float> pointVertices;

		for (const rslt_pulsenode_store& node : this->rsltnodes)
		{
			const glm::vec3& pt = node.node_pt;

			// Position
			pointVertices.push_back(pt.x);
			pointVertices.push_back(pt.y);
			pointVertices.push_back(pt.z);

			float pulse_displ_val = (node.node_displ_magnitude[time_step] - minimum_displacement) /
				(maximim_displacement - minimum_displacement);

			const glm::vec3& pulse_displ = node.node_displ[time_step] * pulse_displ_val;

			// Modal displacement values
			pointVertices.push_back(pulse_displ.x);
			pointVertices.push_back(pulse_displ.y);
			pointVertices.push_back(pulse_displ.z);


			// Modal displacement value
			pointVertices.push_back(pulse_displ_val);

		}

		this->point_vbo.updateVertexBuffer(pointVertices.data(), point_vertex_count * sizeof(float));
	}
	else
	{

		// Write directly to mapped memory (no temporary vector!)
		size_t offset = 0;

		for (const rslt_pulsenode_store& node : this->rsltnodes)
		{
			const glm::vec3& pt = node.node_pt;

			// Position
			vertexPtr[offset++] = pt.x;
			vertexPtr[offset++] = pt.y;
			vertexPtr[offset++] = pt.z;


			// Get original displacement
			const glm::vec3& original_displ = node.node_displ[time_step];
			float original_magnitude = node.node_displ_magnitude[time_step];

			// Scale magnitude to [-1, 1] range
			// First normalize to [0, 1], then map to [-1, 1]
			float t = (original_magnitude - minimum_displacement) / (maximim_displacement - minimum_displacement);
			float normalized_magnitude = t; // *2.0f - 1.0f;  // Convert [0,1] to [-1,1]

			// Scale the displacement vector preserving direction
			glm::vec3 scaled_displ = original_displ;

			if (original_magnitude > 1e-6f) 
			{
				// Scale to unit vector then multiply by normalized magnitude
				scaled_displ = glm::normalize(original_displ) * normalized_magnitude;
			}
			else {
				scaled_displ = glm::vec3(0.0f);
			}

			// Displacement values (scaled to [-1, 1] range)
			vertexPtr[offset++] = scaled_displ.x;
			vertexPtr[offset++] = scaled_displ.y;
			vertexPtr[offset++] = scaled_displ.z;

			// Normalized magnitude value in [-1, 1]
			vertexPtr[offset++] = normalized_magnitude;

		}

		// Unmap to make data available to GPU
		this->point_vbo.unmapBuffer();
	}

	// this->point_vbo.UnBind();

	//
}

void rslt_pulsemesh_store::paint_mesh()
{

	this->rsltmesh_shader.Bind();

	// Paint the mesh triangles, quadrilaterals
	this->point_vao.Bind();

	// Paint the triangle mesh
	this->triangle_ibo.Bind();

	glDrawElements(GL_TRIANGLES,
		static_cast<unsigned int>(triangleIndexData.size()),
		GL_UNSIGNED_INT,
		0);

	glBindBuffer(GL_ELEMENT_ARRAY_BUFFER, 0);

	this->triangle_ibo.UnBind();
	this->point_vao.UnBind();
	this->rsltmesh_shader.UnBind();


}

void rslt_pulsemesh_store::paint_mesh_wireframe()
{
	this->rsltmesh_shader.Bind();

	// Paint the mesh wireframe
	this->point_vao.Bind();
	this->wireframe_ibo.Bind();

	glDrawElements(GL_LINES,
		static_cast<unsigned int>(wireframeIndexData.size()),
		GL_UNSIGNED_INT,
		0);

	this->wireframe_ibo.UnBind();
	this->point_vao.UnBind();
	this->rsltmesh_shader.UnBind();

}


void rslt_pulsemesh_store::paint_mesh_points()
{
	this->rsltmesh_shader.Bind();

	// Paint the mesh points
	this->point_vao.Bind();
	this->point_ibo.Bind();

	glDrawElements(GL_POINTS,
		static_cast<unsigned int>(pointIndexData.size()),
		GL_UNSIGNED_INT,
		0);


	this->point_ibo.UnBind();
	this->point_vao.UnBind();
	this->rsltmesh_shader.UnBind();

}

void rslt_pulsemesh_store::update_openGLuniforms()
{

	// Update the shader uniforms for the mesh shader
	float zoomScale = static_cast<float>(geom_param_ptr->zoom_scale);
	glm::mat4 scalingMatrix = glm::scale(glm::mat4(1.0f),
		glm::vec3(zoomScale, zoomScale, zoomScale));

	// Note: Matrix4.Transpose in C#
	glm::mat4 viewMatrix = glm::transpose(geom_param_ptr->panTranslation) * scalingMatrix;

	// Compute MVP matrix
	glm::mat4 mvp = geom_param_ptr->projectionMatrix *
		viewMatrix *
		geom_param_ptr->rotateTranslation *
		geom_param_ptr->modelMatrix;


	rsltmesh_shader.setUniform("uMVP", mvp, false);
	rsltmesh_shader.setUniform("vTransparency", 0.8f);// Updating uniforms unBinds shader


}


void rslt_pulsemesh_store::update_animation_openGLuniforms()
{
	// Scale the visualization scale to match the geometry scale
	float visualization_defl_scale = this->geom_param_ptr->pulse_visualization_defl_scale *
		(this->geom_param_ptr->node_circle_radii / this->geom_param_ptr->geom_scale);

	rsltmesh_shader.setUniform("uDeflScale", visualization_defl_scale);

}


void rslt_pulsemesh_store::create_buffer_data()
{
	// Create the OpenGL buffer for the mesh
	this->pointIdToIndex.clear();
	this->pointIndexData.clear();

	for (int i = 0; i < static_cast<int>(this->rsltnodes.size()); i++)
	{
		this->pointIdToIndex[rsltnodes[i].node_id] = i;

		this->pointIndexData.push_back(i);
	}


	//_______________________________________________________________
	// prepare wireframe index data for openGL
	this->wireframeIndexData.clear();

	for (const elementline_store& line : this->wireframe)
	{
		auto it_start = this->pointIdToIndex.find(line.startnd_id);
		auto it_end = this->pointIdToIndex.find(line.endnd_id);

		if (it_start != this->pointIdToIndex.end() && it_end != this->pointIdToIndex.end())
		{
			this->wireframeIndexData.push_back(it_start->second);
			this->wireframeIndexData.push_back(it_end->second);
		}
	}

	//_______________________________________________________________
	// prepare triangle index data for openGL
	this->triangleIndexData.clear();

	for (const elementtri_store& tri : this->tris)
	{
		auto it_nd1 = this->pointIdToIndex.find(tri.nd1_id);
		auto it_nd2 = this->pointIdToIndex.find(tri.nd2_id);
		auto it_nd3 = this->pointIdToIndex.find(tri.nd3_id);

		if (it_nd1 != this->pointIdToIndex.end() && it_nd2 != this->pointIdToIndex.end() &&
			it_nd3 != this->pointIdToIndex.end())
		{
			this->triangleIndexData.push_back(it_nd1->second);
			this->triangleIndexData.push_back(it_nd2->second);
			this->triangleIndexData.push_back(it_nd3->second);
		}
	}

	//
}