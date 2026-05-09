#include "model_mesh_store.h"

model_mesh_store::model_mesh_store()
{
	// Empty constructor
}


void model_mesh_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the mesh shader
	 auto shaderSrc = ShaderLibrary::Get(ShaderLibrary::ShaderType::MeshShader);

	 mesh_shader.create_shader_data(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

	// Clear mesh
	clear_mesh();

}


void model_mesh_store::add_mesh(std::vector<node_store> nodes,
	std::vector<elementtri_store> tris,
	std::vector<elementquad_store> quads)
{
	// Clear the existing mesh
	clear_mesh();

	// Move instead of copy (O(1) pointer exchange)
	this->nodes = std::move(nodes);
	this->tris = std::move(tris);
	this->quads = std::move(quads);

	// Create the wire frame
	create_wireframe();

	// Create the openGL objects
	create_buffer_data();

}


void model_mesh_store::clear_mesh()
{
	// Clear mesh
	this->nodes.clear();
	this->wireframe.clear();
	this->tris.clear();
	this->quads.clear();

	// Clear the OpenGL variables
	this->pointIdToIndex.clear();

	this->vertexData.clear();
	this->vertexnormalData.clear();
	this->pointIndexData.clear();
	this->selectedpointIndexData.clear();
	this->wireframeIndexData.clear();
	this->triangleIndexData.clear();
	this->quadrilateralIndexData.clear();
	//
}


void model_mesh_store::create_buffer_data()
{
	// Create the OpenGL buffer for the mesh
	this->pointIdToIndex.clear();
	this->pointIndexData.clear();

	// Store normals for each point
	std::vector<glm::vec3> vnormals;

	for (int i = 0; i < static_cast<int>(this->nodes.size()); i++)
	{
		this->pointIdToIndex[nodes[i].node_id] = i;

		// Initialize normals to zero
		vnormals.push_back(glm::vec3(0));

		this->pointIndexData.push_back(i);
	}

	//_______________________________________________________________
	// prepare the Vertex data for openGL
	this->vertexData.clear();

	for (const node_store& node : this->nodes)
	{
		const glm::vec3& pt = node.node_pt;
		this->vertexData.push_back(pt.x);
		this->vertexData.push_back(pt.y);
		this->vertexData.push_back(pt.z);

		//int nd_id = node.node_id;
		//glm::vec3 ptcoord = pt;

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

	//_______________________________________________________________
	// prepare quadrilateral index data for openGL
	this->quadrilateralIndexData.clear();

	for (const elementquad_store& quad : this->quads)
	{
		auto it_nd1 = this->pointIdToIndex.find(quad.nd1_id);
		auto it_nd2 = this->pointIdToIndex.find(quad.nd2_id);
		auto it_nd3 = this->pointIdToIndex.find(quad.nd3_id);
		auto it_nd4 = this->pointIdToIndex.find(quad.nd4_id);


		if (it_nd1 != this->pointIdToIndex.end() && it_nd2 != this->pointIdToIndex.end() &&
			it_nd3 != this->pointIdToIndex.end() && it_nd4 != this->pointIdToIndex.end())
		{
			// Make two triangles from the quad (pt1, pt2, pt3) and (pt1, pt3, pt4)
			// Triangle 1 (nd1, nd2, nd3)
			this->quadrilateralIndexData.push_back(it_nd1->second);
			this->quadrilateralIndexData.push_back(it_nd2->second);
			this->quadrilateralIndexData.push_back(it_nd3->second);

			// Triangle 2 (nd1, nd3, nd4)
			this->quadrilateralIndexData.push_back(it_nd1->second);
			this->quadrilateralIndexData.push_back(it_nd3->second);
			this->quadrilateralIndexData.push_back(it_nd4->second);
		}
	}

	//_______________________________________________________________
	// Prepare vertex normal data for OpenGL
	create_vertex_normals(vnormals);
	//
}


void model_mesh_store::create_vertex_normals(std::vector<glm::vec3>& vnormals)
{
	const float EPSILON = 1e-12f;

	// Helper lambda to get vertex position by index
	auto get_vertex = [this](int index) -> glm::vec3
		{
			int i3 = index * 3;
			return glm::vec3(
				this->vertexData[i3 + 0],
				this->vertexData[i3 + 1],
				this->vertexData[i3 + 2]
			);
		};

	// Process triangles
	for (size_t i = 0; i < this->triangleIndexData.size(); i += 3)
	{
		int i0 = static_cast<int>(this->triangleIndexData[i]);
		int i1 = static_cast<int>(this->triangleIndexData[i + 1]);
		int i2 = static_cast<int>(this->triangleIndexData[i + 2]);

		glm::vec3 v0 = get_vertex(i0);
		glm::vec3 v1 = get_vertex(i1);
		glm::vec3 v2 = get_vertex(i2);

		// Edge vectors
		glm::vec3 e1 = v1 - v0;
		glm::vec3 e2 = v2 - v0;

		// Face normal (area-weighted)
		glm::vec3 faceNormal = glm::cross(e1, e2);

		// Accumulate to vertices
		vnormals[i0] += faceNormal;
		vnormals[i1] += faceNormal;
		vnormals[i2] += faceNormal;
	}

	// Process quads (split into two triangles)
	for (size_t i = 0; i < this->quadrilateralIndexData.size(); i += 6)
	{
		// Triangle 1 indices
		int i0 = static_cast<int>(this->quadrilateralIndexData[i]);
		int i1 = static_cast<int>(this->quadrilateralIndexData[i + 1]);
		int i2 = static_cast<int>(this->quadrilateralIndexData[i + 2]);

		glm::vec3 v0 = get_vertex(i0);
		glm::vec3 v1 = get_vertex(i1);
		glm::vec3 v2 = get_vertex(i2);

		glm::vec3 e1 = v1 - v0;
		glm::vec3 e2 = v2 - v0;
		glm::vec3 faceNormal1 = glm::cross(e1, e2);

		vnormals[i0] += faceNormal1;
		vnormals[i1] += faceNormal1;
		vnormals[i2] += faceNormal1;

		// Triangle 2 indices
		int i3 = static_cast<int>(this->quadrilateralIndexData[i + 3]);
		int i4 = static_cast<int>(this->quadrilateralIndexData[i + 4]);
		int i5 = static_cast<int>(this->quadrilateralIndexData[i + 5]);

		glm::vec3 v3 = get_vertex(i3);
		glm::vec3 v4 = get_vertex(i4);
		glm::vec3 v5 = get_vertex(i5);

		glm::vec3 e3 = v4 - v3;
		glm::vec3 e4 = v5 - v3;
		glm::vec3 faceNormal2 = glm::cross(e3, e4);

		vnormals[i3] += faceNormal2;
		vnormals[i4] += faceNormal2;
		vnormals[i5] += faceNormal2;
	}

	// Normalize and store final normals
	this->vertexnormalData.clear();
	this->vertexnormalData.reserve(vnormals.size() * 3);

	const glm::vec3 DEFAULT_NORMAL(0.0f, 0.0f, 1.0f);

	for (const auto& normal : vnormals)
	{
		float length1 = glm::length(normal);
		float lengthSq =  length1 * length1;
		glm::vec3 normalized;

		if (lengthSq > EPSILON)
		{
			normalized = glm::normalize(normal);
		}
		else
		{
			normalized = DEFAULT_NORMAL;
		}

		this->vertexnormalData.push_back(normalized.x);
		this->vertexnormalData.push_back(normalized.y);
		this->vertexnormalData.push_back(normalized.z);
	}
	//
}


void model_mesh_store::create_buffer()
{
	// Create the point vertices
	// Define the node vertices of the model for a node (3 position) 
	int node_count = static_cast<int>(this->nodes.size());
	const unsigned int point_vertex_count = 3 * node_count;

	std::vector<float> pointVertices;

	for (int i = 0; i < node_count; i++)
	{
		int vi = i * 3;

		// X, Y, Z Co-ordinate
		pointVertices.push_back(vertexData[vi + 0]);
		pointVertices.push_back(vertexData[vi + 1]);
		pointVertices.push_back(vertexData[vi + 2]);

	}


	//1.  Create the vertex buffer (VBO) for the points
	unsigned int point_vertex_size = point_vertex_count * sizeof(float);
	this->point_vbo.createVertexBuffer(pointVertices.data(), point_vertex_size);
	// this->point_vbo.createDynamicVertexBuffer(point_vertex_count * sizeof(float));
	// this->point_vbo.updateVertexBuffer(pointVertices.data(), point_vertex_count * sizeof(float));


	//2. Create and add to the buffer layout
	VertexBufferLayout point_BufferLayout;
	point_BufferLayout.AddFloat(3); // Vertex layout

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

	// 4D. Create the quad index data (2 Triangles per quad)
	this->quadrilateral_ibo.createIndexBuffer(this->quadrilateralIndexData.data(),
		static_cast<unsigned int>(this->quadrilateralIndexData.size()));


}


void model_mesh_store::paint_mesh()
{
	glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.triangle_color, geom_param_ptr->geom_transparency);
	this->mesh_shader.setUniform("vertexColor", vertexColor);// Updating uniforms unBinds shader
	this->mesh_shader.Bind();


	// Paint the mesh triangles, quadrilaterals
	this->point_vao.Bind();

	// Paint the triangle mesh
	this->triangle_ibo.Bind();
	
	glDrawElements(GL_TRIANGLES,
		static_cast<unsigned int>(triangleIndexData.size()),
		GL_UNSIGNED_INT,
		0);

	this->triangle_ibo.UnBind();

	// Paint the quadrilateral mesh
	this->quadrilateral_ibo.Bind();

	glDrawElements(GL_TRIANGLES,
		static_cast<unsigned int>(quadrilateralIndexData.size()),
		GL_UNSIGNED_INT,
		0);

	this->quadrilateral_ibo.UnBind();
	this->point_vao.UnBind();
	this->mesh_shader.UnBind();

}


void model_mesh_store::paint_mesh_wireframe()
{
	
	glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.line_color, geom_param_ptr->geom_transparency);
	this->mesh_shader.setUniform("vertexColor", vertexColor);// Updating uniforms unBinds shader
	this->mesh_shader.Bind();

	// Paint the mesh wireframe
	this->point_vao.Bind();
	this->wireframe_ibo.Bind();

	glDrawElements(GL_LINES,
		static_cast<unsigned int>(wireframeIndexData.size()),
		GL_UNSIGNED_INT,
		0);

	this->wireframe_ibo.UnBind();
	this->point_vao.UnBind();
	this->mesh_shader.UnBind();
}



void model_mesh_store::paint_mesh_points()
{

	glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.node_color, geom_param_ptr->geom_transparency);
	this->mesh_shader.setUniform("vertexColor", vertexColor);// Updating uniforms unBinds shader
	this->mesh_shader.Bind();

	// Paint the mesh points
	this->point_vao.Bind();
	this->point_ibo.Bind();

	glDrawElements(GL_POINTS,
		static_cast<unsigned int>(pointIndexData.size()),
		GL_UNSIGNED_INT,
		0);


	this->point_ibo.UnBind();
	this->point_vao.UnBind();
	this->mesh_shader.UnBind();

}

void model_mesh_store::paint_selected_mesh_points()
{

	glm::vec4 vertexColor = glm::vec4(geom_param_ptr->geom_colors.selection_color, geom_param_ptr->geom_transparency);
	this->mesh_shader.setUniform("vertexColor", vertexColor);// Updating uniforms unBinds shader
	this->mesh_shader.Bind();

	// Paint the selected mesh points
	this->point_vao.Bind();
	this->selected_point_ibo.Bind();

	glDrawElements(GL_POINTS,
		static_cast<int>(selectedpointIndexData.size()),
		GL_UNSIGNED_INT,
		0);

	this->selected_point_ibo.UnBind();
	this->point_vao.UnBind();
	this->mesh_shader.UnBind();

}


void model_mesh_store::update_openGLuniforms()
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


	mesh_shader.setUniform("uMVP", mvp, false);

	//
}


void model_mesh_store::create_wireframe()
{
	// Create the wireframe from the mesh data
	this->wireframe.clear();

	// Unordered map to track the unique edges
	std::set<std::pair<int, int>> edgeSet;  // Automatic uniqueness
	int wireframeLineId = 0;

	auto add_unique_edge = [&](int a, int b)
		{
			auto edge = std::make_pair(std::min(a, b), std::max(a, b));
			if (edgeSet.insert(edge).second)
			{
				wireframe.emplace_back(wireframeLineId++, edge.first, edge.second);
			}
		};


	// Triangles
	for (const elementtri_store& tri : this->tris)
	{
		add_unique_edge(tri.nd1_id, tri.nd2_id);
		add_unique_edge(tri.nd2_id, tri.nd3_id);
		add_unique_edge(tri.nd3_id, tri.nd1_id);
	}

	// Quadrilaterals
	for (const elementquad_store& quad : this->quads)
	{
		add_unique_edge(quad.nd1_id, quad.nd2_id);
		add_unique_edge(quad.nd2_id, quad.nd3_id);
		add_unique_edge(quad.nd3_id, quad.nd4_id);
		add_unique_edge(quad.nd4_id, quad.nd1_id);
	}

}


void model_mesh_store::add_selection_nodes(std::vector<int> selected_node_ids)
{
	this->selectedpointIndexData.clear();


	for (int nd_id : selected_node_ids)
	{
		int index =	this->pointIdToIndex[nd_id];

		this->selectedpointIndexData.push_back(index);
	}


	// Create the selected point index data 
	// this->selected_point_ibo.clear();
	this->selected_point_ibo.updateIndexBuffer(this->selectedpointIndexData.data(),
		static_cast<int>(this->selectedpointIndexData.size()));

}


std::vector<int> model_mesh_store::is_node_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2)
{
	// Return the node id of node which is inside the rectangle
	// Covert mouse location to screen location
	int max_dim = geom_param_ptr->window_width > geom_param_ptr->window_height ? geom_param_ptr->window_width : geom_param_ptr->window_height;

	// Selected node list index;
	std::vector<int> selected_node_index;

	// Transform the mouse location to openGL screen coordinates
	// Corner Point 1
	glm::vec2 screen_cpt1 = glm::vec2(2.0f * ((corner_pt1.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt1.y) / max_dim));

	// Corner Point 2
	glm::vec2 screen_cpt2 = glm::vec2(2.0f * ((corner_pt2.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt2.y) / max_dim));

	// Nodal location
	glm::mat4 scaling_matrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
	scaling_matrix[3][3] = 1.0f;

	glm::mat4 scaledModelMatrix = geom_param_ptr->rotateTranslation * scaling_matrix * geom_param_ptr->modelMatrix;


	// Loop through all nodes in map
	for (auto it = nodes.begin(); it != nodes.end(); ++it)
	{
		const auto& node = it->node_pt;
		glm::vec4 finalPosition = scaledModelMatrix * glm::vec4(node.x, node.y, node.z, 1.0f) * geom_param_ptr->panTranslation;

		double node_position_x = finalPosition.x;
		double node_position_y = finalPosition.y;

		// Check whether the point inside a rectangle
		if (geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, finalPosition) == true)
		{
			selected_node_index.push_back(it->node_id);
		}
	}

	// Return the node index find
	return selected_node_index;
}




