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

	mesh_shader.create_shader(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

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
	this->pointIndexData.clear();
	this->selectedpointIndexData.clear();
	this->wireframeIndexData.clear();
	this->triangleIndexData.clear();
	this->quadrilateralIndexData.clear();
	//
}



void model_mesh_store::create_buffer()
{
	// Create the OpenGL buffer for the mesh
	this->pointIdToIndex.clear();
	this->pointIndexData.clear();

	for (int i = 0; i < static_cast<int>(this->nodes.size()); i++)
	{
		this->pointIdToIndex[nodes[i].node_id] = i;

		this->pointIndexData.emplace_back(i);
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

	}

	//_______________________________________________________________
	// prepare wireframe index data for openGL
	this->wireframeIndexData.clear();

	for (const elementline_store& line : this->wireframe)
	{
		auto it_start = this->pointIdToIndex.find(line.startnd_id);
		auto it_end = this->pointIdToIndex.find(line.endnd_id);

		if (it_start == this->pointIdToIndex.end() || it_end == this->pointIdToIndex.end())
			continue;

		
		this->wireframeIndexData.push_back(it_start->second);
		this->wireframeIndexData.push_back(it_end->second);
	}

	//_______________________________________________________________
	// prepare triangle index data for openGL
	this->triangleIndexData.clear();

	for (const elementtri_store& tri : this->tris)
	{
		auto it_nd1 = this->pointIdToIndex.find(tri.nd1_id);
		auto it_nd2 = this->pointIdToIndex.find(tri.nd2_id);
		auto it_nd3 = this->pointIdToIndex.find(tri.nd3_id);

		if (it_nd1 == this->pointIdToIndex.end() ||	it_nd2 == this->pointIdToIndex.end() ||
			it_nd3 == this->pointIdToIndex.end())
			continue;

		this->triangleIndexData.push_back(it_nd1->second);
		this->triangleIndexData.push_back(it_nd2->second);
		this->triangleIndexData.push_back(it_nd3->second);

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


		if (it_nd1 == this->pointIdToIndex.end() ||	it_nd2 == this->pointIdToIndex.end() ||
			it_nd3 == this->pointIdToIndex.end() || it_nd4 == this->pointIdToIndex.end())
			continue;

		// Make two triangles from the quad (pt1, pt2, pt3) and (pt1, pt3, pt4)
					// Triangle 1
		this->quadrilateralIndexData.push_back(it_nd1->second);
		this->quadrilateralIndexData.push_back(it_nd2->second);
		this->quadrilateralIndexData.push_back(it_nd3->second);

		// Triangle 2
		this->quadrilateralIndexData.push_back(it_nd1->second);
		this->quadrilateralIndexData.push_back(it_nd3->second);
		this->quadrilateralIndexData.push_back(it_nd4->second);

	}


}


void model_mesh_store::paint_mesh()
{
	// Paint the mesh triangles, quadrilaterals

}


void model_mesh_store::paint_mesh_wireframe()
{
	// Paint the mesh wireframe

}



void model_mesh_store::paint_mesh_points()
{
	// Paint the mesh points

}



void model_mesh_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, 
	bool set_rotatetranslation,	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	if (set_modelmatrix == true)
	{
		// set the model matrix
		mesh_shader.setUniform("geom_scale", static_cast<float>(geom_param_ptr->geom_scale));
		mesh_shader.setUniform("transparency", 1.0f);

		mesh_shader.setUniform("projectionMatrix", geom_param_ptr->projectionMatrix, false);
		mesh_shader.setUniform("viewMatrix", geom_param_ptr->viewMatrix, false);
		mesh_shader.setUniform("modelMatrix", geom_param_ptr->modelMatrix, false);
	}

	if (set_pantranslation == true)
	{
		// set the pan translation
		mesh_shader.setUniform("panTranslation", geom_param_ptr->panTranslation, false);
	}

	if (set_rotatetranslation == true)
	{
		// set the rotate translation
		mesh_shader.setUniform("rotateTranslation", geom_param_ptr->rotateTranslation, false);
	}

	if (set_zoomtranslation == true)
	{
		// set the zoom translation
		mesh_shader.setUniform("zoomscale", static_cast<float>(geom_param_ptr->zoom_scale));
	}

	if (set_transparency == true)
	{
		// set the alpha transparency
		mesh_shader.setUniform("transparency", static_cast<float>(geom_param_ptr->geom_transparency));
	}

	if (set_deflscale == true)
	{
		// set the deflection scale
		mesh_shader.setUniform("normalized_deflscale", static_cast<float>(geom_param_ptr->normalized_defl_scale));
		mesh_shader.setUniform("deflscale", static_cast<float>(geom_param_ptr->defl_scale));
	}
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
	for(const elementtri_store& tri : this->tris)
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

