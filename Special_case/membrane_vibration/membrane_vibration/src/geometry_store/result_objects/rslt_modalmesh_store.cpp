#include "rslt_modalmesh_store.h"

rslt_modalmesh_store::rslt_modalmesh_store()
{
	// Empty constructor
}


void rslt_modalmesh_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Create the mesh shader
	auto shaderSrc = ShaderLibrary::Get(ShaderLibrary::ShaderType::MeshShader);

	rsltmesh_shader.create_shader_data(shaderSrc.vertex.c_str(), shaderSrc.fragment.c_str());

	// Clear mesh
	clear_mesh();
}


void rslt_modalmesh_store::initialize_mesh(std::vector<rslt_modalnode_store> rsltnodes, 
	std::vector<elementtri_store> tris, 
	std::vector<elementquad_store> quads)
{
	// Clear the existing mesh
	clear_mesh();

	// Move instead of copy (O(1) pointer exchange)
	this->rsltnodes = std::move(rsltnodes);
	this->tris = std::move(tris);
	this->quads = std::move(quads);

	// Create the wire frame
	create_wireframe();

	// Create the openGL objects
	create_buffer_data();

}

void rslt_modalmesh_store::clear_mesh()
{
	// Clear mesh
	this->rsltnodes.clear();
	this->wireframe.clear();
	this->tris.clear();
	this->quads.clear();

	// Clear the OpenGL variables
	this->pointIdToIndex.clear();

	this->vertexData.clear();
	this->vertexnormalData.clear();
	this->pointIndexData.clear();
	this->wireframeIndexData.clear();
	this->triangleIndexData.clear();
	this->quadrilateralIndexData.clear();
	//
}

void rslt_modalmesh_store::create_buffer()
{

}

void rslt_modalmesh_store::paint_mesh()
{

}

void rslt_modalmesh_store::paint_mesh_wireframe()
{


}

void rslt_modalmesh_store::paint_mesh_points()
{


}


void rslt_modalmesh_store::update_openGLuniforms()
{


}


void rslt_modalmesh_store::create_wireframe()
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


void rslt_modalmesh_store::create_vertex_normals(std::vector<glm::vec3>& vnormals)
{

}



void rslt_modalmesh_store::create_buffer_data()
{


}