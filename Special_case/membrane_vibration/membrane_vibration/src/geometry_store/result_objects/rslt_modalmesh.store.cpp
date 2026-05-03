#include "rslt_modalmesh.store.h"

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


void rslt_modalmesh_store::initialize_mesh(std::vector<elementline_store> wireframe, 
	std::vector<elementtri_store> tris, 
	std::vector<elementquad_store> quads)
{
	// Clear the existing mesh
	clear_mesh();

	// Cannot use Move
	//// Move instead of copy (O(1) pointer exchange)
	//this->nodes = std::move(nodes);
	//this->tris = std::move(tris);
	//this->quads = std::move(quads);


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


void rslt_modalmesh_store::create_vertex_normals(std::vector<glm::vec3>& vnormals)
{

}



void rslt_modalmesh_store::create_buffer_data()
{


}