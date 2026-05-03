#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <unordered_map>
#include "../geom_parameters.h"
#include "../geometry_buffers/gBuffers.h"

#include "../fe_objects/model_mesh_store.h"

#include <set>
#include <utility>



struct rslt_modalnode_store
{
	int node_id = 0;
	glm::vec3 node_pt = glm::vec3(0);

	// Modal results (x, y, z)
	std::vector<glm::vec3> node_modal_displ;
	std::vector<double> node_modal_displ_magnitude;


	rslt_modalnode_store(int id, const glm::vec3& pt,
		std::vector<glm::vec3>&& node_modal_displ,
		std::vector<double>&& node_modal_displ_magnitude)
		: node_id(id), node_pt(pt), node_modal_displ(std::move(node_modal_displ)), 
		  node_modal_displ_magnitude(std::move(node_modal_displ_magnitude))
	{

	}

};





class rslt_modalmesh_store
{
public:
	std::vector<rslt_modalnode_store> rsltnodes;
	std::vector<elementline_store> wireframe;
	std::vector<elementtri_store> tris;
	std::vector<elementquad_store> quads;


	rslt_modalmesh_store();
	~rslt_modalmesh_store() = default;


	void init(geom_parameters* geom_param_ptr);

	void initialize_mesh(std::vector<elementline_store> wireframe,
		std::vector<elementtri_store> tris,
		std::vector<elementquad_store> quads);

	void clear_mesh();

	void create_buffer();

	void paint_mesh();
	void paint_mesh_wireframe();
	void paint_mesh_points();


	void update_openGLuniforms();


private:
	geom_parameters* geom_param_ptr = nullptr;

	Shader rsltmesh_shader;

	// Vertex Buffer object and Vertex Array object 
	VertexBuffer point_vbo;
	VertexArray point_vao;

	// Index buffer for the points, lines and triangles, quadrilaterals (EBO)
	IndexBuffer point_ibo;
	IndexBuffer wireframe_ibo;
	IndexBuffer triangle_ibo;
	IndexBuffer quadrilateral_ibo;

	// Geometry data for OpenGL
	std::unordered_map<int, int> pointIdToIndex;

	std::vector<float> vertexData;
	std::vector<float> vertexnormalData;
	std::vector<unsigned int> pointIndexData;
	std::vector<unsigned int> wireframeIndexData;
	std::vector<unsigned int> triangleIndexData;
	std::vector<unsigned int> quadrilateralIndexData;


	void create_vertex_normals(std::vector<glm::vec3>& vnormals);

	void create_buffer_data();

};







