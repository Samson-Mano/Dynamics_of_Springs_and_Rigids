#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <unordered_map>
#include "../geom_parameters.h"
#include "../geometry_buffers/gBuffers.h"

#include <set>
#include <utility>



struct node_store
{
	int node_id = 0;
	glm::vec3 node_pt = glm::vec3(0);

	node_store(int id, float x, float y, float z)
		: node_id(id), node_pt(x, y, z)
	{ }

};



struct elementline_store
{
	int line_id = 0; // ID of the line
	int startnd_id = 0; // start node id
	int endnd_id = 0; // end node id

	elementline_store(int id, int startnd_id, int endnd_id)
		: line_id(id), startnd_id(startnd_id), endnd_id(endnd_id)
	{ }
};


struct elementtri_store
{
	int tri_id = 0; // ID of the triangle element
	int nd1_id = 0; // node 1 id
	int nd2_id = 0; // node 2 id
	int nd3_id = 0; // node 3 id

	elementtri_store(int id, int nd1_id, int nd2_id, int nd3_id)
		: tri_id(id), nd1_id(nd1_id), nd2_id(nd2_id), nd3_id(nd3_id)
	{ }
};

struct elementquad_store
{
	int quad_id = 0; // ID of the quadrilateral element
	int nd1_id = 0; // node 1 id
	int nd2_id = 0; // node 2 id
	int nd3_id = 0; // node 3 id
	int nd4_id = 0; // node 4 id

	elementquad_store(int id, int nd1_id, int nd2_id, int nd3_id, int nd4_id)
		:quad_id(id), nd1_id(nd1_id), nd2_id(nd2_id), nd3_id(nd3_id), nd4_id(nd4_id)
	{ }
};




class model_mesh_store
{
public:
	std::vector<node_store> nodes;
	std::vector<elementline_store> wireframe;
	std::vector<elementtri_store> tris;
	std::vector<elementquad_store> quads;


	model_mesh_store();
	~model_mesh_store() = default;


	void init(geom_parameters* geom_param_ptr);

	void add_mesh(std::vector<node_store> nodes,
					std::vector<elementtri_store> tris,
					std::vector<elementquad_store> quads);

	void clear_mesh();

	void create_buffer();

	void paint_mesh();

	void paint_mesh_wireframe();

	void paint_mesh_points();

	void paint_selected_mesh_points();


	std::vector<int> is_node_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2);

	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);



private:
	geom_parameters* geom_param_ptr = nullptr;

	Shader mesh_shader;

	// Vertex Buffer object and Vertex Array object 
	VertexBuffer point_vbo;
	VertexArray point_vao;

	// Index buffer for the points, lines and triangles, quadrilaterals (EBO)
	IndexBuffer point_ibo;
	IndexBuffer selected_point_ibo;
	IndexBuffer wireframe_ibo;
	IndexBuffer triangle_ibo;
	IndexBuffer quadrilateral_ibo;

	// Geometry data for OpenGL
	std::unordered_map<int, int> pointIdToIndex;

	std::vector<float> vertexData;
	std::vector<float> vertexnormalData;
	std::vector<unsigned int> pointIndexData;
	std::vector<unsigned int> selectedpointIndexData;
	std::vector<unsigned int> wireframeIndexData;
	std::vector<unsigned int> triangleIndexData;
	std::vector<unsigned int> quadrilateralIndexData;


	void create_wireframe();

	void create_vertex_normals(std::vector<glm::vec3>& vnormals);

	void create_buffer_data();

};