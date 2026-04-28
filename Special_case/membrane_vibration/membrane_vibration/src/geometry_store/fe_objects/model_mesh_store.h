#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <unordered_map>
#include "../geom_parameters.h"
#include "../geometry_buffers/gBuffers.h"




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
	std::vector<elementline_store> wierframe;
	std::vector<elementtri_store> tris;
	std::vector<elementquad_store> quads;


	model_mesh_store();
	~model_mesh_store() = default;



private:




};