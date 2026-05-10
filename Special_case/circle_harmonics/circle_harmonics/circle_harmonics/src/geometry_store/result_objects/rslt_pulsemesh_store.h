#pragma once
#include <glm/vec2.hpp>
#include <glm/vec3.hpp>
#include <unordered_map>
#include "../geom_parameters.h"
#include "../geometry_buffers/gBuffers.h"

#include "../fe_objects/model_mesh_store.h"

#include <set>
#include <utility>


struct rslt_pulsenode_store
{
	int node_id = 0;
	glm::vec3 node_pt = glm::vec3(0);

	std::vector <glm::vec3> node_displ; // Nodal  displacement at time t
	std::vector<double> node_displ_magnitude; // Displacmenet magnitude at time t


	// Default constructor
	rslt_pulsenode_store() = default;


	// Constructor for lvalues (copies)
	rslt_pulsenode_store(int id, const glm::vec3& pt,
		const std::vector<glm::vec3>& displ,
		const std::vector<double>& displ_mag)
		: node_id(id), node_pt(pt), node_displ(displ), node_displ_magnitude(displ_mag)
	{

	}

};



class rslt_pulsemesh_store
{
public:
	std::vector<rslt_pulsenode_store> rsltnodes;
	std::vector<elementline_store> wireframe;
	std::vector<elementtri_store> tris;

	int prev_time_step = 0;

	double maximim_displacement = 0.0;
	double minimum_displacement = 0.0;
	int number_of_timesteps = 0;
	std::vector<double> time_points; // Result time t list

	rslt_pulsemesh_store();
	~rslt_pulsemesh_store() = default;


	void init(geom_parameters* geom_param_ptr);

	void add_result_mesh(std::vector<rslt_pulsenode_store> rsltnodes,
		std::vector<elementline_store> wireframe,
		std::vector<elementtri_store> tris);

	void clear_mesh();

	void create_buffer();
	void update_buffer(int time_step);

	void paint_mesh();
	void paint_mesh_wireframe();
	void paint_mesh_points();

	void update_openGLuniforms();
	void update_animation_openGLuniforms();

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


	// Geometry data for OpenGL
	std::unordered_map<int, int> pointIdToIndex;

	std::vector<unsigned int> pointIndexData;
	std::vector<unsigned int> wireframeIndexData;
	std::vector<unsigned int> triangleIndexData;


	void create_buffer_data();


};