#pragma once
#include "../geometry_buffers/gBuffers.h"
#include "../geom_parameters.h"
#include "../geometry_objects/label_list_store.h"

#include <unordered_set>
#include <glm/gtx/rotate_vector.hpp> 

struct nodeinl_condition_data
{
	int node_id = 0;
	glm::vec3 inlcond_loc = glm::vec3(0);
	double inl_amplitude_z = 0.0; // initial amplitude z

};

class nodeinlcond_list_store
{
public:
	const double epsilon = 0.000001;
	unsigned int inlcond_count = 0;
	std::unordered_map<int, nodeinl_condition_data> inlcondMap;
	int inlcond_type = 0; //0 - Displacement, 1 - Velocity
	int model_type = 0; // 0,1 - Line, 2,3 - Circle

	nodeinlcond_list_store();
	~nodeinlcond_list_store() = default;
	void init(geom_parameters* geom_param_ptr);
	void set_zero_condition(int inlcond_type, const int& model_type);
	void add_inlcondition(int& node_id, glm::vec3& inlcond_loc, double& inl_amplitude_z);
	void delete_inlcondition(int& node_id);
	void set_buffer();
	void paint_inlcond();
	void paint_inlcond_label();
	void update_openGLuniforms();

private:
	double inlcond_max = 0.0;

	geom_parameters* geom_param_ptr = nullptr;
	gBuffers inlcond_buffer;
	Shader inlcond_shader;

	label_list_store inlcond_value_labels;

	void get_inlcond_buffer(nodeinl_condition_data& inlcond, float* inlcond_vertices, unsigned int& inlcond_v_index,
		unsigned int* inlcond_indices, unsigned int& inlcond_i_index);

};

