#pragma once
#include "nodes_list_store.h"

struct load_data
{
	int load_id = 0; // Load id
	int load_setid = 0; // Load set id
	int node_id = 0; // id of the node its applied to
	glm::vec2 load_loc = glm::vec2(0); // Load location
	double load_start_time = 0.0; // Load start time
	double load_end_time = 0.0; // Load end time
	double load_value = 0.0; // Load value
	double load_angle = 0.0; // Load angle
	bool show_load_label = false;

};

class nodeload_list_store
{
public:
	const double epsilon = 0.000001;
	int load_count = 0;
	std::unordered_map<int, load_data> loadMap;

	double model_total_length = 0.0;
	int number_of_nodes = 0;
	int model_type = 0; // 0,1 - Line, 2,3 - Circle

	nodeload_list_store();
	~nodeload_list_store();
	void init(geom_parameters* geom_param_ptr);
	void set_zero_condition(double& model_total_length, const int& number_of_nodes, const int& model_type);
	void add_loads(nodes_list_store& model_nodes, double& load_start_time, double& load_end_time,
		double& load_value, int& node_start_id, int& node_end_id, int& interpolation_type);
	void delete_all_loads();
	void set_buffer();
	void paint_loads();
	void paint_load_labels();
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	geom_parameters* geom_param_ptr = nullptr;
	gBuffers load_buffer;
	Shader load_shader;
	label_list_store load_value_labels;
	double load_max = 0.0;
	std::vector<int> all_load_ids;
	std::vector<int> all_load_setids;

	glm::vec2 linear_interpolation(glm::vec2 pt1, glm::vec2 pt2, double t_val);
	glm::vec2 cubic_bezier_interpolation(glm::vec2 pt1, glm::vec2 pt2, glm::vec2 pt3, glm::vec2 pt4, double t_val);
	glm::vec2 half_sine_interpolation(glm::vec2 pt1, glm::vec2 pt2, glm::vec2 pt3, double t_val);

	void get_load_buffer(load_data& ld, float* load_vertices, unsigned int& load_v_index, unsigned int* load_indices, unsigned int& load_i_index);
	int get_unique_load_id(std::vector<int>& all_ids);

	double get_load_angle(const glm::vec2& node_pt);
};
