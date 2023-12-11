#pragma once
#include "../fe_objects/nodes_list_store.h"
#include "../geometry_objects/dynamic_point_list_store.h"

struct result_node
{
	std::vector<int> index; // index
	std::vector<double> time_val; // at time t list
	std::vector<double> displ_magnitude; // Displacmenet magnitude at time t
	std::vector <glm::vec2> normalized_displ; // Nodal normalized displacement at time t
};

struct result_node_store
{
	int node_id = 0;
	glm::vec2 node_pt = glm::vec2(0);

	// Node results (index, time, displacement, (x, y))
	result_node node_result_val;
	int number_of_timesteps = 0;
};


class result_node_list_store
{
public:
	unsigned int node_count = 0;
	std::unordered_map<int, result_node_store> result_nodeMap; // Create an unordered_map to store nodes with ID as key
	double max_node_displ = 0.0; // Maximum nodal displacement


	result_node_list_store();
	~result_node_list_store();
	void init(geom_parameters* geom_param_ptr);
	void clear_data();
	void add_result_node(int& node_id, glm::vec2& node_pt, result_node node_result_val, const int& number_of_time_steps);
	void set_buffer();
	void paint_result_nodes(const int& dyn_index);
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, 
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);

private:
	geom_parameters* geom_param_ptr = nullptr;
	dynamic_point_list_store result_node_points;

};
