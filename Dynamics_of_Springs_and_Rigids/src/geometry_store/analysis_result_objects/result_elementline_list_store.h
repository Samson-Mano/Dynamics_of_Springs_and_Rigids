#pragma once
#include "result_node_list_store.h"
#include "../geometry_objects/dynamic_line_list_store.h"


struct result_elementline_store
{
	int line_id = 0; // ID of the line
	result_node_store* startNode = nullptr; // start node
	result_node_store* endNode = nullptr; // end node

	bool is_rigid = false; // Check whether the line is rigid or not

	// Line result displacement data
	// Line displacement magnitude
	std::vector<double> startpt_displ_magnitude; // Pt1 displacmenet magnitude at time t
	std::vector<double> endpt_displ_magnitude; // Pt2 displacmenet magnitude at time t

	// Line normalized displacement vector
	std::vector<glm::vec2> startpt_normalized_displ;
	std::vector<glm::vec2> endpt_normalized_displ;
};


class result_elementline_list_store
{
public:
	unsigned int result_elementline_count = 0;
	std::unordered_map<int, result_elementline_store> result_elementlineMap; // Create an unordered_map to store lines with ID as key
	double max_line_displ = 0.0; // Maximum line displacement
	double element_max_length = 0.0;
	double element_min_length = DBL_MAX;

	result_elementline_list_store();
	~result_elementline_list_store();
	void init(geom_parameters* geom_param_ptr);
	void clear_data();
	void add_result_elementline(int& line_id, result_node_store* startNode, result_node_store* endNode, bool& is_rigid);
	void set_buffer();
	void paint_result_elementlines(const int& dyn_index);
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation,
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);

private:
	geom_parameters* geom_param_ptr = nullptr;
	dynamic_line_list_store result_element_lines;

	void set_rigid_element_line(const result_elementline_store& rline, dynamic_line_list_store& result_element_lines);
	void set_spring_element_line(const result_elementline_store& rline, dynamic_line_list_store& result_element_lines);
};
