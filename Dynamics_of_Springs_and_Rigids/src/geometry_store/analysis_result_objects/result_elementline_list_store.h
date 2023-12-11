#pragma once
#include "result_node_list_store.h"
#include "../geometry_objects/dynamic_line_list_store.h"


struct result_line_points
{
	int split_line_id = 0; // line id of the individual hermitian interpolation line 
	// Point coordinate
	glm::vec2 pt1 = glm::vec2(0);
	glm::vec2 pt2 = glm::vec2(0);

	// Points displacement magnitude
	std::vector<double> pt1_displ_magnitude; // Pt1 displacmenet magnitude at time t
	std::vector<double> pt2_displ_magnitude; // Pt2 displacmenet magnitude at time t

	// Points normalized displacement
	std::vector<glm::vec2> pt1_normalized_displ;
	std::vector<glm::vec2> pt2_normalized_displ;
};

struct result_elementline_store
{
	int line_id = 0; // ID of the line
	result_node_store* startNode = nullptr; // start node
	result_node_store* endNode = nullptr; // end node

	// Line result displacement data
	std::vector<result_line_points> discretized_bar_line_data;
};


class result_elementline_list_store
{
public:
	unsigned int result_elementline_count = 0;
	std::unordered_map<int, result_elementline_store> result_elementlineMap; // Create an unordered_map to store lines with ID as key
	double max_line_displ = 0.0; // Maximum line displacement

	result_elementline_list_store();
	~result_elementline_list_store();
	void init(geom_parameters* geom_param_ptr);
	void clear_data();
	void add_result_elementline(int& line_id, result_node_store* startNode, result_node_store* endNode);
	void set_buffer();
	void paint_result_elementlines(const int& dyn_index);
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation,
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);

private:
	geom_parameters* geom_param_ptr = nullptr;
	dynamic_line_list_store result_element_lines;

};
