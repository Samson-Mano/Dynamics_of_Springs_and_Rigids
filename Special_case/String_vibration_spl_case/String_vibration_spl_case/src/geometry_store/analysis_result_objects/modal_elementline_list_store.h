#pragma once
#include "modal_nodes_list_store.h"
#include "../geometry_objects/line_list_store.h"


struct modal_elementline_store
{
	int line_id = 0; // ID of the line
	modal_node_store* startNode = nullptr; // start node
	modal_node_store* endNode = nullptr; // end node

	// Line modal displacement data
	bool is_rigid = false; // Check whether the line is rigid or not

	// Line normalized displacement vector
	std::unordered_map<int, glm::vec2> startpt_modal_displ;
	std::unordered_map<int, glm::vec2> endpt_modal_displ;

};

class modal_elementline_list_store
{
public:
	const int colormap_type = 1;
	unsigned int modal_elementline_count = 0;
	std::unordered_map<int, modal_elementline_store> modal_elementlineMap; // Create an unordered_map to store lines with ID as key
	//std::unordered_map<int, double> max_node_displ; // Stores the maximum nodal displacement for the whole model
	//std::unordered_map<int, double> min_node_displ; // Stores the minimum nodal displacement for the whole model
	double element_max_length = 0.0;
	double element_min_length = DBL_MAX;


	modal_elementline_list_store();
	~modal_elementline_list_store();
	void init(geom_parameters* geom_param_ptr);
	void clear_data();
	void add_modal_elementline(int& line_id, modal_node_store* startNode, modal_node_store* endNode,bool& is_rigid);

	void set_buffer(int selected_mode);
	void paint_modal_elementlines();
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	geom_parameters* geom_param_ptr = nullptr;
	line_list_store modal_element_lines;

	void set_rigid_element_line(modal_elementline_store& rline, const int& selected_mode, line_list_store& modal_element_lines);
	void set_spring_element_line(modal_elementline_store& rline, const int& selected_mode, line_list_store& modal_element_lines);
};
