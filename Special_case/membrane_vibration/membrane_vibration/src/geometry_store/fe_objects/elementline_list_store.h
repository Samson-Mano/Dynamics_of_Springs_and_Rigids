#pragma once
#include "nodes_list_store.h"
#include "../geometry_objects/line_list_store.h"

struct elementline_store
{
	int line_id = 0; // ID of the line
	node_store* startNode = nullptr; // start node
	node_store* endNode = nullptr; // end node
};


class elementline_list_store
{
public:
	unsigned int elementline_count = 0;
	std::unordered_map<int, elementline_store> elementlineMap; // Create an unordered_map to store nodes with ID as key

	elementline_list_store();
	~elementline_list_store();
	void init(geom_parameters* geom_param_ptr);
	void add_elementline(int& line_id, node_store* startNode, node_store* endNode);
	void set_buffer();
	void paint_elementlines();

	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);

private:
	geom_parameters* geom_param_ptr = nullptr;
	line_list_store element_lines;
};
