#pragma once
#include "modal_elementline_list_store.h"
#include "../geometry_objects/dynamic_tri_list_store.h"


struct modal_elementquad_store
{
	int quad_id = 0; // ID of the quadrilateral element
	modal_node_store* nd1 = nullptr; // node 1
	modal_node_store* nd2 = nullptr; // node 2
	modal_node_store* nd3 = nullptr; // node 3
	modal_node_store* nd4 = nullptr; // node 4

	// Point coordinate
	glm::vec3 nd1pt = glm::vec3(0);
	glm::vec3 nd2pt = glm::vec3(0);
	glm::vec3 nd3pt = glm::vec3(0);
	glm::vec3 nd4pt = glm::vec3(0);

	// Point displacements
	std::unordered_map<int, glm::vec3> nd1_modal_displ;
	std::unordered_map<int, glm::vec3> nd2_modal_displ;
	std::unordered_map<int, glm::vec3> nd3_modal_displ;
	std::unordered_map<int, glm::vec3> nd4_modal_displ;
};


class modal_elementquad_list_store
{
public:
	unsigned int modal_elementquad_count = 0;
	std::unordered_map<int, modal_elementquad_store> modal_elementquadMap; // Create an unordered_map to store Quadrilaterals with ID as key

	modal_elementquad_list_store();
	~modal_elementquad_list_store();
	void init(geom_parameters* geom_param_ptr);
	void add_modal_elementquadrilateral(int& quad_id, modal_node_store* nd1, modal_node_store* nd2,
		modal_node_store* nd3, modal_node_store* nd4);
	void clear_data();
	void set_buffer();
	void update_buffer(int selected_mode);
	void paint_modal_elementquadrilaterals();
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	geom_parameters* geom_param_ptr = nullptr;
	//   4______3     4____3      3
	//   |      |     |   /     / |
	//   |      |     | /     /   | 
	//   1______2     1      1____2      

	dynamic_tri_list_store modal_element_tris12m; // Tri 12m
	dynamic_tri_list_store modal_element_tris23m; // Tri 23m
	dynamic_tri_list_store modal_element_tris34m; // Tri 34m
	dynamic_tri_list_store modal_element_tris41m; // Tri 41m


};
