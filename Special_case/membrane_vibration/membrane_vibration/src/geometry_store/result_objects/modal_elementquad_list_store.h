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
	std::vector<glm::vec3> nd1_modal_displ;
	std::vector<glm::vec3> nd2_modal_displ;
	std::vector<glm::vec3> nd3_modal_displ;
	std::vector<glm::vec3> nd4_modal_displ;

	std::vector<glm::vec3> edge13_025modal_displ; // edge 13 0.25
	std::vector<glm::vec3> edge13_050modal_displ; // edge 13 0.50
	std::vector<glm::vec3> edge13_075modal_displ; // edge 13 0.75

	std::vector<glm::vec3> edge32_025modal_displ; // edge 32 0.25
	std::vector<glm::vec3> edge32_050modal_displ; // edge 32 0.50
	std::vector<glm::vec3> edge32_075modal_displ; // edge 32 0.75

	std::vector<glm::vec3> edge21_025modal_displ; // edge 21 0.25
	std::vector<glm::vec3> edge21_050modal_displ; // edge 21 0.50
	std::vector<glm::vec3> edge21_075modal_displ; // edge 21 0.75

	std::vector<glm::vec3> edge14_025modal_displ; // edge 14 0.25
	std::vector<glm::vec3> edge14_050modal_displ; // edge 14 0.50
	std::vector<glm::vec3> edge14_075modal_displ; // edge 14 0.75

	std::vector<glm::vec3> edge43_025modal_displ; // edge 43 0.25
	std::vector<glm::vec3> edge43_050modal_displ; // edge 43 0.50
	std::vector<glm::vec3> edge43_075modal_displ; // edge 43 0.75


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
		modal_node_store* nd3, modal_node_store* nd4,
		std::vector<glm::vec3> edge13_025modal_displ, // edge 13 0.25
		std::vector<glm::vec3> edge13_050modal_displ, // edge 13 0.50
		std::vector<glm::vec3> edge13_075modal_displ, // edge 13 0.75
		std::vector<glm::vec3> edge32_025modal_displ, // edge 32 0.25
		std::vector<glm::vec3> edge32_050modal_displ, // edge 32 0.50
		std::vector<glm::vec3> edge32_075modal_displ, // edge 32 0.75
		std::vector<glm::vec3> edge21_025modal_displ, // edge 21 0.25
		std::vector<glm::vec3> edge21_050modal_displ, // edge 21 0.50
		std::vector<glm::vec3> edge21_075modal_displ, // edge 21 0.75
		std::vector<glm::vec3> edge14_025modal_displ, // edge 14 0.25
		std::vector<glm::vec3> edge14_050modal_displ, // edge 14 0.50
		std::vector<glm::vec3> edge14_075modal_displ, // edge 14 0.75
		std::vector<glm::vec3> edge43_025modal_displ, // edge 43 0.25
		std::vector<glm::vec3> edge43_050modal_displ, // edge 43 0.50
		std::vector<glm::vec3> edge43_075modal_displ); // edge 43 0.75

	void clear_data();
	void set_buffer();
	void update_buffer(int selected_mode);
	void paint_modal_elementquadrilaterals();
	void update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
		bool set_zoomtranslation, bool set_transparency, bool set_deflscale);
private:
	geom_parameters* geom_param_ptr = nullptr;
	// Anti clockwise
	//   4____m34____3		
	//   |		     |     
	//   m41	m	m23
	//   |			 |    
	//   1_____m12___2        

	dynamic_tri_list_store modal_element_tris132; // Tri 132
	dynamic_tri_list_store modal_element_tris143; // Tri 143


};
