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

	// Edge point
	glm::vec3 mid12pt = glm::vec3(0);
	glm::vec3 mid23pt = glm::vec3(0);
	glm::vec3 mid34pt = glm::vec3(0);
	glm::vec3 mid41pt = glm::vec3(0);
	glm::vec3 midqdpt = glm::vec3(0);

	// Point displacements
	std::vector<glm::vec3> nd1_modal_displ;
	std::vector<glm::vec3> nd2_modal_displ;
	std::vector<glm::vec3> nd3_modal_displ;
	std::vector<glm::vec3> nd4_modal_displ;

	std::vector<glm::vec3> v12_modal_displ; // eigen vector at mid of 1-2
	std::vector<glm::vec3> v23_modal_displ; // eigen vector at mid of 2-3
	std::vector<glm::vec3> v34_modal_displ; // eigen vector at mid of 3-4
	std::vector<glm::vec3> v41_modal_displ; // eigen vector at mid of 4-1
	std::vector<glm::vec3> v_mid_modal_displ; // eigen vector at quad mid
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
		std::vector<glm::vec3>& v12,
		std::vector<glm::vec3>& v23,
		std::vector<glm::vec3>& v34,
		std::vector<glm::vec3>& v41,
		std::vector<glm::vec3>& v_mid);
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

	dynamic_tri_list_store modal_element_tris1_m_m12; // Tri 1_m_m12
	dynamic_tri_list_store modal_element_tris1_m41_m; // Tri 1_m41_m
	dynamic_tri_list_store modal_element_tris2_m12_m; // Tri 2_m12_m
	dynamic_tri_list_store modal_element_tris2_m_m23; // Tri 2_m_m23
	dynamic_tri_list_store modal_element_tris3_m23_m; // Tri 3_m23_m
	dynamic_tri_list_store modal_element_tris3_m_m34; // Tri 3_m_m34
	dynamic_tri_list_store modal_element_tris4_m34_m; // Tri 4_m34_m
	dynamic_tri_list_store modal_element_tris4_m_m41; // Tri 4_m_m41

};
