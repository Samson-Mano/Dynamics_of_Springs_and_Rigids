#include "modal_elementquad_list_store.h"

modal_elementquad_list_store::modal_elementquad_list_store()
{
	// Empty Constructor
}

modal_elementquad_list_store::~modal_elementquad_list_store()
{
	// Empty Destructor
}

void modal_elementquad_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameters for the quads (and clear the quads)
	modal_element_tris132.init(geom_param_ptr); // Tri 132
	modal_element_tris143.init(geom_param_ptr); // Tri 143

	// Clear the element quads
	modal_elementquad_count = 0;
	modal_elementquadMap.clear();
}

void modal_elementquad_list_store::clear_data()
{
	// Clear the element quads
	modal_element_tris132.clear_triangles(); // Tri 132
	modal_element_tris143.clear_triangles(); // Tri 143

	// Clear the element quads
	modal_elementquad_count = 0;
	modal_elementquadMap.clear();

}

void modal_elementquad_list_store::add_modal_elementquadrilateral(int& quad_id, modal_node_store* nd1,
	modal_node_store* nd2, modal_node_store* nd3, modal_node_store* nd4,
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
	std::vector<glm::vec3> edge43_075modal_displ) // edge 43 0.75
{
	// Add result quad element
	modal_elementquad_store temp_quad;
	temp_quad.quad_id = quad_id;
	temp_quad.nd1 = nd1;
	temp_quad.nd2 = nd2;
	temp_quad.nd3 = nd3;
	temp_quad.nd4 = nd4;

	// Add the point coordinate
	temp_quad.nd1pt = (*nd1).node_pt;
	temp_quad.nd2pt = (*nd2).node_pt;
	temp_quad.nd3pt = (*nd3).node_pt;
	temp_quad.nd4pt = (*nd4).node_pt;

	// Add the modal displacement
	temp_quad.nd1_modal_displ = (*nd1).node_modal_displ;
	temp_quad.nd2_modal_displ = (*nd2).node_modal_displ;
	temp_quad.nd3_modal_displ = (*nd3).node_modal_displ;
	temp_quad.nd4_modal_displ = (*nd4).node_modal_displ;

	temp_quad.edge13_025modal_displ = edge13_025modal_displ; // edge 13 0.25
	temp_quad.edge13_050modal_displ = edge13_050modal_displ; // edge 13 0.50
	temp_quad.edge13_075modal_displ = edge13_075modal_displ; // edge 13 0.75

	temp_quad.edge32_025modal_displ = edge32_025modal_displ; // edge 32 0.25
	temp_quad.edge32_050modal_displ = edge32_050modal_displ; // edge 32 0.50
	temp_quad.edge32_075modal_displ = edge32_075modal_displ; // edge 32 0.75

	temp_quad.edge21_025modal_displ = edge21_025modal_displ; // edge 21 0.25
	temp_quad.edge21_050modal_displ = edge21_050modal_displ; // edge 21 0.50
	temp_quad.edge21_075modal_displ = edge21_075modal_displ; // edge 21 0.75

	temp_quad.edge14_025modal_displ = edge14_025modal_displ; // edge 14 0.25
	temp_quad.edge14_050modal_displ = edge14_050modal_displ; // edge 14 0.50
	temp_quad.edge14_075modal_displ = edge14_075modal_displ; // edge 14 0.75

	temp_quad.edge43_025modal_displ = edge43_025modal_displ; // edge 43 0.25
	temp_quad.edge43_050modal_displ = edge43_050modal_displ; // edge 43 0.50
	temp_quad.edge43_075modal_displ = edge43_075modal_displ; // edge 43 0.75


	// Check whether the node_id is already there
	if (modal_elementquadMap.find(quad_id) != modal_elementquadMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	// Insert to the lines
	modal_elementquadMap.insert({ quad_id, temp_quad });
	modal_elementquad_count++;

}

void modal_elementquad_list_store::set_buffer()
{
	// Clear existing modal quads (8 triangles)
	modal_element_tris132.clear_triangles(); // Tri 132
	modal_element_tris143.clear_triangles(); // Tri 143

	// Add the lines
	// Loop throug every line element
	int i = 0;
	for (auto& quad_m : modal_elementquadMap)
	{
		modal_elementquad_store quad = quad_m.second;

		// Node 1
		std::vector<glm::vec3> nd1_point_displ; // Node 1 displ
		std::vector<glm::vec3> nd1_point_color; // Node 1 color

		// Node 2
		std::vector<glm::vec3> nd2_point_displ; // Node 2 displ
		std::vector<glm::vec3> nd2_point_color; // Node 2 color

		// Node 3
		std::vector<glm::vec3> nd3_point_displ; // Node 3 displ
		std::vector<glm::vec3> nd3_point_color; // Node 3 color

		// Node 4
		std::vector<glm::vec3> nd4_point_displ; // Node 4 displ
		std::vector<glm::vec3> nd4_point_color; // Node 4 color

		// Edge 13
		std::vector<glm::vec3> edge13_025point_color; // Edge 13 025color
		std::vector<glm::vec3> edge13_050point_color; // Edge 13 050color
		std::vector<glm::vec3> edge13_075point_color; // Edge 13 075color

		// Edge 32
		std::vector<glm::vec3> edge32_025point_color; // Edge 32 025color
		std::vector<glm::vec3> edge32_050point_color; // Edge 32 050color
		std::vector<glm::vec3> edge32_075point_color; // Edge 32 075color

		// Edge 21
		std::vector<glm::vec3> edge21_025point_color; // Edge 21 025color
		std::vector<glm::vec3> edge21_050point_color; // Edge 21 050color
		std::vector<glm::vec3> edge21_075point_color; // Edge 21 075color

		// Edge 14
		std::vector<glm::vec3> edge14_025point_color; // Edge 14 025color
		std::vector<glm::vec3> edge14_050point_color; // Edge 14 050color
		std::vector<glm::vec3> edge14_075point_color; // Edge 14 075color

		// Edge 43
		std::vector<glm::vec3> edge43_025point_color; // Edge 43 025color
		std::vector<glm::vec3> edge43_050point_color; // Edge 43 050color
		std::vector<glm::vec3> edge43_075point_color; // Edge 43 075color


		for (int j = 0; j < static_cast<int>(quad.nd1_modal_displ.size()); j++)
		{
			//____________________________________________________________________________________________________________
			// Node 1
			glm::vec3 nd1_displ = glm::vec3(quad.nd1_modal_displ[j].x,
				quad.nd1_modal_displ[j].y,
				quad.nd1_modal_displ[j].z);

			// Find the displacment value
			double nd1_displ_value = glm::length(nd1_displ);

			glm::vec3 nd1_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - nd1_displ_value));


			//____________________________________________________________________________________________________________
			// Node 2
			glm::vec3 nd2_displ = glm::vec3(quad.nd2_modal_displ[j].x,
				quad.nd2_modal_displ[j].y,
				quad.nd2_modal_displ[j].z);

			// Find the displacment value
			double nd2_displ_value = glm::length(nd2_displ);

			glm::vec3 nd2_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - nd2_displ_value));


			//____________________________________________________________________________________________________________
			// Node 3
			glm::vec3 nd3_displ = glm::vec3(quad.nd3_modal_displ[j].x,
				quad.nd3_modal_displ[j].y,
				quad.nd3_modal_displ[j].z);

			// Find the displacment value
			double nd3_displ_value = glm::length(nd3_displ);

			glm::vec3 nd3_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - nd3_displ_value));


			//____________________________________________________________________________________________________________
			// Node 4
			glm::vec3 nd4_displ = glm::vec3(quad.nd4_modal_displ[j].x,
				quad.nd4_modal_displ[j].y,
				quad.nd4_modal_displ[j].z);

			// Find the displacment value
			double nd4_displ_value = glm::length(nd4_displ);

			glm::vec3 nd4_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - nd4_displ_value));

			//____________________________________________________________________________________________________________
			//____________________________________________________________________________________________________________
			// Edge 13 0.25
			glm::vec3 edge13_025displ = glm::vec3(quad.edge13_025modal_displ[j].x,
				quad.edge13_025modal_displ[j].y,
				quad.edge13_025modal_displ[j].z);

			// Find the displacment value
			double edge13_025displ_value = glm::length(edge13_025displ);

			glm::vec3 edge13_025contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge13_025displ_value));

			//____________________________________________________________________________________________________________
			// Edge 13 0.50
			glm::vec3 edge13_050displ = glm::vec3(quad.edge13_050modal_displ[j].x,
				quad.edge13_050modal_displ[j].y,
				quad.edge13_050modal_displ[j].z);

			// Find the displacment value
			double edge13_050displ_value = glm::length(edge13_050displ);

			glm::vec3 edge13_050contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge13_050displ_value));

			//____________________________________________________________________________________________________________
			// Edge 13 0.75
			glm::vec3 edge13_075displ = glm::vec3(quad.edge13_075modal_displ[j].x,
				quad.edge13_075modal_displ[j].y,
				quad.edge13_075modal_displ[j].z);

			// Find the displacment value
			double edge13_075displ_value = glm::length(edge13_075displ);

			glm::vec3 edge13_075contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge13_075displ_value));

			//____________________________________________________________________________________________________________
			//____________________________________________________________________________________________________________
			// Edge 32 0.25
			glm::vec3 edge32_025displ = glm::vec3(quad.edge32_025modal_displ[j].x,
				quad.edge32_025modal_displ[j].y,
				quad.edge32_025modal_displ[j].z);

			// Find the displacment value
			double edge32_025displ_value = glm::length(edge32_025displ);

			glm::vec3 edge32_025contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge32_025displ_value));

			//____________________________________________________________________________________________________________
			// Edge 32 0.50
			glm::vec3 edge32_050displ = glm::vec3(quad.edge32_050modal_displ[j].x,
				quad.edge32_050modal_displ[j].y,
				quad.edge32_050modal_displ[j].z);

			// Find the displacment value
			double edge32_050displ_value = glm::length(edge32_050displ);

			glm::vec3 edge32_050contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge32_050displ_value));

			//____________________________________________________________________________________________________________
			// Edge 32 0.75
			glm::vec3 edge32_075displ = glm::vec3(quad.edge32_075modal_displ[j].x,
				quad.edge32_075modal_displ[j].y,
				quad.edge32_075modal_displ[j].z);

			// Find the displacment value
			double edge32_075displ_value = glm::length(edge32_075displ);

			glm::vec3 edge32_075contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge32_075displ_value));

			//____________________________________________________________________________________________________________
			//____________________________________________________________________________________________________________
			// Edge 21 0.25
			glm::vec3 edge21_025displ = glm::vec3(quad.edge21_025modal_displ[j].x,
				quad.edge21_025modal_displ[j].y,
				quad.edge21_025modal_displ[j].z);

			// Find the displacment value
			double edge21_025displ_value = glm::length(edge21_025displ);

			glm::vec3 edge21_025contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge21_025displ_value));

			//____________________________________________________________________________________________________________
			// Edge 21 0.50
			glm::vec3 edge21_050displ = glm::vec3(quad.edge21_050modal_displ[j].x,
				quad.edge21_050modal_displ[j].y,
				quad.edge21_050modal_displ[j].z);

			// Find the displacment value
			double edge21_050displ_value = glm::length(edge21_050displ);

			glm::vec3 edge21_050contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge21_050displ_value));

			//____________________________________________________________________________________________________________
			// Edge 21 0.75
			glm::vec3 edge21_075displ = glm::vec3(quad.edge21_075modal_displ[j].x,
				quad.edge21_075modal_displ[j].y,
				quad.edge21_075modal_displ[j].z);

			// Find the displacment value
			double edge21_075displ_value = glm::length(edge21_075displ);

			glm::vec3 edge21_075contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge21_075displ_value));

			//____________________________________________________________________________________________________________
			//____________________________________________________________________________________________________________
			// Edge 14 0.25
			glm::vec3 edge14_025displ = glm::vec3(quad.edge14_025modal_displ[j].x,
				quad.edge14_025modal_displ[j].y,
				quad.edge14_025modal_displ[j].z);

			// Find the displacment value
			double edge14_025displ_value = glm::length(edge14_025displ);

			glm::vec3 edge14_025contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge14_025displ_value));

			//____________________________________________________________________________________________________________
			// Edge 14 0.50
			glm::vec3 edge14_050displ = glm::vec3(quad.edge14_050modal_displ[j].x,
				quad.edge14_050modal_displ[j].y,
				quad.edge14_050modal_displ[j].z);

			// Find the displacment value
			double edge14_050displ_value = glm::length(edge14_050displ);

			glm::vec3 edge14_050contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge14_050displ_value));

			//____________________________________________________________________________________________________________
			// Edge 14 0.75
			glm::vec3 edge14_075displ = glm::vec3(quad.edge14_075modal_displ[j].x,
				quad.edge14_075modal_displ[j].y,
				quad.edge14_075modal_displ[j].z);

			// Find the displacment value
			double edge14_075displ_value = glm::length(edge14_075displ);

			glm::vec3 edge14_075contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge14_075displ_value));

			//____________________________________________________________________________________________________________
			//____________________________________________________________________________________________________________
			// Edge 43 0.25
			glm::vec3 edge43_025displ = glm::vec3(quad.edge43_025modal_displ[j].x,
				quad.edge43_025modal_displ[j].y,
				quad.edge43_025modal_displ[j].z);

			// Find the displacment value
			double edge43_025displ_value = glm::length(edge43_025displ);

			glm::vec3 edge43_025contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge43_025displ_value));

			//____________________________________________________________________________________________________________
			// Edge 43 0.50
			glm::vec3 edge43_050displ = glm::vec3(quad.edge43_050modal_displ[j].x,
				quad.edge43_050modal_displ[j].y,
				quad.edge43_050modal_displ[j].z);

			// Find the displacment value
			double edge43_050displ_value = glm::length(edge43_050displ);

			glm::vec3 edge43_050contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge43_050displ_value));

			//____________________________________________________________________________________________________________
			// Edge 43 0.75
			glm::vec3 edge43_075displ = glm::vec3(quad.edge43_075modal_displ[j].x,
				quad.edge43_075modal_displ[j].y,
				quad.edge43_075modal_displ[j].z);

			// Find the displacment value
			double edge43_075displ_value = glm::length(edge43_075displ);

			glm::vec3 edge43_075contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - edge43_075displ_value));


			//____________________________________________________________________________________________________________
			//____________________________________________________________________________________________________________
			//____________________________________________________________________________________________________________

			// Node 1
			nd1_point_displ.push_back(nd1_displ); // Node 1 displ
			nd1_point_color.push_back(nd1_contour_color); // Node 1 color

			// Node 2
			nd2_point_displ.push_back(nd2_displ); // Node 2 displ
			nd2_point_color.push_back(nd2_contour_color); // Node 2 color

			// Node 3
			nd3_point_displ.push_back(nd3_displ); // Node 3 displ
			nd3_point_color.push_back(nd3_contour_color);  // Node 3 color

			// Node 4
			nd4_point_displ.push_back(nd4_displ); // Node 4 displ
			nd4_point_color.push_back(nd4_contour_color);  // Node 4 color

			// Edge 13 
			edge13_025point_color.push_back(edge13_025contour_color); // Edge 13 0.25
			edge13_050point_color.push_back(edge13_050contour_color); // Edge 13 0.50
			edge13_075point_color.push_back(edge13_075contour_color); // Edge 13 0.75

			// Edge 32
			edge32_025point_color.push_back(edge32_025contour_color); // Edge 32 0.25
			edge32_050point_color.push_back(edge32_050contour_color); // Edge 32 0.50
			edge32_075point_color.push_back(edge32_075contour_color); // Edge 32 0.75

			// Edge 21
			edge21_025point_color.push_back(edge21_025contour_color); // Edge 21 0.25
			edge21_050point_color.push_back(edge21_050contour_color); // Edge 21 0.50
			edge21_075point_color.push_back(edge21_075contour_color); // Edge 21 0.75

			// Edge 14
			edge14_025point_color.push_back(edge14_025contour_color); // Edge 14 0.25
			edge14_050point_color.push_back(edge14_050contour_color); // Edge 14 0.50
			edge14_075point_color.push_back(edge14_075contour_color); // Edge 14 0.75

			// Edge 43
			edge43_025point_color.push_back(edge43_025contour_color); // Edge 43 0.25
			edge43_050point_color.push_back(edge43_050contour_color); // Edge 43 0.50
			edge43_075point_color.push_back(edge43_075contour_color); // Edge 43 0.75

		}


		// Add to the 8 triangle list
		modal_element_tris132.add_tri(i, quad.nd1pt, quad.nd3pt, quad.nd2pt,
			nd1_point_displ, nd3_point_displ, nd2_point_displ,
			nd1_point_color, nd3_point_color, nd2_point_color,
			edge13_025point_color, edge13_050point_color, edge13_075point_color,
			edge32_025point_color, edge32_050point_color, edge32_075point_color,
			edge21_025point_color, edge21_050point_color, edge21_075point_color); // Tri 132
		modal_element_tris143.add_tri(i, quad.nd1pt, quad.nd4pt, quad.nd3pt,
			nd1_point_displ, nd4_point_displ, nd3_point_displ,
			nd1_point_color, nd4_point_color, nd3_point_color,
			edge14_025point_color, edge14_050point_color, edge14_075point_color,
			edge43_025point_color, edge43_050point_color, edge43_075point_color,
			edge13_075point_color, edge21_050point_color, edge13_025point_color); // Tri 143

		i++;
	}

	// Set the buffer
	modal_element_tris132.set_buffer(); // Tri 132
	modal_element_tris143.set_buffer(); // Tri 143

}


void modal_elementquad_list_store::update_buffer(int selected_mode)
{
	// Update the buffer based on the selected mode
	modal_element_tris132.update_buffer(selected_mode); // Tri 132
	modal_element_tris143.update_buffer(selected_mode); // Tri 143

}


void modal_elementquad_list_store::paint_modal_elementquadrilaterals()
{
	// Paint the triangles
	modal_element_tris132.paint_triangles(); // Tri 132
	modal_element_tris143.paint_triangles(); // Tri 143

}

void modal_elementquad_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation,
	bool set_rotatetranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	modal_element_tris132.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation,
		set_zoomtranslation, set_transparency, set_deflscale);
	modal_element_tris143.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation,
		set_zoomtranslation, set_transparency, set_deflscale);

}
