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
	modal_element_tris12m.init(geom_param_ptr); // Tri 12m
	modal_element_tris23m.init(geom_param_ptr); // Tri 23m
	modal_element_tris34m.init(geom_param_ptr); // Tri 34m
	modal_element_tris41m.init(geom_param_ptr); // Tri 41m


	// Clear the element quads
	modal_elementquad_count = 0;
	modal_elementquadMap.clear();
}

void modal_elementquad_list_store::clear_data()
{
	// Clear the element quads
	modal_element_tris12m.clear_triangles(); // Tri 12m
	modal_element_tris23m.clear_triangles(); // Tri 23m
	modal_element_tris34m.clear_triangles(); // Tri 34m
	modal_element_tris41m.clear_triangles(); // Tri 41m
	modal_elementquad_count = 0;
	modal_elementquadMap.clear();

}

void modal_elementquad_list_store::add_modal_elementquadrilateral(int& quad_id, modal_node_store* nd1,
	modal_node_store* nd2, modal_node_store* nd3, modal_node_store* nd4)
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
	// Clear existing modal quads (2 triangles)
	modal_element_tris12m.clear_triangles(); // Tri 12m
	modal_element_tris23m.clear_triangles(); // Tri 23m
	modal_element_tris34m.clear_triangles(); // Tri 34m
	modal_element_tris41m.clear_triangles(); // Tri 41m

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

		// Mid Node
		std::vector<glm::vec3> mid_point_displ; // Mid displ
		std::vector<glm::vec3> mid_point_color; // Mid color

		// Mid Node 2 & 3 
		glm::vec3 mid_nd23 = geom_param_ptr->linear_interpolation3d(quad.nd2pt, quad.nd3pt, 0.5);

		// Mid Node 4 & 1 
		glm::vec3 mid_nd41 = geom_param_ptr->linear_interpolation3d(quad.nd4pt, quad.nd1pt, 0.5);

		// Mid Node
		glm::vec3 mid_nd = geom_param_ptr->linear_interpolation3d(mid_nd23, mid_nd41, 0.5);


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

			// Mid Node 2 & 3 
			glm::vec3 mid_nd23_displ = geom_param_ptr->linear_interpolation3d(nd2_displ, nd3_displ, 0.5);
			glm::vec3 mid_nd23_contour_color = geom_param_ptr->linear_interpolation3d(nd2_contour_color, nd3_contour_color, 0.5);

			// Mid Node 4 & 1 
			glm::vec3 mid_nd41_displ = geom_param_ptr->linear_interpolation3d(nd4_displ, nd1_displ, 0.5);
			glm::vec3 mid_nd41_contour_color = geom_param_ptr->linear_interpolation3d(nd4_contour_color, nd1_contour_color, 0.5);

			// Mid Node
			glm::vec3 mid_nd_displ = geom_param_ptr->linear_interpolation3d(mid_nd23_displ, mid_nd41_displ, 0.5);
			glm::vec3 mid_nd_contour_color = geom_param_ptr->linear_interpolation3d(mid_nd23_contour_color, mid_nd41_contour_color, 0.5);



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

			// Mid Node
			mid_point_displ.push_back(mid_nd_displ); // Mid displ
			mid_point_color.push_back(mid_nd_contour_color); // Mid color
		}



		// Add to the line list
		modal_element_tris12m.add_tri(i, quad.nd1pt, quad.nd2pt, mid_nd,
			nd1_point_displ, nd2_point_displ, mid_point_displ,
			nd1_point_color, nd2_point_color, mid_point_color);
			
		modal_element_tris23m.add_tri(i, quad.nd2pt, quad.nd3pt, mid_nd,
			nd2_point_displ, nd3_point_displ, mid_point_displ,
			nd2_point_color, nd3_point_color, mid_point_color);

		modal_element_tris34m.add_tri(i, quad.nd3pt, quad.nd4pt, mid_nd,
			nd3_point_displ, nd4_point_displ, mid_point_displ,
			nd3_point_color, nd4_point_color, mid_point_color);

		modal_element_tris41m.add_tri(i, quad.nd4pt, quad.nd1pt, mid_nd,
			nd4_point_displ, nd1_point_displ, mid_point_displ,
			nd4_point_color, nd1_point_color, mid_point_color);

		i++;
	}

	// Set the buffer
	modal_element_tris12m.set_buffer(); // Tri 12m
	modal_element_tris23m.set_buffer(); // Tri 23m
	modal_element_tris34m.set_buffer(); // Tri 34m
	modal_element_tris41m.set_buffer(); // Tri 41m
}


void modal_elementquad_list_store::update_buffer(int selected_mode)
{
	modal_element_tris12m.update_buffer(selected_mode); // Tri 12m
	modal_element_tris23m.update_buffer(selected_mode); // Tri 23m
	modal_element_tris34m.update_buffer(selected_mode); // Tri 34m
	modal_element_tris41m.update_buffer(selected_mode); // Tri 41m

}


void modal_elementquad_list_store::paint_modal_elementquadrilaterals()
{
	// Paint the triangles
	modal_element_tris12m.paint_triangles(); // Tri 12m
	modal_element_tris23m.paint_triangles(); // Tri 23m
	modal_element_tris34m.paint_triangles(); // Tri 34m
	modal_element_tris41m.paint_triangles(); // Tri 41m

}

void modal_elementquad_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, 
	bool set_rotatetranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	modal_element_tris12m.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation,
		set_zoomtranslation, set_transparency, set_deflscale);
	modal_element_tris23m.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation,
		set_zoomtranslation, set_transparency, set_deflscale);
	modal_element_tris34m.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation,
		set_zoomtranslation, set_transparency, set_deflscale);
	modal_element_tris41m.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation,
		set_zoomtranslation, set_transparency, set_deflscale);

}
