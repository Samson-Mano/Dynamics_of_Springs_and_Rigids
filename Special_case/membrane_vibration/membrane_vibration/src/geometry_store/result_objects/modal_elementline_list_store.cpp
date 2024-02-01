#include "modal_elementline_list_store.h"

modal_elementline_list_store::modal_elementline_list_store()
{
	// Empty constructor
}

modal_elementline_list_store::~modal_elementline_list_store()
{
	// Empty destructor
}

void modal_elementline_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameters for the lines (and clear the lines)
	modal_element_lines.init(geom_param_ptr);

	// Clear the element lines
	modal_elementline_count = 0;
	modal_elementlineMap.clear();
}

void  modal_elementline_list_store::clear_data()
{
	// Clear the element lines
	modal_element_lines.clear_lines();
	modal_elementline_count = 0;
	modal_elementlineMap.clear();
}

void modal_elementline_list_store::add_modal_elementline(int& line_id, modal_node_store* startNode, modal_node_store* endNode)
{
	// Add result line element
	modal_elementline_store temp_line;
	temp_line.line_id = line_id;
	temp_line.startNode = startNode;
	temp_line.endNode = endNode;

	// Add the point coordinate
	temp_line.startpt = (*startNode).node_pt;
	temp_line.endpt = (*endNode).node_pt;

	// Add the modal displacement
	temp_line.startnd_modal_displ = (*startNode).node_modal_displ;
	temp_line.endnd_modal_displ = (*endNode).node_modal_displ;

	// Check whether the node_id is already there
	if (modal_elementlineMap.find(line_id) != modal_elementlineMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	// Insert to the lines
	modal_elementlineMap.insert({ line_id, temp_line });
	modal_elementline_count++;
}


void modal_elementline_list_store::set_buffer(int selected_mode)
{
	// Clear existing modal line
	modal_element_lines.clear_lines();

	double mode_max_displ = 1.0;
	double mode_min_displ = 0.0;

	// Add the lines
	// Loop throug every line element
	int i = 0;
	for (auto& line_m : modal_elementlineMap)
	{
		modal_elementline_store ln = line_m.second;

		// start Pt
		// Scale the displacement with maximum displacement
		glm::vec3 startpt_displ = glm::vec3(ln.startnd_modal_displ[selected_mode].x,
			ln.startnd_modal_displ[selected_mode].y, 
			ln.startnd_modal_displ[selected_mode].z);

		// Find the displacment value
		double startpt_displ_value =glm::length(startpt_displ);

		// scale the value with mode max displacement
		double startpt_displ_scale = (startpt_displ_value - mode_min_displ) / (mode_max_displ - mode_min_displ);

		glm::vec3 startpt_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - startpt_displ_scale));

		//___________________________________________________________________________________________________
		// end Pt
		// Scale the displacement with maximum displacement
		glm::vec3 endpt_displ = glm::vec3(ln.endnd_modal_displ[selected_mode].x,
			ln.endnd_modal_displ[selected_mode].y,
			ln.endnd_modal_displ[selected_mode].z);

		// Find the displacment value
		double endpt_displ_value = glm::length(endpt_displ);

		// scale the value with mode max displacement
		double endpt_displ_scale = (endpt_displ_value - mode_min_displ) / (mode_max_displ - mode_min_displ);

		glm::vec3 endpt_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - endpt_displ_scale));

		// Add to the line list
		modal_element_lines.add_line(i, ln.startpt, ln.endpt, startpt_displ, endpt_displ, 
			startpt_contour_color, endpt_contour_color, true);

		i++;
	}

	// Set the buffer
	modal_element_lines.set_buffer();
}

void modal_elementline_list_store::paint_modal_elementlines()
{
	// Paint the lines
	modal_element_lines.paint_lines();
}

void modal_elementline_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	modal_element_lines.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation, set_zoomtranslation, set_transparency, set_deflscale);
}
