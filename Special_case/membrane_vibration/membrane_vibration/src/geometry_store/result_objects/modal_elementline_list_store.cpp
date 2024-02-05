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


void modal_elementline_list_store::set_buffer()
{
	// Clear existing modal line
	modal_element_lines.clear_lines();

	// Add the lines
	// Loop throug every line element
	int i = 0;
	for (auto& line_m : modal_elementlineMap)
	{
		modal_elementline_store ln = line_m.second;

		// start Pt
		std::vector<glm::vec3> startpt_point_displ; // start point displ
		
		// end Pt
		std::vector<glm::vec3> endpt_point_displ; // end point displ

		for (int j = 0; j < static_cast<int>(ln.startnd_modal_displ.size()); j++)
		{
			glm::vec3 startpt_displ = glm::vec3(ln.startnd_modal_displ[j].x,
												ln.startnd_modal_displ[j].y,
												ln.startnd_modal_displ[j].z);

			// Add to the list
			startpt_point_displ.push_back(startpt_displ);

			glm::vec3 endpt_displ = glm::vec3(ln.endnd_modal_displ[j].x,
				ln.endnd_modal_displ[j].y,
				ln.endnd_modal_displ[j].z);

			// Add to the list
			endpt_point_displ.push_back(endpt_displ);
		}

		// Add to the line list
		modal_element_lines.add_line(i, ln.startpt, ln.endpt, startpt_point_displ, endpt_point_displ);

		i++;
	}

	// Set the buffer
	modal_element_lines.set_buffer();
}

void modal_elementline_list_store::update_buffer(int selected_mode)
{
	// Update the buffer (because selected mode is changed)
	modal_element_lines.update_buffer(selected_mode);
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
