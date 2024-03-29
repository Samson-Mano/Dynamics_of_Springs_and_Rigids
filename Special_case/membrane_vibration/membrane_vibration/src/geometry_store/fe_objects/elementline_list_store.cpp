#include "elementline_list_store.h"

elementline_list_store::elementline_list_store()
{
	// Empty constructor
}

elementline_list_store::~elementline_list_store()
{
	// Empty destructor
}

void elementline_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameters for the labels (and clear the labels)
	element_lines.init(geom_param_ptr);

	// Clear the lines
	elementline_count = 0;
	elementlineMap.clear();
}

void elementline_list_store::add_elementline(int& line_id, node_store* startNode, node_store* endNode)
{
	// Add the line to the list
	elementline_store temp_line;
	temp_line.line_id = line_id;
	temp_line.startNode = startNode;
	temp_line.endNode = endNode;

	// Check whether the line_id is already there
	if (elementlineMap.find(line_id) != elementlineMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	// Check whether the element already exists
	for (auto& ln_m : elementlineMap)
	{
		elementline_store ln = ln_m.second;

		if ((ln.startNode->node_id == startNode->node_id &&
			ln.endNode->node_id == endNode->node_id) ||
			(ln.startNode->node_id == endNode->node_id &&
				ln.endNode->node_id == startNode->node_id))
		{
			// Element already exists
			return;
		}
	}


	// Insert to the lines
	elementlineMap.insert({ line_id, temp_line });
	elementline_count++;

	//__________________________ Add the node points
	glm::vec3 temp_color = geom_param_ptr->geom_colors.line_color;
	glm::vec3 start_node_pt = (*startNode).node_pt;
	glm::vec3 end_node_pt = (*endNode).node_pt;

	//__________________________ Add the lines
	element_lines.add_line(line_id, start_node_pt, end_node_pt,
									 temp_color, temp_color);

}

void elementline_list_store::set_buffer()
{
	// Set the buffers for the Model
	element_lines.set_buffer();
}

void elementline_list_store::paint_elementlines()
{
	// Paint the model lines
	element_lines.paint_lines();
}


void elementline_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	// Update model openGL uniforms
	element_lines.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation, set_zoomtranslation, set_transparency, set_deflscale);

}