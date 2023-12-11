#include "result_elementline_list_store.h"

result_elementline_list_store::result_elementline_list_store()
{
	// Empty constructor
}

result_elementline_list_store::~result_elementline_list_store()
{
	// Empty destructor
}

void result_elementline_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameters for the line
	result_element_lines.init(geom_param_ptr);

	// Clear the element lines
	result_elementline_count = 0;
	result_elementlineMap.clear();
}

void result_elementline_list_store::clear_data()
{
	// Clear the element lines
	result_elementline_count = 0;
	result_elementlineMap.clear();

	// Dynamic lines
	result_element_lines.clear_lines();
}

void result_elementline_list_store::add_result_elementline(int& line_id, result_node_store* startNode, result_node_store* endNode)
{
	// Add result line element
	result_elementline_store temp_line;
	temp_line.line_id = line_id;
	temp_line.startNode = startNode;
	temp_line.endNode = endNode;

	// Check whether the node_id is already there
	if (result_elementlineMap.find(line_id) != result_elementlineMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	////__________________________ Add Hermite interpolation for Beam Element
	//temp_line.discretized_bar_line_data = set_line_bar_interpolation(interpolation_count, startNode, endNode);

	// Insert to the lines
	result_elementlineMap.insert({ line_id, temp_line });
	result_elementline_count++;
}

void result_elementline_list_store::set_buffer()
{




}

void result_elementline_list_store::paint_result_elementlines(const int& dyn_index)
{
	// Paint the lines
	result_element_lines.paint_lines(dyn_index);
}

void result_elementline_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, 
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	// Result line update geometry 
	result_element_lines.update_opengl_uniforms(set_modelmatrix, set_pantranslation, 
		set_zoomtranslation, set_transparency, set_deflscale);

}
