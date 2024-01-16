#include "pulse_elementline_list_store.h"

pulse_elementline_list_store::pulse_elementline_list_store()
{
	// Empty constructor
}

pulse_elementline_list_store::~pulse_elementline_list_store()
{
	// Empty destructor
}

void pulse_elementline_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameters for the line
	pulse_element_lines.init(geom_param_ptr);

	max_line_displ = 0.0;
	element_max_length = 0.0;
	element_min_length = DBL_MAX;

	// Clear the element lines
	pulse_elementline_count = 0;
	pulse_elementlineMap.clear();
}

void pulse_elementline_list_store::clear_data()
{
	// Dynamic lines
    pulse_element_lines.clear_lines();

	max_line_displ = 0.0;
	element_max_length = 0.0;
	element_min_length = DBL_MAX;

	// Clear the element lines
	pulse_elementline_count = 0;
	pulse_elementlineMap.clear();
}

void pulse_elementline_list_store::add_pulse_elementline(int& line_id, pulse_node_store* startNode, pulse_node_store* endNode)
{
	// Add result line element
	pulse_elementline_store temp_pulse_line;
	temp_pulse_line.line_id = line_id;
	temp_pulse_line.startNode = startNode;
	temp_pulse_line.endNode = endNode;

	// Check whether the node_id is already there
	if (pulse_elementlineMap.find(line_id) != pulse_elementlineMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	//__________________________ Get the node points
	glm::vec3 temp_color = glm::vec3(0);
	glm::vec2 start_node_pt = (*startNode).node_pt;
	glm::vec2 end_node_pt = (*endNode).node_pt;

	// Find the maximum - minimum line length
	double element_length = geom_parameters::get_line_length(start_node_pt, end_node_pt);

	if (element_max_length < element_length)
	{
		element_max_length = element_length;
	}

	if (element_min_length > element_length)
	{
		element_min_length = element_length;
	}

	//__________________________ Add result lines Displacement results
	// Displacement magnitude list
	temp_pulse_line.startpt_displ_magnitude = startNode->node_pulse_result.displ_magnitude;
	temp_pulse_line.endpt_displ_magnitude = endNode->node_pulse_result.displ_magnitude;

	// Displacement vector list
	temp_pulse_line.startpt_normalized_displ = startNode->node_pulse_result.normalized_displ;
	temp_pulse_line.endpt_normalized_displ = endNode->node_pulse_result.normalized_displ;


	// Insert to the lines
	pulse_elementlineMap.insert({ line_id, temp_pulse_line });
	pulse_elementline_count++;
}

void pulse_elementline_list_store::set_buffer()
{
	// Clear the lines
	pulse_element_lines.clear_lines();

	//__________________________ Add the Dynamic lines
	for (auto& line_m : pulse_elementlineMap)
	{
		pulse_elementline_store  rline = line_m.second;

			std::vector<glm::vec2> line_startpt_offset; // list of start points offset
			std::vector<glm::vec2> line_endpt_offset; // list of end points offset

			std::vector<glm::vec3> line_startpt_color; // list of start point color
			std::vector<glm::vec3> line_endpt_color; // list of end point color

			// Add each individual segment of main line to list
			int i = 0;
			for (auto& pt1 : rline.startpt_normalized_displ)
			{
				// Pt1
				// Point1 displacement
				double pt_displ1 = rline.startpt_displ_magnitude[i];

				// Distance ratio1  Scale the displacement with maximum displacement
				double dist_ratio1 = pt_displ1 / max_line_displ;

				glm::vec2 pt1_offset = static_cast<float>(dist_ratio1) * pt1;

				// Add to the list
				line_startpt_offset.push_back(pt1_offset);

				// pt1 contour color
				glm::vec3 pt1_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - dist_ratio1));

				// Add to the list
				line_startpt_color.push_back(pt1_contour_color);

				i++;
			}

			i = 0;
			for (auto& pt2 : rline.endpt_normalized_displ)
			{
				// Pt2
				// Point2 displacement
				double pt_displ2 = rline.endpt_displ_magnitude[i];

				// Distance ratio1  Scale the displacement with maximum displacement
				double dist_ratio2 = pt_displ2 / max_line_displ;

				glm::vec2 pt2_offset = static_cast<float>(dist_ratio2) * pt2;

				// Add to the list
				line_endpt_offset.push_back(pt2_offset);

				// pt1 contour color
				glm::vec3 pt2_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - dist_ratio2));

				// Add to the list
				line_endpt_color.push_back(pt2_contour_color);

				i++;
			}

			// Add to the line list
			pulse_element_lines.add_line(rline.line_id, 
				rline.startNode->node_pt, 
				rline.endNode->node_pt,
				line_startpt_offset, line_endpt_offset, line_startpt_color, line_endpt_color);
			
	}

	// Set the buffer (Only the index buffer is set because its a dynamic paint)
	pulse_element_lines.set_buffer();
}

void pulse_elementline_list_store::paint_pulse_elementlines(const int& dyn_index)
{
	// Paint the lines
	pulse_element_lines.paint_lines(dyn_index);
}

void pulse_elementline_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	// Result line update geometry 
	pulse_element_lines.update_opengl_uniforms(set_modelmatrix, set_pantranslation,
		set_zoomtranslation, set_transparency, set_deflscale);
}
