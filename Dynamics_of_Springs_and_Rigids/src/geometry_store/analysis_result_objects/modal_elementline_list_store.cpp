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

	// Set the geometry parameters for the labels (and clear the labels)
	modal_element_lines.init(geom_param_ptr);

	// Clear the element lines
	modal_elementline_count = 0;
	modal_elementlineMap.clear();
}

void  modal_elementline_list_store::clear_data()
{
	modal_element_lines.clear_lines();

	// Clear the element lines
	modal_elementline_count = 0;
	modal_elementlineMap.clear();
}

void modal_elementline_list_store::add_modal_elementline(int& line_id, modal_node_store* startNode, modal_node_store* endNode, bool& is_rigid)
{
	// Add result line element
	modal_elementline_store temp_line;
	temp_line.line_id = line_id;
	temp_line.startNode = startNode;
	temp_line.endNode = endNode;


	// Check whether the node_id is already there
	if (modal_elementlineMap.find(line_id) != modal_elementlineMap.end())
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

	temp_line.is_rigid = is_rigid; // set whether the line is rigid or not

	//__________________________ Add result lines Displacement results
	// Node Displacement vector list
	temp_line.startpt_modal_displ = startNode->node_modal_displ;
	temp_line.endpt_modal_displ = endNode->node_modal_displ;


	// Insert to the lines
	modal_elementlineMap.insert({ line_id, temp_line });
	modal_elementline_count++;
}


void modal_elementline_list_store::set_buffer(int selected_mode)
{
	// Clear existing modal line
	modal_element_lines.clear_lines();

	double mode_max_displ = max_node_displ[selected_mode];
	double mode_min_displ = min_node_displ[selected_mode];

	// Add the lines
	// Loop throug every line element
	for (auto& line_m : modal_elementlineMap)
	{
		modal_elementline_store rline = line_m.second;

		if (rline.is_rigid == true)
		{
			// Line is rigid (so no deformation along the length)
			set_rigid_element_line(rline, selected_mode, modal_element_lines);
		}
		else
		{
			// Line is spring
			set_spring_element_line(rline, selected_mode, modal_element_lines);

		}

	}

	// Set the buffer
	modal_element_lines.set_buffer();
}

void modal_elementline_list_store::paint_modal_elementlines()
{
	modal_element_lines.paint_lines();
}

void modal_elementline_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	modal_element_lines.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
}

void modal_elementline_list_store::set_rigid_element_line(modal_elementline_store& rline, const int& selected_mode, line_list_store& modal_element_lines)
{
	// Set the rigid element displacement lines
	// Get the Start node pt and End node pt
	int line_id = rline.line_id;

	glm::vec2 start_node_pt = rline.startNode->node_pt;
	glm::vec2 end_node_pt = rline.endNode->node_pt;

	// Line nodal displacement vector
	glm::vec2 startpt_modal_displ = rline.startpt_modal_displ[selected_mode];
	glm::vec2 endpt_modal_displ = rline.endpt_modal_displ[selected_mode];

	// Line length
	double element_length = geom_parameters::get_line_length(start_node_pt, end_node_pt);

	// Direction cosines
	double l_cos = (end_node_pt.x - start_node_pt.x) / element_length; // l cosine
	double m_sin = (start_node_pt.y - end_node_pt.y) / element_length; // m sine


	int temp_line_id = -1; // to store the temporary line id
	// To store the temporary start and end point
	glm::vec2 temp_pt1 = glm::vec2(0); // Pt 1
	glm::vec2 temp_pt2 = glm::vec2(0); // Pt 2
	glm::vec2 origin = glm::vec2(0);
	glm::vec2 temp_pt1_offset = glm::vec2(0); // temporary start point offset
	glm::vec2 temp_pt2_offset = glm::vec2(0); // temporary end point offset

	glm::vec3 temp_pt1_color = glm::vec3(0); // to store start point color
	glm::vec3 temp_pt2_color = glm::vec3(0); // to store end point color

	// Displacement normailized
	float displ_ratio_1 = 0.0f;
	float displ_ratio_2 = 0.0f;


	// Flat ends of rigid
	// Flat end 1
	temp_pt1 = start_node_pt;
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f);

	// offset
	temp_pt1_offset = startpt_modal_displ;
	temp_pt2_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.25f);

	displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
	displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

	// color
	temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
	temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color

	temp_line_id = modal_element_lines.line_count;
	modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);

	//_________________________________________________________________________________________________
	// Flat end 2
	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);
	temp_pt2 = end_node_pt;

	// offset
	temp_pt1_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.75f); 
	temp_pt2_offset = endpt_modal_displ;

	displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
	displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

	// color
	temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
	temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color

	temp_line_id = modal_element_lines.line_count;
	modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);


	//_________________________________________________________________________________________________
	// Rigid segment 1
	double rigid_width_amplitude = geom_param_ptr->rigid_element_width *
		(geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale);

	double pt_x = 0;
	double pt_y = rigid_width_amplitude * 1;

	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));

	// offset
	temp_pt1_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.25f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
	temp_pt2_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.75f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));

	displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
	displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

	// color
	temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
	temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color

	temp_line_id = modal_element_lines.line_count;
	modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);

	//_________________________________________________________________________________________________
	// Rigid segment 2
	pt_x = 0;
	pt_y = rigid_width_amplitude * -1;

	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));

	// offset
	temp_pt1_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.25f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
	temp_pt2_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.75f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));

	displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
	displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

	// color
	temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
	temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color

	temp_line_id = modal_element_lines.line_count;
	modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);


}

void modal_elementline_list_store::set_spring_element_line(modal_elementline_store& rline, const int& selected_mode, line_list_store& modal_element_lines)
{
	// Set the spring element displacement lines
	// Get the Start node pt and End node pt
	int line_id = rline.line_id;

	glm::vec2 start_node_pt = rline.startNode->node_pt;
	glm::vec2 end_node_pt = rline.endNode->node_pt;

	// Line nodal displacement vector
	glm::vec2 startpt_modal_displ = rline.startpt_modal_displ[selected_mode];
	glm::vec2 endpt_modal_displ = rline.endpt_modal_displ[selected_mode];

	// Line length
	double element_length = geom_parameters::get_line_length(start_node_pt, end_node_pt);

	// Direction cosines
	double l_cos = (end_node_pt.x - start_node_pt.x) / element_length; // l cosine
	double m_sin = (start_node_pt.y - end_node_pt.y) / element_length; // m sine


	int temp_line_id = -1; // to store the temporary line id
	// To store the temporary start and end point
	glm::vec2 temp_pt1 = glm::vec2(0); // Pt 1
	glm::vec2 temp_pt2 = glm::vec2(0); // Pt 2
	glm::vec2 origin = glm::vec2(0);
	glm::vec2 temp_pt1_offset = glm::vec2(0); // temporary start point offset
	glm::vec2 temp_pt2_offset = glm::vec2(0); // temporary end point offset

	glm::vec3 temp_pt1_color = glm::vec3(0); // to store start point color
	glm::vec3 temp_pt2_color = glm::vec3(0); // to store end point color

	// Displacement normailized
	float displ_ratio_1 = 0.0f;
	float displ_ratio_2 = 0.0f;


	// Flat ends of spring
	// Flat end 1
	temp_pt1 = start_node_pt;
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f);

	// offset
	temp_pt1_offset = startpt_modal_displ;
	temp_pt2_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.25f);

	displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
	displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

	// color
	temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
	temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color

	temp_line_id = modal_element_lines.line_count;
	modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);

	//_________________________________________________________________________________________________
	// Flat end 2
	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);
	temp_pt2 = end_node_pt;

	// offset
	temp_pt1_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.75f);
	temp_pt2_offset = endpt_modal_displ;

	displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
	displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

	// color
	temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
	temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color

	temp_line_id = modal_element_lines.line_count;
	modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);


	//_________________________________________________________________________________________________
	// Spring portion
	int turn_count = static_cast<int>(geom_parameters::get_remap(element_max_length, element_min_length,
		geom_param_ptr->spring_turn_max, geom_param_ptr->spring_turn_min, element_length)); // spring turn frequency

	double spring_width_amplitude = geom_param_ptr->spring_element_width *
		(geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale);

	// Parameter t
	double param_t_prev = 0.0;
	double param_t = 0.0;

	double pt_x = 0.0;
	double pt_y = 0.0;

	glm::vec2 origin_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f);
	temp_pt1 = origin_pt;

	for (int i = 1; i < turn_count; i++)
	{
		param_t_prev = (i - 1) / static_cast<double>(turn_count);
		param_t = i / static_cast<double>(turn_count);

		pt_x = (param_t * element_length * 0.5f);
		pt_y = spring_width_amplitude * ((i % 2 == 0) ? 1 : -1);

		temp_pt2 = origin_pt + glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));

		// offset
		temp_pt1_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.25f + (param_t_prev * 0.5f));
		temp_pt2_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.25f + (param_t * 0.5f));

		displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
		displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

		// color
		temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
		temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color

		temp_line_id = modal_element_lines.line_count;
		modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
			temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);

		// set the previous pt
		temp_pt1 = temp_pt2;
	}

	// Last point
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);

	// offset
	temp_pt1_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.25f + (param_t * 0.5f));
	temp_pt2_offset = geom_parameters::linear_interpolation(startpt_modal_displ, endpt_modal_displ, 0.75f);

	displ_ratio_1 = geom_parameters::get_line_length(origin, temp_pt1_offset);
	displ_ratio_2 = geom_parameters::get_line_length(origin, temp_pt2_offset);

	// color
	temp_pt1_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_1); // to store start point color
	temp_pt2_color = geom_parameters::getContourColor_d(1.0f - displ_ratio_2); // to store end point color


	temp_line_id = modal_element_lines.line_count;
	modal_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color, true);

}

