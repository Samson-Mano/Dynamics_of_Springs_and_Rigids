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

void pulse_elementline_list_store::add_pulse_elementline(int& line_id, pulse_node_store* startNode, pulse_node_store* endNode, bool& is_rigid)
{
	// Add result line element
	pulse_elementline_store temp_pulse_line;
	temp_pulse_line.line_id = line_id;
	temp_pulse_line.startNode = startNode;
	temp_pulse_line.endNode = endNode;
	temp_pulse_line.is_rigid = is_rigid; // set whether the line is rigid or not

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

		if (rline.is_rigid == true)
		{
			// Line is rigid (so no deformation along the length)
			set_rigid_element_line(rline, pulse_element_lines);

		}
		else
		{
			// Line is spring
			set_spring_element_line(rline, pulse_element_lines);

		}
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

void pulse_elementline_list_store::set_rigid_element_line(const pulse_elementline_store& rline, dynamic_line_list_store& pulse_element_lines)
{
	// Set the rigid element displacement lines
		// Get the Start node pt and End node pt
	glm::vec2 start_node_pt = rline.startNode->node_pt;
	glm::vec2 end_node_pt = rline.endNode->node_pt;

	// Line displacement magnitude
	std::vector<double> startpt_displ_magnitude = rline.startpt_displ_magnitude;
	std::vector<double> endpt_displ_magnitude = rline.endpt_displ_magnitude;

	// Line normalized displacement vector
	std::vector<glm::vec2> startpt_normalized_displ = rline.startpt_normalized_displ;
	std::vector<glm::vec2> endpt_normalized_displ = rline.endpt_normalized_displ;

	int time_step_count = static_cast<int>(rline.startpt_displ_magnitude.size());

	// Line length
	double element_length = geom_parameters::get_line_length(start_node_pt, end_node_pt);

	// Direction cosines
	double l_cos = (end_node_pt.x - start_node_pt.x) / element_length; // l cosine
	double m_sin = (start_node_pt.y - end_node_pt.y) / element_length; // m sine

	int i = 0; // loop variable
	int temp_line_id = -1; // to store the temporary line id
	// To store the temporary start and end point
	glm::vec2 temp_pt1 = glm::vec2(0);
	glm::vec2 temp_pt2 = glm::vec2(0);
	glm::vec2 temp_offset_pt = glm::vec2(0); // temporary offset point to add to the list

	// Displacement normailized
	float displ_ratio_1 = 0.0f;
	float displ_ratio_2 = 0.0f;

	std::vector<glm::vec3> temp_pt1_color; // to store start point color
	std::vector<glm::vec3> temp_pt2_color; // to store end point color

	std::vector<glm::vec2> temp_pt1_offset; // to store the start point offset
	std::vector<glm::vec2> temp_pt2_offset; // to store the end point offset


	// Flat ends of rigid
			// Flat end 1
	temp_pt1 = start_node_pt;
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f);

	for (i = 0; i < time_step_count; i++)
	{
		displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[i] / max_line_displ);
		displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[i] / max_line_displ);

		// Add the start point offset and start point color
		temp_offset_pt = displ_ratio_1 * startpt_normalized_displ[i];
		temp_pt1_offset.push_back(temp_offset_pt);
		temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f - displ_ratio_1));

		// Add the end point offset and end point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.25f);
		temp_pt2_offset.push_back(temp_offset_pt);
		temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.25f)));

	}

	temp_line_id = pulse_element_lines.dyn_line_count;

	pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);

	//_________________________________________________________________________________________________
	// Flat end 2
	temp_pt1_color.clear(); // clear start point color
	temp_pt2_color.clear(); // clear end point color

	temp_pt1_offset.clear(); // clear the start point offset
	temp_pt2_offset.clear(); // clear the end point offset


	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);
	temp_pt2 = end_node_pt;

	for (i = 0; i < time_step_count; i++)
	{
		displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[i] / max_line_displ);
		displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[i] / max_line_displ);

		// Add the start point offset and start point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.75f);
		temp_pt1_offset.push_back(temp_offset_pt);
		temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.75f)));

		// Add the end point offset and end point color
		temp_offset_pt = displ_ratio_2 * endpt_normalized_displ[i];
		temp_pt2_offset.push_back(temp_offset_pt);
		temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f - displ_ratio_2));

	}

	temp_line_id = pulse_element_lines.dyn_line_count;

	pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);

	//_________________________________________________________________________________________________
	// Rigid segment 1
	double rigid_width_amplitude = geom_param_ptr->rigid_element_width *
		(geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale);

	temp_pt1_color.clear(); // clear start point color
	temp_pt2_color.clear(); // clear end point color

	temp_pt1_offset.clear(); // clear the start point offset
	temp_pt2_offset.clear(); // clear the end point offset

	double pt_x = 0;
	double pt_y = rigid_width_amplitude * 1;

	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));

	for (i = 0; i < time_step_count; i++)
	{
		displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[i] / max_line_displ);
		displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[i] / max_line_displ);

		// Add the start point offset and start point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.25f);
		temp_pt1_offset.push_back(temp_offset_pt);
		temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.25f)));

		// Add the end point offset and end point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.75f);
		temp_pt2_offset.push_back(temp_offset_pt);
		temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.75f)));

	}

	temp_line_id = pulse_element_lines.dyn_line_count;

	pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);

	//_________________________________________________________________________________________________
	// Rigid segment 2
	temp_pt1_color.clear(); // clear start point color
	temp_pt2_color.clear(); // clear end point color

	temp_pt1_offset.clear(); // clear the start point offset
	temp_pt2_offset.clear(); // clear the end point offset

	pt_x = 0;
	pt_y = rigid_width_amplitude * -1;

	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f) +
		glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));



	for (i = 0; i < time_step_count; i++)
	{
		displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[i] / max_line_displ);
		displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[i] / max_line_displ);

		// Add the start point offset and start point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.25f);
		temp_pt1_offset.push_back(temp_offset_pt);
		temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.25f)));

		// Add the end point offset and end point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.75f);
		temp_pt2_offset.push_back(temp_offset_pt);
		temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.75f)));

	}

	temp_line_id = pulse_element_lines.dyn_line_count;

	pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);
}

void pulse_elementline_list_store::set_spring_element_line(const pulse_elementline_store& rline, dynamic_line_list_store& pulse_element_lines)
{
	// Set the spring element displacement lines
		// Get the Start node pt and End node pt
	glm::vec2 start_node_pt = rline.startNode->node_pt;
	glm::vec2 end_node_pt = rline.endNode->node_pt;

	// Line displacement magnitude
	std::vector<double> startpt_displ_magnitude = rline.startpt_displ_magnitude;
	std::vector<double> endpt_displ_magnitude = rline.endpt_displ_magnitude;

	// Line normalized displacement vector
	std::vector<glm::vec2> startpt_normalized_displ = rline.startpt_normalized_displ;
	std::vector<glm::vec2> endpt_normalized_displ = rline.endpt_normalized_displ;

	int time_step_count = static_cast<int>(rline.startpt_displ_magnitude.size());

	// Line length
	double element_length = geom_parameters::get_line_length(start_node_pt, end_node_pt);

	// Direction cosines
	double l_cos = (end_node_pt.x - start_node_pt.x) / element_length; // l cosine
	double m_sin = (start_node_pt.y - end_node_pt.y) / element_length; // m sine

	int i = 0; // loop variable
	int temp_line_id = -1; // to store the temporary line id
	// To store the temporary start and end point
	glm::vec2 temp_pt1 = glm::vec2(0);
	glm::vec2 temp_pt2 = glm::vec2(0);
	glm::vec2 temp_offset_pt = glm::vec2(0); // temporary offset point to add to the list

	// Displacement normailized
	float displ_ratio_1 = 0.0f;
	float displ_ratio_2 = 0.0f;

	std::vector<glm::vec3> temp_pt1_color; // to store start point color
	std::vector<glm::vec3> temp_pt2_color; // to store end point color

	std::vector<glm::vec2> temp_pt1_offset; // to store the start point offset
	std::vector<glm::vec2> temp_pt2_offset; // to store the end point offset


	// Flat ends of spring
	// Flat end 1
	temp_pt1 = start_node_pt;
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f);

	for (i = 0; i < time_step_count; i++)
	{
		displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[i] / max_line_displ);
		displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[i] / max_line_displ);

		// Add the start point offset and start point color
		temp_offset_pt = displ_ratio_1 * startpt_normalized_displ[i];
		temp_pt1_offset.push_back(temp_offset_pt);
		temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f - displ_ratio_1));

		// Add the end point offset and end point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.25f);
		temp_pt2_offset.push_back(temp_offset_pt);
		temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.25f)));

	}

	temp_line_id = pulse_element_lines.dyn_line_count;

	pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);

	//_________________________________________________________________________________________________
	// Flat end 2
	temp_pt1_color.clear(); // clear start point color
	temp_pt2_color.clear(); // clear end point color

	temp_pt1_offset.clear(); // clear the start point offset
	temp_pt2_offset.clear(); // clear the end point offset


	temp_pt1 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);
	temp_pt2 = end_node_pt;

	for (i = 0; i < time_step_count; i++)
	{
		displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[i] / max_line_displ);
		displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[i] / max_line_displ);

		// Add the start point offset and start point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.75f);
		temp_pt1_offset.push_back(temp_offset_pt);
		temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.75f)));

		// Add the end point offset and end point color
		temp_offset_pt = displ_ratio_2 * endpt_normalized_displ[i];
		temp_pt2_offset.push_back(temp_offset_pt);
		temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f - displ_ratio_2));

	}

	temp_line_id = pulse_element_lines.dyn_line_count;

	pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);

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

	for (i = 1; i < turn_count; i++)
	{
		param_t_prev = (i - 1) / static_cast<double>(turn_count);
		param_t = i / static_cast<double>(turn_count);

		pt_x = (param_t * element_length * 0.5f);
		pt_y = spring_width_amplitude * ((i % 2 == 0) ? 1 : -1);

		temp_pt2 = origin_pt + glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));

		temp_pt1_color.clear(); // clear start point color
		temp_pt2_color.clear(); // clear end point color

		temp_pt1_offset.clear(); // clear the start point offset
		temp_pt2_offset.clear(); // clear the end point offset

		for (int j = 0; j < time_step_count; j++)
		{
			displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[j] / max_line_displ);
			displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[j] / max_line_displ);

			// Add the start point offset and start point color
			temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
				displ_ratio_2 * endpt_normalized_displ[i], 0.25f + (param_t_prev * 0.5f));
			temp_pt1_offset.push_back(temp_offset_pt);
			temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f -
				geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.25f + (param_t_prev * 0.5f))));

			// Add the end point offset and end point color
			temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
				displ_ratio_2 * endpt_normalized_displ[i], 0.25f + (param_t * 0.5f));
			temp_pt2_offset.push_back(temp_offset_pt);
			temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f -
				geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.25f + (param_t * 0.5f))));

		}

		temp_line_id = pulse_element_lines.dyn_line_count;

		pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
			temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);

		// set the previous pt
		temp_pt1 = temp_pt2;
	}

	// Last point
	temp_pt2 = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);

	temp_pt1_color.clear(); // clear start point color
	temp_pt2_color.clear(); // clear end point color

	temp_pt1_offset.clear(); // clear the start point offset
	temp_pt2_offset.clear(); // clear the end point offset

	for (int j = 0; j < time_step_count; j++)
	{
		displ_ratio_1 = static_cast<float>(startpt_displ_magnitude[j] / max_line_displ);
		displ_ratio_2 = static_cast<float>(endpt_displ_magnitude[j] / max_line_displ);

		// Add the start point offset and start point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.25f + (param_t * 0.5f));
		temp_pt1_offset.push_back(temp_offset_pt);
		temp_pt1_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.25f + (param_t * 0.5f))));

		// Add the end point offset and end point color
		temp_offset_pt = geom_parameters::linear_interpolation(displ_ratio_1 * startpt_normalized_displ[i],
			displ_ratio_2 * endpt_normalized_displ[i], 0.75f);
		temp_pt2_offset.push_back(temp_offset_pt);
		temp_pt2_color.push_back(geom_parameters::getContourColor_d(1.0f -
			geom_parameters::get_lerp(displ_ratio_1, displ_ratio_2, 0.75f)));

	}

	temp_line_id = pulse_element_lines.dyn_line_count;

	pulse_element_lines.add_line(temp_line_id, temp_pt1, temp_pt2,
		temp_pt1_offset, temp_pt2_offset, temp_pt1_color, temp_pt2_color);
}
