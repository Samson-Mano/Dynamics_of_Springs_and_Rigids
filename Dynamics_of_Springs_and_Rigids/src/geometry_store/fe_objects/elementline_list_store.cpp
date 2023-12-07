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
	line_id_labels.init(geom_param_ptr);
	line_length_labels.init(geom_param_ptr);
	selected_element_lines.init(geom_param_ptr);
	line_material_id_labels.init(geom_param_ptr);

	// Reset the line max - min
	element_max_length = 0.0;
	element_min_length = DBL_MAX;

	// Clear the lines
	elementline_count = 0;
	elementlineMap.clear();
}

void elementline_list_store::add_elementline(int& line_id, node_store* startNode, node_store* endNode, int& material_id)
{
	// Add the line to the list
	elementline_store temp_line;
	temp_line.line_id = line_id;
	temp_line.material_id = material_id;
	temp_line.startNode = startNode;
	temp_line.endNode = endNode;

	// Check whether the line id is already there
	if (elementlineMap.find(line_id) != elementlineMap.end())
	{
		// Element ID already exist (do not add)
		return;
	}

	// Check whether the startNode and endNode already exists (regardless of order)
	for (const auto& line : elementlineMap)
	{
		const elementline_store& existing_line = line.second;

		if ((existing_line.startNode->node_id == startNode->node_id && existing_line.endNode->node_id == endNode->node_id) ||
			(existing_line.startNode->node_id == endNode->node_id && existing_line.endNode->node_id == startNode->node_id))
		{
			// Line with the same start and end nodes already exists (do not add)
			return;
		}
	}

	// Insert to the lines
	elementlineMap.insert({ line_id, temp_line });
	elementline_count++;

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


	//__________________________ Add the Line ID labels
	std::string temp_str = "";
	temp_color = geom_param_ptr->geom_colors.line_length_color;

	temp_str = "[" + std::to_string(line_id) + "]";
	glm::vec2 line_mid_pt = (start_node_pt + end_node_pt) / 2.0f;

	// Calculate the angle between the line segment and the x-axis
	float line_angle = atan2(end_node_pt.y - start_node_pt.y, end_node_pt.x - start_node_pt.x);

	line_id_labels.add_text(temp_str, line_mid_pt, glm::vec2(0), temp_color, line_angle, true, false);

	// Add the Line Length label
	float line_length = glm::distance(start_node_pt, end_node_pt);

	std::stringstream ss;
	ss << std::fixed << std::setprecision(geom_param_ptr->length_precision) << line_length;

	temp_str = ss.str();

	line_length_labels.add_text(temp_str, line_mid_pt, glm::vec2(0), temp_color, line_angle, false, false);
}

void elementline_list_store::add_selection_lines(const std::vector<int>& selected_edge_ids)
{
	// Clear the existing selected lines
	selected_element_lines.clear_lines();

	// Add to Selected Edges
	glm::vec3 temp_color = geom_param_ptr->geom_colors.selection_color;

	for (const auto& it : selected_edge_ids)
	{
		// ID, Start node pt and End node pt of the selected line
		glm::vec2 start_node_pt = elementlineMap[it].startNode->node_pt;
		glm::vec2 end_node_pt = elementlineMap[it].endNode->node_pt;

		// Line length
		double element_length = geom_parameters::get_line_length(start_node_pt, end_node_pt);
		
		// Flat ends of the spring
		int line_id = selected_element_lines.line_count;
		glm::vec2 curr_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f);

		// Flat end 1
		selected_element_lines.add_line(line_id, start_node_pt, curr_pt,
			glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

		curr_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);

		// Flat end 2
		line_id = selected_element_lines.line_count;
		selected_element_lines.add_line(line_id, curr_pt, end_node_pt,
			glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

		// Spring portion
		double l_cos = (end_node_pt.x - start_node_pt.x) / element_length; // l cosine
		double m_sin = (start_node_pt.y - end_node_pt.y) / element_length; // m sine

		glm::vec2 origin_pt = geom_parameters::linear_interpolation(start_node_pt, 
			end_node_pt, 0.25f); // origin pt for adding the spring portion
		glm::vec2 prev_pt = glm::vec2(0);

		if (elementlineMap[it].material_id < 1)
		{
			// Rigid elements
			//__________________________ Add the lines
				double rigid_width_amplitude = geom_param_ptr->rigid_element_width * 
				(geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale);

			// Rigid segment 1
			double pt_x = 0;
			double pt_y = rigid_width_amplitude*1;

			curr_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			curr_pt = curr_pt + origin_pt;

			pt_x = element_length * 0.5f;
			pt_y = rigid_width_amplitude * 1;

			prev_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			prev_pt = prev_pt + origin_pt;

			line_id = selected_element_lines.line_count;
			selected_element_lines.add_line(line_id, curr_pt, prev_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

			// Rigid segment 2
			pt_x = 0;
			pt_y = rigid_width_amplitude * -1;

			curr_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			curr_pt = curr_pt + origin_pt;

			pt_x = element_length * 0.5f;
			pt_y = rigid_width_amplitude * -1;

			prev_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			prev_pt = prev_pt + origin_pt;

			line_id = selected_element_lines.line_count;
			selected_element_lines.add_line(line_id, curr_pt, prev_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);
		}
		else
		{
		
			int turn_count = static_cast<int>(geom_parameters::get_remap(element_max_length, element_min_length,
				spring_turn_max, spring_turn_min, element_length)); // spring turn frequency

			glm::vec2 prev_pt = origin_pt;
			curr_pt = glm::vec2(0);

			double spring_width_amplitude = geom_param_ptr->spring_element_width * 
				(geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale);

			// Points of springs
			for (int i = 1; i < turn_count; i++)
			{
				double param_t = i / static_cast<double>(turn_count);

				double pt_x = (param_t * element_length * 0.5f);
				double pt_y = spring_width_amplitude * ((i % 2 == 0) ? 1 : -1);

				curr_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
				curr_pt = curr_pt + origin_pt;

				line_id = selected_element_lines.line_count;

				selected_element_lines.add_line(line_id, prev_pt, curr_pt,
					glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

				// set the previous pt
				prev_pt = curr_pt;
			}

			// Last point
			curr_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);

			line_id = selected_element_lines.line_count;

			selected_element_lines.add_line(line_id, prev_pt, curr_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);
		}
	}

	// Set the selected element lines buffer
	selected_element_lines.set_buffer();
}


void elementline_list_store::update_material(const std::vector<int> selected_element_line, const int& material_id)
{
	// Update the material ID
	for (const int& it : selected_element_line)
	{
		elementlineMap[it].material_id = material_id;
	}

	// Update the material ID label
	update_material_id_labels();
}


void elementline_list_store::execute_delete_material(const int& del_material_id)
{
	// Update delete material
	bool is_del_material_found = false; // Flag to check whether material id deleted

	// Delete the material
	for (const auto& ln : elementlineMap)
	{
		int id = ln.first; // get the id
		if (elementlineMap[id].material_id == del_material_id)
		{
			// Delete material is removed and the material ID of that element to 1 (Default spring element)
			elementlineMap[id].material_id = 1;
			is_del_material_found = true;
		}
	}

	// Update the material ID label
	if (is_del_material_found == true)
	{
		update_material_id_labels();
	}

}


void elementline_list_store::set_buffer()
{
	// Set the buffers for the Model
	recreate_element_lines();
	update_material_id_labels();
	// Line ID and Line Length labels
	line_id_labels.set_buffer();
	line_length_labels.set_buffer();
}

void elementline_list_store::paint_elementlines()
{
	// Paint the model lines
	element_lines.paint_lines();
}

void elementline_list_store::paint_selected_elementlines()
{
	//Paint the selected model lines
	selected_element_lines.paint_lines();
}


void elementline_list_store::paint_label_line_ids()
{
	// Paint the line id labels
	line_id_labels.paint_text();
}

void elementline_list_store::paint_label_line_lengths()
{
	// Paint the line length labels
	line_length_labels.paint_text();
}

void elementline_list_store::paint_lines_material_id()
{
	// Paint line material ID
	line_material_id_labels.paint_text();
}

void elementline_list_store::recreate_element_lines()
{
	// Re-create the element lines
		// clear the element lines
	element_lines.clear_lines();

	glm::vec3 temp_color = glm::vec3(0);

	for (const auto& ln_m : elementlineMap)
	{
		elementline_store ln = ln_m.second;

		// ID, Start node pt and End node pt
		glm::vec2 start_node_pt = ln.startNode->node_pt;
		glm::vec2 end_node_pt = ln.endNode->node_pt;
		glm::vec2 curr_pt = glm::vec2(0);
		glm::vec2 origin_pt = glm::vec2(0);
		glm::vec2 prev_pt = glm::vec2(0);

		// Line length
		double element_length = geom_parameters::get_line_length(start_node_pt, end_node_pt);

		// Direction cosines
		double l_cos = (end_node_pt.x - start_node_pt.x) / element_length; // l cosine
		double m_sin = (start_node_pt.y - end_node_pt.y) / element_length; // m sine


		if (ln.material_id < 1)
		{
			// Rigid elements
			//__________________________ Add the lines
			temp_color = geom_param_ptr->geom_colors.rigid_line_color;

			// Flat ends of rigid
			// Flat end 1
			curr_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f);

			int line_id = element_lines.line_count;
			element_lines.add_line(line_id, start_node_pt, curr_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

			// Flat end 2
			curr_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);

			line_id = element_lines.line_count;
			element_lines.add_line(line_id, curr_pt, end_node_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);


			// Rigid segment 1
			double rigid_width_amplitude = geom_param_ptr->rigid_element_width *
				(geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale);


			origin_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f); // origin point

			double pt_x = 0;
			double pt_y = rigid_width_amplitude * 1;

			curr_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			curr_pt = curr_pt + origin_pt;

			pt_x = element_length * 0.5f;
			pt_y = rigid_width_amplitude * 1;

			prev_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			prev_pt = prev_pt + origin_pt;

			line_id = element_lines.line_count;
			element_lines.add_line(line_id, curr_pt, prev_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

			// Rigid segment 2
			pt_x = 0;
			pt_y = rigid_width_amplitude * -1;

			curr_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			curr_pt = curr_pt + origin_pt;

			pt_x = element_length * 0.5f;
			pt_y = rigid_width_amplitude * -1;

			prev_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
			prev_pt = prev_pt + origin_pt;

			line_id = element_lines.line_count;
			element_lines.add_line(line_id, curr_pt, prev_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);
		}
		else
		{
			// Flat ends of the spring
			int line_id = element_lines.line_count;
			temp_color = geom_param_ptr->geom_colors.spring_line_color;
			curr_pt = geom_parameters::linear_interpolation(start_node_pt,end_node_pt,0.25f);

			// Flat end 1
			element_lines.add_line(line_id, start_node_pt, curr_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

			curr_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);

			// Flat end 2
			line_id = element_lines.line_count;
			element_lines.add_line(line_id, curr_pt, end_node_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);


			// Spring portion
					int turn_count = static_cast<int>(geom_parameters::get_remap(element_max_length, element_min_length,
				spring_turn_max,spring_turn_min,element_length)); // spring turn frequency

			origin_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.25f); // origin point
			prev_pt = origin_pt;
			curr_pt = glm::vec2(0);

			double spring_width_amplitude = geom_param_ptr->spring_element_width *
				(geom_param_ptr->node_circle_radii / geom_param_ptr->geom_scale);

			// Points of springs
			for (int i = 1; i < turn_count; i++)
			{
				double param_t = i / static_cast<double>(turn_count);

				double pt_x = (param_t * element_length * 0.5f);
				double pt_y = spring_width_amplitude * ((i % 2 == 0) ? 1 : -1);

				curr_pt = glm::vec2(((l_cos * pt_x) + (m_sin * pt_y)), ((-1.0 * m_sin * pt_x) + (l_cos * pt_y)));
				curr_pt = curr_pt + origin_pt;

				line_id = element_lines.line_count;

				element_lines.add_line(line_id, prev_pt, curr_pt,
					glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);

				// set the previous pt
				prev_pt = curr_pt;
			}

			// Last point
			curr_pt = geom_parameters::linear_interpolation(start_node_pt, end_node_pt, 0.75f);

			line_id = element_lines.line_count;

			element_lines.add_line(line_id, prev_pt, curr_pt,
				glm::vec2(0), glm::vec2(0), temp_color, temp_color, false);
		}
	}

	element_lines.set_buffer();

}

int elementline_list_store::is_line_hit(glm::vec2& loc)
{
	// Return the line id of line which is clicked
	// Covert mouse location to screen location
	int max_dim = geom_param_ptr->window_width > geom_param_ptr->window_height ? geom_param_ptr->window_width : geom_param_ptr->window_height;

	// Transform the mouse location to openGL screen coordinates
	glm::vec2 screenPt = glm::vec2(2.0f * ((loc.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - loc.y) / max_dim));

	// Nodal location
	glm::mat4 scaling_matrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
	scaling_matrix[3][3] = 1.0f;

	glm::mat4 scaledModelMatrix = scaling_matrix * geom_param_ptr->modelMatrix;

	// Loop through all nodes in map and update min and max values
	for (auto it = elementlineMap.begin(); it != elementlineMap.end(); ++it)
	{
		elementline_store elementline = it->second;

		glm::vec2 s_node = elementline.startNode->node_pt;
		glm::vec2 e_node = elementline.endNode->node_pt;

		glm::vec4 s_node_finalPosition = scaledModelMatrix * glm::vec4(s_node.x, s_node.y, 0, 1.0f) * geom_param_ptr->panTranslation;
		glm::vec4 e_node_finalPosition = scaledModelMatrix * glm::vec4(e_node.x, e_node.y, 0, 1.0f) * geom_param_ptr->panTranslation;

		// S & E Point 
		glm::vec2 spt = glm::vec2(s_node_finalPosition.x, s_node_finalPosition.y);
		glm::vec2 ept = glm::vec2(e_node_finalPosition.x, e_node_finalPosition.y);

		float threshold = 8 * geom_param_ptr->node_circle_radii;

		if (isClickPointOnLine(screenPt, spt, ept, threshold) == true)
		{
			// Return the Id of the line if hit == true
			return it->first;
		}
	}

	return -1;
}

std::vector<int> elementline_list_store::is_line_selected(const glm::vec2& corner_pt1, const glm::vec2& corner_pt2)
{
	// Return the node id of node which is inside the rectangle
	// Covert mouse location to screen location
	int max_dim = geom_param_ptr->window_width > geom_param_ptr->window_height ? geom_param_ptr->window_width : geom_param_ptr->window_height;

	// Selected node list index;
	std::vector<int> selected_edge_index;

	// Transform the mouse location to openGL screen coordinates
	// Corner Point 1
	glm::vec2 screen_cpt1 = glm::vec2(2.0f * ((corner_pt1.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt1.y) / max_dim));

	// Corner Point 2
	glm::vec2 screen_cpt2 = glm::vec2(2.0f * ((corner_pt2.x - (geom_param_ptr->window_width * 0.5f)) / max_dim),
		2.0f * (((geom_param_ptr->window_height * 0.5f) - corner_pt2.y) / max_dim));

	// Nodal location
	glm::mat4 scaling_matrix = glm::mat4(1.0) * static_cast<float>(geom_param_ptr->zoom_scale);
	scaling_matrix[3][3] = 1.0f;

	glm::mat4 scaledModelMatrix = scaling_matrix * geom_param_ptr->modelMatrix;

	// Loop through all edges in map
	for (auto it = elementlineMap.begin(); it != elementlineMap.end(); ++it)
	{
		const glm::vec2& start_pt = it->second.startNode->node_pt;
		const glm::vec2& end_pt = it->second.endNode->node_pt;
		glm::vec2 pt_025 = geom_param_ptr->linear_interpolation(start_pt, end_pt, 0.25);
		glm::vec2 pt_050 = geom_param_ptr->linear_interpolation(start_pt, end_pt, 0.50);
		glm::vec2 pt_075 = geom_param_ptr->linear_interpolation(start_pt, end_pt, 0.75);

		glm::vec4 start_pt_fp = scaledModelMatrix * glm::vec4(start_pt.x, start_pt.y, 0, 1.0f) * geom_param_ptr->panTranslation;
		glm::vec4 end_pt_fp = scaledModelMatrix * glm::vec4(end_pt.x, end_pt.y, 0, 1.0f) * geom_param_ptr->panTranslation;
		glm::vec4 pt_025_fp = scaledModelMatrix * glm::vec4(pt_025.x, pt_025.y, 0, 1.0f) * geom_param_ptr->panTranslation;
		glm::vec4 pt_050_fp = scaledModelMatrix * glm::vec4(pt_050.x, pt_050.y, 0, 1.0f) * geom_param_ptr->panTranslation;
		glm::vec4 pt_075_fp = scaledModelMatrix * glm::vec4(pt_075.x, pt_075.y, 0, 1.0f) * geom_param_ptr->panTranslation;

		// Check whether the point inside a rectangle
		if (geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, start_pt_fp) == true || 
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, end_pt_fp) == true || 
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, pt_025_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, pt_050_fp) == true ||
			geom_param_ptr->isPointInsideRectangle(screen_cpt1, screen_cpt2, pt_075_fp) == true)
		{
			selected_edge_index.push_back(it->first);
		}
	}

	// Return the edge index find
	return selected_edge_index;

}


bool elementline_list_store::isClickPointOnLine(const glm::vec2& clickPoint, const glm::vec2& lineStart, 
	const glm::vec2& lineEnd, float threshold)
{
	glm::vec2 lineDirection = lineEnd - lineStart;
	float lineLengthSq = glm::dot(lineDirection, lineDirection);

	glm::vec2 clickToLineStart = clickPoint - lineStart;
	float dotProduct = glm::dot(clickToLineStart, lineDirection);

	// Calculate the normalized projection of clickToLineStart onto the line
	glm::vec2 projection = (dotProduct / lineLengthSq) * lineDirection;

	// Calculate the squared normal distance between the click point and the line
	float normalDistanceSq = glm::dot(clickToLineStart - projection, clickToLineStart - projection);

	// Check if the click point is within the line segment's bounding box
	if (dotProduct >= 0.0f && dotProduct <= lineLengthSq)
	{
		// Check if the normal distance is less than or equal to the threshold
		if (normalDistanceSq <= threshold * threshold)
		{
			return true; // Click point is on the line segment
		}
	}

	return false; // Click point is not on the line segment
}

void elementline_list_store::update_material_id_labels()
{
	line_material_id_labels.clear_labels();

	// Update the material ids
	glm::vec3 temp_color;
	std::string temp_str;

	for (auto it = elementlineMap.begin(); it != elementlineMap.end(); ++it)
	{
		elementline_store elementline = it->second;

		// Add the line id
		glm::vec2 start_pt = elementline.startNode->node_pt;
		glm::vec2 end_pt = elementline.endNode->node_pt;

		// Calculate the midpoint of the line segment
		glm::vec2 line_mid_pt = glm::vec2((start_pt.x + end_pt.x) * 0.5f, (start_pt.y + end_pt.y) * 0.5f);

		float line_angle = atan2(end_pt.y - start_pt.y, end_pt.x - start_pt.x);

		// Add the material ID
		temp_color = geom_parameters::get_standard_color(elementline.material_id);
		temp_str = "";
		if (elementline.material_id < 1)
		{
			// Rigid element
			temp_str = "                       Rigid [" + std::to_string(elementline.material_id) + "]";
		}
		else
		{
			// Spring element
			temp_str = "                         Spring [" + std::to_string(elementline.material_id) + "]";
		}
		
		line_material_id_labels.add_text(temp_str, line_mid_pt, glm::vec2(0), temp_color, line_angle, true, false);
	}

	line_material_id_labels.set_buffer();

	// re-create the element lines
	recreate_element_lines();
}

void elementline_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	// Update model openGL uniforms
	element_lines.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
	selected_element_lines.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
	line_id_labels.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
	line_length_labels.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
	line_material_id_labels.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_zoomtranslation, set_transparency, set_deflscale);
}

int elementline_list_store::get_line_id(const int& startNode_id, const int& endNode_id)
{
	// Return the edge id
	for (const auto& line_m : elementlineMap)
	{
		const elementline_store& line = line_m.second;

		if ((line.startNode->node_id == startNode_id && line.endNode->node_id == endNode_id) ||
			(line.startNode->node_id == endNode_id && line.endNode->node_id == startNode_id))
		{
			// Line with the same start and end nodes already exists (do not add)
			return line.line_id;
		}
	}

	// Non found
	return -1;
}
