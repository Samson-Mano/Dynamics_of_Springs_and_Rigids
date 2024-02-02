#include "modal_nodes_list_store.h"

modal_nodes_list_store::modal_nodes_list_store()
{
	// Empty constructor
}

modal_nodes_list_store::~modal_nodes_list_store()
{
	// Empty destructor
}

void modal_nodes_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameters for the labels (and clear the labels)
	modal_node_points.init(geom_param_ptr);

	// Clear the results
	node_count = 0;
	modal_nodeMap.clear();
}

void modal_nodes_list_store::clear_data()
{
	modal_node_points.clear_points();

	// Clear the results
	node_count = 0;
	modal_nodeMap.clear();
}

void modal_nodes_list_store::add_result_node(int& node_id, glm::vec3& node_pt, std::unordered_map<int, glm::vec3> node_modal_displ)
{
	// Add result nodes
	modal_node_store temp_node;
	temp_node.node_id = node_id;
	temp_node.node_pt = node_pt;
	temp_node.node_modal_displ = node_modal_displ;

	// Check whether the node_id is already there
	if (modal_nodeMap.find(node_id) != modal_nodeMap.end())
	{
		// Node ID already exist (do not add)
		return;
	}

	// Insert to the nodes
	modal_nodeMap.insert({ node_id, temp_node });
	node_count++;
}

void modal_nodes_list_store::set_buffer()
{
	// Clear existing modal line
	modal_node_points.clear_points();

	// Add the dynamic points
	// Loop through every point
	int i = 0;
	for (auto& nd_m : modal_nodeMap)
	{
		modal_node_store nd = nd_m.second;

		std::vector<glm::vec3> point_displ; // point displ
		std::vector<glm::vec3> point_color; // point color

		for (int i = 0; i < static_cast<int>(nd.node_modal_displ.size()); i++)
		{
			glm::vec3 pt_displ = glm::vec3(nd.node_modal_displ[i].x,
				nd.node_modal_displ[i].y,
				nd.node_modal_displ[i].z);

			// Find the displacment value
			double pt_displ_value = glm::length(pt_displ);

			glm::vec3 pt_contour_color = geom_parameters::getContourColor_d(static_cast<float>(1.0 - pt_displ_value));

			// Add to the list
			point_displ.push_back(pt_displ);
			point_color.push_back(pt_contour_color);

		}

		// Add all the points
		modal_node_points.add_point(nd.node_id, nd.node_pt, point_displ, point_color);

	}

	// Set the buffer
	modal_node_points.set_buffer();
}


void modal_nodes_list_store::update_buffer(const int& selected_mode)
{
	modal_node_points.update_buffer(selected_mode);
}


void modal_nodes_list_store::paint_modal_nodes()
{
	modal_node_points.paint_points();
}


void modal_nodes_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation, 
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	modal_node_points.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation, set_zoomtranslation, set_transparency, set_deflscale);
}
