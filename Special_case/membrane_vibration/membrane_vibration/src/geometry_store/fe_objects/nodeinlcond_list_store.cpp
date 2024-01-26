#include "nodeinlcond_list_store.h"

nodeinlcond_list_store::nodeinlcond_list_store()
{
	// Empty constructor
}

nodeinlcond_list_store::~nodeinlcond_list_store()
{
	// Empty destructor
}

void nodeinlcond_list_store::init(geom_parameters* geom_param_ptr)
{
	// Set the geometry parameters
	this->geom_param_ptr = geom_param_ptr;

	// Set the geometry parameter for the points
	inlcond_points.init(geom_param_ptr);
	inl_condition_labels.init(geom_param_ptr);

}

void nodeinlcond_list_store::set_zero_condition(int inl_cond_type,const int& model_type)
{
	this->inl_cond_type = inl_cond_type; // Initial condition type 0 - Displacement, 1 - Velocity
	this->model_type = model_type; // Model type 0, 1 Line, 2,3 Circle

}

void nodeinlcond_list_store::add_inlcondition(int& node_id, glm::vec2& inlcond_loc, double& inl_amplitude_z)
{
	// Add the initial condition to the particular node
	nodeinl_condition_data temp_inl_condition_data;
	temp_inl_condition_data.node_id = node_id;
	temp_inl_condition_data.inlcond_loc = inlcond_loc;
	temp_inl_condition_data.inl_amplitude_z = inl_amplitude_z;

	// Insert the inital condition data to unordered map
	// Searching for node_id
	if (inlcondMap.find(node_id) != inlcondMap.end())
	{
		// Node is already have constraint
		// so remove the constraint
		inlcondMap[node_id] = temp_inl_condition_data;

		return;
	}

	// Insert the constraint to nodes
	inlcondMap.insert({ node_id, temp_inl_condition_data });
	inlcond_count++;

}


void nodeinlcond_list_store::delete_inlcondition(int& node_id)
{
	// Delete the initial condition in this node
	if (inlcond_count != 0)
	{
		// Remove the intial condition data to unordered map
		// Searching for node_id
		// Check there is already a initial conditon in the found node
		if (inlcondMap.find(node_id) != inlcondMap.end())
		{
			// Node is already have initial condition
			// so remove the intial condition
			inlcondMap.erase(node_id);

			// Update the buffer
			set_buffer();

			// adjust the initial condition count
			inlcond_count--;
		}
	}
}

void nodeinlcond_list_store::set_buffer()
{
	inlcond_points.set_buffer();
	inl_condition_labels.set_buffer();
}

void nodeinlcond_list_store::paint_inlcond()
{
	// Paint the initial displacement points
	inlcond_points.paint_points();
}

void nodeinlcond_list_store::paint_inlcond_label()
{
	// Paint the peak displacement label
	inl_condition_labels.paint_text();
}

void nodeinlcond_list_store::update_geometry_matrices(bool set_modelmatrix, bool set_pantranslation, bool set_rotatetranslation,
	bool set_zoomtranslation, bool set_transparency, bool set_deflscale)
{
	// Update model openGL uniforms
	inlcond_points.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation, set_zoomtranslation, set_transparency, set_deflscale);
	inl_condition_labels.update_opengl_uniforms(set_modelmatrix, set_pantranslation, set_rotatetranslation, set_zoomtranslation, set_transparency, set_deflscale);
}
