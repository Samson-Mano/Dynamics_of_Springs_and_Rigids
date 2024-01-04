#include "geom_store.h"

geom_store::geom_store()
{
	// Empty Constructor
}

geom_store::~geom_store()
{
	// Empty Destructor
}

void geom_store::init(modal_analysis_window* modal_solver_window,
	pulse_analysis_window* pulse_solver_window,
	options_window* op_window,
	node_load_window* nd_load_window,
	inlcondition_window* nd_inlcond_window,
	new_model_window* md_window)
{
	// Initialize
	// Initialize the geometry parameters
	geom_param.init();

	// Intialize the selection rectangle
	selection_rectangle.init(&geom_param);

	is_geometry_set = false;

	// Initialize the solvers
	modal_solver.clear_results(); 
	pulse_solver.clear_results();


	// Add the window pointers
	this->md_window = md_window;
	this->op_window = op_window; // Option window
	this->nd_load_window = nd_load_window; // Node Load window
	this->nd_inlcond_window = nd_inlcond_window; // Node initial condition window

	// Add the solver window pointers
	this->modal_solver_window = modal_solver_window; // Modal Analysis Solver window
	this->pulse_solver_window = pulse_solver_window; // Pulse Analysis Solver window
}

void geom_store::fini()
{
	// Deinitialize
	is_geometry_set = false;
}

void geom_store::load_model(const int& model_type, std::vector<std::string> input_data)
{

	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of raw data input started" << std::endl;

	int j = 0, i = 0;

	// Initialize the model items
	this->model_nodes.init(&geom_param);
	this->model_lineelements.init(&geom_param);

	// Node constraints
	this->node_constraints.init(&geom_param);
	this->node_loads.init(&geom_param);
	this->node_ptmass.init(&geom_param);
	this->node_inlcond.init(&geom_param);

	// Re-initialize the result elements
	this->modal_result_nodes.init(&geom_param);
	this->modal_result_lineelements.init(&geom_param);
	this->pulse_result_nodes.init(&geom_param);
	this->pulse_result_lineelements.init(&geom_param);

	// Re-initialized the analysis window
	this->modal_solver_window->init();
	this->pulse_solver_window->init();

	// Material data list
	material_data mat_data;

	// Re-Initialize the solver
	modal_solver.clear_results();
	pulse_solver.clear_results();

	//Node Point list
	std::vector<glm::vec2> node_pts_list;

	int node_count = 0;

	// Process the lines
	while (j < input_data.size())
	{
		std::string line = input_data[j];
		std::string type = line.substr(0, 4);  // Extract the first 4 characters of the line

		// Split the line into comma-separated fields
		std::istringstream iss(line);
		std::string field;
		std::vector<std::string> fields;
		while (std::getline(iss, field, ','))
		{
			fields.push_back(field);
		}

		if (fields[0] == "NodeCount")
		{
			// Node count
			node_count = std::stoi(fields[1]);
		}
		else if (fields[0] == "Tension")
		{
			// Tension
			mat_data.line_tension = std::stod(fields[1]);

		}
		else if (type == "Length")
		{
			// Length
			mat_data.line_length = std::stod(fields[1]);
		}
		else if (type == "Density")
		{
			// Density
			mat_data.material_density = std::stod(fields[1]);
		}

		// Iterate line
		j++;
	}


	// Model input
	int material_id = 0;
	if (model_type == 0)
	{
		// Line (Fixed - Fixed)
		int i = 0;
		double dl = mat_data.line_length / static_cast<double>(node_count);

		// Add first node
		glm::vec2 node_pt = glm::vec2(i*dl, 0);
		this->model_nodes.add_node(i, node_pt);

		// Add other nodes
		for (i = 1; i < node_count; i++)
		{
			// Node point
			node_pt = glm::vec2(i * dl, 0);
			this->model_nodes.add_node(i, node_pt);

			// Add element
			int line_id = i - 1;
			this->model_lineelements.add_elementline(line_id, &this->model_nodes.nodeMap[i - 1], &this->model_nodes.nodeMap[i], material_id);
		}

		// Add end constraint
		int constraint_type = 0;
		double constraint_angle = 90;
		i = 0;
		this->node_constraints.add_constraint(i, this->model_nodes.nodeMap[i].node_pt, constraint_type, constraint_angle);

		i = node_count - 1;
		this->node_constraints.add_constraint(i, this->model_nodes.nodeMap[i].node_pt, constraint_type, constraint_angle);
	}





	// Input read failed??
	if (model_nodes.node_count < 1 || model_lineelements.elementline_count < 1)
	{
		is_geometry_set = false;
		std::cerr << "Input error !!" << std::endl;
		return;
	}

	// Geometry is loaded
	is_geometry_set = true;

	// Set the boundary of the geometry
	std::pair<glm::vec2, glm::vec2> result = geom_parameters::findMinMaxXY(node_pts_list);
	this->geom_param.min_b = result.first;
	this->geom_param.max_b = result.second;
	this->geom_param.geom_bound = geom_param.max_b - geom_param.min_b;

	// Set the center of the geometry
	this->geom_param.center = geom_parameters::findGeometricCenter(node_pts_list);

	// Set the geometry
	update_model_matrix();
	update_model_zoomfit();

	// Set the geometry buffers
	this->model_nodes.set_buffer();
	this->model_lineelements.set_buffer();

	// Set the constraints buffer
	this->node_constraints.set_buffer();
	this->node_loads.set_buffer();
	this->node_ptmass.set_buffer();
	this->node_inlcond.set_buffer();

	// Set the result object buffers

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Model read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
}


void geom_store::update_WindowDimension(const int& window_width, const int& window_height)
{
	// Update the window dimension
	this->geom_param.window_width = window_width;
	this->geom_param.window_height = window_height;

	if (is_geometry_set == true)
	{
		// Update the model matrix
		update_model_matrix();
		// !! Zoom to fit operation during window resize is handled in mouse event class !!
	}
}


void geom_store::update_model_matrix()
{
	// Set the model matrix for the model shader
	// Find the scale of the model (with 0.9 being the maximum used)
	int max_dim = geom_param.window_width > geom_param.window_height ? geom_param.window_width : geom_param.window_height;

	double normalized_screen_width = 1.8f * (static_cast<double>(geom_param.window_width) / static_cast<double>(max_dim));
	double normalized_screen_height = 1.8f * (static_cast<double>(geom_param.window_height) / static_cast<double>(max_dim));


	geom_param.geom_scale = std::min(normalized_screen_width / geom_param.geom_bound.x,
		normalized_screen_height / geom_param.geom_bound.y);

	// Translation
	glm::vec3 geom_translation = glm::vec3(-1.0f * (geom_param.max_b.x + geom_param.min_b.x) * 0.5f * geom_param.geom_scale,
		-1.0f * (geom_param.max_b.y + geom_param.min_b.y) * 0.5f * geom_param.geom_scale,
		0.0f);

	glm::mat4 g_transl = glm::translate(glm::mat4(1.0f), geom_translation);

	geom_param.modelMatrix = g_transl * glm::scale(glm::mat4(1.0f), glm::vec3(static_cast<float>(geom_param.geom_scale)));

	// Update the model matrix
	model_nodes.update_geometry_matrices(true, false, false, true, false);
	model_lineelements.update_geometry_matrices(true, false, false, true, false);
	node_constraints.update_geometry_matrices(true, false, false, true, false);
	node_loads.update_geometry_matrices(true, false, false, true, false);
	node_ptmass.update_geometry_matrices(true, false, false, true, false);
	node_inlcond.update_geometry_matrices(true, false, false, true, false);

	// Update the modal analysis result matrix
	modal_result_nodes.update_geometry_matrices(true, false, false, false, false);
	modal_result_lineelements.update_geometry_matrices(true, false, false, false, false);
	pulse_result_nodes.update_geometry_matrices(true, false, false, false, false);
	pulse_result_lineelements.update_geometry_matrices(true, false, false, false, false);

}

void geom_store::update_model_zoomfit()
{
	if (is_geometry_set == false)
		return;

	// Set the pan translation matrix
	geom_param.panTranslation = glm::mat4(1.0f);

	// Set the zoom scale
	geom_param.zoom_scale = 1.0f;

	// Update the zoom scale and pan translation
	model_nodes.update_geometry_matrices(false, true, true, false, false);
	model_lineelements.update_geometry_matrices(false, true, true, false, false);
	node_constraints.update_geometry_matrices(false, true, true, false, false);
	node_loads.update_geometry_matrices(false, true, true, false, false);
	node_ptmass.update_geometry_matrices(false, true, true, false, false);
	node_inlcond.update_geometry_matrices(false, true, true, false, false);

	// Update the modal analysis result matrix
	modal_result_nodes.update_geometry_matrices(false, true, true, false, false);
	modal_result_lineelements.update_geometry_matrices(false, true, true, false, false);
	pulse_result_nodes.update_geometry_matrices(false, true, true, false, false);
	pulse_result_lineelements.update_geometry_matrices(false, true, true, false, false);

}

void geom_store::update_model_pan(glm::vec2& transl)
{
	if (is_geometry_set == false)
		return;

	// Pan the geometry
	geom_param.panTranslation = glm::mat4(1.0f);

	geom_param.panTranslation[0][3] = -1.0f * transl.x;
	geom_param.panTranslation[1][3] = transl.y;

	// Update the pan translation
	model_nodes.update_geometry_matrices(false, true, false, false, false);
	model_lineelements.update_geometry_matrices(false, true, false, false, false);
	node_constraints.update_geometry_matrices(false, true, false, false, false);
	node_loads.update_geometry_matrices(false, true, false, false, false);
	node_ptmass.update_geometry_matrices(false, true, false, false, false);
	node_inlcond.update_geometry_matrices(false, true, false, false, false);

	// Update the modal analysis result matrix
	modal_result_nodes.update_geometry_matrices(false, true, false, false, false);
	modal_result_lineelements.update_geometry_matrices(false, true, false, false, false);
	pulse_result_nodes.update_geometry_matrices(false, true, false, false, false);
	pulse_result_lineelements.update_geometry_matrices(false, true, false, false, false);

}

void geom_store::update_model_zoom(double& z_scale)
{
	if (is_geometry_set == false)
		return;

	// Zoom the geometry
	geom_param.zoom_scale = z_scale;

	// Update the Zoom
	model_nodes.update_geometry_matrices(false, false, true, false, false);
	model_lineelements.update_geometry_matrices(false, false, true, false, false);
	node_constraints.update_geometry_matrices(false, false, true, false, false);
	node_loads.update_geometry_matrices(false, false, true, false, false);
	node_ptmass.update_geometry_matrices(false, false, true, false, false);
	node_inlcond.update_geometry_matrices(false, false, true, false, false);

	// Update the modal analysis result matrix
	modal_result_nodes.update_geometry_matrices(false, false, true, false, false);
	modal_result_lineelements.update_geometry_matrices(false, false, true, false, false);
	pulse_result_nodes.update_geometry_matrices(false, false, true, false, false);
	pulse_result_lineelements.update_geometry_matrices(false, false, true, false, false);

}

void geom_store::update_model_transperency(bool is_transparent)
{
	if (is_geometry_set == false)
		return;

	if (is_transparent == true)
	{
		// Set the transparency value
		geom_param.geom_transparency = 0.2f;
	}
	else
	{
		// remove transparency
		geom_param.geom_transparency = 1.0f;
	}

	// Update the model transparency
	model_nodes.update_geometry_matrices(false, false, false, true, false);
	model_lineelements.update_geometry_matrices(false, false, false, true, false);
	node_constraints.update_geometry_matrices(false, false, false, true, false);
	node_loads.update_geometry_matrices(false, false, false, true, false);
	node_ptmass.update_geometry_matrices(false, false, false, true, false);
	node_inlcond.update_geometry_matrices(false, false, false, true, false);

	// Donot update result elements transparency

}

void geom_store::update_selection_rectangle(const glm::vec2& o_pt, const glm::vec2& c_pt,
	const bool& is_paint, const bool& is_select, const bool& is_rightbutton)
{
	// Draw the selection rectangle
	selection_rectangle.update_selection_rectangle(o_pt, c_pt, is_paint);

	// Selection commence (mouse button release)
	if (is_paint == false && is_select == true)
	{

	}
}


void geom_store::paint_geometry()
{
	if (is_geometry_set == false)
		return;

	// Clean the back buffer and assign the new color to it
	glClear(GL_COLOR_BUFFER_BIT);

	// Paint the model
	paint_model();

	// Paint the results
	paint_model_results();

}

void geom_store::paint_model()
{
	if (modal_solver_window->is_show_window == true ||
		pulse_solver_window->is_show_window == true)
	{
		if (modal_solver_window->is_show_window == true && 
			modal_solver.is_modal_analysis_complete == true &&
			modal_solver_window->show_undeformed_model == false)
		{
			// Modal Analysis complete, window open and user turned off model view
			return;
		}
		//________________________________________________________________________________________
		if (pulse_solver_window->is_show_window == true && pulse_solver.is_pulse_analysis_complete == true &&
			pulse_solver_window->show_undeformed_model == false)
		{
			// Pulse analysis complete, window open and user turned off model view
			return;
		}
	}

	//______________________________________________
	// Paint the model
	if (op_window->is_show_modelelements == true)
	{
		// Show the model elements
		model_lineelements.paint_elementlines();

		if (op_window->is_show_modelelementids == true)
		{
			// Show model element ids
			model_lineelements.paint_label_line_ids();

		}
		if (op_window->is_show_modelelementlengths == true)
		{
			// Show model element lengths
			model_lineelements.paint_label_line_lengths();
		}

	}

	if (op_window->is_show_modelnodes == true)
	{
		// Show the model nodes
		model_nodes.paint_model_nodes();

		if (op_window->is_show_modelnodeids == true)
		{
			// Show model node ids
			model_nodes.paint_label_node_ids();
		}
		if (op_window->is_show_modelnodecoords == true)
		{
			// Show model node co-ordinates
			model_nodes.paint_label_node_coords();
		}
	}

	if (op_window->is_show_ptmass == true)
	{
		// Show the node point mass
		node_ptmass.paint_pointmass();

		if (op_window->is_show_ptmass_labels == true)
		{
			// Show the node point mass label
			node_ptmass.paint_pointmass_label();
		}
	}

	if (op_window->is_show_constraint == true)
	{
		// Show the node constraints
		node_constraints.paint_constraints();
	}

	if (op_window->is_show_loads == true)
	{
		// Show the node Loads
		node_loads.paint_loads();

		if (op_window->is_show_loadvalues == true)
		{
			// Show the node load values
			node_loads.paint_load_labels();
		}
	}

	if (op_window->is_show_inlcondition == true)
	{
		// Show the node initial condition
		node_inlcond.paint_inlcondition_label();
	}

	if (md_window->is_show_window == true)
	{
		// New Model Window
		if (md_window->execute_create_model == true)
		{
			// Load a model


			md_window->execute_create_model = false;
		}

	}


}

void geom_store::paint_model_results()
{
	// Paint the results
	// Modal Analysis 
	paint_modal_analysis_results();

	// Pulse Analysis
	paint_pulse_analysis_results();

}


void geom_store::paint_modal_analysis_results()
{
	// Paint the modal analysis results
	// Closing sequence for the modal analysis window
	if (modal_solver_window->execute_modal_close == true)
	{
		// Execute the close sequence
		if (modal_solver.is_modal_analysis_complete == true)
		{
			// Pulse response analysis is complete
			update_model_transperency(false);
		}

		modal_solver_window->execute_modal_close = false;
	}

	// Check whether the modal analysis solver window is open or not
	if (modal_solver_window->is_show_window == false)
	{
		return;
	}

	// Paint the modal analysis results
	if (modal_solver.is_modal_analysis_complete == true)
	{
		// Change the buffer depending on the selected mode
		if (modal_solver_window->is_mode_selection_changed == true)
		{
			// Update the buffers
			// Modal Line buffer
			modal_result_lineelements.set_buffer(modal_solver_window->selected_modal_option);

			// Modal Node buffer
			modal_result_nodes.set_buffer(modal_solver_window->selected_modal_option);

			modal_solver_window->is_mode_selection_changed = false;
		}

		// Update the deflection scale
		geom_param.normalized_defl_scale = std::abs(modal_solver_window->normailzed_defomation_scale);
		geom_param.defl_scale = modal_solver_window->deformation_scale;

		// Update the deflection scale
		modal_result_lineelements.update_geometry_matrices(false, false, false, false, true);
		modal_result_nodes.update_geometry_matrices(false, false, false, false, true);
		// ______________________________________________________________________________________
		// Paint the modal lines
		modal_result_lineelements.paint_modal_elementlines();

		// Paint the modal nodes
		modal_result_nodes.paint_modal_nodes();

		// Paint result text
		if (modal_solver_window->show_result_text_values == true)
		{
			// Paint the modal result vector
			modal_result_nodes.paint_label_mode_vectors();
		}

	}

	// Open sequence for the modal analysis window
	if (modal_solver_window->execute_modal_open == true)
	{
		// Execute the open sequence
		if (modal_solver.is_modal_analysis_complete == true)
		{
			// update the modal window list box
			modal_solver_window->mode_result_str = modal_solver.mode_result_str;

			// Set the buffer
			modal_solver_window->is_mode_selection_changed = true;

			// Modal analysis is already complete so set the transparency for the model
			update_model_transperency(true);
		}
		modal_solver_window->execute_modal_open = false;
	}

	// Modal Analysis 
	if (modal_solver_window->execute_modal_analysis == true)
	{
		// Execute the Modal Analysis
		modal_solver.modal_analysis_start(model_nodes,
			model_lineelements,
			node_constraints,
			node_ptmass,
			mat_data);

		// reset the frequency response and pulse response solution
		pulse_solver.clear_results();

		// Check whether the modal analysis is complete or not
		if (modal_solver.is_modal_analysis_complete == true)
		{
			// update the modal window list box
			modal_solver_window->mode_result_str = modal_solver.mode_result_str;

			// Set the buffer
			modal_solver_window->is_mode_selection_changed = true;

			// Modal analysis is already complete so set the transparency for the model
			update_model_transperency(true);
		}


		modal_solver_window->execute_modal_analysis = false;
	}



}


void geom_store::paint_pulse_analysis_results()
{
	// Paint the pulse analysis results
	// Check closing sequence for Pulse response analysis window
	if (pulse_solver_window->execute_pulse_close == true)
	{
		// Execute the close sequence
		if (pulse_solver.is_pulse_analysis_complete == true)
		{
			// Pulse response analysis is complete
			update_model_transperency(false);
		}

		pulse_solver_window->execute_pulse_close = false;
	}

	// Check whether the modal analysis solver window is open or not
	if (pulse_solver_window->is_show_window == false)
	{
		return;
	}


	// Paint the pulse analysis result
	if (pulse_solver.is_pulse_analysis_complete == true)
	{
		// Update the deflection scale
		geom_param.normalized_defl_scale = 1.0f;
		geom_param.defl_scale = pulse_solver_window->deformation_scale_max;

		// Update the deflection scale
		pulse_result_lineelements.update_geometry_matrices(false, false, false, false, true);
		pulse_result_nodes.update_geometry_matrices(false, false, false, false, true);
		// ______________________________________________________________________________________

		// Paint the pulse lines
		pulse_result_lineelements.paint_pulse_elementlines(pulse_solver_window->time_step);

		// Paint the pulse nodes
		pulse_result_nodes.paint_pulse_nodes(pulse_solver_window->time_step);
	}


	if (pulse_solver_window->execute_pulse_open == true)
	{
		// Execute the open sequence
		if (modal_solver.is_modal_analysis_complete == false)
		{
			// Exit the window (when modal analysis is not complete)
			pulse_solver_window->is_show_window = false;
		}
		else
		{
			// Modal analysis Results
			pulse_solver_window->number_of_modes = static_cast<int>(modal_solver.m_eigenvalues.size());
			pulse_solver_window->modal_first_frequency = std::sqrt(modal_solver.m_eigenvalues.at(0)) / (2.0 * m_pi); // std::sqrt(modal_results.eigen_values[i]) / (2.0 * m_pi);
			pulse_solver_window->modal_end_frequency = std::sqrt(modal_solver.m_eigenvalues.at(pulse_solver_window->number_of_modes - 1)) / (2.0 * m_pi);
			pulse_solver_window->mode_result_str = modal_solver.mode_result_str;

			// Modal analysis is complete (check whether frequency response analysis is complete or not)
			if (pulse_solver.is_pulse_analysis_complete == true)
			{
				// Set the pulse response analysis result
				pulse_solver_window->time_interval_atrun = pulse_solver.time_interval;
				pulse_solver_window->time_step_count = pulse_solver.time_step_count;

				// Reset the buffers for pulse result nodes and lines
				pulse_result_lineelements.set_buffer();
				pulse_result_nodes.set_buffer();

				// Pulse response analysis is complete
				update_model_transperency(true);
			}

		}
		pulse_solver_window->execute_pulse_open = false;
	}

	if (pulse_solver_window->execute_pulse_analysis == true)
	{
		// Execute the Pulse response Analysis
		pulse_solver.pulse_analysis_start(model_nodes,
			model_lineelements,
			node_constraints,
			node_loads,
			node_ptmass,
			node_inlcond,
			mat_data,
			modal_solver,
			pulse_solver_window->total_simulation_time,
			pulse_solver_window->time_interval,
			pulse_solver_window->damping_ratio,
			pulse_solver_window->selected_modal_option1,
			pulse_solver_window->selected_modal_option2,
			pulse_solver_window->selected_pulse_option);

		// Check whether the modal analysis is complete or not
		if (pulse_solver.is_pulse_analysis_complete == true)
		{
			// Set the pulse response analysis result
			pulse_solver_window->time_interval_atrun = pulse_solver.time_interval;
			pulse_solver_window->time_step_count = pulse_solver.time_step_count;

			// Reset the buffers for pulse result nodes and lines
			pulse_result_lineelements.set_buffer();
			pulse_result_nodes.set_buffer();

			// Pulse response analysis is complete
			update_model_transperency(true);
		}
		pulse_solver_window->execute_pulse_analysis = false;
	}

}

