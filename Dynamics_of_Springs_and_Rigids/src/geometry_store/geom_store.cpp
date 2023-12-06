#include "geom_store.h"

geom_store::geom_store()
{
	// Empty Constructor
}

geom_store::~geom_store()
{
	// Empty Destructor
}

void geom_store::init(analysis_window* sol_window, options_window* op_window,
	node_constraint_window* nd_cnst_window, node_load_window* nd_load_window,  
	pointmass_window* nd_ptmass_window, inlcondition_window* nd_inlcond_window,
	element_prop_window* elm_prop_window)
{
	// Initialize
	// Initialize the geometry parameters
	geom_param.init();

	// Intialize the selection rectangle
	selection_rectangle.init(&geom_param);

	is_geometry_set = false;
	is_heat_analysis_complete = false;

	// Add the window pointers
	this->sol_window = sol_window; // Solver window
	this->op_window = op_window; // Option window
	this->nd_cnst_window = nd_cnst_window; // Node constraint window
	this->nd_load_window = nd_load_window; // Node Load window
	this->nd_ptmass_window = nd_ptmass_window; // Node Point mass window
	this->nd_inlcond_window = nd_inlcond_window; // Node initial condition window
	this->elm_prop_window = elm_prop_window; // Element property window
}

void geom_store::fini()
{
	// Deinitialize
	is_geometry_set = false;
}

void geom_store::read_varai2d(std::ifstream& input_file)
{
	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of Varai2D input started" << std::endl;

	// Read the varai2D
	// Read the entire file into a string
	std::string file_contents((std::istreambuf_iterator<char>(input_file)),
		std::istreambuf_iterator<char>());

	// Split the string into lines
	std::istringstream iss(file_contents);
	std::string line;
	std::vector<std::string> lines;
	while (std::getline(iss, line))
	{
		lines.push_back(line);
	}

	int j = 0, i = 0;

	// Re initialize the geometry variable
	this->model_nodes.init(&geom_param);
	this->model_lineelements.init(&geom_param);

	std::vector<glm::vec2> all_nodepts;

	// Process the lines
	while (j < lines.size())
	{
		std::cout << "Line: " << lines[j] << std::endl;
		// Check for the start of nodes input
		if (lines[j].find("[+] End Points") != std::string::npos)
		{
			int num_nodes;
			// Read the number of nodes
			std::stringstream ss(lines[j]);
			std::string token;
			std::getline(ss, token, ','); // Get the string "[+] End Points"
			std::getline(ss, token, ','); // Get the number of nodes as a string
			num_nodes = std::stoi(token) + j; // Convert the string to an integer

			// Read and store the nodes
			for (i = j; i < num_nodes; i++)
			{
				int node_id;
				double x, y;

				std::stringstream ss(lines[i + 1]);
				std::string token;

				std::getline(ss, token, ','); // read the node ID
				node_id = std::stoi(token);

				std::getline(ss, token, ','); // read the x-coordinate
				x = std::stod(token);

				std::getline(ss, token, ','); // read the y-coordinate
				y = std::stod(token);

				// Add to node store list
				glm::vec2 node_pt = glm::vec2(x, y);
				this->model_nodes.add_node(node_id, node_pt);
				all_nodepts.push_back(node_pt);
				j++;
			}

			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Nodes read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		}
		// Check for the start of lines input
		else if (lines[j].find("[+] Lines") != std::string::npos) {
			int num_lines;
			// Read the number of nodes
			std::stringstream ss(lines[j]);
			std::string token;
			std::getline(ss, token, ','); // Get the string "[+] Lines"
			std::getline(ss, token, ','); // Get the number of nodes as a string
			num_lines = std::stoi(token) + j; // Convert the string to an integer

			// Read and store the lines
			for (i = j; i < num_lines; i++)
			{
				int line_id, start_node_id, end_node_id;
				std::stringstream ss(lines[i + 1]);
				std::string token;

				std::getline(ss, token, ','); // read the line ID
				line_id = std::stoi(token);

				std::getline(ss, token, ','); // read the start node ID
				start_node_id = std::stoi(token);

				std::getline(ss, token, ','); // read the end node ID
				end_node_id = std::stoi(token);

				// Create lines_store object using references to startNode and endNode
				int mat_id = 1;
				this->model_lineelements.add_elementline(line_id, &model_nodes.nodeMap[start_node_id],
					&model_nodes.nodeMap[end_node_id],mat_id);

				j++;
			}

			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Lines read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		}

		// iterate line
		j++;
	}

	if (model_nodes.node_count < 1 || model_lineelements.elementline_count < 1)
	{
		// No elements added
		return;
	}

	// add a default Rigid to the material list
	material_data inpt_material;
	inpt_material.material_id = 0; // Get the material id
	inpt_material.material_name = "Default Rigid"; //Default material name
	inpt_material.material_stiffness = INFINITY; // N/mm

	// Add to materail list
	elm_prop_window->material_list.clear();
	elm_prop_window->material_list[inpt_material.material_id] = inpt_material;

	// add a default Spring to the material list
	inpt_material.material_id = 1; // Get the material id
	inpt_material.material_name = "Default Spring"; //Default material name
	inpt_material.material_stiffness = 100; // N/mm

	// Add to the material list
	elm_prop_window->material_list[inpt_material.material_id] = inpt_material;


	// Re-initalized the ptmass, constraints, initial conditions & loads
	this->node_constraints.init(&geom_param);
	this->node_ptmass.init(&geom_param);
	this->node_loads.init(&geom_param);
	this->node_inlcond.init(&geom_param);

	// Set the buffer
	// Geometry is loaded
	is_geometry_set = true;

	// Set the boundary of the geometry
	std::pair<glm::vec2, glm::vec2> result = geom_parameters::findMinMaxXY(all_nodepts);
	this->geom_param.min_b = result.first;
	this->geom_param.max_b = result.second;
	this->geom_param.geom_bound = geom_param.max_b - geom_param.min_b;

	// Set the center of the geometry
	this->geom_param.center = geom_parameters::findGeometricCenter(all_nodepts);

	// Set the geometry
	update_model_matrix();
	update_model_zoomfit();

	// Set the geometry buffers
	this->model_nodes.set_buffer();
	this->model_lineelements.set_buffer();
	this->node_constraints.set_buffer();
	this->node_loads.set_buffer();
	this->node_ptmass.set_buffer();
	this->node_inlcond.set_buffer();

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Model read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
}


void geom_store::read_dxfdata(std::ostringstream& input_data)
{


}


void geom_store::read_rawdata(std::ifstream& input_file)
{
	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of input started" << std::endl;

	// Read the Raw Data
	// Read the entire file into a string
	std::string file_contents((std::istreambuf_iterator<char>(input_file)),
		std::istreambuf_iterator<char>());

	// Split the string into lines
	std::istringstream iss(file_contents);
	std::string line;
	std::vector<std::string> lines;
	while (std::getline(iss, line))
	{
		lines.push_back(line);
	}

	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Lines loaded at " << stopwatch_elapsed_str.str() << " secs" << std::endl;

	int j = 0, i = 0;

	// Reinitialize the model geometry
	is_geometry_set = false;
	is_heat_analysis_complete = false;

	// Initialize the model items
	this->model_nodes.init(&geom_param);
	this->model_lineelements.init(&geom_param);

	// Node constraints
	this->node_constraints.init(&geom_param);
	this->node_loads.init(&geom_param);
	this->node_ptmass.init(&geom_param);
	this->node_inlcond.init(&geom_param);

	// Initialize the result store

	//Node Point list
	std::vector<glm::vec2> node_pts_list;

	// Process the lines
	while (j < lines.size())
	{
		std::istringstream iss(lines[j]);

		std::string inpt_type;
		char comma;
		iss >> inpt_type;

		if (inpt_type == "*NODE")
		{
			// Nodes
			while (j < lines.size())
			{
				std::istringstream nodeIss(lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(nodeIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 3)
				{
					break;
				}

				int node_id = std::stoi(splitValues[0]); // node ID
				double x = std::stod(splitValues[1]); // Node coordinate x
				double y = std::stod(splitValues[2]); // Node coordinate y

				glm::vec2 node_pt = glm::vec2(x, y);
				node_pts_list.push_back(node_pt);

				// Add the nodes
				this->model_nodes.add_node(node_id, node_pt);
				j++;
			}

			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Nodes read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		}

		if (inpt_type == "*ELEMENT,TYPE=S3")
		{
			// Triangle Element
			while (j < lines.size())
			{
				std::istringstream elementIss(lines[j + 1]);

				// Vector to store the split values
				std::vector<std::string> splitValues;

				// Split the string by comma
				std::string token;
				while (std::getline(elementIss, token, ','))
				{
					splitValues.push_back(token);
				}

				if (static_cast<int>(splitValues.size()) != 4)
				{
					break;
				}

				int tri_id = std::stoi(splitValues[0]); // triangle ID
				int nd1 = std::stoi(splitValues[1]); // Node id 1
				int nd2 = std::stoi(splitValues[2]); // Node id 2
				int nd3 = std::stoi(splitValues[3]); // Node id 3

				// Add the Triangle Elements
				/*this->model_trielements.add_elementtriangle(tri_id, &model_nodes.nodeMap[nd1], &model_nodes.nodeMap[nd2],
					&model_nodes.nodeMap[nd3]);*/
				j++;
			}


			stopwatch_elapsed_str.str("");
			stopwatch_elapsed_str << stopwatch.elapsed();
			std::cout << "Elements read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
		}

		// Iterate the line
		j++;
	}

	// Input read failed??
	if (model_nodes.node_count == 0)
	{
		std::cerr << "Input error !!" << std::endl;
		return;
	}

	// Create the default material
	material_data inpt_material;
	inpt_material.material_id = 0; // Get the material id
	inpt_material.material_name = "Default material"; //Default material name
	inpt_material.material_stiffness = 100; // N/m

	// Add to materail list
	elm_prop_window->material_list.clear();
	elm_prop_window->material_list[inpt_material.material_id] = inpt_material;

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

	// Set the result object buffers

	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Model read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;
}

void geom_store::write_rawdata(std::ofstream& output_file)
{

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
	model_nodes.update_geometry_matrices(true, false, false, false, false);
	model_lineelements.update_geometry_matrices(true, false, false, false, false);
	node_constraints.update_geometry_matrices(true, false, false, false, false);
	node_loads.update_geometry_matrices(true, false, false, false, false);
	node_ptmass.update_geometry_matrices(true, false, false, false, false);
	node_inlcond.update_geometry_matrices(true, false, false, false, false);

	// Update the modal analysis result matrix

	
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
}

void geom_store::update_selection_rectangle(const glm::vec2& o_pt, const glm::vec2& c_pt,
	const bool& is_paint, const bool& is_select, const bool& is_rightbutton)
{
	// Draw the selection rectangle
	selection_rectangle.update_selection_rectangle(o_pt, c_pt, is_paint);

	// Selection commence (mouse button release)
	if (is_paint == false && is_select == true)
	{
		// Node Constraint Window
		if (nd_cnst_window->is_show_window == true)
		{
			// Selected Node index
			std::vector<int> selected_node_ids = model_nodes.is_node_selected(o_pt, c_pt);
			nd_cnst_window->add_to_node_list(selected_node_ids, is_rightbutton);
		}

		// Node Load Window
		if (nd_load_window->is_show_window == true)
		{
			// Selected Node Index
			std::vector<int> selected_node_ids = model_nodes.is_node_selected(o_pt, c_pt);
			nd_load_window->add_to_node_list(selected_node_ids, is_rightbutton);
		}

		// Node Point Mass Window
		if (nd_ptmass_window->is_show_window == true)
		{
			// Selected Node Index
			std::vector<int> selected_node_ids = model_nodes.is_node_selected(o_pt, c_pt);
			nd_ptmass_window->add_to_node_list(selected_node_ids, is_rightbutton);
		}

		// Node initial condition window
		if (nd_inlcond_window->is_show_window == true)
		{
			// Selected Node Index
			std::vector<int> selected_node_ids = model_nodes.is_node_selected(o_pt, c_pt);
			nd_inlcond_window->add_to_node_list(selected_node_ids, is_rightbutton);
		}


		// Element Properties Window
		if (elm_prop_window->is_show_window == true)
		{
			// Selected Element Index
			/*std::vector<int> selected_elm_ids = model_trielements.is_tri_selected(o_pt, c_pt);
			elm_prop_window->add_to_element_list(selected_elm_ids, is_rightbutton);*/

		}
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
	if (sol_window->is_show_window == true && is_heat_analysis_complete == true && sol_window->show_model == false)
	{
		// Analysis complete and user turned off model view
		return;
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


	if (nd_cnst_window->is_show_window == true)
	{
		// Node Constraint Window 
		paint_node_constraint_operation();
	}

	if (nd_load_window->is_show_window == true)
	{
		// Node Load Window 
		paint_node_load_operation();
	}

	if (nd_ptmass_window->is_show_window == true)
	{
		// Node point mass window
		paint_node_ptmass_operation();
	}

	if (nd_inlcond_window->is_show_window == true)
	{
		// Node initial condition window
		paint_node_inlcond_operation();
	}


	if (elm_prop_window->is_show_window == true)
	{
		// Element Properties Window 
			// Selection rectangle
		selection_rectangle.paint_selection_rectangle();

				// Paint the selected elements
		if (elm_prop_window->is_selected_count == true)
		{
			//model_trielements.paint_selected_elementtriangles();
		}

		// Check whether the selection changed
		if (elm_prop_window->is_selection_changed == true)
		{
			//model_trielements.add_selection_triangles(elm_prop_window->selected_elements);
			elm_prop_window->is_selection_changed = false;
		}

		// Material deleted
		if (elm_prop_window->execute_delete_materialid != -1)
		{
			// Delete material
			// Execute delete material id
			//model_trielements.execute_delete_material(elm_prop_window->execute_delete_materialid);
			elm_prop_window->execute_delete_materialid = -1;

			// Remove the selection
			elm_prop_window->selected_elements.clear();
			elm_prop_window->is_selection_changed = true;
		}

		// Apply the Element properties
		if (elm_prop_window->apply_element_properties == true)
		{
			// Apply material properties to the selected triangle elements
			int material_id = elm_prop_window->material_list[elm_prop_window->selected_material_option].material_id; // get the material id
			//model_trielements.update_material(elm_prop_window->selected_elements, material_id);
			elm_prop_window->apply_element_properties = false;

			// Remove the selection
			elm_prop_window->selected_elements.clear();
			elm_prop_window->is_selection_changed = true;
		}

		// Paint the material ID
		//model_trielements.paint_tri_material_id();
	}

}

void geom_store::paint_model_results()
{
	// Paint the results
	
}


void geom_store::paint_node_constraint_operation()
{
	// Selection rectangle
	selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_cnst_window->is_selected_count == true)
	{
		glPointSize(6.0f);
		model_nodes.paint_selected_model_nodes();
		glPointSize(3.0f);
	}

	// Check whether the selection changed
	if (nd_cnst_window->is_selection_changed == true)
	{
		model_nodes.add_selection_nodes(nd_cnst_window->selected_nodes);
		nd_cnst_window->is_selection_changed = false;
	}

	// Apply the Node constraint
	if (nd_cnst_window->apply_nodal_constraint == true)
	{
		int constraint_type = nd_cnst_window->selected_constraint_option; // selected constraint type
		double constraint_angle = nd_cnst_window->constraint_angle; // constraint angle

		for (int& id : nd_cnst_window->selected_nodes)
		{
			// Add the constraints
			node_constraints.add_constraint(id, model_nodes.nodeMap[id].node_pt, constraint_type, constraint_angle);
		}

		node_constraints.set_buffer();

		// Constraint application ends
		nd_cnst_window->apply_nodal_constraint = false;

		// Remove the selection
		nd_cnst_window->selected_nodes.clear();
		nd_cnst_window->is_selection_changed = true;
	}

	// Delete all the Node constraint
	if (nd_cnst_window->delete_nodal_constraint == true)
	{
		for (int& id : nd_cnst_window->selected_nodes)
		{
			// Delete the constraints
			node_constraints.delete_constraint(id);
		}

		node_constraints.set_buffer();

		// Constraint delete ends
		nd_cnst_window->delete_nodal_constraint = false;

		// Remove the selection
		nd_cnst_window->selected_nodes.clear();
		nd_cnst_window->is_selection_changed = true;
	}
}


void  geom_store::paint_node_load_operation()
{
	// Selection rectangle
	selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_load_window->is_selected_count == true)
	{
		glPointSize(6.0f);
		model_nodes.paint_selected_model_nodes();
		glPointSize(3.0f);
	}

	// Check whether the selection changed
	if (nd_load_window->is_selection_changed == true)
	{
		model_nodes.add_selection_nodes(nd_load_window->selected_nodes);
		nd_load_window->is_selection_changed = false;
	}

	// Apply the Node load
	if (nd_load_window->apply_nodal_load == true)
	{
		double load_amplitude = nd_load_window->load_amplitude; // load amplitude
		double load_start_time = nd_load_window->load_start_time; // load start time
		double load_end_time = nd_load_window->load_end_time; // load end time
		double load_angle = nd_load_window->load_angle; // load angle

		for (int& id : nd_load_window->selected_nodes)
		{
			// Add the loads
			node_loads.add_load(id, model_nodes.nodeMap[id].node_pt, 
				load_start_time, load_end_time, load_amplitude, load_angle);
		}

		node_loads.set_buffer();

		// Load application ends
		nd_load_window->apply_nodal_load = false;

		// Remove the selection
		nd_load_window->selected_nodes.clear();
		nd_load_window->is_selection_changed = true;
	}

	// Delete all the Node load
	if (nd_load_window->delete_nodal_load == true)
	{
		for (int& id : nd_load_window->selected_nodes)
		{
			// Delete the loads
			node_loads.delete_load(id);
		}

		node_loads.set_buffer();

		// Load delete ends
		nd_load_window->delete_nodal_load = false;

		// Remove the selection
		nd_load_window->selected_nodes.clear();
		nd_load_window->is_selection_changed = true;
	}
}


void geom_store::paint_node_ptmass_operation()
{
	// Paint the node point mass pre processing
		// Selection rectangle
	selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_ptmass_window->is_selected_count == true)
	{
		glPointSize(6.0f);
		model_nodes.paint_selected_model_nodes();
		glPointSize(3.0f);
	}

	// Check whether the selection changed
	if (nd_ptmass_window->is_selection_changed == true)
	{
		model_nodes.add_selection_nodes(nd_ptmass_window->selected_nodes);
		nd_ptmass_window->is_selection_changed = false;
	}

	// Apply the Node Point Mass
	if (nd_ptmass_window->apply_nodal_ptmass == true)
	{
		double pt_mass_x = nd_ptmass_window->mass_x; // Point mass x values
		double pt_mass_y = nd_ptmass_window->mass_y; // Point mass y values
		
		for (int& id : nd_ptmass_window->selected_nodes)
		{
			// Add the point mass
			node_ptmass.add_pointmass(id, model_nodes.nodeMap[id].node_pt,glm::vec2(0),
				pt_mass_x,pt_mass_y,false);
		}

		node_ptmass.set_buffer();

		// Point Mass application ends
		nd_ptmass_window->apply_nodal_ptmass = false;

		// Remove the selection
		nd_ptmass_window->selected_nodes.clear();
		nd_ptmass_window->is_selection_changed = true;
	}

	// Delete all the Node point mass
	if (nd_ptmass_window->delete_nodal_ptmass == true)
	{
		for (int& id : nd_ptmass_window->selected_nodes)
		{
			// Delete the point mass
			node_ptmass.delete_pointmass(id);
		}

		node_ptmass.set_buffer();

		// Point Mass delete ends
		nd_ptmass_window->delete_nodal_ptmass = false;

		// Remove the selection
		nd_ptmass_window->selected_nodes.clear();
		nd_ptmass_window->is_selection_changed = true;
	}
}


void geom_store::paint_node_inlcond_operation()
{
	// Paint the node initial condition pre processing
		// Selection rectangle
	selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_inlcond_window->is_selected_count == true)
	{
		glPointSize(6.0f);
		model_nodes.paint_selected_model_nodes();
		glPointSize(3.0f);
	}

	// Check whether the selection changed
	if (nd_inlcond_window->is_selection_changed == true)
	{
		model_nodes.add_selection_nodes(nd_inlcond_window->selected_nodes);
		nd_inlcond_window->is_selection_changed = false;
	}

	// Apply the Node Initial Condition
	if (nd_inlcond_window->apply_nodal_inlcond == true)
	{
		// Intial displacement
		double initial_displacement_x = nd_inlcond_window->initial_displacement_x; // initial displacement x
		double initial_displacement_y = nd_inlcond_window->initial_displacement_y; // initial displacement y
		// Initial velocity
		double initial_velocity_x = nd_inlcond_window->initial_velocity_x; // initial velocity x
		double initial_velocity_y = nd_inlcond_window->initial_velocity_y; // initial velocity y

		for (int& id : nd_inlcond_window->selected_nodes)
		{
			// Add the initial condition
			node_inlcond.add_inlcondition(id, model_nodes.nodeMap[id].node_pt,
				initial_displacement_x, initial_displacement_y, initial_velocity_x, initial_velocity_y);
		}

		node_inlcond.set_buffer();

		// initial condition application ends
		nd_inlcond_window->apply_nodal_inlcond = false;

		// Remove the selection
		nd_inlcond_window->selected_nodes.clear();
		nd_inlcond_window->is_selection_changed = true;
	}

	// Delete all the Node initial condition
	if (nd_inlcond_window->delete_nodal_inlcond == true)
	{
		for (int& id : nd_inlcond_window->selected_nodes)
		{
			// Delete the initial condition
			node_inlcond.delete_inlcondition(id);
		}

		node_inlcond.set_buffer();

		// Initial condition delete ends
		nd_inlcond_window->delete_nodal_inlcond = false;

		// Remove the selection
		nd_inlcond_window->selected_nodes.clear();
		nd_inlcond_window->is_selection_changed = true;
	}
}

