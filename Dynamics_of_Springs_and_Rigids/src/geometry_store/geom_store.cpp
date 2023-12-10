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
	forced_analysis_window* forced_solver_window, options_window* op_window,
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
	is_modal_analysis_complete = false;
	is_pulse_analysis_complete = false;
	is_forced_analysis_complete = false;


	// Add the window pointers
	this->op_window = op_window; // Option window
	this->nd_cnst_window = nd_cnst_window; // Node constraint window
	this->nd_load_window = nd_load_window; // Node Load window
	this->nd_ptmass_window = nd_ptmass_window; // Node Point mass window
	this->nd_inlcond_window = nd_inlcond_window; // Node initial condition window
	this->elm_prop_window = elm_prop_window; // Element property window

	// Add the solver window pointers
	this->modal_solver_window = modal_solver_window; // Modal Analysis Solver window
	this->pulse_solver_window = pulse_solver_window; // Pulse Analysis Solver window
	this->forced_solver_window = forced_solver_window; // Forced Response Analysis Solver window
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
		else if (lines[j].find("[+] Lines") != std::string::npos)
		{
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
					&model_nodes.nodeMap[end_node_id], mat_id);

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
		is_geometry_set = false;
		std::cerr << "Input error !!" << std::endl;
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

	// Initialize the result store
	is_modal_analysis_complete = false;
	is_pulse_analysis_complete = false;
	is_forced_analysis_complete = false;


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
	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of DXF Data input started" << std::endl;

	// Read the data from string
	std::string inputStr = input_data.str();
	std::stringstream ss(inputStr);

	std::string temp;
	std::vector<std::string> lines;
	while (std::getline(ss, temp))
	{
		lines.push_back(temp);
	}

	int j = 0, i = 0;

	// Re initialize the geometry variable
	this->model_nodes.init(&geom_param);
	this->model_lineelements.init(&geom_param);

	std::vector<glm::vec2> all_nodepts;

	// Process the lines
	while (j < lines.size())
	{
		std::string line = lines[j];
		std::string type = line.substr(0, 4);  // Extract the first 4 characters of the line

		// Split the line into comma-separated fields
		std::istringstream iss(line);
		std::string field;
		std::vector<std::string> fields;
		while (std::getline(iss, field, ','))
		{
			fields.push_back(field);
		}

		if (type == "node")
		{
			// Read the nodes
			int node_id = std::stoi(fields[1]); // node ID
			double x = std::stod(fields[2]); // Node coordinate x
			double y = std::stod(fields[3]); // Node coordinate y

			// Add to node Map
			glm::vec2 node_pt = glm::vec2(x, y);
			this->model_nodes.add_node(node_id, node_pt);

			all_nodepts.push_back(node_pt);
		}
		else if (type == "line")
		{
			int line_id = std::stoi(fields[1]); // line ID
			int start_node_id = std::stoi(fields[2]); // line id start node
			int end_node_id = std::stoi(fields[3]); // line id end node
			int material_id = std::stoi(fields[4]); // materail ID of the line

			// Add to line Map (Note that Nodes needed to be added before the start of line addition !!!!)
			int mat_id = 1;
			this->model_lineelements.add_elementline(line_id, &model_nodes.nodeMap[start_node_id], &model_nodes.nodeMap[end_node_id], mat_id);
		}

		// Iterate line
		j++;
	}


	stopwatch_elapsed_str.str("");
	stopwatch_elapsed_str << stopwatch.elapsed();
	std::cout << "Nodes & Lines read completed at " << stopwatch_elapsed_str.str() << " secs" << std::endl;


	if (model_nodes.node_count < 1 || model_lineelements.elementline_count < 1)
	{
		// No elements added
		is_geometry_set = false;
		std::cerr << "Input error !!" << std::endl;
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

	// Initialize the result store
	is_modal_analysis_complete = false;
	is_pulse_analysis_complete = false;
	is_forced_analysis_complete = false;



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


void geom_store::read_rawdata(std::ifstream& input_file)
{

	// Create stopwatch
	Stopwatch_events stopwatch;
	stopwatch.start();
	std::stringstream stopwatch_elapsed_str;
	stopwatch_elapsed_str << std::fixed << std::setprecision(6);

	std::cout << "Reading of raw data input started" << std::endl;

	// Initialize the model items
	this->model_nodes.init(&geom_param);
	this->model_lineelements.init(&geom_param);

	// Node constraints
	this->node_constraints.init(&geom_param);
	this->node_loads.init(&geom_param);
	this->node_ptmass.init(&geom_param);
	this->node_inlcond.init(&geom_param);

	// Material data list
	std::unordered_map<int, material_data> mat_data;

	// Initialize the result store
	is_modal_analysis_complete = false;
	is_pulse_analysis_complete = false;
	is_forced_analysis_complete = false;



	//Node Point list
	std::vector<glm::vec2> node_pts_list;

	// Read all the nodes
	while (!input_file.eof())
	{
		char type[5];
		input_file.read(type, sizeof(type));

		if (std::strcmp(type, "node") == 0)
		{
			int nd_id; // node id
			glm::vec2 nd_pt; // node pt

			input_file.read(reinterpret_cast<char*>(&nd_id), sizeof(nd_id));
			input_file.read(reinterpret_cast<char*>(&nd_pt), sizeof(nd_pt));

			node_pts_list.push_back(nd_pt); // node points

			// Add to the nodes
			this->model_nodes.add_node(nd_id, nd_pt);
		}
		else if (std::strcmp(type, "line") == 0)
		{
			int line_id, start_node_id, end_node_id, material_id; // line id, start node id, end node id, material id

			input_file.read(reinterpret_cast<char*>(&line_id), sizeof(line_id));
			input_file.read(reinterpret_cast<char*>(&start_node_id), sizeof(start_node_id));
			input_file.read(reinterpret_cast<char*>(&end_node_id), sizeof(end_node_id));
			input_file.read(reinterpret_cast<char*>(&material_id), sizeof(material_id));

			// Add to the lines
			this->model_lineelements.add_elementline(line_id, &this->model_nodes.nodeMap[start_node_id],
				&this->model_nodes.nodeMap[end_node_id], material_id);
		}
		else if (std::strcmp(type, "cnst") == 0)
		{
			int node_id, constraint_type; // constraint node id, constraint type
			glm::vec2 constraint_loc; // constraint location
			double constraint_angle; // constraint angle

			input_file.read(reinterpret_cast<char*>(&node_id), sizeof(node_id)); // node id
			input_file.read(reinterpret_cast<char*>(&constraint_loc), sizeof(constraint_loc)); // constraint location
			input_file.read(reinterpret_cast<char*>(&constraint_type), sizeof(constraint_type)); // constraint type
			input_file.read(reinterpret_cast<char*>(&constraint_angle), sizeof(constraint_angle)); // constraint angle

			// Add to the constraints
			this->node_constraints.add_constraint(node_id, constraint_loc, constraint_type, constraint_angle);
		}
		else if (std::strcmp(type, "load") == 0)
		{
			int load_id, node_id; // Load id, id of the node its applied to
			glm::vec2 load_loc = glm::vec2(0); // Load location
			// Load start time, Load end time, Load value, Load angle, Load phase angle
			double load_start_time, load_end_time, load_value, load_angle, load_phase; 

			input_file.read(reinterpret_cast<char*>(&load_id), sizeof(load_id)); // Load id
			input_file.read(reinterpret_cast<char*>(&node_id), sizeof(node_id)); // Node id
			input_file.read(reinterpret_cast<char*>(&load_loc), sizeof(load_loc)); // Load location
			input_file.read(reinterpret_cast<char*>(&load_start_time), sizeof(load_start_time)); // Load start time
			input_file.read(reinterpret_cast<char*>(&load_end_time), sizeof(load_end_time)); // Load end time
			input_file.read(reinterpret_cast<char*>(&load_value), sizeof(load_value)); // Load value
			input_file.read(reinterpret_cast<char*>(&load_angle), sizeof(load_angle)); // Load angle
			input_file.read(reinterpret_cast<char*>(&load_phase), sizeof(load_phase)); // Load phase angle

			// Add to the loads
			this->node_loads.add_load(load_id, load_loc, load_start_time, load_end_time, 
				load_value, load_angle, load_phase);
		}
		else if (std::strcmp(type, "mass") == 0)
		{
			int node_id; // node id
			glm::vec2 ptmass_loc, ptmass_defl; // pt mass location
			double ptmass_x, ptmass_y; // value of ptmass x and ptmass y
			bool is_offset; // offset

			input_file.read(reinterpret_cast<char*>(&node_id), sizeof(node_id)); // Node id
			input_file.read(reinterpret_cast<char*>(&ptmass_loc), sizeof(ptmass_loc)); // ptmass location
			input_file.read(reinterpret_cast<char*>(&ptmass_defl), sizeof(ptmass_defl)); // ptmass deflection
			input_file.read(reinterpret_cast<char*>(&ptmass_x), sizeof(ptmass_x)); // ptmass x value
			input_file.read(reinterpret_cast<char*>(&ptmass_y), sizeof(ptmass_y)); // ptmass y value
			input_file.read(reinterpret_cast<char*>(&is_offset), sizeof(is_offset)); // is offset

			// Add to the point mass
			this->node_ptmass.add_pointmass(node_id, ptmass_loc, ptmass_defl, ptmass_x, ptmass_y, is_offset);
		}
		else if (std::strcmp(type, "ilcd") == 0)
		{
			int node_id; // node id
			glm::vec2 inlcond_loc; // initial condition location
			// initial displacement x, initial displacement y, initial velocity x, initial velocity y
			double inl_displacement_x,  inl_displacement_y,inl_velocity_x, inl_velocity_y;

			input_file.read(reinterpret_cast<char*>(&node_id), sizeof(node_id)); // Node id
			input_file.read(reinterpret_cast<char*>(&inlcond_loc), sizeof(inlcond_loc)); // Initial condition location
			input_file.read(reinterpret_cast<char*>(&inl_displacement_x), sizeof(inl_displacement_x)); // Initial displacement x
			input_file.read(reinterpret_cast<char*>(&inl_displacement_y), sizeof(inl_displacement_y)); // Initial displacement y
			input_file.read(reinterpret_cast<char*>(&inl_velocity_x), sizeof(inl_velocity_x)); // Initial velocity x
			input_file.read(reinterpret_cast<char*>(&inl_velocity_y), sizeof(inl_velocity_y)); // Initial velocity y

			// Add to the initial condition
			this->node_inlcond.add_inlcondition(node_id, inlcond_loc,
				inl_displacement_x, inl_displacement_y, inl_velocity_x, inl_velocity_y);
		}
		else if (std::strcmp(type, "matr") == 0)
		{
			unsigned int material_id;
			size_t material_name_length;
			double material_stiffness;

			input_file.read(reinterpret_cast<char*>(&material_id), sizeof(material_id)); // Material id
			input_file.read(reinterpret_cast<char*>(&material_name_length), sizeof(material_name_length)); // Material name length

			// Allocate a buffer to hold the material name
			char buffer[256]; // Assuming a reasonable maximum length for the material name
			input_file.read(buffer, material_name_length);
			buffer[material_name_length] = '\0'; // Null-terminate the string

			std::string material_name(buffer);
			input_file.read(reinterpret_cast<char*>(&material_stiffness), sizeof(material_stiffness)); // Material stiffness

			// Create the temporary material
			material_data inpt_material;
			inpt_material.material_id = material_id; // Get the material id
			inpt_material.material_name = material_name; //Default material name
			inpt_material.material_stiffness = material_stiffness; // N/m

			 // Add to material list
			 mat_data[material_id] = inpt_material;
		}
	}

	// Clear the material list
	elm_prop_window->material_list.clear();
	elm_prop_window->material_list = mat_data;


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


void geom_store::write_rawdata(std::ofstream& output_file)
{

	// Write all the nodes
	for (auto& nd_m : model_nodes.nodeMap)
	{
		char type[5] = { 'n','o','d','e' }; // Identifier for node
		output_file.write(type, sizeof(type));

		const node_store& node = nd_m.second;

		// Write node data to the binary file
		output_file.write(reinterpret_cast<const char*>(&node.node_id), sizeof(node.node_id)); // node id
		output_file.write(reinterpret_cast<const char*>(&node.node_pt), sizeof(node.node_pt)); // node pt
	}

	// Write all the lines
	for (auto& ln_m : model_lineelements.elementlineMap)
	{
		char type[5] = { 'l','i','n','e' };  // Identifier for line
		output_file.write(type, sizeof(type));

		const elementline_store& line = ln_m.second;

		// Write line data to the binary file
		output_file.write(reinterpret_cast<const char*>(&line.line_id), sizeof(line.line_id)); // line id
		output_file.write(reinterpret_cast<const char*>(&line.startNode->node_id), sizeof(line.startNode->node_id)); // line start node id
		output_file.write(reinterpret_cast<const char*>(&line.endNode->node_id), sizeof(line.endNode->node_id)); // line end node id
		output_file.write(reinterpret_cast<const char*>(&line.material_id), sizeof(line.material_id)); // line end node id
	}

	// Write all the constraints
	for (auto& cnst_m : node_constraints.constraintMap)
	{
		char type[5] = { 'c','n','s','t' };  // Identifier for constraint
		output_file.write(type, sizeof(type));

		const constraint_data& cnst = cnst_m.second;

		// Write constraint data to the binary file
		output_file.write(reinterpret_cast<const char*>(&cnst.node_id), sizeof(cnst.node_id)); // node id
		output_file.write(reinterpret_cast<const char*>(&cnst.constraint_loc), sizeof(cnst.constraint_loc)); // constraint location
		output_file.write(reinterpret_cast<const char*>(&cnst.constraint_type), sizeof(cnst.constraint_type)); // constraint type
		output_file.write(reinterpret_cast<const char*>(&cnst.constraint_angle), sizeof(cnst.constraint_angle)); // constraint angle
	}

	// Write all the loads
	for (auto& ld_m : node_loads.loadMap)
	{
		char type[5] = { 'l','o','a','d' };  // Identifier for load
		output_file.write(type, sizeof(type));

		const load_data ld = ld_m.second;

		// Write load data to the binary file
		output_file.write(reinterpret_cast<const char*>(&ld.load_id), sizeof(ld.load_id)); // Load id
		output_file.write(reinterpret_cast<const char*>(&ld.node_id), sizeof(ld.node_id)); // Node id
		output_file.write(reinterpret_cast<const char*>(&ld.load_loc), sizeof(ld.load_loc)); // Load location
		output_file.write(reinterpret_cast<const char*>(&ld.load_start_time), sizeof(ld.load_start_time)); // Load start time
		output_file.write(reinterpret_cast<const char*>(&ld.load_end_time), sizeof(ld.load_end_time)); // Load end time
		output_file.write(reinterpret_cast<const char*>(&ld.load_value), sizeof(ld.load_value)); // Load value
		output_file.write(reinterpret_cast<const char*>(&ld.load_angle), sizeof(ld.load_angle)); // Load angle
		output_file.write(reinterpret_cast<const char*>(&ld.load_phase), sizeof(ld.load_phase)); // Load phase angle
	}

	// Write all the point mass
	for (auto& ptm_m : node_ptmass.ptmassMap)
	{
		char type[5] = { 'm','a','s','s' };  // Identifier for point mass
		output_file.write(type, sizeof(type));

		const nodepointmass_data ptm = ptm_m.second;

		// Write point mass data to the binary file
		output_file.write(reinterpret_cast<const char*>(&ptm.node_id), sizeof(ptm.node_id)); // Node id
		output_file.write(reinterpret_cast<const char*>(&ptm.ptmass_loc), sizeof(ptm.ptmass_loc)); // ptmass location
		output_file.write(reinterpret_cast<const char*>(&ptm.ptmass_defl), sizeof(ptm.ptmass_defl)); // ptmass deflection
		output_file.write(reinterpret_cast<const char*>(&ptm.ptmass_x), sizeof(ptm.ptmass_x)); // ptmass x value
		output_file.write(reinterpret_cast<const char*>(&ptm.ptmass_y), sizeof(ptm.ptmass_y)); // ptmass y value
		output_file.write(reinterpret_cast<const char*>(&ptm.is_offset), sizeof(ptm.is_offset)); // is offset
		
	}

	// Write all the initial condition data
	for (auto& inl_m : node_inlcond.inlcondMap)
	{
		char type[5] = { 'i','l','c','d' };  // Identifier for initial condition
		output_file.write(type, sizeof(type));

		const nodeinl_condition_data inl = inl_m.second;

		// Write initial condition data to the binary file
		output_file.write(reinterpret_cast<const char*>(&inl.node_id), sizeof(inl.node_id)); // Node id
		output_file.write(reinterpret_cast<const char*>(&inl.inlcond_loc), sizeof(inl.inlcond_loc)); // Initial condition location
		output_file.write(reinterpret_cast<const char*>(&inl.inl_displacement_x), sizeof(inl.inl_displacement_x)); // Initial displacement x
		output_file.write(reinterpret_cast<const char*>(&inl.inl_displacement_y), sizeof(inl.inl_displacement_y)); // Initial displacement y
		output_file.write(reinterpret_cast<const char*>(&inl.inl_velocity_x), sizeof(inl.inl_velocity_x)); // Initial velocity x
		output_file.write(reinterpret_cast<const char*>(&inl.inl_velocity_y), sizeof(inl.inl_velocity_y)); // Initial velocity y

	}

	// Write all the material property
	for (auto& mat_d : elm_prop_window->material_list)
	{
		char type[5] = { 'm','a','t','r' };  // Identifier for material properties
		output_file.write(type, sizeof(type));

		material_data mat = mat_d.second;

		// Write material property data to the binary file
		output_file.write(reinterpret_cast<const char*>(&mat.material_id), sizeof(mat.material_id)); // Material id

		// Write the length of the material_name and then write the characters
		size_t material_name_length = mat.material_name.size(); //size of the material name
		output_file.write(reinterpret_cast<const char*>(&material_name_length), sizeof(material_name_length));
		output_file.write(mat.material_name.c_str(), material_name_length);
		output_file.write(reinterpret_cast<const char*>(&mat.material_stiffness), sizeof(mat.material_stiffness)); // Material stiffness

	}

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
			std::vector<int> selected_elm_ids = model_lineelements.is_line_selected(o_pt, c_pt);
			elm_prop_window->add_to_element_list(selected_elm_ids, is_rightbutton);
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
	if (modal_solver_window->is_show_window == true ||
		pulse_solver_window->is_show_window == true ||
		forced_solver_window->is_show_window == true)
	{
		if (modal_solver_window->is_show_window == true && is_modal_analysis_complete == true &&
			modal_solver_window->show_undeformed_model == false)
		{
			// Modal Analysis complete, window open and user turned off model view
			return;
		}
		//________________________________________________________________________________________
		if (pulse_solver_window->is_show_window == true && is_pulse_analysis_complete == true &&
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
		// Element properties window
		paint_element_prop_operation();
	}
}

void geom_store::paint_model_results()
{
	// Paint the results
	// Modal Analysis 
	paint_modal_analysis_results();

	// Pulse Analysis
	paint_pulse_analysis_results();

	// Forced Response Analysis
	paint_forced_resp_analysis_results();

}


void geom_store::paint_node_constraint_operation()
{
	// Selection rectangle
	selection_rectangle.paint_selection_rectangle();

	// Paint the selected nodes
	if (nd_cnst_window->is_selected_count == true)
	{
		glPointSize(geom_param.selected_point_size);
		model_nodes.paint_selected_model_nodes();
		glPointSize(geom_param.point_size);
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
		glPointSize(geom_param.selected_point_size);
		model_nodes.paint_selected_model_nodes();
		glPointSize(geom_param.point_size);
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
		double load_phase = nd_load_window->load_phase_angle; // load phase angle

		for (int& id : nd_load_window->selected_nodes)
		{
			// Add the loads
			node_loads.add_load(id, model_nodes.nodeMap[id].node_pt,
				load_start_time, load_end_time, load_amplitude, load_angle, load_phase);
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
		glPointSize(geom_param.selected_point_size);
		model_nodes.paint_selected_model_nodes();
		glPointSize(geom_param.point_size);
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
			node_ptmass.add_pointmass(id, model_nodes.nodeMap[id].node_pt, glm::vec2(0),
				pt_mass_x, pt_mass_y, false);
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
		glPointSize(geom_param.selected_point_size);
		model_nodes.paint_selected_model_nodes();
		glPointSize(geom_param.point_size);
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


void geom_store::paint_element_prop_operation()
{
	// Paint the element properties application pre processing
	// Element Properties Window 
			// Selection rectangle
	selection_rectangle.paint_selection_rectangle();

	// Paint the selected elements
	if (elm_prop_window->is_selected_count == true)
	{
		glLineWidth(geom_param.selected_line_width);
		model_lineelements.paint_selected_elementlines();
		glLineWidth(geom_param.line_width);
	}

	// Check whether the selection changed
	if (elm_prop_window->is_selection_changed == true)
	{
		model_lineelements.add_selection_lines(elm_prop_window->selected_elements);
		elm_prop_window->is_selection_changed = false;
	}

	// Material deleted
	if (elm_prop_window->execute_delete_materialid != -1)
	{
		// Delete material
		// Execute delete material id
		model_lineelements.execute_delete_material(elm_prop_window->execute_delete_materialid);
		elm_prop_window->execute_delete_materialid = -1;

		// Remove the selection
		elm_prop_window->selected_elements.clear();
		elm_prop_window->is_selection_changed = true;
	}

	// Apply the Element properties
	if (elm_prop_window->apply_element_properties == true)
	{
		// Apply material properties to the selected line elements
		int material_id = elm_prop_window->material_list[elm_prop_window->selected_material_option].material_id; // get the material id
		model_lineelements.update_material(elm_prop_window->selected_elements, material_id);
		elm_prop_window->apply_element_properties = false;

		// Remove the selection
		elm_prop_window->selected_elements.clear();
		elm_prop_window->is_selection_changed = true;
	}

	// Paint the material ID
	model_lineelements.paint_lines_material_id();

}


void geom_store::paint_modal_analysis_results()
{
	// Paint the modal analysis results
	// Closing sequency for the modal analysis window
	if (modal_solver_window->execute_modal_close == true)
	{
		// Execute the close sequence
		if (is_modal_analysis_complete == true)
		{
			// Modal analysis is complete (but clear the results anyway beacuse results will be loaded at open)
			modal_solver_window->modal_analysis_complete = false;

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





}


void geom_store::paint_pulse_analysis_results()
{
	// Paint the pulse analysis results

}


void geom_store::paint_forced_resp_analysis_results()
{
	// Paint the forced response analysis results


}