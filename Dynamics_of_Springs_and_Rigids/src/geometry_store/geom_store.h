#pragma once
#include "geom_parameters.h"

// File system
#include <fstream>
#include <sstream>
#include <iomanip>

// Window includes
#include "../tool_window/node_constraint_window.h"
#include "../tool_window/node_load_window.h"
#include "../tool_window/inlcondition_window.h"
#include "../tool_window/pointmass_window.h"
#include "../tool_window/options_window.h"
#include "../tool_window/element_prop_window.h"
#include "../tool_window/modal_analysis_window.h"
#include "../tool_window/pulse_analysis_window.h"
#include "../tool_window/forcedresp_analysis_window.h"

// Solver
// 1) Penalty method
#include "../fe_solver/penalty_method/modal_penalty_solver.h"
#include "../fe_solver/penalty_method/pulse_penalty_solver.h"


// 2) Lagrange method



// 3) Elimination method
#include "../fe_solver/elimination_method/modal_elim_solver.h"
#include "../fe_solver/elimination_method/pulse_elim_solver.h"
#include "../fe_solver/elimination_method/forcedresp_elim_solver.h"




// FE Objects
#include "fe_objects/nodes_list_store.h"
#include "fe_objects/elementline_list_store.h"
#include "fe_objects/nodeconstraint_list_store.h"
#include "fe_objects/nodeload_list_store.h"
#include "fe_objects/nodeinlcond_list_store.h"
#include "fe_objects/nodepointmass_list_store.h"

// Geometry Objects
#include "geometry_objects/dynamic_selrectangle_store.h"

// FE Result Objects Modal analysis
#include "analysis_result_objects/modal_nodes_list_store.h";
#include "analysis_result_objects/modal_elementline_list_store.h";

// FE Result Objects Pulse analysis
#include "analysis_result_objects/pulse_node_list_store.h"
#include "analysis_result_objects/pulse_elementline_list_store.h"


class geom_store
{
public: 
	const double m_pi = 3.14159265358979323846;
	bool is_geometry_set = false;

	// Main Variable to strore the geometry parameters
	geom_parameters geom_param;

	geom_store();
	~geom_store();

	void init(modal_analysis_window* modal_solver_window,
		pulse_analysis_window* pulse_solver_window,
		forcedresp_analysis_window* forcedresp_solver_window,
		options_window* op_window,
		node_constraint_window* nd_cnst_window, node_load_window* nd_load_window, 
		pointmass_window* nd_ptmass_window,	inlcondition_window* nd_inlcond_window,
		element_prop_window* elm_prop_window);
	void fini();

	// Reading and writing the geometry file
	void read_varai2d(std::ifstream& input_file);
	void read_dxfdata(std::ostringstream& input_data);
	void read_rawdata(std::ifstream& input_file);
	void write_rawdata(std::ofstream& output_file);

	// Functions to control the drawing area
	void update_WindowDimension(const int& window_width, const int& window_height);
	void update_model_matrix();
	void update_model_zoomfit();
	void update_model_pan(glm::vec2& transl);
	void update_model_zoom(double& z_scale);
	void update_model_transperency(bool is_transparent);

	// Function to paint the selection rectangle
	void update_selection_rectangle(const glm::vec2& o_pt, const glm::vec2& c_pt,
		const bool& is_paint, const bool& is_select, const bool& is_rightbutton);

	// Functions to paint the geometry and results
	void paint_geometry();
private:
	dynamic_selrectangle_store selection_rectangle;

	// Geometry objects
	nodes_list_store model_nodes;
	elementline_list_store model_lineelements;

	// Node initial condition, loads & Constraints
	nodeconstraint_list_store node_constraints;
	nodeload_list_store node_loads;
	nodepointmass_list_store node_ptmass;
	nodeinlcond_list_store node_inlcond;

	// Modal analysis result 
	modal_nodes_list_store modal_result_nodes;
	modal_elementline_list_store modal_result_lineelements;

	// Pulse analysis result
	pulse_node_list_store pulse_result_nodes;
	pulse_elementline_list_store pulse_result_lineelements;

	// Solver object
	int solver_type = -1;

	// 1) Penalty method
	modal_penalty_solver modal_penalty_s;
	pulse_penalty_solver pulse_penalty_s;


	// 2) Lagrange method


	// 3) Elimination method 
	modal_elim_solver modal_elim_s;
	pulse_elim_solver pulse_elim_s;
	forcedresp_elim_solver freq_elim_s;




	// Window pointers
	options_window* op_window = nullptr;
	node_constraint_window* nd_cnst_window = nullptr;
	node_load_window* nd_load_window = nullptr;
	pointmass_window* nd_ptmass_window = nullptr;
	inlcondition_window* nd_inlcond_window = nullptr;
	element_prop_window* elm_prop_window = nullptr;

	// Analysis window
	modal_analysis_window* modal_solver_window = nullptr;
	pulse_analysis_window* pulse_solver_window = nullptr;
	forcedresp_analysis_window* forcedresp_solver_window = nullptr;

	// Analysis results
	bool is_modal_analysis_complete = false;
	bool is_pulse_analysis_complete = false;
	bool is_forcedresp_analysis_complete = false;


	void paint_model(); // Paint the model
	void paint_model_results(); // Paint the results
	void paint_node_constraint_operation(); // Paint the node constraint pre processing
	void paint_node_load_operation(); // Paint the node load pre processing
	void paint_node_ptmass_operation(); // Paint the node point mass pre processing
	void paint_node_inlcond_operation(); // Paint the node initial condition pre processing
	void paint_element_prop_operation(); // Paint the Element properties pre processing

	//_____________________________________________________________________________________
	void paint_modal_analysis_results(); // Paint the modal analysis results
	void paint_pulse_analysis_results(); // Paint the pulse analysis results
	void paint_forced_resp_analysis_results(); // Paint the forced response analysis results

};

