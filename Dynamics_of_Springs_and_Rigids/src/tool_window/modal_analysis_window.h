#pragma once
#include <iostream>
#include <vector>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"


class modal_analysis_window
{
public:
	bool is_show_window = false;
	bool execute_modal_analysis = false; // Main solver run event flag
	bool execute_modal_open = false; // Solver window execute opening event flag
	bool execute_modal_close = false; // Closing of solution window event flag

	int solver_type = 0; // Solver type 0 - Penalty method, 1 - Lagrange method, 2 - Elimination method

	// Modal analysis result list
	int selected_modal_option = 0;
	std::vector<std::string> mode_result_str;
	int selection_change_flag = 0;
	bool is_mode_selection_changed = false;

	// analysis results
	bool show_undeformed_model = true; // show undeformed model 
	bool show_result_text_values = true; // show the result text values

	// Animation control
	bool animate_play = true;
	bool animate_pause = false;
	double time_val = 0.0;
	double deformation_scale_max = 10.0;
	double animation_speed = 20.0;
	double normailzed_defomation_scale = 0.0;
	double deformation_scale = 0.0;

	modal_analysis_window();
	~modal_analysis_window();
	void init(); // initialize
	void render_window();
private:
	Stopwatch_events stopwatch;

};
