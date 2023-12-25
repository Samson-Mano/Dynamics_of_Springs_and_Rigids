#pragma once
#include <iostream>
#include <vector>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"

class forcedresp_analysis_window
{
public:
	bool is_show_window = false;
	bool execute_forcedresp_analysis = false; // Main solver run event flag
	bool execute_forcedresp_open = false; // Solver window execute opening event flag
	bool execute_forcedresp_close = false; // Closing of solution window event flag


	// Forced response analysis results
	bool forcedresp_analysis_complete = false;




	forcedresp_analysis_window();
	~forcedresp_analysis_window();
	void init();// initialize
	void render_window();
private:
	Stopwatch_events stopwatch;

};
