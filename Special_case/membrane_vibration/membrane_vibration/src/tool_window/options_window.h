#pragma once
#include <iostream>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"

class options_window
{
public:
	// Model constraints
	bool is_show_constraint = true; // Show constraints
	bool is_show_loads = true; // Show loads
	bool is_show_loadvalues = true; // Show load values;
	bool is_show_ptmass = true; // Show point mass
	bool is_show_ptmass_labels = true; // show point mass labels
	bool is_show_inlcondition = true; // show initial condition

	// Model Nodes
	bool is_show_modelnodes = true; // Show model nodes
	bool is_show_modelnodeids = true; // Show model node ids 
	bool is_show_modelnodecoords = true; // Show model node co-ordinates

	// Model elements
	bool is_show_modeledeges = true; // Show model edges
	bool is_show_modelelements = true; // show model elements

	// Window
	bool is_show_window = false;

	options_window();
	~options_window();
	void init();
	void render_window();
private:

};