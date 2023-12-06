#pragma once
#include <iostream>
#include <GL/glew.h>
#include <GLFW/glfw3.h>
#include <glm/vec2.hpp>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"

class inlcondition_window
{
public:
	bool is_show_window = false;
	bool is_selection_changed = false;
	bool is_selected_count = false;

	bool apply_nodal_inlcond = false;// Apply nodal initial condition
	bool delete_nodal_inlcond = false; // Delete nodal initial condition
	std::vector<int> selected_nodes;

	// Initial displacement
	double initial_displacement_x = 0.0;
	double initial_displacement_y = 0.0;
	// Initial velocity
	double initial_velocity_x = 0.0;
	double initial_velocity_y = 0.0;
	
	inlcondition_window();
	~inlcondition_window();
	void init();
	void render_window();
	void add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right);
private:
	void get_idisplx_value_input();
	void get_idisply_value_input();
	void get_ivelox_value_input();
	void get_iveloy_value_input();

};
