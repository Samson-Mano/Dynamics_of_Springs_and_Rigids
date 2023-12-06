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

class pointmass_window
{
public:
	bool is_show_window = false;
	bool is_selection_changed = false;
	bool is_selected_count = false;

	bool apply_nodal_ptmass = false;// Apply point masss
	bool delete_nodal_ptmass = false; // Delete point mass
	std::vector<int> selected_nodes;

	// Point Mass
	double mass_x = 100;
	double mass_y = 100;


	pointmass_window();
	~pointmass_window();
	void init();
	void render_window();
	void add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right);
private:

	void get_massx_value_input();
	void get_massy_value_input();


};
