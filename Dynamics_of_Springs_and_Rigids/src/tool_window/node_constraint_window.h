#pragma once
#include <iostream>
#include <cstring>
#include "../ImGui/imgui.h"
#include "../ImGui/imgui_impl_glfw.h"
#include "../ImGui/imgui_impl_opengl3.h"
#include "../ImGui/stb_image.h"
#include "../ImGui/stb_implement.h"
#include "../geometry_store/geom_parameters.h"

struct cnst_image_data
{
	int image_width = 0;
	int image_height = 0;
	unsigned int image_texture_ID = 0;
	bool is_loaded = false;
};

class node_constraint_window
{
public:
	bool is_show_window = false;
	bool is_selection_changed = false;
	bool is_selected_count = false;

	bool apply_nodal_constraint = false;// Apply nodal constraint
	bool delete_nodal_constraint = false; // Delete nodal constraint
	int selected_constraint_option = 0; 	// Constraint type 0 - fixed support, 1 - fixed roller, 2 - pin support, 3 - pin roller
	std::vector<int> selected_nodes;

	double constraint_angle = 90.0; // Constraint angle

	node_constraint_window();
	~node_constraint_window();
	void init(); // initialize bind images
	void render_window();
	void add_to_node_list(const std::vector<int>& selected_nodes, const bool& is_right);
private:
	cnst_image_data cnst_image;

	void get_constraint_angle_input();
	void draw_support();
	void LoadTextureFromFile(const char* filename, cnst_image_data& cnst_image);
	bool get_image_min_max_coord(ImVec2& window_pos, ImVec2& window_size, ImVec2& img_pos_top_left,
		ImVec2& img_pos_top_right, ImVec2& img_pos_bot_right, ImVec2& img_pos_bot_left, double& orientation);
};

