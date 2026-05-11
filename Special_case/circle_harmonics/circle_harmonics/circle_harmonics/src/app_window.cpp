#include "app_window.h"

int app_window::window_width = 800;
int app_window::window_height = 600;
bool app_window::isWindowSizeChanging = false;


app_window::app_window()
{
	// Empty constructor
}


void app_window::init()
{
	// Initialize of the main app window
	// Initialize the window width, window height, window size
	is_glwindow_success = false;

	// Initialize GLFW
	if (!glfwInit())
	{
		// ShowWindow(GetConsoleWindow(), SW_RESTORE);
		std::cerr << "Failed to initialize GLFW" << std::endl;
		return;
	}

	// Request OpenGL 4.6 with Core Profile
	glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 4);
	glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 6);
	glfwWindowHint(GLFW_OPENGL_PROFILE, GLFW_OPENGL_CORE_PROFILE);
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);  // Required on macOS


#ifdef __APPLE__
	glfwWindowHint(GLFW_OPENGL_FORWARD_COMPAT, GL_TRUE);
#endif

	// Enable debug context for development (optional)
#ifdef _DEBUG
	glfwWindowHint(GLFW_OPENGL_DEBUG_CONTEXT, GL_TRUE);
#endif

	// Request higher quality for better performance
	glfwWindowHint(GLFW_SAMPLES, 4);  // MSAA
	glfwWindowHint(GLFW_DOUBLEBUFFER, GLFW_TRUE);


	// Create a window
	window = glfwCreateWindow(window_width, window_height, "Circle Harmonnics", nullptr, nullptr);

	if (!window)
	{
		std::cerr << "Failed to create GLFW window" << std::endl;
		glfwTerminate();
		return;
	}

	// Make the window's context current
	glfwMakeContextCurrent(window);

	// Initialize GLEW with experimental features
	glewExperimental = GL_TRUE;  // Needed for core profile
	GLenum err = glewInit();
	if (err != GLEW_OK)
	{
		std::cerr << "Failed to initialize GLEW: " << glewGetErrorString(err) << std::endl;
		glfwTerminate();
		return;
	}

	// Clear any GL error that might have occurred during GLEW init
	glGetError();


	// Verify OpenGL version
	checkOpenGLVersion();

	// Setup debug callback (OpenGL 4.3+)
#ifdef _DEBUG
	setupDebugCallback();
#endif


	// Set viewport and callbacks
	glfwGetFramebufferSize(window, &window_width, &window_height);
	glfwSetFramebufferSizeCallback(window, framebufferSizeCallback);

	// Set the icon for the window
	GLFWwindow_set_icon(window);

	// Initialize Modern OpenGL 4.6 features
	initModernOpenGL();

	// Window initialize success
	is_glwindow_success = true;

	// Intialize tool windows
	inl_window.init(); // Initial condition window
	md_window.init(); // Model window
	nd_load_window.init(); // Node load window
	op_window.init(); // Option window
	sol_modal_window.init(); // Modal Analysis solver window
	sol_pulse_window.init(); // Pulse Analysis solver window

	geom.update_WindowDimension(window_width, window_height);

	// Initialize the geometry (initialize only after model window is initialized)
	geom.init(&sol_modal_window,
		&sol_pulse_window,
		&op_window,
		&nd_load_window,
		&inl_window,
		&md_window);

	// Set the mouse button callback function with the user pointer pointing to the mouseHandler object
	glfwSetWindowUserPointer(window, &mouse_Handler);

	// Passing the address of geom and window dimensions to mouse handler
	mouse_Handler.init(geom);

	// Pass the address of options window, material window, solver window
	// geom.add_window_ptr(&op_window, &mat_window, &fe_window);

	glfwSetMouseButtonCallback(window, mouse_event_handler::mouseButtonCallback);

	// Set the mouse move callback function with the user pointer pointing to the mouseHandler object
	glfwSetCursorPosCallback(window, mouse_event_handler::mouseMoveCallback);

	// Set the mouse scroll callback function with the user pointer pointing to the mouseHandler object
	glfwSetScrollCallback(window, mouse_event_handler::mouseScrollCallback);

	// Set key input callback function with the user pointer pointing to the mouseHandler object
	glfwSetKeyCallback(window, mouse_event_handler::keyDownCallback);



	// Setup ImGui with OpenGL 4.6
	IMGUI_CHECKVERSION();
	ImGui::CreateContext();
	ImGui_ImplGlfw_InitForOpenGL(window, true);
	ImGui_ImplOpenGL3_Init("#version 460");  // Updated to 4.6

	ImGui::StyleColorsDark();
	customizeImGuiStyle();

	framebufferSizeCallback(window, window_width, window_height);

}



void app_window::setupDebugCallback()
{
	// OpenGL 4.6 debug callback
	glEnable(GL_DEBUG_OUTPUT);
	glEnable(GL_DEBUG_OUTPUT_SYNCHRONOUS);

	glDebugMessageCallback([](GLenum source, GLenum type, GLuint id,
		GLenum severity, GLsizei length,
		const GLchar* message, const void* userParam) {
			// Ignore non-error messages
			if (severity == GL_DEBUG_SEVERITY_NOTIFICATION) return;

			std::cerr << "OpenGL Debug: " << message << std::endl;

			if (type == GL_DEBUG_TYPE_ERROR) {
				std::cerr << "OpenGL Error! Severity: " << severity << std::endl;
				__debugbreak();  // Break on error in debug mode
			}
		}, nullptr);
}


void app_window::initModernOpenGL()
{
	// Enable OpenGL 4.6 features

	// 1. Direct State Access (DSA) - OpenGL 4.5+
	// Use glCreateBuffers, glNamedBufferData, etc.

	// 2. Enable depth testing
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);  // Better for modern rendering
	glDepthMask(GL_TRUE);

	// 3. Disable backface culling
	glDisable(GL_CULL_FACE);

	//glEnable(GL_CULL_FACE);
	//glCullFace(GL_BACK);
	//glFrontFace(GL_CCW);

	// 4. Enable seamless cubemaps (OpenGL 3.2+)
	glEnable(GL_TEXTURE_CUBE_MAP_SEAMLESS);

	// 5. Set anisotropic filtering (if using textures)
	if (glewIsSupported("GL_EXT_texture_filter_anisotropic")) {
		GLfloat maxAniso;
		glGetFloatv(GL_MAX_TEXTURE_MAX_ANISOTROPY_EXT, &maxAniso);
		glTexParameterf(GL_TEXTURE_2D, GL_TEXTURE_MAX_ANISOTROPY_EXT, maxAniso);
	}

	// 6. Enable multisample
	glEnable(GL_MULTISAMPLE);

	// 7. Set clear color
	glClearColor(0.2f, 0.2f, 0.2f, 1.0f);

	// 8. Use pixel buffer objects for efficient pixel transfer
	// Initialize PBOs if needed

	std::cout << "Modern OpenGL 4.6 initialized" << std::endl;
}



void app_window::fini()
{
	// Deinitialize ImGui and GLFW
	// Cleanup Geometry
	geom.fini();

	// Cleanup ImGui
	ImGui_ImplOpenGL3_Shutdown();
	ImGui_ImplGlfw_Shutdown();
	ImGui::DestroyContext();

	// Terminate GLFW
	glfwTerminate();
}

void app_window::app_render()
{
	// Create a custom font for the menu bar
	ImGuiIO& io = ImGui::GetIO();
	imgui_font = io.Fonts->AddFontFromFileTTF("./resources/fonts/FreeSans.ttf", 18);


	// Set the point size and line width
	// Set the point size
	// glPointSize(geom.geom_param.point_size);
	// glLineWidth(geom.geom_param.line_width);


	// Main rendering loop
	while (!glfwWindowShouldClose(window))
	{
		// Start ImGui frame
		ImGui_ImplOpenGL3_NewFrame();
		ImGui_ImplGlfw_NewFrame();
		ImGui::NewFrame();

		// menu events
		menu_events();

		// Status bar
		draw_status_bar();

		// Render OpenGL graphics here
		glClearColor(geom.geom_param.geom_colors.background_color.x,
			geom.geom_param.geom_colors.background_color.y,
			geom.geom_param.geom_colors.background_color.z, 1.0f);  // Set the clear color to black
		glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);  // Clear the color buffer

		// Window size change event
		if (isWindowSizeChanging == true)
		{
			geom.update_WindowDimension(window_width, window_height);
			mouse_Handler.zoom_to_fit();
		}
		isWindowSizeChanging = false;

		// Paint the geometry
		geom.paint_geometry();

		// Render ImGui UI
		ImGui::Render();
		ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

		// Swap buffers
		glfwSwapBuffers(window);

		// Poll for events
		glfwPollEvents();
	}
}

void app_window::menu_events()
{
	// Control the menu events
// Change the font for the menu bar
	ImGui::PushFont(imgui_font);


	// Create a menu bar
	if (ImGui::BeginMainMenuBar(), ImGuiWindowFlags_MenuBar)
	{
		// File menu item
		if (ImGui::BeginMenu("File"))
		{
			if (ImGui::MenuItem("New model"))
			{
				// Model data menu
				md_window.is_show_window = true;
			}
			if (ImGui::MenuItem("Options"))
			{
				// Options menu
				op_window.is_show_window = true;
			}
			if (ImGui::BeginMenu("View"))  // BeginMenu, not MenuItem
			{
				if (ImGui::MenuItem("Front view"))
				{
					// Handle front view
					// setFrontView();
					mouse_Handler.mouse_evnt.change_viewport(4);
				}
				if (ImGui::MenuItem("Top view"))
				{
					// Handle top view
					// setTopView();
					mouse_Handler.mouse_evnt.change_viewport(2);
				}
				if (ImGui::MenuItem("Side view"))
				{
					// Handle side view
					// setSideView();
					mouse_Handler.mouse_evnt.change_viewport(6);
				}
				ImGui::EndMenu();  // End the submenu
			}

			ImGui::Separator();  // Optional separator

			if (ImGui::MenuItem("Exit"))
			{
				// Handle menu Exit
				exit(0);
			}
			ImGui::EndMenu();
		}
		// Pre-Processing menu item
		if (ImGui::BeginMenu("Pre-Processing"))
		{
			if (ImGui::MenuItem("Intial Condition"))
			{
				// Initial condition
				inl_window.is_show_window = true;
			}
			if (ImGui::MenuItem("Nodal Loads"))
			{
				// Nodal Loads
				nd_load_window.is_show_window = true;
			}

			ImGui::EndMenu();
		}
		// Solve
		if (ImGui::BeginMenu("Solve"))
		{
			if (ImGui::MenuItem("Modal Analysis Solve"))
			{
				// Modal Analysis Solve
				sol_modal_window.execute_modal_open = true;
				sol_modal_window.is_show_window = true;
			}
			if (ImGui::MenuItem("Pulse Analysis Solve"))
			{
				// Pulse Analysis Solve
				sol_pulse_window.execute_pulse_open = true;
				sol_pulse_window.is_show_window = true;
			}


			ImGui::EndMenu();
		}
		// Add more menu items here as needed
		ImGui::EndMainMenuBar();
	}

	// Execute window render operation
	md_window.render_window(); // model window
	inl_window.render_window(); // initial condition window
	nd_load_window.render_window(); // Nodal load window
	op_window.render_window(); // Option window
	sol_modal_window.render_window(); // Modal Analysis Solver window
	sol_pulse_window.render_window(); // Pulse Analysis Solver window

	// Pop the custom font after using it
	ImGui::PopFont();
}

// Static callback function for framebuffer size changes
// static keyword makes the function a class-level function rather than an instance-level function
// allows it to be used as a callback function for the GLFW library
void app_window::framebufferSizeCallback(GLFWwindow* window, int window_width, int window_height)
{
	// Triggers when the openGL window is resized
	app_window::window_width = window_width;
	app_window::window_height = window_height;

	int max_dim = window_width > window_height ? window_width : window_height;
	int x_offset = (max_dim - window_width) / 2; // Calculate x offset to center the viewport
	int y_offset = (max_dim - window_height) / 2; // Calculate y offset to center the viewport

	// Set the viewport to the maximum dimension and center it at (0, 0)
	glViewport(-x_offset, -y_offset, max_dim, max_dim);

	app_window::isWindowSizeChanging = true;

}

void app_window::GLFWwindow_set_icon(GLFWwindow* window)
{
	// Get the image
	stb_implement stb("./resources/images/logo_ico_png.png");

	// Set the window icon using GLFW's API for Windows
	GLFWimage icon;
	icon.width = stb.image_width;
	icon.height = stb.image_height;
	icon.pixels = stb.image;
	glfwSetWindowIcon(window, 1, &icon);
}




void app_window::draw_status_bar()
{
	ImGuiViewport* viewport = ImGui::GetMainViewport();


	float height = ImGui::GetFrameHeight();

	ImGui::SetNextWindowPos(
		ImVec2(viewport->Pos.x, viewport->Pos.y + viewport->Size.y - height));

	ImGui::SetNextWindowSize(
		ImVec2(viewport->Size.x, height));

	ImGuiWindowFlags flags =
		ImGuiWindowFlags_NoTitleBar |
		ImGuiWindowFlags_NoResize |
		ImGuiWindowFlags_NoMove |
		ImGuiWindowFlags_NoScrollbar |
		ImGuiWindowFlags_NoSavedSettings;


	ImGui::Begin("StatusBar", nullptr, flags);

	float fps = ImGui::GetIO().Framerate;
	float ms = 1000.0f / fps;

	ImGui::Text("FPS: %.1f | Frame: %.2f ms", fps, ms);

	ImGui::End();
}



bool app_window::checkOpenGLVersion()
{
	GLint major, minor;
	glGetIntegerv(GL_MAJOR_VERSION, &major);
	glGetIntegerv(GL_MINOR_VERSION, &minor);

	std::cout << "OpenGL Version: " << major << "." << minor << std::endl;

	if (major > 4 || (major == 4 && minor >= 6))
	{
		std::cout << "OpenGL 4.6 supported!" << std::endl;
		return true;
	}
	else if (major == 4 && minor >= 3)
	{
		std::cout << "OpenGL 4.3+ supported. Some features may be limited." << std::endl;
		return false;
	}
	else
	{
		std::cerr << "OpenGL 4.0 not supported!" << std::endl;
		return false;
	}
}





//_____________________________________________________________________________________________________
//_____________________________________________________________________________________________________
//__________________________ Customize ImGUI Style ____________________________________________________
//_____________________________________________________________________________________________________
//_____________________________________________________________________________________________________


void app_window::customizeImGuiStyle()
{
	ImGuiStyle& style = ImGui::GetStyle();
	ImVec4* colors = style.Colors;

	// ============================================
	// 1. Style Preset Selection
	// ============================================

	// Option 1: Dark Theme (Default)
	// ImGui::StyleColorsDark();

	// Option 2: Light Theme
	// ImGui::StyleColorsLight();

	// Option 3: Classic Theme
	// ImGui::StyleColorsClassic();

	applyTheme(3);

}



// Helper function to apply a theme preset
void app_window::applyTheme(int themeIndex)
{
	switch (themeIndex) {
	case 0:
		ImGui::StyleColorsDark();
		break;
	case 1:
		ImGui::StyleColorsLight();
		break;
	case 2:
		ImGui::StyleColorsClassic();
		break;
	case 3:
		customizeEngineeringTheme();
		break;
	case 4:
		customizeBlueTheme();
		break;
	case 5:
		customizeHighContrastTheme();
		break;
	default:
		customizeEngineeringTheme();
		break;
	}
}

//// Add theme selector to your menu
//void app_window::renderThemeSelector()
//{
//	if (ImGui::BeginMenu("Themes"))
//	{
//		if (ImGui::MenuItem("Dark Theme")) applyTheme(0);
//		if (ImGui::MenuItem("Light Theme")) applyTheme(1);
//		if (ImGui::MenuItem("Classic Theme")) applyTheme(2);
//		if (ImGui::MenuItem("Engineering Theme")) applyTheme(3);
//		if (ImGui::MenuItem("Blue Theme")) applyTheme(4);
//		if (ImGui::MenuItem("High Contrast")) applyTheme(5);
//		ImGui::EndMenu();
//	}
//}




// Option 1: Engineering/Technical Theme
void app_window::customizeEngineeringTheme()
{
	ImGuiStyle& style = ImGui::GetStyle();
	ImVec4* colors = style.Colors;

	// Style parameters
	style.WindowRounding = 4.0f;
	style.WindowBorderSize = 1.0f;
	style.FrameRounding = 2.0f;
	style.FrameBorderSize = 0.0f;
	style.PopupRounding = 3.0f;
	style.ScrollbarRounding = 3.0f;
	style.GrabRounding = 2.0f;
	style.TabRounding = 3.0f;
	style.ChildRounding = 3.0f;

	// Sizing
	style.WindowPadding = ImVec2(8, 8);
	style.FramePadding = ImVec2(6, 4);
	style.ItemSpacing = ImVec2(8, 6);
	style.ItemInnerSpacing = ImVec2(6, 4);
	style.IndentSpacing = 21.0f;
	style.ScrollbarSize = 14.0f;
	style.GrabMinSize = 12.0f;

	// Colors - Dark Engineering Theme
	ImVec4 darkBg = ImVec4(0.18f, 0.18f, 0.20f, 1.00f);
	ImVec4 panelBg = ImVec4(0.22f, 0.22f, 0.24f, 1.00f);
	ImVec4 accent = ImVec4(0.26f, 0.59f, 0.98f, 1.00f);     // Professional blue
	ImVec4 accentHover = ImVec4(0.35f, 0.67f, 1.00f, 1.00f);
	ImVec4 accentActive = ImVec4(0.20f, 0.50f, 0.90f, 1.00f);
	ImVec4 textColor = ImVec4(0.95f, 0.95f, 0.97f, 1.00f);
	ImVec4 textDisabled = ImVec4(0.60f, 0.60f, 0.65f, 1.00f);
	ImVec4 borderColor = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
	ImVec4 errorColor = ImVec4(0.95f, 0.25f, 0.25f, 1.00f);
	ImVec4 warningColor = ImVec4(0.98f, 0.70f, 0.20f, 1.00f);
	ImVec4 successColor = ImVec4(0.25f, 0.75f, 0.35f, 1.00f);

	// Window backgrounds
	colors[ImGuiCol_WindowBg] = darkBg;
	colors[ImGuiCol_ChildBg] = panelBg;
	colors[ImGuiCol_PopupBg] = panelBg;

	// Borders
	colors[ImGuiCol_Border] = borderColor;
	colors[ImGuiCol_BorderShadow] = ImVec4(0.10f, 0.10f, 0.12f, 1.00f);

	// Headers
	colors[ImGuiCol_Header] = ImVec4(0.28f, 0.28f, 0.32f, 1.00f);
	colors[ImGuiCol_HeaderHovered] = ImVec4(0.30f, 0.30f, 0.35f, 1.00f);
	colors[ImGuiCol_HeaderActive] = accentActive;

	// Buttons
	colors[ImGuiCol_Button] = ImVec4(0.26f, 0.26f, 0.30f, 1.00f);
	colors[ImGuiCol_ButtonHovered] = accentHover;
	colors[ImGuiCol_ButtonActive] = accentActive;

	// Frame backgrounds (input boxes, dropdowns)
	colors[ImGuiCol_FrameBg] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
	colors[ImGuiCol_FrameBgHovered] = ImVec4(0.18f, 0.18f, 0.22f, 1.00f);
	colors[ImGuiCol_FrameBgActive] = ImVec4(0.20f, 0.20f, 0.25f, 1.00f);

	// Tabs
	colors[ImGuiCol_Tab] = ImVec4(0.20f, 0.20f, 0.24f, 1.00f);
	colors[ImGuiCol_TabHovered] = accentHover;
	colors[ImGuiCol_TabActive] = accent;
	colors[ImGuiCol_TabUnfocused] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
	colors[ImGuiCol_TabUnfocusedActive] = ImVec4(0.18f, 0.18f, 0.22f, 1.00f);

	// Title bar
	colors[ImGuiCol_TitleBg] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
	colors[ImGuiCol_TitleBgActive] = accent;
	colors[ImGuiCol_TitleBgCollapsed] = ImVec4(0.15f, 0.15f, 0.18f, 0.75f);

	// Scrollbar
	colors[ImGuiCol_ScrollbarBg] = ImVec4(0.12f, 0.12f, 0.15f, 1.00f);
	colors[ImGuiCol_ScrollbarGrab] = ImVec4(0.28f, 0.28f, 0.32f, 1.00f);
	colors[ImGuiCol_ScrollbarGrabHovered] = ImVec4(0.35f, 0.35f, 0.40f, 1.00f);
	colors[ImGuiCol_ScrollbarGrabActive] = accent;

	// Slider grabs
	colors[ImGuiCol_SliderGrab] = accent;
	colors[ImGuiCol_SliderGrabActive] = accentActive;

	// Checkbox
	colors[ImGuiCol_CheckMark] = accent;

	// Separator
	colors[ImGuiCol_Separator] = borderColor;
	colors[ImGuiCol_SeparatorHovered] = accentHover;
	colors[ImGuiCol_SeparatorActive] = accent;

	// Resize grip
	colors[ImGuiCol_ResizeGrip] = ImVec4(0.25f, 0.25f, 0.30f, 1.00f);
	colors[ImGuiCol_ResizeGripHovered] = accentHover;
	colors[ImGuiCol_ResizeGripActive] = accent;

	// Text
	colors[ImGuiCol_Text] = textColor;
	colors[ImGuiCol_TextDisabled] = textDisabled;
	colors[ImGuiCol_TextSelectedBg] = accent;

	// Drag and drop
	colors[ImGuiCol_DragDropTarget] = accent;

	// Plot lines
	colors[ImGuiCol_PlotLines] = ImVec4(0.60f, 0.80f, 1.00f, 1.00f);
	colors[ImGuiCol_PlotLinesHovered] = warningColor;
	colors[ImGuiCol_PlotHistogram] = successColor;
	colors[ImGuiCol_PlotHistogramHovered] = warningColor;

	// Menu bar
	colors[ImGuiCol_MenuBarBg] = ImVec4(0.14f, 0.14f, 0.16f, 1.00f);

	// Modal window dimming
	colors[ImGuiCol_ModalWindowDimBg] = ImVec4(0.0f, 0.0f, 0.0f, 0.60f);
}

// Option 2: Professional Blue Theme
void app_window::customizeBlueTheme()
{
	ImGuiStyle& style = ImGui::GetStyle();
	ImVec4* colors = style.Colors;

	// Style rounding
	style.WindowRounding = 6.0f;
	style.FrameRounding = 4.0f;
	style.PopupRounding = 4.0f;
	style.ScrollbarRounding = 4.0f;
	style.GrabRounding = 4.0f;
	style.TabRounding = 4.0f;

	// Professional blue color scheme
	ImVec4 primaryBlue = ImVec4(0.20f, 0.45f, 0.85f, 1.00f);
	ImVec4 primaryBlueHover = ImVec4(0.25f, 0.55f, 0.95f, 1.00f);
	ImVec4 primaryBlueActive = ImVec4(0.15f, 0.35f, 0.75f, 1.00f);

	colors[ImGuiCol_WindowBg] = ImVec4(0.06f, 0.06f, 0.08f, 0.94f);
	colors[ImGuiCol_ChildBg] = ImVec4(0.08f, 0.08f, 0.10f, 0.80f);
	colors[ImGuiCol_PopupBg] = ImVec4(0.08f, 0.08f, 0.10f, 0.94f);

	colors[ImGuiCol_Header] = primaryBlue;
	colors[ImGuiCol_HeaderHovered] = primaryBlueHover;
	colors[ImGuiCol_HeaderActive] = primaryBlueActive;

	colors[ImGuiCol_Button] = ImVec4(0.20f, 0.20f, 0.25f, 1.00f);
	colors[ImGuiCol_ButtonHovered] = primaryBlueHover;
	colors[ImGuiCol_ButtonActive] = primaryBlueActive;

	colors[ImGuiCol_FrameBg] = ImVec4(0.12f, 0.12f, 0.15f, 1.00f);
	colors[ImGuiCol_FrameBgHovered] = ImVec4(0.15f, 0.15f, 0.18f, 1.00f);
	colors[ImGuiCol_FrameBgActive] = ImVec4(0.18f, 0.18f, 0.22f, 1.00f);

	colors[ImGuiCol_Tab] = ImVec4(0.12f, 0.12f, 0.15f, 1.00f);
	colors[ImGuiCol_TabHovered] = primaryBlueHover;
	colors[ImGuiCol_TabActive] = primaryBlue;

	colors[ImGuiCol_SliderGrab] = primaryBlue;
	colors[ImGuiCol_SliderGrabActive] = primaryBlueActive;

	colors[ImGuiCol_CheckMark] = primaryBlue;

	colors[ImGuiCol_Text] = ImVec4(0.92f, 0.92f, 0.95f, 1.00f);
}

// Option 3: High Contrast Theme (Accessibility)
void app_window::customizeHighContrastTheme()
{
	ImGuiStyle& style = ImGui::GetStyle();
	ImVec4* colors = style.Colors;

	style.WindowRounding = 0.0f;
	style.FrameRounding = 0.0f;

	colors[ImGuiCol_WindowBg] = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
	colors[ImGuiCol_Text] = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);
	colors[ImGuiCol_TextDisabled] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);

	colors[ImGuiCol_Button] = ImVec4(0.20f, 0.20f, 0.20f, 1.00f);
	colors[ImGuiCol_ButtonHovered] = ImVec4(0.40f, 0.40f, 0.40f, 1.00f);
	colors[ImGuiCol_ButtonActive] = ImVec4(0.60f, 0.60f, 0.60f, 1.00f);

	colors[ImGuiCol_Header] = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
	colors[ImGuiCol_HeaderHovered] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
	colors[ImGuiCol_HeaderActive] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);

	colors[ImGuiCol_FrameBg] = ImVec4(0.0f, 0.0f, 0.0f, 1.00f);
	colors[ImGuiCol_FrameBgHovered] = ImVec4(0.30f, 0.30f, 0.30f, 1.00f);
	colors[ImGuiCol_FrameBgActive] = ImVec4(0.50f, 0.50f, 0.50f, 1.00f);

	colors[ImGuiCol_Border] = ImVec4(1.0f, 1.0f, 1.0f, 1.00f);
	colors[ImGuiCol_CheckMark] = ImVec4(0.0f, 1.0f, 0.0f, 1.00f);
	colors[ImGuiCol_SliderGrab] = ImVec4(0.0f, 1.0f, 0.0f, 1.00f);
}





