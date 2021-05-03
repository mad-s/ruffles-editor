#include "common/common.h"
#include <imgui/imgui.h>

using ruffles::real;

namespace ImGui {
	bool InputReal(const char *label, real *v, real step = 0.0, real step_fast = 0.0, const char *format = "%.6f", ImGuiInputTextFlags flags = 0);
}
