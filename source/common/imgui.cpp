#include "common/imgui.h"

namespace ImGui {

using ruffles::real;

bool InputReal(const char *label, real *v, real step, real step_fast, const char *format, ImGuiInputTextFlags flags) {
	if (std::is_same<real, double>::value) {
		return InputDouble(label,(double*)(void*)v,step,step_fast,format,flags);
	} else if (std::is_same<real, float>::value) {
		return InputFloat(label,(float*)(void*)v,step,step_fast,format,flags);
	}
	return false;
}

}
