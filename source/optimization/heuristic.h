#pragma once

#include "common/common.h"
#include "ruffle/ruffle.h"
#include "optimization/target_shape.h"

namespace ruffles::optimization {

class Heuristic {
public:
	TargetShape target;
	real eta_outer = 1.5;
	real eta_inner = 0.1;
	real inner_ratio = 0.6;
	real min_length = 0.1; // TODO: how to set?

	Heuristic();
	Heuristic(TargetShape target);

	void step(Ruffle &ruffle);
	void step_inner(Ruffle &ruffle);
	void step_outer(Ruffle &ruffle);
};
}
