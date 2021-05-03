#pragma once

#include "common/common.h"
#include "ruffle/ruffle.h"
#include "optimization/target_shape.h"

#include <random>

namespace ruffles::optimization {

class ParticleSwarm {
public:
	struct Particle {
		Ruffle ruffle;

		VectorX x;
		VectorX v;

		VectorX best;
		real best_value;

		Particle(Ruffle &ruffle_);

		void set_lengths();
	};

	TargetShape target_shape;
	int m;

	vector<Particle> particles;
	VectorX global_best;
	real global_best_value;

	Ruffle *global_best_ruffle;

	real omega = 1.0;
	real phi_p = 2.0;
	real phi_g = 2.0;

	real learning_rate = 1.0;


	ParticleSwarm(TargetShape target_shape, Ruffle &ruffle, int n);

	void update_best();

	void physics_solve();

	void step();
};

}
