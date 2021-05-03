#include "optimization/particle_swarm.h"

#include "simulation/verlet.h"
#include "simulation/lbfgs.h"
#include "simulation/combination.h"

namespace ruffles::optimization {
// implement particle
ParticleSwarm::Particle::Particle(Ruffle &ruffle_)
	: ruffle(ruffle_.clone()), best(ruffle_.sections.size()), best_value(infinity) {
	//ruffle.simulator.reset(new simulation::Verlet(ruffle.simulation_mesh));
	//ruffle.simulator.reset(new simulation::LBFGS(ruffle.simulation_mesh));
	ruffle.simulator.reset(new simulation::Combination(ruffle.simulation_mesh));
}

void ParticleSwarm::Particle::set_lengths() {
	int i = 0;
	for (auto it = ruffle.sections.begin(); it != ruffle.sections.end(); ++it) {
		it->length = x(i);
		i++;
	}
	ruffle.update_simulation_mesh();
}

ParticleSwarm::ParticleSwarm(TargetShape target_shape, Ruffle &ruffle, int n)
	: target_shape(target_shape), global_best_value(infinity), global_best_ruffle(nullptr) {
	m = ruffle.sections.size();
	VectorX x0(m);

	int i = 0;
	for (auto it = ruffle.sections.begin(); it != ruffle.sections.end(); ++it) {
		x0[i] = it->length;
		i++;
	}

	VectorX lb = 0.5 * x0;
	VectorX ub = 1.5 * x0;

	for (int i = 0; i < n; i++) {
		particles.emplace_back(ruffle);
		Particle &particle = particles.back();
		VectorX x = lb + random_vector(m).cwiseProduct(ub-lb);
		particle.x = x;
		// ?
		particle.v = VectorX::Zero(m);
		particle.set_lengths();

		//particles.push_back(particle);
	}

	physics_solve();
	update_best();
}

void ParticleSwarm::update_best() {
	for (auto &particle : particles) {
		real value = target_shape.energy(particle.ruffle);
		if (value < particle.best_value) {
			particle.best_value = value;
			particle.best = particle.x;
			if (value < global_best_value) {
				global_best_value = value;
				global_best = particle.x;
				global_best_ruffle = &particle.ruffle;
			}
		}
	}
}

void ParticleSwarm::physics_solve() {
	int i = 0;
	cerr << "Solving " << particles.size() << " ruffles!" << endl;
	// TODO: parallelize
	for (auto &particle : particles) {
		particle.ruffle.physics_solve();
		i++;
		cerr << i << "/" << particles.size() << endl;
	}
}

void ParticleSwarm::step() {
	for (auto &particle : particles) {
		particle.v =
			omega*particle.v
		      + phi_p*random_vector(m).cwiseProduct(particle.best-particle.x)
		      + phi_g*random_vector(m).cwiseProduct(global_best  -particle.x);
		particle.x += learning_rate * particle.v;
		particle.set_lengths();
	}

	physics_solve();
	update_best();
}


}
