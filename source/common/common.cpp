#include "common/common.h"

#include <iostream>


namespace ruffles {

real max(real a, real b) {
	return a > b ? a : b;
}
real min(real a, real b) {
	return a < b ? a : b;
}

void panic_impl(std::string fname, int line, std::string what) {
	std::cerr << "Panic at " << fname << ":" << line << ": \"" << what << "\"" << std::endl;
	std::exit(1);
}

VectorX random_vector(int m) {
	return VectorX::Random(m).array()*0.5+0.5;
}


real angle(Vector6 x, Vector6 *grad, Matrix6 *hessian) {
	real dx1 = x(2*1+0) - x(2*0+0);
	real dy1 = x(2*1+1) - x(2*0+1);
	real dx2 = x(2*2+0) - x(2*1+0);
	real dy2 = x(2*2+1) - x(2*1+1);
	real dx3 = x(2*2+0) - x(2*0+0);
	real dy3 = x(2*2+1) - x(2*0+1);

	real l1_sq = dx1*dx1 + dy1*dy1;
	real l2_sq = dx2*dx2 + dy2*dy2;
	real l3_sq = dx3*dx3 + dy3*dy3;

	real l1 = sqrt(l1_sq);
	real l2 = sqrt(l2_sq);
	real l3 = sqrt(l3_sq);

	real a = l1_sq + l2_sq - l3_sq;
	real b = 2*l1*l2;
	real c = min(1., max(-1., a/b));
	real theta = acos(c);
	//dbg(theta);

	if (grad) {
		Vector6 dl1_sq, dl2_sq, dl3_sq;
		dl1_sq << -2*dx1, -2*dy1, 2*dx1,  2*dy1, 0., 0.;
		dl2_sq << 0.,   0.,  -2*dx2, -2*dy2, 2*dx2, 2*dy2;
		dl3_sq << -2*dx3, -2*dy3, 0.,   0.,  2*dx3, 2*dy3;


		Vector6 dl1 = 1./(2*l1) * dl1_sq;
		Vector6 dl2 = 1./(2*l2) * dl2_sq;
		Vector6 dl3 = 1./(2*l3) * dl3_sq;
		(void)dl3;


		Vector6 da = dl1_sq + dl2_sq - dl3_sq;
		Vector6 db = 2*l1*dl2 + 2*l2*dl1;
		Vector6 dc = da/b - a*db/(b*b);

		real s = sqrt(max(0., 1-c*c));
		bool degen = s < 1e-6;
		if (degen) {
			s = 1e-6;
			// TODO: accurate derivatives in the limit
		}
		//dbg(degen);

		Vector6 dtheta = -1/s * dc;
		*grad = dtheta;

		if (hessian) {
			if (degen) {
				hessian->setZero();
			} else {
				Matrix2 t = 2*Matrix2::Identity();
				Matrix2 z =   Matrix2::Zero();

				Matrix6 hl1_sq,hl2_sq,hl3_sq;
				hl1_sq <<
					 t,-t, z,
					-t, t, z,
					 z, z, z;
				hl2_sq <<
					 z, z, z,
					 z, t,-t,
					 z,-t, t;
				hl3_sq <<
					 t, z,-t,
					 z, z, z,
					-t, z, t;

				// outer product
				auto outer = [](Vector6 a, Vector6 b) {
					return a * b.transpose();
				};

				Matrix6 hl1 = hl1_sq / (2*l1) - outer(dl1_sq, dl1_sq) / (4*l1_sq*l1);
				Matrix6 hl2 = hl2_sq / (2*l2) - outer(dl2_sq, dl2_sq) / (4*l2_sq*l2);
				Matrix6 hl3 = hl3_sq / (2*l3) - outer(dl3_sq, dl3_sq) / (4*l3_sq*l3);
				(void)hl3;
				

				Matrix6 ha = hl1_sq + hl2_sq - hl3_sq;
				Matrix6 hb = 2*(outer(dl1,dl2) + outer(dl2,dl1) + l1*hl2 + l2*hl1);

				Matrix6 hc = ha / b - a*hb/(b*b) + 2*a*outer(db,db)/(b*b*b) - (outer(da,db)+outer(db,da))/(b*b);

				Matrix6 htheta = -c * outer(dc,dc) /(s*s*s) - hc / s;
				*hessian = htheta;
			}
		}

	}

	return theta;
}



}
