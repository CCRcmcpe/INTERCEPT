#ifndef SOLVER_H
#define SOLVER_H

#include <format>
#include <functional>
#include <iostream>
#include <numbers>

#include "Eigen/Eigen"
#include <gsl/gsl_roots.h>
#include "gsl/gsl_multiroots.h"
#include "gsl/gsl_poly.h"

using namespace Eigen;
using namespace std;

constexpr double G = 9.8066;
constexpr double DEGREE = std::numbers::pi_v<double> / 180.;

constexpr double TOLERANCE = 1.;
constexpr double HORIZONAL_EPS = 1e-05;
constexpr double FALLBACK_TO_NUMERICAL_LOSS_TOLERANCE = 1000.;
constexpr double STEP_SIZE = 0.05;
constexpr int EQUATION_ORDER = 4;
constexpr size_t COEFF_COUNT = static_cast<size_t>(EQUATION_ORDER) + 1;

const Vector3d G_ACC(0., 0., -G);

using LossFunction = function<Vector2d(double, double, double*)>;

inline LossFunction Bounded(const LossFunction& func, const Vector2d& lower, const Vector2d& upper,
                            const double minIncr = 0.001)
{
	return [=](const double h, const double v, double* outT) -> Vector2d
	{
		Vector2d x(h, v);
		Vector2d xBorder(h, v);
		double distFromBorder = 0;
		for (int i = 0; i < 2; i++)
		{
			if (x[i] < lower[i])
			{
				xBorder[i] = lower[i];
				distFromBorder += lower[i] - x[i];
			}
			if (x[i] > upper[i])
			{
				xBorder[i] = upper[i];
				distFromBorder += x[i] - upper[i];
			}
		}
		if (distFromBorder == 0)
		{
			return func(h, v, outT);
		}

		Vector2d fBorder = func(xBorder[0], xBorder[1], outT);

		Vector2d fBorderMinIncr(0, 0);
		for (int i = 0; i < 2; i++)
		{
			if (fBorder[i] > 0)
			{
				fBorderMinIncr[i] = minIncr;
			}
			else
			{
				fBorderMinIncr[i] = -minIncr;
			}
		}

		return fBorder + (fBorder + fBorderMinIncr) * distFromBorder;
	};
}

inline double AngleBetween(const Vector3d& v1, const Vector3d& v2)
{
	Vector3d v1C = v1;
	Vector3d v2C = v1;
	v1C.normalize();
	v2C.normalize();
	return acos(clamp(v1.dot(v2), -1.0, 1.0));
}


struct Solution
{
	double Horizonal;
	double Vertical;
	double T;
};

inline std::ostream& operator<<(std::ostream& os, const Solution& arg)
{
	return os << format("Horizonal = {}бу, Vertical = {}бу, TOF = {}s", arg.Horizonal / DEGREE, arg.Vertical / DEGREE,
	                    arg.T);
}

struct IntegrationResult
{
	Vector3d LastPos;
	double T;
};

inline void PrintState(const gsl_multiroot_fsolver* solver, int iter)
{
	const string info = format(
		"iter = {} | x = {:.3f}, {:.3f} | loss(x) = {:.3f}, {:.3f}",
		iter + 1,
		gsl_vector_get(solver->x, 0) / DEGREE,
		gsl_vector_get(solver->x, 1) / DEGREE,
		gsl_vector_get(solver->f, 0),
		gsl_vector_get(solver->f, 1)
	);

	cout << info << endl;
}

class BallisticsSolver
{
	gsl_poly_complex_workspace* PolynomialWorkspace = gsl_poly_complex_workspace_alloc(COEFF_COUNT);
	gsl_multiroot_fsolver* NumericalSolver = gsl_multiroot_fsolver_alloc(gsl_multiroot_fsolver_hybrids, 2);

	Vector3d TargetPos;
	Vector3d TargetVelocity;
	Vector3d TargetAcceleration;
	double InitSpeed = 0;
	double AirFriction = 0;

	Vector3d TargetPosF(double t) const;

	Vector3d HvToVelocity(double horizonal, double vertical) const;

	Vector2d VelocityToHv(const Vector3d& velocity) const;

	bool SimplisticSolution(shared_ptr<Solution>* outSolution) const
	{
		const Vector3d relAcc = TargetAcceleration - G_ACC;
		const double coeffs[COEFF_COUNT] = {
			TargetPos.dot(TargetPos),
			(2.0 * TargetVelocity.dot(TargetPos)),
			relAcc.dot(TargetPos) + TargetVelocity.dot(TargetVelocity) - InitSpeed * InitSpeed,
			relAcc.dot(TargetVelocity),
			relAcc.dot(relAcc) / 4.0
		};
		Vector<double, EQUATION_ORDER * 2> results;

		gsl_poly_complex_solve(coeffs, COEFF_COUNT, PolynomialWorkspace, results.data());

		const double* t = nullptr;
		for (auto col : results.reshaped(2, EQUATION_ORDER).colwise())
		{
			const double& real = col[0];
			const double& imag = col[1];
			if (imag != 0 || real < 0) continue;

			if (t == nullptr || real < *t)
			{
				t = &real;
			}
		}

		if (t == nullptr)
		{
			return false;
		}

		const double tof = *t;

		const Vector3d vSol = tof * relAcc / 2.0 + TargetVelocity + TargetPos / tof;
		Vector2d hv = VelocityToHv(vSol);

		*outSolution = make_shared<Solution>(Solution{hv[0], hv[1], tof});
		return true;
	}

	bool NumericalSolution(const double initialGuessH, const double initialGuessV,
	                       shared_ptr<Solution>* outSolution) const
	{
		int status;
		int iter = 0;

		gsl_vector* initX = gsl_vector_alloc(2);
		gsl_vector_set(initX, 0, initialGuessH);
		gsl_vector_set(initX, 1, initialGuessV);

		gsl_multiroot_fsolver_set(NumericalSolver, const_cast<gsl_multiroot_function*>(&GslAdaptedLoss), initX);

#if _DEBUG
		PrintState(NumericalSolver, iter);
#endif

		double finalH = -1, finalV = -1;

		do
		{
			iter++;
			status = gsl_multiroot_fsolver_iterate(NumericalSolver);

#if _DEBUG
			PrintState(NumericalSolver, iter);
#endif

			if (status) // check if solver is stuck -- GSL_EBADFUNC or GSL_ENOPROG
				break;

			finalH = gsl_vector_get(NumericalSolver->x, 0);
			finalV = gsl_vector_get(NumericalSolver->x, 1);
			Vector2d loss(gsl_vector_get(NumericalSolver->f, 0), gsl_vector_get(NumericalSolver->f, 1));

			if (loss.norm() < TOLERANCE)
			{
				status = GSL_SUCCESS;
				break;
			}

			status = gsl_multiroot_test_residual(NumericalSolver->f, 1e-7);
		}
		while (status == GSL_CONTINUE && iter < 100);

#if _DEBUG
		cout << "status = " << gsl_strerror(status) << endl;
#endif

		gsl_vector_free(initX);

		double tof;
		Loss(finalH, finalV, &tof);
		*outSolution = make_shared<Solution>(Solution{finalH, finalV, tof});
		return status == GSL_SUCCESS;
	}

	struct LocateDtParams
	{
		const BallisticsSolver* Solver;
		Vector3d ProjPos;
		Vector3d ProjVel;
		double T;
		double AirFriction;
	};

	static double LocateDeltaT(LocateDtParams& exactTimeParams)
	{
		auto exactTimeEq = [](const double deltaT, void* params)
		{
			auto& [solver, projPos, projVel, t, airFriction] = *static_cast<LocateDtParams*>(params);
			const Vector3d projAcc = airFriction * projVel.array().pow(2).matrix() + G_ACC;
			const Vector3d projVel2 = projVel + projAcc * deltaT;
			const Vector3d projPos2 = projPos + projVel * deltaT;
			return 90 * DEGREE - AngleBetween(solver->TargetPosF(t + deltaT) - projPos2, projVel2);
		};

		double lo = 0, hi = STEP_SIZE;

		// cout << exactTimeEq(lo, &exactTimeParams) << endl;
		// cout << exactTimeEq(hi, &exactTimeParams) << endl;

		gsl_function function{exactTimeEq, &exactTimeParams};
		gsl_root_fsolver* const brent = gsl_root_fsolver_alloc(gsl_root_fsolver_brent);
		gsl_root_fsolver_set(brent, &function, lo, hi);

		int iter = 0;
		int status;
		double root;
		do
		{
			iter++;
			gsl_root_fsolver_iterate(brent);
			root = gsl_root_fsolver_root(brent);

			lo = gsl_root_fsolver_x_lower(brent);
			hi = gsl_root_fsolver_x_upper(brent);
			status = gsl_root_test_interval(lo, hi, 0, 0.001);
		}
		while (status == GSL_CONTINUE && iter < 100);

		gsl_root_fsolver_free(brent);

		return root;
	}

	IntegrationResult Integrate(const double horizonal, const double vertical) const
	{
		// if (isnan(horizonal) || isnan(vertical) || horizonal < -90 * DEGREE || horizonal > 90 * DEGREE || vertical < -90
		// 	* DEGREE || vertical > 90 * DEGREE)
		// {
		// 	throw exception("OUT OF RANGE");
		// }
		Vector3d projPos(0, 0, 0);
		Vector3d projVel = HvToVelocity(horizonal, vertical);
		double t = 0;
		while (true)
		{
			Vector3d lastPos = projPos;
			Vector3d lastVel = projVel;
			const double lastT = t;

			Vector3d projAcc = AirFriction * projVel.array().pow(2).matrix() + G_ACC;
			projVel += projAcc * STEP_SIZE;
			projPos += projVel * STEP_SIZE;
			t += STEP_SIZE;

			Vector3d targetPos = TargetPosF(t);
			if (const double angle = 90 * DEGREE - AngleBetween(targetPos - projPos, projVel); angle < 0)
			{
				LocateDtParams params{this, lastPos, lastVel, lastT, AirFriction};
				const double deltaT = LocateDeltaT(params);

				projAcc = AirFriction * lastVel.array().pow(2).matrix() + G_ACC;
				lastVel += projAcc * deltaT;
				lastPos += projVel * deltaT;

				return IntegrationResult{lastPos, lastT + deltaT};
			}
		}
	}

	Vector2d Loss(const double horizonal, const double vertical, double* outT = nullptr) const
	{
		auto [LastPos, T] = Integrate(horizonal, vertical);
		const Vector3d lastTargetPos = TargetPosF(T);
		// cout << (LastPos - lastTargetPos).head(1).norm() << "|" << LastPos.z() - lastTargetPos.z() << endl;
		if (outT != nullptr) *outT = T;
		return Vector2d(
			(LastPos - lastTargetPos).head(1).norm(),
			LastPos.z() - lastTargetPos.z()
		);
	}

	const LossFunction BoundedLossFunction;
	const gsl_multiroot_function GslAdaptedLoss;

public:
	BallisticsSolver() :
		BoundedLossFunction(
			Bounded([this](const double h, const double v, double* outT)
			{
				return Loss(h, v, outT);
			}, Vector2d(-40 * DEGREE, -20 * DEGREE), Vector2d(40 * DEGREE, 90 * DEGREE))
		),
		GslAdaptedLoss({
			[](const gsl_vector* x, void* params, gsl_vector* f) -> int
			{
				const BallisticsSolver* solver = static_cast<BallisticsSolver*>(params);
				const double horizonal = gsl_vector_get(x, 0);
				const double vertical = gsl_vector_get(x, 1);
				Vector2d loss = solver->BoundedLossFunction(horizonal, vertical, nullptr);
				gsl_vector_set(f, 0, loss[0]);
				gsl_vector_set(f, 1, loss[1]);
				return GSL_SUCCESS;
			},
			2, this
		})
	{
	}

	~BallisticsSolver()
	{
		gsl_poly_complex_workspace_free(PolynomialWorkspace);
		gsl_multiroot_fsolver_free(NumericalSolver);
	}

	shared_ptr<Solution> operator()(
		// TARGET
		const Vector3d& targetPos,
		const Vector3d& targetVelocity,
		const Vector3d& targetAcceleration,
		// PROJECTILE
		const double initSpeed,
		const double airFriction)
	{
		TargetPos = targetPos;
		TargetVelocity = targetVelocity;
		TargetAcceleration = targetAcceleration;
		InitSpeed = initSpeed;
		AirFriction = airFriction;

		shared_ptr<Solution> result;

		if (!SimplisticSolution(&result))
			return nullptr;

		if (AirFriction == 0)
			return result;

		const double loss = Loss(result->Horizonal, result->Vertical).norm();

		if (loss < TOLERANCE)
			return result;

		if (loss > FALLBACK_TO_NUMERICAL_LOSS_TOLERANCE)
			return nullptr;

		if (NumericalSolution(result->Horizonal, result->Vertical, &result))
		{
			if (abs(result->Horizonal) < HORIZONAL_EPS)
			{
				result->Horizonal = 0;
			}
		}

		return result;
	}
};

inline Vector3d BallisticsSolver::TargetPosF(const double t) const
{
	return TargetPos + TargetVelocity * t + (TargetAcceleration * t).array().pow(2).matrix() / 2;
}

inline Vector3d BallisticsSolver::HvToVelocity(const double horizonal, const double vertical) const
{
	return Vector3d(
		sin(horizonal) * cos(vertical),
		cos(horizonal) * cos(vertical),
		sin(vertical)
	) * InitSpeed;
}

inline Vector2d BallisticsSolver::VelocityToHv(const Vector3d& velocity) const
{
	const double horizonal = atan2(velocity[0], velocity[1]);
	const double vertical = asin(velocity[2] / InitSpeed);
	return Vector2d(horizonal, vertical);
}

#endif // SOLVER_H
