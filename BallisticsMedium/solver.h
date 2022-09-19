#include "Eigen/Eigen"
#include "gsl/gsl_poly.h"
#include "gsl/gsl_multiroots.h"
#include <numbers>
#include <functional>
#include <iostream>
using namespace Eigen;
using namespace std;

constexpr float G = 9.8066f;
constexpr float TOLERANCE = 1.f;
constexpr float HORIZONAL_EPS = 1e-05f;
constexpr float STEP_SIZE = 0.05f;
constexpr float DEGREE = std::numbers::pi_v<float> / 180.f;

const Vector3f G_ACC(0, 0, -G);


inline float AngleBetween(Vector3f v1, Vector3f v2)
{
	v1.normalize();
	v2.normalize();
	return acosf(clamp(v1.dot(v2), -1.0f, 1.0f));
}

struct IntegrationResult
{
	Vector3f LastPos;
	float T;
};

struct CostFunctor
{
	explicit CostFunctor(const function<double(double, double)> from): From(move(from))
	{
	}

	bool operator()(const double* const x, double* residual) const
	{
		cout << From(x[0], x[1]) << endl;
		residual[0] = From(x[0], x[1]);
		return true;
	}

private:
	function<double(double, double)> From;
};

inline void PrintState(size_t iter, gsl_multiroot_fsolver* s)
{
	printf("iter = %3u x = % .3f % .3f "
	       "f(x) = % .3e % .3e\n",
	       iter,
	       gsl_vector_get(s->x, 0),
	       gsl_vector_get(s->x, 1),
	       gsl_vector_get(s->f, 0),
	       gsl_vector_get(s->f, 1));
}

class BallisticsSolver
{
	Vector3f TargetPos;
	Vector3f TargetVelocity;
	Vector3f TargetAcceleration;
	float InitSpeed;
	float AirFriction;

	Vector3f TargetPosF(float t) const;

	Vector3f HVToVelocity(float horizonal, float vertical) const;

	IntegrationResult Integrate(const float horizonal, const float vertical) const
	{
		if (isnan(horizonal) || isnan(vertical) || horizonal < -90 * DEGREE || horizonal > 90 * DEGREE || vertical < -90 * DEGREE || vertical > 90 * DEGREE)
		{
			throw exception("FUCK YOUR ");
		}
		Vector3f projPos(0, 0, 0);
		Vector3f projVel = HVToVelocity(horizonal, vertical);
		float t = 0;
		while (true)
		{
			const Vector3f lastPos = projPos;
			const float lastT = t;

			Vector3f projAcc = AirFriction * projVel.array().pow(2).matrix() + G_ACC;
			projVel += projAcc * STEP_SIZE;
			projPos += projVel * STEP_SIZE;
			t += STEP_SIZE;

			Vector3f targetPos = TargetPosF(t);
			if (const float angle = 90 * DEGREE - AngleBetween(targetPos - projPos, projVel); angle < 0)
			{
				const float tInterpolate = STEP_SIZE * (projPos - targetPos).norm() / (projPos - lastPos).norm();
				const Vector3f interpolatedPos = projPos + projVel * tInterpolate;
				const float interpolatedT = lastT + tInterpolate;
				return IntegrationResult{interpolatedPos, interpolatedT};
			}
		}
	}

	Vector2f Loss(const float horizonal, const float vertical) const
	{
		auto [LastPos, T] = Integrate(horizonal, vertical);
		const Vector3f lastTargetPos = TargetPosF(T);
		cout << (LastPos - lastTargetPos).head(2).norm() << endl << LastPos.z() - lastTargetPos.z() << endl;
		return Vector2f(
			(LastPos - lastTargetPos).head(2).norm(),
			LastPos.z() - lastTargetPos.z()
		);
	}

public:
	BallisticsSolver(
		// TARGET
		Vector3f targetPos,
		Vector3f targetVelocity,
		Vector3f targetAcceleration,
		// PROJECTILE
		const float initSpeed,
		const float airFriction) :
		TargetPos(move(targetPos)),
		TargetVelocity(move(targetVelocity)),
		TargetAcceleration(move(targetAcceleration)),
		InitSpeed(initSpeed),
		AirFriction(airFriction)
	{
	}

	void operator()() const
	{
		int status;
		size_t iter = 0;

		auto gslAdapter = [](const gsl_vector* x, void* params, gsl_vector* f) -> int
		{
			const BallisticsSolver* solver = static_cast<BallisticsSolver*>(params);
			Vector2f loss = solver->Loss(gsl_vector_get(x, 0), gsl_vector_get(x, 1));
			gsl_vector_set(f, 0, loss[0]);
			gsl_vector_set(f, 1, loss[1]);
			return GSL_SUCCESS;
		};

		void* pThis = const_cast<BallisticsSolver*>(this);
		gsl_multiroot_function function = {gslAdapter, 2, pThis};

		constexpr double init[2] = {0, 0.1};
		gsl_vector* initX = gsl_vector_alloc(2);
		gsl_vector_set(initX, 0, init[0]);
		gsl_vector_set(initX, 1, init[1]);

		const gsl_multiroot_fsolver_type* solverType = gsl_multiroot_fsolver_broyden;
		gsl_multiroot_fsolver* solver = gsl_multiroot_fsolver_alloc(solverType, 2);
		gsl_multiroot_fsolver_set(solver, &function, initX);

		PrintState(iter, solver);

		do
		{
			iter++;
			status = gsl_multiroot_fsolver_iterate(solver);

			PrintState(iter, solver);

			if (status) /* check if solver is stuck */
				break;

			status =
				gsl_multiroot_test_residual(solver->f, 1e-7);
		}
		while (status == GSL_CONTINUE && iter < 1000);

		printf("status = %s\n", gsl_strerror(status));

		gsl_multiroot_fsolver_free(solver);
		gsl_vector_free(initX);
	}
};

inline Vector3f BallisticsSolver::TargetPosF(const float t) const
{
	return TargetPos + TargetVelocity * t + (TargetAcceleration * t).array().pow(2).matrix() / 2;
}

inline Vector3f BallisticsSolver::HVToVelocity(const float horizonal, const float vertical) const
{
	return Vector3f(
		sin(horizonal) * cos(vertical),
		cos(horizonal) * cos(vertical),
		sin(vertical)
	) * InitSpeed;
}
