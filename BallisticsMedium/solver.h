#include "Eigen/Eigen"
#include "ceres/problem.h"
#include "ceres/numeric_diff_cost_function.h"
#include "ceres/solver.h"
#include <numbers>
#include <functional>
#include <iostream>
using namespace ceres;
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

	float Loss(const float horizonal, const float vertical) const
	{
		auto [LastPos, T] = Integrate(horizonal, vertical);
		const Vector3f lastTargetPos = TargetPosF(T);
		return (LastPos - lastTargetPos).norm();
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
		Problem problem;
		double x0[] = {0,0};
		double* parameterBlocks[] = {x0};
		auto adapter = [this](double h, double v) -> double { return Loss(h, v); };
		problem.AddResidualBlock(
			new NumericDiffCostFunction<CostFunctor, CENTRAL, 1, 2>(new CostFunctor(adapter)),
			nullptr, parameterBlocks, 1
		);
		Solver::Options options;
		options.minimizer_type = LINE_SEARCH;
		options.use_explicit_schur_complement = true;
		options.linear_solver_type = DENSE_QR;
		options.minimizer_progress_to_stdout = true;
		Solver::Summary summary;

		ceres::Solve(options, &problem, &summary);

		std::cout << summary.FullReport() << endl;
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
