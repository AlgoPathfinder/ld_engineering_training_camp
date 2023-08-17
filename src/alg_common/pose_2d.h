#pragma once

#include <inttypes.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

class Pose2D
{
private:
	double mPhi;
	mutable double mSinPhi, mCosPhi;
	mutable bool mCosSinUpdated;

	void UpdateSinCosCache() const;

public:	
	double X, Y;

	Pose2D()
	{
		X = Y = mPhi = 0;
		mSinPhi = 0;
		mCosPhi = 0;
		mCosSinUpdated = false;
	};

	~Pose2D() {};

	Pose2D(const double x, const double y, const double phi);

	Pose2D operator + (const Pose2D& b) const;
	Pose2D operator - (const Pose2D& b) const;
	Pose2D operator-();
	void operator *=(const double  s);

	void SetPhi(double phi);
	inline double Phi() const { return mPhi; }

	inline double Normal() const {return sqrt(X * X + Y * Y);}

	void NormalizePhi();

	double LineDistanceSq(const Pose2D *pose);
	double AngleDifference(const Pose2D *pose);

	void Reset(double x = 0.0f, double y = 0.0, double phi = 0.0f);
	float GetDistance(Pose2D point);
	void ComposeFrom(const Pose2D &A, const Pose2D &B);
	void InverseComposeFrom(const Pose2D& A, const Pose2D& B);


};

struct  Pose2DInt
{
	Pose2DInt() : x(0), y(0), phi(0) { }

	int32_t	x, y, phi; //!< Bin indices
	
	struct  lt_operator
	{
		inline bool operator()(const Pose2DInt& s1, const Pose2DInt& s2) const
		{
			if (s1.x < s2.x)  return true;
			if (s1.x > s2.x)  return false;
			if (s1.y < s2.y)  return true;
			if (s1.y > s2.y)  return false;
			return s1.phi < s2.phi;
		}
	};
};