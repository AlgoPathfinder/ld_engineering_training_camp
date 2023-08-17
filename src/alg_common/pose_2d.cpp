#include "pose_2d.h"
#include "lidar_data_common.h"


inline void  Pose2D::UpdateSinCosCache() const {

	mCosPhi = cos(mPhi);
	mSinPhi = sin(mPhi);

	//m_GetSinCos(mSinPhi, mCosPhi, mPhi);
	mCosSinUpdated = true;
}

Pose2D::Pose2D(const double x, const double y, const double phi)
{
	X = x;
	Y = y;
	SetPhi(phi);
	mCosSinUpdated = false;
}

Pose2D Pose2D::operator+(const Pose2D & b) const
{
	UpdateSinCosCache();
	return Pose2D(
		X + b.X * mCosPhi - b.Y * mSinPhi,
		Y + b.X * mSinPhi + b.Y * mCosPhi,
		mPhi + b.mPhi);
}

Pose2D Pose2D::operator-(const Pose2D & b) const
{
	Pose2D ret;
	ret.InverseComposeFrom(*this, b);
	return ret;
	//Pose2D b_;
	//b_.InverseComposeFrom(Pose2D(0,0,0),b);
	//return *this + b_;
}

Pose2D Pose2D::operator-()
{	
	Pose2D b_;
	b_.InverseComposeFrom(Pose2D(0,0,0),*this);
	return b_;
}

void Pose2D::operator*=(const double s)
{
	X *= s;
	Y *= s;
	mPhi *= s;
	NormalizePhi();
	mCosSinUpdated = false;
}

void Pose2D::SetPhi(double phi)
{
	mPhi = phi;
	NormalizePhi();
	mCosSinUpdated = false;
}

void Pose2D::NormalizePhi()
{
	mPhi = wrapToPi(mPhi);
}

double Pose2D::LineDistanceSq(const Pose2D * pose)
{
	double dx = X - pose->X;
	double dy = Y - pose->Y;
	return dx * dx + dy * dy;
}

double Pose2D::AngleDifference(const Pose2D * pose)
{
	double v = mPhi - pose->mPhi;
	return fabs(wrapToPi(v));
}

void Pose2D::Reset(double x, double y, double phi)
{
	X = x;
	Y = y;
	mPhi = phi;
}

float Pose2D::GetDistance(Pose2D point)
{
	return sqrt((X - point.X)*(X - point.X) + (Y - point.Y)*(Y - point.Y));	
}

void Pose2D::ComposeFrom(const Pose2D & A, const Pose2D & B)
{
	A.UpdateSinCosCache();

	// Use temporary variables for the cases (A==this) or (B==this)
	const double new_x = A.X + B.X * A.mCosPhi - B.Y * A.mSinPhi;
	const double new_y = A.Y + B.X * A.mSinPhi + B.Y * A.mCosPhi;
	X = new_x;
	Y = new_y;
	SetPhi(A.mPhi + B.mPhi);
	NormalizePhi();

}

void Pose2D::InverseComposeFrom(const Pose2D & A, const Pose2D & B)
{
	B.UpdateSinCosCache();

	X = (A.X - B.X) * B.mCosPhi + (A.Y - B.Y) * B.mSinPhi;
	Y = -(A.X - B.X) * B.mSinPhi + (A.Y - B.Y) * B.mCosPhi;
	SetPhi(A.mPhi - B.mPhi);
	NormalizePhi();
}
