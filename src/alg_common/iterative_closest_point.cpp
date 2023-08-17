#include "iterative_closest_point.h"

using namespace std;

#ifndef FLT_MAX
#define FLT_MAX          3.402823466e+38F        // max value
#endif

IterativeClosestPoint::IterativeClosestPoint()
{
	mOffsetOtherMapPoints = 0;	
	mThresholdDist = 0.3f;
	mThresholdAng = (float)DEG2RAD(9);

	mOpDecimateOtherMapPoints = 10;
	mOpALFA = 0.8f;
	mOpMinAbsTransStep = 1e-6f;
	mOpMinAbsRotStep = 1e-6f;
	mOpSmallestThresholdDist = 0.05f;
	mOpMaxIterations = 80;

	mObsPointsX = NULL;
	mObsPointsY = NULL;
	mNewObsPointsX = NULL;
	mNewObsPointsY = NULL;
	mMatchPairs = NULL;
	mObsPointsCount = 0;
}

IterativeClosestPoint::~IterativeClosestPoint()
{
	if (mObsPointsX) delete[] mObsPointsX;
	if (mObsPointsY) delete[] mObsPointsY;
	if (mMatchPairs) delete[] mMatchPairs;
	if (mNewObsPointsX) delete[] mNewObsPointsX;
	if (mNewObsPointsY) delete[] mNewObsPointsY;
}

bool IterativeClosestPoint::Align(PointsMap2D *this_map, const PointDataFrame *new_frame,
	Pose2D *robot_pose_in, Pose2D *slam_pose_out, int32_t *goodness /*= NULL*/, int32_t *total_goodness)
{
	//static TimeInterval tictac;
	//tictac.Tic();

	 

	int32_t scan_size = new_frame->data.size();
	int32_t match_count = 0;
	bool keep_approaching = false;

	if (mObsPointsCount < scan_size) {
		if (mObsPointsX) delete[] mObsPointsX;
		if (mObsPointsY) delete[] mObsPointsY;
		if (mMatchPairs) delete[] mMatchPairs;
		if (mNewObsPointsX) delete[] mNewObsPointsX;
		if (mNewObsPointsY) delete[] mNewObsPointsY;

		mNewObsPointsX = new float[scan_size];
		mNewObsPointsY = new float[scan_size];
		mObsPointsX = new float[scan_size];
		mObsPointsY = new float[scan_size];

		mMatchPairs = new MatchPair[scan_size];		
		mObsPointsCount = scan_size;			
	}


	//int points_count = obs->RangeToXYPointsWithFilter(NULL, mObsPointsX, mObsPointsY, NULL, 0.02f * 0.02f);
	int points_count=scan_size;
	
	for (size_t i = 0; i < new_frame->data.size(); i++)
	{
		mObsPointsX[i] = new_frame->data[i].x;
		mObsPointsY[i] = new_frame->data[i].y;
	}


	Pose2D mean = *robot_pose_in;
	Pose2D last_mean = mean;
	int32_t iter_count = 0;
	//int32_t incount = 0;
	mOffsetOtherMapPoints = 0;
	mMaxAngularDistForCorrespondence = mThresholdAng;
	mMaxDistForCorrespondence = mThresholdDist;

	if (points_count)
	{
		mOpDecimateOtherMapPoints = points_count / 25;
	}
	if (mOpDecimateOtherMapPoints < 2)
	{
		mOpDecimateOtherMapPoints = 2;
	}
	 
	//printf("AAAA time= %lld ms\n", tictac.Tac() / 1000);
	float avg_dis_sq = .0f;
	do {
		keep_approaching = false;
		mAngularDistPivotX = (float)mean.X; mAngularDistPivotY = (float)mean.Y;
		match_count = DetermineMatching(this_map, mObsPointsX, mObsPointsY, points_count, &mean, mMatchPairs,&avg_dis_sq);
		 
		//printf("DetermineMatching time= %lld ms\n", tictac.Tac() / 1000);

		//printf("match_count = %d, mean = (%.2f, %.2f, %.2f)\n",
		//	match_count, mean.X, mean.Y,mean.Phi());
		//if (match_count>10)
		//{
		//	printf("******************************************");
		//}
		if (match_count) {
			LeastSquareErrorRigidTransformation(this_map, mMatchPairs, match_count, &mean);

			//printf("LeastSquareErrorRigidTransformation time= %lld ms\n", tictac.Tac() / 1000);

			keep_approaching = true;

			if (!(fabs(last_mean.X - mean.X) > mOpMinAbsTransStep ||
				fabs(last_mean.Y - mean.Y) > mOpMinAbsTransStep ||
				fabs(wrapToPi(last_mean.Phi() - mean.Phi())) > mOpMinAbsRotStep)) {

				mMaxAngularDistForCorrespondence *= mOpALFA;
				mMaxDistForCorrespondence *= mOpALFA;

				if (mMaxDistForCorrespondence < mOpSmallestThresholdDist)
					keep_approaching = false;

				if (++mOffsetOtherMapPoints >= mOpDecimateOtherMapPoints)
					mOffsetOtherMapPoints = 0;
				
			}
			
			last_mean = mean;
			iter_count++;
			
			if (iter_count >= mOpMaxIterations && mMaxDistForCorrespondence > mOpSmallestThresholdDist)
				mMaxDistForCorrespondence *= mOpALFA;
		}		
	} while ((keep_approaching && iter_count < mOpMaxIterations) || 
		(iter_count >= mOpMaxIterations && mMaxDistForCorrespondence > mOpSmallestThresholdDist));

	if(slam_pose_out) *slam_pose_out = mean;

	//LOGD("IterativeClosestPoint::Align():goodness = " << (mOpDecimateOtherMapPoints * match_count) * 100 / points_count);

	uint32_t current_mOpDecimateOtherMapPoints = mOpDecimateOtherMapPoints;
	mOpDecimateOtherMapPoints = 2;
	match_count = DetermineMatching(this_map, mObsPointsX, mObsPointsY, points_count, &mean, mMatchPairs, &avg_dis_sq);
	 
	//MatchPair *p = mMatchPairs;

	float TotalGoodness = 0;
	int last_id = mMatchPairs->other_id;
	float str_val = 1;

	for (int i = 1; i < match_count; i++)
	{
		MatchPair *p = mMatchPairs + i;
		if (p->other_id - last_id <= 4)
		{
			if (str_val < 3325)
			{
				str_val *= 1.5;
			}
			else
			{
				TotalGoodness += str_val;
				str_val = 1;
			}
		}
		else
		{
			TotalGoodness += str_val;
			str_val = 1;
		}
		last_id = p->other_id;
		//printf("%d ", p->other_id);
	}
	if (str_val != 1)
	{
		TotalGoodness += str_val;
	}


	//LOGD("IterativeClosestPoint::Align():goodness = " << (mOpDecimateOtherMapPoints * match_count) * 100 / points_count<<", avg_dis_sq = "<< avg_dis_sq);

	if (points_count==0)
	{
		if (goodness) *goodness = 0;
	}
	//else if (avg_dis_sq>1600)
	//{
	//	if (goodness) *goodness = 0;
	//	LOGD("IterativeClosestPoint::Align():goodness = " << (mOpDecimateOtherMapPoints * match_count) * 100 / points_count<<", avg_dis_sq = "<< avg_dis_sq);
	//}
	else
	{
		if(goodness) *goodness = (mOpDecimateOtherMapPoints * match_count) * 100 / points_count;
	}

	mOpDecimateOtherMapPoints = current_mOpDecimateOtherMapPoints;

	//LA_PRINT("match_count= %d   points_count= %d   TotalGoodness= %f   goodness= %d   avg_dis_sq= %f   \n", match_count, points_count, TotalGoodness, *goodness, avg_dis_sq);

	//printf("Align time= %lld ms\n", tictac.Tac() / 1000);
	/*printf("goodness = %3d, iter_count = %3d,  slam_pose(%6.2f, %6.2f, %6.2f)\n",
		 *goodness, iter_count,(float)mean.X, (float)mean.Y, (float)mean.Phi());*/
	*total_goodness = (int)TotalGoodness;
	 
	return true;
}

int32_t IterativeClosestPoint::DetermineMatching(PointsMap2D *this_map, float *other_x, float *other_y, 
	int32_t points_count, Pose2D * other_pose, MatchPair *pairs, float* avg_dist_sq)
{
	int32_t match_count = 0;
	int sum_dist_sq = 0;
	//max_x, min_x, max_y, min_y
	float map_bound[4], obs_bound[4] = {-FLT_MAX, FLT_MAX, -FLT_MAX, FLT_MAX};

	if (!this_map->GetPointsCount() || !points_count)
		return 0;  // No

	//const double sin_phi = sin(other_pose->Phi());
	//const double cos_phi = cos(other_pose->Phi());
	Pose2D local_pose;
	

	float *x_locals = mNewObsPointsX;
	float *y_locals = mNewObsPointsY;

	const double sin_phi = sin(other_pose->Phi());
	const double cos_phi = cos(other_pose->Phi());

	// float sin_phi;
	// float cos_phi;
	// m_GetSinCos(sin_phi, cos_phi, other_pose->Phi());

	for (int32_t i = 0; i < points_count; i++) {
		float x_value = other_pose->X + cos_phi*other_x[i] - sin_phi*other_y[i];
		if (obs_bound[1] > x_value)obs_bound[1] = x_value;
		if (obs_bound[0] < x_value)obs_bound[0] = x_value;
		x_locals[i] = x_value;

		float y_value = other_pose->Y + sin_phi*other_x[i] + cos_phi*other_y[i];
		if (obs_bound[3] > y_value)obs_bound[3] = y_value;
		if (obs_bound[2] < y_value)obs_bound[2] = y_value;
		y_locals[i] = y_value;
	}

	this_map->GetBoundingBox(map_bound);

	if (obs_bound[1] > map_bound[0] ||
		obs_bound[0] < map_bound[1] ||
		obs_bound[3] > map_bound[2] ||
		obs_bound[2] < map_bound[3])
		return 0;

	//const float *this_px = this_map->GetXPtr();
	//const float *this_py = this_map->GetYPtr();

	//static TimeInterval tictac;
	//tictac.Tic();

	int mAngularDistPivotX_INT = mAngularDistPivotX * 1000;
	int mAngularDistPivotY_INT = mAngularDistPivotY * 1000;
	int mMaxDistForCorrespondence_INT = mMaxDistForCorrespondence * 1000;

	int itTime = 0;

	for (int32_t i = mOffsetOtherMapPoints; i < points_count; i += mOpDecimateOtherMapPoints) {
		// For speed-up:
		int x_local = x_locals[i] * 1000; // *x_locals_it;
		int y_local = y_locals[i] * 1000; // *y_locals_it;
		//if (x_local > -7.77 && x_local<-7.76 && y_local>-0.14&&y_local < -0.13)
		//{
		//	int test = 0;
		//}
		int32_t this_i;
		int pos_in[2] = { x_local , y_local };
		//float match_dist_sq = this_map->Nearest(pos_in, NULL, &this_i);
		int match_dist_sq = this_map->RTreeNearest(pos_in, NULL, &this_i);

		//cout << mMaxAngularDistForCorrespondence << "   " << mAngularDistPivotX - x_local << "   " << mAngularDistPivotY - y_local << "   " << mMaxDistForCorrespondence << endl;
		
		int dist = (int)(mMaxAngularDistForCorrespondence *
			sqrt((mAngularDistPivotX_INT - x_local)*(mAngularDistPivotX_INT - x_local) + (mAngularDistPivotY_INT - y_local)*(mAngularDistPivotY_INT - y_local))) +
			mMaxDistForCorrespondence_INT;

		int dist_sq = dist*dist;

		if (match_dist_sq< dist_sq) {
			// Save all the correspondences:
			MatchPair *p = pairs + match_count;

			p->this_id = this_i;
			p->other_id = i;
			p->dist_sq = match_dist_sq;

			match_count++;
			sum_dist_sq += match_dist_sq;
		}
		itTime++;
	}

	//printf("itTime= %d\n", itTime);

	//printf("timeAll = %lld ms\n", tictac.Tac()/1000);
	sum_dist_sq = match_count ? sum_dist_sq / match_count : 0;
	if (avg_dist_sq!=NULL)
		*avg_dist_sq = sum_dist_sq;

	return match_count;
}

bool IterativeClosestPoint::LeastSquareErrorRigidTransformation(PointsMap2D *this_map, MatchPair * pairs, int32_t match_count, Pose2D * pose)
{
	const size_t N = match_count;
	if (N < 2) return false;
	const float N_inv = 1.0f / N;  // For efficiency, keep this value.

	int SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
	float Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;

	const PointsType *this_px = this_map->GetXPtr();
	const PointsType *this_py = this_map->GetYPtr();
	for (int32_t i = 0; i < match_count; i++) {
		MatchPair *corrIt = pairs + i;
		// Get the pair of points in the correspondence:
		int32_t this_i = corrIt->this_id;
		int32_t other_i = corrIt->other_id;
		const int xa = this_px[this_i];
		const int ya = this_py[this_i];
		const int xb = mObsPointsX[other_i] * 1000;
		const int yb = mObsPointsY[other_i] * 1000;

		// Compute the terms:
		SumXa += xa;
		SumYa += ya;

		SumXb += xb;
		SumYb += yb;

		Sxx += xa * xb;
		Sxy += xa * yb;
		Syx += ya * xb;
		Syy += ya * yb;

	}	// End of "for all correspondences"...

	const float	SumXa_f = SumXa;
	const float	SumYa_f = SumYa;
	const float	SumXb_f = SumXb;
	const float	SumYb_f = SumYb;

	const float	mean_x_a = SumXa_f * N_inv;
	const float	mean_y_a = SumYa_f * N_inv;
	const float	mean_x_b = SumXb_f * N_inv;
	const float	mean_y_b = SumYb_f * N_inv;
	const float	N_f = N;

	// Auxiliary variables Ax,Ay:
	const float Ax = N_f*(Sxx + Syy) - SumXa_f*SumXb_f - SumYa_f*SumYb_f;
	const float Ay = SumXa_f * SumYb_f + N_f*(Syx - Sxy) - SumXb_f * SumYa_f;

	//printf("%d   %d   %d   %d   %f   %f   %f   %f\n", SumXa, SumXb, SumYa, SumYb, N*(Sxx + Syy), Ax, Ay, (Ax != 0 || Ay != 0) ? atan2(Ay, Ax) : 0);

	pose->SetPhi((Ax != 0 || Ay != 0) ? atan2(Ay, Ax) : 0);

	// float ccos;
	// float csin;
	// m_GetSinCos(csin, ccos, (float)pose->Phi());

	const float ccos = (float)cos(pose->Phi());
	const float csin = (float)sin(pose->Phi());

	pose->X = (mean_x_a - mean_x_b * ccos + mean_y_b * csin) / 1000;
	pose->Y = (mean_y_a - mean_x_b * csin - mean_y_b * ccos) / 1000;

	return true;
}


//bool IterativeClosestPoint::LeastSquareErrorRigidTransformation(PointsMap2D *this_map, MatchPair * pairs, int32_t match_count, Pose2D * pose)
//{
//	const size_t N = match_count;
//	if (N < 2) return false;
//	const float N_inv = 1.0f / N;  // For efficiency, keep this value.
//	float SumXa = 0, SumXb = 0, SumYa = 0, SumYb = 0;
//	float Sxx = 0, Sxy = 0, Syx = 0, Syy = 0;
//
//	const PointsType *this_px = this_map->GetXPtr();
//	const PointsType *this_py = this_map->GetYPtr();
//	for (int32_t i = 0; i < match_count; i++) {
//		MatchPair *corrIt = pairs + i;
//		// Get the pair of points in the correspondence:
//		int32_t this_i = corrIt->this_id;
//		int32_t other_i = corrIt->other_id;
//		const float xa = this_px[this_i] / 1000.0;
//		const float ya = this_py[this_i] / 1000.0;
//		const float xb = mObsPointsX[other_i];
//		const float yb = mObsPointsY[other_i];
//
//		// Compute the terms:
//		SumXa += xa;
//		SumYa += ya;
//
//		SumXb += xb;
//		SumYb += yb;
//
//		Sxx += xa * xb;
//		Sxy += xa * yb;
//		Syx += ya * xb;
//		Syy += ya * yb;
//	}	// End of "for all correspondences"...
//
//	const float	mean_x_a = SumXa * N_inv;
//	const float	mean_y_a = SumYa * N_inv;
//	const float	mean_x_b = SumXb * N_inv;
//	const float	mean_y_b = SumYb * N_inv;
//
//	// Auxiliary variables Ax,Ay:
//	const float Ax = N*(Sxx + Syy) - SumXa*SumXb - SumYa*SumYb;
//	const float Ay = SumXa * SumYb + N*(Syx - Sxy) - SumXb * SumYa;
//
//	printf("-----%f-----%f-----\n", Ax, Ay);
//
//	pose->SetPhi((Ax != 0 || Ay != 0) ? atan2(Ay, Ax) : 0);
//
//	float ccos;
//	float csin;
//	m_GetSinCos(csin, ccos, (float)pose->Phi());
//
//	//const float ccos = (float)cos(pose->Phi());
//	//const float csin = (float)sin(pose->Phi());
//
//	pose->X = (mean_x_a - mean_x_b * ccos + mean_y_b * csin);
//	pose->Y = (mean_y_a - mean_x_b * csin - mean_y_b * ccos);
//
//	return true;
//}
