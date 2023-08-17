/*
 * @Author: Ang.Lee.
 * @Date: 2023-08-15 16:54:57
 * @LastEditTime: 2023-08-16 16:20:02
 * @LastEditors: Ang.Lee.
 * @Description: 
 * 
 */
#pragma once
#include <vector>
#include<stdio.h>
#include "points_map_2d.h"

class IterativeClosestPoint
{
public:

	struct MatchPair
	{
		int32_t this_id;
		int32_t other_id;
		int dist_sq;
	};

	IterativeClosestPoint();
	~IterativeClosestPoint();

	bool Align(PointsMap2D * this_map, const PointDataFrame * new_frame, Pose2D * robot_pose_in, Pose2D * slam_pose_out, int32_t * goodness, int32_t * total_goodness);

	
private:
	float *mObsPointsX, *mObsPointsY;
	float *mNewObsPointsX, *mNewObsPointsY;
	MatchPair *mMatchPairs;
	int32_t mObsPointsCount;

	//Options
	uint32_t mOpDecimateOtherMapPoints;
	float mOpALFA;
	float mOpMinAbsTransStep, mOpMinAbsRotStep;
	float mOpSmallestThresholdDist;
	int32_t mOpMaxIterations;

	//Params
	uint32_t mOffsetOtherMapPoints;
	float mThresholdAng, mThresholdDist;
	float mMaxAngularDistForCorrespondence, mMaxDistForCorrespondence;
	float mAngularDistPivotX, mAngularDistPivotY;

	int32_t DetermineMatching(PointsMap2D *this_map, float *other_x, float *other_y, int32_t points_count, Pose2D * other_pose, MatchPair *pairs, float* avg_dist_sq=NULL);

	bool LeastSquareErrorRigidTransformation(PointsMap2D *this_map, MatchPair *pairs, int32_t match_count, Pose2D *pose);
};

