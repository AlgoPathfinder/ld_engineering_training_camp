#pragma once

#include <inttypes.h>
#include <math.h>
#include <limits>
#include <utility>
#include <fstream>
#include <cfloat>

#include "pose_2d.h"
#include "rtree.h"
#include "lidar_data_common.h"

#define MAX_POINTS_COUNT_IN_MAP (2000000)

typedef  int  PointsType; 

class PointsMap2D
{
public:
	PointsMap2D();
	PointsMap2D(PointsType *x, PointsType *y, int32_t count);
	~PointsMap2D();

	bool RebuildRTree(bool* IsBreak);

	inline const PointsType *GetXPtr() { return mX; }
	inline const PointsType *GetYPtr() { return mY; }
	inline int32_t GetPointsCount() { return mPointsCount; }
	bool InsertObservation(Pose2D *pose, const PointDataFrame *insert_frame);
	bool GetPoint(int32_t index, Pose2D *pose);

	void GetBoundingBox(float *min_x, float *max_x, float *min_y, float *max_y);
	void GetBoundingBox(float *max_min_buf);

	void UpdateBoundingBox();
	void UpdatePoints(PointsType *x, PointsType *y, int32_t count);

	PointsType Nearest(PointsType *pos_in, PointsType *pos_out, int32_t *index);
	PointsType Nearest(PointsType x, PointsType y, PointsType *pos_out, int32_t *index);

	int RTreeNearest(int *pos_in, int *pos_out, int32_t *index);

	//void GetTMapData(TMapData * tMapData, GridMap2D * gridMap2D);

	// // for KD Tree Method
	// inline unsigned int kdtree_get_point_count() const { return mPointsCount; }
	// template <typename KDBOX>
	// inline bool kdtree_get_bbox(KDBOX &bb) const
	// {
	// 	bb[0].low = mMinX;
	// 	bb[0].high = mMaxX;
	// 	bb[1].low = mMinY;
	// 	bb[1].high = mMaxY;
	// 	return true;
	// }
	// /// Returns the dim'th component of the idx'th point in the class:
	// inline float kdtree_get_pt(const unsigned int idx, int dim) const {
	// 	if (dim == 0) return this->mX[idx];
	// 	else if (dim == 1) return this->mY[idx];
	// 	else return 0;
	// }
	// /// Returns the distance between the vector "p1[0:size-1]" and the data point with index "idx_p2" stored in the class:
	// inline float kdtree_distance(const PointsType *p1, const unsigned int idx_p2, unsigned int size) const
	// {
	// 	const PointsType d0 = p1[0] - mX[idx_p2];
	// 	const PointsType d1 = p1[1] - mY[idx_p2];
	// 	return d0*d0 + d1*d1;
	// }

private:
	bool Resize(int32_t new_size);
	//int32_t fuseWith(Pose2D* oterMapPose, PointsType *other_x, PointsType *other_y, int32_t count);
	int32_t RTreefuseWith(Pose2D* oterMapPose, PointsType * other_x, PointsType * other_y, int32_t count);
private:
	int32_t mPointsCount;
	int32_t mMaxPointsCount;
	float mMaxX, mMinX, mMaxY, mMinY;
	PointsType mOpMinSquareDistBetweenPoints;
	bool mFuseWithOtherMap;
	bool mBoundingBoxIsUpdated;
	PointsType *mX, *mY;
	rbox::RTree2i *tree;
	//CriticalSection mCs;
};

