#include "points_map_2d.h"


using namespace std;

PointsMap2D::PointsMap2D()
{
	mOpMinSquareDistBetweenPoints = 0.05f * 0.05f;
	mPointsCount = 0;
	mMaxPointsCount = 1000000;
	mX = new PointsType[mMaxPointsCount];
	mY = new PointsType[mMaxPointsCount];

	mFuseWithOtherMap = true;
	mBoundingBoxIsUpdated = false;

	mMaxY = mMaxX = FLT_MIN;
	mMinY = mMinX = FLT_MAX;
	
	tree = new rbox::RTree2i;

}

PointsMap2D::PointsMap2D(PointsType * x, PointsType * y, int32_t count)
{
	mOpMinSquareDistBetweenPoints = 0.05f * 0.05f;
	mPointsCount = 0;
	mMaxPointsCount = count;
	mX = new PointsType[mMaxPointsCount];
	mY = new PointsType[mMaxPointsCount];

	memcpy(mX, x, count * sizeof(PointsType));
	memcpy(mY, y, count * sizeof(PointsType));
	mPointsCount = count;

	mFuseWithOtherMap = true;

	tree = new rbox::RTree2i;

	UpdateBoundingBox();
	
	//kdtree_mark_as_outdated();
}

PointsMap2D::~PointsMap2D()
{
	if (mX) delete[] mX;
	if (mY) delete[] mY;
	if (tree)
	{
		delete tree;
	}
}

bool PointsMap2D::Resize(int32_t new_size)
{
	if (new_size > mMaxPointsCount) {
		new_size += 10000;
		if (new_size > MAX_POINTS_COUNT_IN_MAP)
			return false;

		PointsType *new_x = new PointsType[new_size];
		PointsType *new_y = new PointsType[new_size];
		memcpy(new_x, mX, mMaxPointsCount * sizeof(PointsType));
		memcpy(new_y, mY, mMaxPointsCount * sizeof(PointsType));
		delete[] mX;
		delete[] mY;
		mX = new_x;
		mY = new_y;
		mMaxPointsCount = new_size;
	}

	return true;
}

int32_t PointsMap2D::RTreefuseWith(Pose2D* oterMapPose, PointsType * other_x, PointsType * other_y, int32_t count)
{
	if (mPointsCount == 0)
	{
		int XYtmp[2];
		for (int i = 0; i < count; i++)
		{
			XYtmp[0] = other_x[i];
			XYtmp[1] = other_y[i];
			tree->Insert(XYtmp, XYtmp, mPointsCount + i);
		}
		return count;
	}

	PointsType        x_local, y_local;
	unsigned int nMapPointsWithCorrespondence = 0;	// Number of points with one corrs. at least

	const float mMaxAngularDistForCorrespondence = 0.0f;
	const float mMaxDistForCorrespondence = 0.02f;

	PointsType * x_locals = other_x;
	PointsType * y_locals = other_y;
	struct MatchPair
	{
		int32_t this_id;
		int32_t other_id;
		double dist_sq;
	};

	int mAngularDistPivotX_INT = oterMapPose->X * 1000;
	int mAngularDistPivotY_INT = oterMapPose->Y * 1000;
	int mMaxDistForCorrespondence_INT = mMaxDistForCorrespondence * 1000;

	for (int localIdx = 0; localIdx<count; localIdx++)
	{
		x_local = x_locals[localIdx]; // *x_locals_it;
		y_local = y_locals[localIdx]; // *y_locals_it;

		int32_t this_i;
		int pos_in[2] = { x_local, y_local };
		int match_dist_sq = RTreeNearest(pos_in, NULL, &this_i);

		int dist = (int)(mMaxAngularDistForCorrespondence *
			sqrt((mAngularDistPivotX_INT - x_local)*(mAngularDistPivotX_INT - x_local) + (mAngularDistPivotY_INT - y_local)*(mAngularDistPivotY_INT - y_local))) +
			mMaxDistForCorrespondence_INT;

		int dist_sq = dist*dist;

		if (match_dist_sq < dist_sq) {
			// Save all the correspondences:

			int old_pos_in[2] = { mX[this_i],  mY[this_i] };
			int new_pos_in[2] = { (x_local + mX[this_i] * 4) / 5,(y_local + mY[this_i] * 4) / 5 };

			mX[this_i] = new_pos_in[0];
			mY[this_i] = new_pos_in[1];

			tree->Remove(old_pos_in, old_pos_in, this_i);
			tree->Insert(new_pos_in, new_pos_in, this_i);

		}
		else
		{
			mX[mPointsCount + nMapPointsWithCorrespondence] = x_local;
			mY[mPointsCount + nMapPointsWithCorrespondence] = y_local;
			tree->Insert(pos_in, pos_in, mPointsCount + nMapPointsWithCorrespondence);
			nMapPointsWithCorrespondence++;
		}
	}
	return (nMapPointsWithCorrespondence);
}


void PointsMap2D::GetBoundingBox(float *min_x, float *max_x, float *min_y, float *max_y)
{
	*min_x = mMinX;
	*min_y = mMinY;
	*max_x = mMaxX;
	*max_y = mMaxY;
}

void PointsMap2D::GetBoundingBox(float *max_min_buf)
{
	max_min_buf[0] = mMaxX;
	max_min_buf[1] = mMinX;
	max_min_buf[2] = mMaxY;
	max_min_buf[3] = mMinY;
}

void PointsMap2D::UpdateBoundingBox()
{
	if (mPointsCount <= 0)
		return;
	else{
		mMaxX = mMinX = mX[0]/1000.0;
		mMaxY = mMinY = mY[0]/1000.0;
	}

	for (int i = 1; i < mPointsCount; i++) {
		if (mMaxX < mX[i] / 1000.0) mMaxX = mX[i] / 1000.0;
		if (mMinX > mX[i] / 1000.0) mMinX = mX[i] / 1000.0;
		if (mMaxY < mY[i] / 1000.0) mMaxY = mY[i] / 1000.0;
		if (mMinY > mY[i] / 1000.0) mMinY = mY[i] / 1000.0;
	}

	mBoundingBoxIsUpdated = true;
}

void PointsMap2D::UpdatePoints(PointsType * x, PointsType * y, int32_t count)
{
	Resize(count);

	memcpy(mX, x, count * sizeof(PointsType));
	memcpy(mY, y, count * sizeof(PointsType));
	mPointsCount = count;

	UpdateBoundingBox();
}

int PointsMap2D::RTreeNearest(int *pos_in, int *pos_out, int32_t *index)
{
	vector<int> search_out;
	int i;
	tree->KNN(pos_in, 1, search_out);
	i = search_out[0];
	//if (search_out.size() > 0)
	//{
	//	i = search_out[0];
	//}
	//else
	//{
	//	i = 0;
	//}

	if (pos_out) {
		pos_out[0] = mX[i];
		pos_out[1] = mY[i];
	}

	*index = i;

	return((pos_in[0] - mX[i])*(pos_in[0] - mX[i]) + (pos_in[1] - mY[i])*(pos_in[1] - mY[i]));
}

// void PointsMap2D::GetTMapData(TMapData * tMapData,GridMap2D* gridMap2D)
// {
// 	mCs.Enter();
// 	gridMap2D->GetTMapData(tMapData, false);
// 	memset(&tMapData->map[0],255,tMapData->map_param.width*tMapData->map_param.height);
// 	for (int i=0; i<mPointsCount; i++)
// 	{
// 		int x = tMapData->x2idx(mX[i]/1000.0);
// 		int y = tMapData->y2idx(mY[i]/1000.0);
// 		tMapData->map[x + tMapData->map_param.width*y] = 0;
// 	}
// 	mCs.Leave();
// }

bool PointsMap2D::InsertObservation(Pose2D * pose, const PointDataFrame *insert_frame)
{
	const int32_t scan_size = insert_frame->data.size();
	if (!scan_size)
		return true;

	if (!Resize(mPointsCount + scan_size))
		return false;

	float *px = new float[scan_size];
	float *py = new float[scan_size];

	for (size_t i = 0; i < insert_frame->data.size(); i++)
	{

		px[i] = insert_frame->data[i].x * cos(pose->Phi()) - insert_frame->data[i].y * sin(pose->Phi()) + pose->X;
		py[i] = insert_frame->data[i].y * cos(pose->Phi()) + insert_frame->data[i].x * sin(pose->Phi()) + pose->Y;
		mMaxX = max(mMaxX, px[i]);
		mMinX = min(mMinX, px[i]);
		mMaxY = max(mMaxY, py[i]);
		mMinY = min(mMinY, py[i]);
	}

	//int32_t n = obs->RangeToXYPointsWithFilter(pose, px, py,max_min_xy, mOpMinSquareDistBetweenPoints, 1, true);

	PointsType *KDTpx = mX + mPointsCount;
	PointsType *KDTpy = mY + mPointsCount;

	for (int i = 0; i < scan_size; i++)
	{
		if (i < scan_size)
		{
			KDTpx[i] = px[i] * 1000;
			KDTpy[i] = py[i] * 1000;
		}
	}

	//int XYtmp[2];
	//for (int i = 0; i < n; i++)
	//{
	//	XYtmp[0] = KDTpx[i] = px[i] * 1000;
	//	XYtmp[1] = KDTpy[i] = py[i] * 1000;
	//	tree.Insert(XYtmp, XYtmp, mPointsCount + i);
	//}

	/*XYtmp[0] = mX[10];
	XYtmp[1] = mY[10];
	tree.Remove(XYtmp, XYtmp, 10);
*/

	int32_t nNeedInsertedPoints = RTreefuseWith(pose, KDTpx, KDTpy,scan_size);

	delete[] px;
	delete[] py;

	mPointsCount += nNeedInsertedPoints;

	//mPointsCount += n;

	//int32_t nNeedInsertedPoints = fuseWith(pose, KDTpx, KDTpy, n);

	//mPointsCount += nNeedInsertedPoints;
	//
	//if(nNeedInsertedPoints > 0)
	//	kdtree_mark_as_outdated();

	return true;
}

bool PointsMap2D::GetPoint(int32_t index, Pose2D * pose)
{
	if (index < mPointsCount) {
		pose->X = mX[index]/1000.0;
		pose->Y = mY[index]/1000.0;
	}
	
	return false;
}

bool PointsMap2D::RebuildRTree(bool* IsBreak)
{
	if (mPointsCount > 0)
	{
		//LOGD("mPointsCount = "<< mPointsCount);
		rbox::RTree2i *newTree=new rbox::RTree2i;
		 
		int XYtmp[2];
		for (int i = 0; i < mPointsCount; i++)
		{
			 
			XYtmp[0] = mX[i];
			XYtmp[1] = mY[i];
			 
			newTree->Insert(XYtmp, XYtmp, i);
			 
			if (*IsBreak)
			{
				 
				delete newTree;
				 
				return(1);
			}
		}
		 
		delete tree;
		 
		tree = newTree;
	}
	return 0;
}
