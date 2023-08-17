/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-26 18:03:02
 * @LastEditTime: 2023-08-17 23:13:55
 * @LastEditors: Ang.Lee.
 * @Description: 
 * 
 */
#pragma once

#include <iostream>
#include "lidar_data_common.h"
#include "grid_map_2d.h"
#include "rtree.h"

class GridMap2DwithRtree :public GridMap2D
{
public:

	using GridMap2D::GridMap2D;

	rbox::RTree2i grid_tree;

	int UpdataMap(PointDataFrame data_frame, float x, float y, float r)
	{
		int start_idx_x = x_to_idx(x);
		int start_idx_y = y_to_idy(y);

		if ((start_idx_x < 0) || (start_idx_y < 0) || (start_idx_x >= map_w) || (start_idx_y >= map_h))
		{
			return 1;
		}

		for (size_t i = 0; i < data_frame.data.size(); i++)
		{
			float end_x = data_frame.data[i].x * cos(r) - data_frame.data[i].y * sin(r) + x;
			float end_y = data_frame.data[i].y * cos(r) + data_frame.data[i].x * sin(r) + y;
			int end_idx_x = x_to_idx(end_x);
			int end_idx_y = y_to_idy(end_y);

			if ((end_idx_x < 0) || (end_idx_y < 0) || (end_idx_x >= map_w) || (end_idx_y >= map_h))
			{
				continue;
			}

			double dx = 0;
			double dy = 0;

			double delta_idx_x = end_idx_x - start_idx_x;
			double delta_idx_y = end_idx_y - start_idx_y;

			if (fabs(delta_idx_x) > fabs(delta_idx_y))
			{

				dx = delta_idx_x / fabs(delta_idx_x);
				dy = delta_idx_y / fabs(delta_idx_x);
			}
			else
			{
				if (fabs(delta_idx_y) > 0.1)
				{
					dx = delta_idx_x / fabs(delta_idx_y);
					dy = delta_idx_y / fabs(delta_idx_y);
				}
				else
				{
					std::cout << "impossible!" << std::endl;
					return 1;
				}
			}

			double f_cur_idx_x = start_idx_x;
			double f_cur_idx_y = start_idx_y;

			int cur_idx_x = start_idx_x;
			int cur_idx_y = start_idx_y;

			while ((cur_idx_x != end_idx_x) || (cur_idx_y != end_idx_y))
			{
				//if ((cur_idx_x == end_idx_x) || (cur_idx_y == end_idx_y))
				//{
				//	std::cout << " " << std::endl;
				//}

				int data_index = cur_idx_y * map_w + cur_idx_x;

				if (mapdata[data_index] > 255 - abs(odd_miss))
				{
					mapdata[data_index] = 255;
				}
				else
				{
					if ((mapdata[data_index] < 127)&& (mapdata[data_index] + odd_miss >= 127))
					{
						int delete_idx[2];
						delete_idx[0] = cur_idx_x;
						delete_idx[1] = cur_idx_y;
						grid_tree.Remove(delete_idx, delete_idx, data_index);
					}
					mapdata[data_index] += odd_miss;
				}
				f_cur_idx_x += dx;
				f_cur_idx_y += dy;

				cur_idx_x = round(f_cur_idx_x);
				cur_idx_y = round(f_cur_idx_y);

			}

			int data_index = cur_idx_y * map_w + cur_idx_x;
			//int data_index = end_idx_y * map_w + end_idx_x;

			if (mapdata[data_index] < abs(odd_hit))
			{
				mapdata[data_index] = 0;
			}
			else
			{
				if ((mapdata[data_index] >= 127) && (mapdata[data_index] + odd_hit < 127))
				{
					int insert_idx[2];
					insert_idx[0] = cur_idx_x;
					insert_idx[1] = cur_idx_y;
					grid_tree.Insert(insert_idx, insert_idx, data_index);
				}
				mapdata[data_index] += odd_hit;
			}
		}
		return 0;
	}

	double CalculatePointDataCost(PointDataFrame data_frame, float x, float y, float r)
	{
		double total_value = 0;
		for (size_t i = 0; i < data_frame.data.size(); i++)
		{
			PointXY tran_point;
			tran_point.x = data_frame.data[i].x * cos(r) - data_frame.data[i].y * sin(r) + x;
			tran_point.y = data_frame.data[i].y * cos(r) + data_frame.data[i].x * sin(r) + y;

			int knn_point[2];
			knn_point[0] = x_to_idx(tran_point.x);
			knn_point[1] = y_to_idy(tran_point.y);

			std::vector<int> dis;
			grid_tree.NearstWithDis(knn_point, dis);

			total_value += double(dis[0]) * (log(mapdata[knn_point[1] * map_w + knn_point[0]] + 2));
			//total_value += double(dis[0]* dis[0]);
		}
		return total_value;
	}

};
