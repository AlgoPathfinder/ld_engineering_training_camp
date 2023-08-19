/*
 * @Author: Ang.Lee.
 * @Date: 2023-08-01 16:32:08
 * @LastEditTime: 2023-08-03 22:17:38
 * @LastEditors: Ang.Lee.
 * @Description: 
 * 
 */
#pragma once
#include <iostream>
#include "lidar_data_common.h"
#include "grid_map_2d_with_rtree.h"

class ScanMatcher {

private:
	const int max_direction = 8;
	const double str_xy_res = 0.1;
	const double str_ang_res = 0.1;

public:
	ScanMatcher() {};
	~ScanMatcher() {};

	int MatchProcess(GridMap2DwithRtree& grid_map_with_rtree, PointDataFrame& new_point_frame, double& car_x, double& car_y, double& car_a)
	{
		int step = 1;
		double xy_step = str_xy_res;
		double ang_step = str_ang_res;
		double dst_x = car_x;
		double dst_y = car_y;
		double dst_a = car_a;

		double min_value = 36000000000000;

		while (step < 6)
		{
			double new_x = dst_x;
			double new_y = dst_y;
			double new_a = dst_a;
			int direction = 1;
			while (direction < max_direction)
			{
				switch (direction)
				{
				case 1:
				{
					new_x = dst_x + xy_step;
				}
				break;
				case 2:
				{
					new_x = dst_x - xy_step;
				}
				break;
				case 3:
				{
					new_y = dst_y + xy_step;
				}
				break;
				case 4:
				{
					new_y = dst_y - xy_step;
				}
				break;
				case 5:
				{
					new_a = dst_a + ang_step;
				}
				break;
				case 6:
				{
					new_a = dst_a - ang_step;
				}
				break;
				default:
					break;
				}

				double cost_value = grid_map_with_rtree.CalculatePointDataCost(new_point_frame, new_x, new_y, new_a);
				if (cost_value < min_value)
				{
					dst_x = new_x;
					dst_y = new_y;
					dst_a = new_a;
					min_value = cost_value;
					direction = 1;
				}
				else
				{
					direction++;
				}
			}
			step++;
			xy_step /= 2;
			ang_step /= 2;
		}

		car_x = dst_x;
		car_y = dst_y;
		car_a = dst_a;

		//std::cout << "dx= " << dst_x << "	dy= " << dst_y << "	da= " << dst_a << std::endl;
		//std::cout << "x= " << car_x << "	y= " << car_y << "	a= " << car_a << std::endl;
		return 0;
	}
};