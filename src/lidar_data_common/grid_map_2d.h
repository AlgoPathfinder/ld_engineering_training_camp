/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-23 11:09:32
 * @LastEditTime: 2023-08-02 22:18:10
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\lidar_data_common\grid_map_2d.h
 * 
 */
#pragma once

#include<iostream>
#include<cstring>
#include"lidar_data_common.h"

class GridMap2D {

protected:
	uint8_t* mapdata;
	int map_w = 800;
	int map_h = 600;
	float resolution = 0.05;
	int8_t odd_hit = -10;
	int8_t odd_miss = 5;
	uint8_t init_value = 127;
	int search_size = 2;

public:

	uint8_t* get_map()
	{
		return mapdata;
	}
	int get_w()
	{
		return map_w;
	}
	int get_h()
	{
		return map_h;
	}

	int x_to_idx(float x)
	{
		return x / resolution + map_w / 2;
	}
	int y_to_idy(float y)
	{
		return y / resolution + map_h / 2;
	}
	float idx_to_x(int x)
	{
		return (float(x) - map_w / 2 + 0.5) * resolution;
	}
	float idy_to_y(int y)
	{
		return (float(y) - map_h / 2 + 0.5) * resolution;
	}


	GridMap2D(int w, int h, float r)
	{
		map_w = w;
		map_h = h;
		resolution = r;
		mapdata = new uint8_t[w * h];
		memset(mapdata, init_value, w * h);
	}

	~GridMap2D()
	{
		delete[] mapdata;
	}

	double CalculateLikelyhood(PointDataFrame data_frame, float x, float y, float r)
	{
		double total_likelyhood = 0;
		for (size_t i = 0; i < data_frame.data.size(); i++)
		{
			PointXY tran_point;
			tran_point.x = data_frame.data[i].x * cos(r) + data_frame.data[i].y * sin(r) + x;
			tran_point.y = data_frame.data[i].y * cos(r) - data_frame.data[i].x * sin(r) + y;

			int idx_x = x_to_idx(tran_point.x);
			int idx_y = y_to_idy(tran_point.y);

			double max_likelyhood = 0;
			
			for (int i = - search_size; i <= search_size; i++)
			{
				for (int j = -search_size; j <= search_size; j++)
				{
					double cur_dis_sq = (i * i + j * j);
					
					//if (cur_dis_sq > min_dis) continue;
					if ((idx_y + j >= 0) && (idx_y + j < map_h) && (idx_x + i > 0) && (idx_x + i < map_w))
					{
						int cur_index = (idx_y + j) * map_w + idx_x + i;
						double cur_likelyhood = 0;
						if (mapdata[cur_index] < 127)
						{
							cur_likelyhood = exp(1 /(cur_dis_sq + 1));
							if (cur_likelyhood > max_likelyhood)
							{
								max_likelyhood = cur_likelyhood;
							}
						}
						
					}
				}
			}
			total_likelyhood += max_likelyhood;
		}
		return total_likelyhood;
	}

	int UpdataMap(PointDataFrame data_frame,float x,float y,float r)
	{
		int start_idx_x = x_to_idx(x);
		int start_idx_y = y_to_idy(y);

		if ((start_idx_x < 0) || (start_idx_y < 0) || (start_idx_x >= map_w) || (start_idx_y >= map_h))
		{
			return 1;
		}

		for (size_t i = 0; i < data_frame.data.size(); i++)
		{
			float end_x = data_frame.data[i].x * cos(r) + data_frame.data[i].y * sin(r) + x;
			float end_y = data_frame.data[i].y * cos(r) - data_frame.data[i].x * sin(r) + y;
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

			while ((cur_idx_x!= end_idx_x)||(cur_idx_y!= end_idx_y))
			{
				//if ((cur_idx_x == end_idx_x) || (cur_idx_y == end_idx_y))
				//{
				//	std::cout << " " << std::endl;
				//}

				int data_index = cur_idx_y * map_w + cur_idx_x;

				if (mapdata[data_index] > 255-abs(odd_miss))
				{
					mapdata[data_index] = 255;
				}
				else
				{
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
				mapdata[data_index] += odd_hit;
			}
		}
		return 0;
	}

};