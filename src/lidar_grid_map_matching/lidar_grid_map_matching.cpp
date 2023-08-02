/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-24 18:03:03
 * @LastEditTime: 2023-08-02 22:12:00
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\lidar_grid_map_matching\lidar_grid_map_matching.cpp
 * 
 */
#include <iostream>
#include "../lidar_data_common/lidar_data_common.h"
#include <opencv2/opencv.hpp>
#include "../lidar_data_common/grid_map_2d.h"

int main()
{
	LidarDataFrameList frame_data_test;
	frame_data_test.ReadDataFromFile("../data/lidar_data008.txt");
	std::cout << "total frame: " << frame_data_test.get_frame_size() << std::endl;

	int count = 0;

	PointDataFrame new_frame;
	PointDataFrame last_frame;

	double car_x = 0;
	double car_y = 0;
	double car_a = 0;

	double last_insert_x = 0;
	double last_insert_y = 0;
	double last_insert_a = 0;

	GridMap2D grid_map(1200, 800, 0.05);
	cv::Mat grid_show(grid_map.get_h(), grid_map.get_w(), CV_8UC1);

	const int show_w = 400;
	const int show_h = 300;
	cv::Mat points_show(show_h, show_w, CV_8UC3);
	memset(&points_show.data[0], 255, show_h * show_w * 3);


	while (count < frame_data_test.get_frame_size())
	{

		LidarDataTransform data_tran_test;
		data_tran_test.set_lidar_data(frame_data_test.data_list[count]);
		data_tran_test.DataGridFilter(0.1);
		//data_tran_test.DataDownSample(2);
		//data_tran_test.CutData(0.7);

		if (count == 0)
		{
			last_frame = data_tran_test.get_point_data();
			count++;
			for (int i = 0; i < 10; i++)
			{
				grid_map.UpdataMap(last_frame, 0, 0, 0);
			}
			continue;
		}

		new_frame = data_tran_test.get_point_data();

		int step = 1;
		float xy_step = 0.06;
		float ang_step = 0.08;
		float dst_x = car_x;
		float dst_y = car_y;
		float dst_a = car_a;

		double max_likelyhood = 0;

		while (step < 6)
		{
			float new_x = dst_x;
			float new_y = dst_y;
			float new_a = dst_a;
			int direction = 1;
			while (direction < 7)
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

				double cur_likelyhood = grid_map.CalculateLikelyhood(new_frame, new_x, new_y, new_a);
				if (cur_likelyhood > max_likelyhood)
				{
					dst_x = new_x;
					dst_y = new_y;
					dst_a = new_a;
					max_likelyhood = cur_likelyhood;
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
		std::cout << "x= " << car_x << "	y= " << car_y << "	a= " << car_a << std::endl;

		//LidarDataTransform lidar_grid_update;
		//lidar_grid_update.set_lidar_data(frame_data_test.data_list[count]);
		//grid_map.UpdataMap(lidar_grid_update.get_point_data(), car_x, car_y, car_a);


		if (((abs(car_x - last_insert_x) + abs(car_y - last_insert_y)) > 0.2) || (abs(car_a - last_insert_a) > 0.3))
		{
			grid_map.UpdataMap(new_frame, car_x, car_y, car_a);
			LidarDataTransform lidar_grid_update;
			lidar_grid_update.set_lidar_data(frame_data_test.data_list[count]);
			grid_map.UpdataMap(lidar_grid_update.get_point_data(), car_x, car_y, car_a);

			last_insert_x = car_x;
			last_insert_y = car_y;
			last_insert_a = car_a;
		}

		int idx_x = car_x * 15;
		idx_x = idx_x + show_w / 2;
		int idx_y = car_y * 15;
		idx_y = -idx_y + show_h / 2;

		if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
		{
			cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(255, 0, 0));
		}

		for (size_t i = 0; i < new_frame.data.size(); i++)
		{
			int idx_x = (new_frame.data[i].x * cos(car_a) + new_frame.data[i].y * sin(car_a) + car_x) * 15;
			idx_x = idx_x + show_w / 2;
			int idx_y = (new_frame.data[i].y * cos(car_a) - new_frame.data[i].x * sin(car_a) + car_y) * 15;
			idx_y = -idx_y + show_h / 2;
			if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
			{
				cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(0, 0, 255));
			}

		}

		memcpy(&grid_show.data[0], grid_map.get_map(), grid_map.get_w() * grid_map.get_h());

		cv::flip(grid_show, grid_show, 0);
		cv::imshow("grid_map", grid_show);

		cv::imshow("trojectory", points_show);
		cv::waitKey(1);

		last_frame = new_frame;
		count++;
	}
	return 0;
}

