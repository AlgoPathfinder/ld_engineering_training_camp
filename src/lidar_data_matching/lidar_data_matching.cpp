/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-21 18:01:51
 * @LastEditTime: 2023-07-25 22:15:12
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\lidar_data_matching\lidar_data_matching.cpp
 * 
 */

#include <iostream>
#include "../lidar_data_common/lidar_data_common.h"
#include <opencv2/opencv.hpp>

int main()
{
	LidarDataFrameList frame_data_test;
	frame_data_test.ReadDataFromFile("../data/lidar_data010.txt");
	std::cout << "total frame: " << frame_data_test.get_frame_size() << std::endl;

	int count = 0;

	PointDataFrame new_frame;
	PointDataFrame last_frame;

	double car_x = 0;
	double car_y = 0;
	double car_a = 0;

	const int show_w = 400;
	const int show_h = 300;
	cv::Mat points_show(show_h, show_w, CV_8UC3);
	memset(&points_show.data[0], 255, show_h * show_w * 3);

	while (count < frame_data_test.get_frame_size())
	{
		LidarDataTransform data_tran_test;
		data_tran_test.set_lidar_data(frame_data_test.data_list[count]);
		data_tran_test.DataTransform();
		data_tran_test.DataGridFilter(0.02);
		//data_tran_test.DataDownSample(1);

		if (count == 0)
		{
			last_frame = data_tran_test.get_point_data();
			count++;
			continue;
		}

		new_frame = data_tran_test.get_point_data();

		int step = 1;
		float xy_step=0.05;
		float ang_step = 4.0;
		float dst_x = 0;
		float dst_y = 0;
		float dst_a = 0;

		double min_value = 36000;

		while (step<10)
		{
			float new_x = dst_x;
			float new_y = dst_y;
			float new_a = dst_a;
			int direction = 1;
			while (direction<7)
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

				double cost_value=CalCulatePointDataCost(last_frame, new_frame, new_x, new_y, new_a/180.0*3.14159);
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

		car_x += dst_x * cos(car_a) + dst_y * sin(car_a);
		car_y += dst_y * cos(car_a) - dst_x * sin(car_a);

		car_a += dst_a/180.0*3.14159;

		//std::cout << "dx= " << dst_x << "	dy= " << dst_y << "	da= " << dst_a << std::endl;
		std::cout << "x= " << car_x << "	y= " << car_y << "	a= " << car_a << std::endl;

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

		cv::imshow("trojectory", points_show);
		cv::waitKey(1);

		last_frame = new_frame;
		count++;
	}
	return 0;
}

