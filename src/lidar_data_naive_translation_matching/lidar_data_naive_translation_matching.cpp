/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-19 16:48:34
 * @LastEditTime: 2023-07-19 17:08:45
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\lidar_data_naive_translation_matching\lidar_data_naive_translation_matching.cpp
 * 
 */
#include <iostream>
#include "../lidar_data_common/lidar_data_common.h"
#include <opencv2/opencv.hpp>

int main()
{
	LidarDataFrameList frame_data_test;
	frame_data_test.ReadDataFromFile("../lidar_data001.txt");
	std::cout << "total frame: " << frame_data_test.get_frame_size() << std::endl;

	int count = 0;

	PointDataFrame new_frame;
	PointDataFrame first_frame;

	double car_x = 0;
	double car_y = 0;
	double car_a = 0;

	const int show_w = 400;
	const int show_h = 300;
	const float to_map_scale = 15.0f;

	cv::Mat points_show(show_h, show_w, CV_8UC3);

	while (count < frame_data_test.get_frame_size())
	{
		memset(&points_show.data[0], 255, show_h * show_w * 3);
		LidarDataTransform data_tran_test;
		data_tran_test.set_lidar_data(frame_data_test.data_list[count]);

		data_tran_test.DataGridFilter(0.05);
		data_tran_test.DataDownSample(2);

		if (count == 0)
		{
			first_frame = data_tran_test.get_point_data();
			count++;
			continue;
		}

		new_frame = data_tran_test.get_point_data();

		float dst_x = 0;

		double min_value = 36000;
		
		//沿X方向搜索可能的范围
		for (int i = -40; i < 40; i++)
		{
			float new_x = i * 0.01 + car_x;
			double cost_value = CalCulatePointDataCost(first_frame, new_frame, new_x, 0, 0);
			if (cost_value < min_value)
			{
				dst_x = new_x;
				min_value = cost_value;
			}
		}

		car_x = dst_x;

		std::cout << "x= " << car_x << "	y= " << car_y << "	a= " << car_a << std::endl;

		int idx_x = car_x * to_map_scale;
		idx_x = idx_x + show_w / 2;
		int idx_y = car_y * to_map_scale;
		idx_y = -idx_y + show_h / 2;


		float car_top_x = 0.4 * cos(car_a) + 0 * sin(car_a) + car_x;
		float car_top_y = 0 * cos(car_a) - 0.4 * sin(car_a) + car_y;

		int cat_top_idx_x = car_top_x * to_map_scale;
		cat_top_idx_x = cat_top_idx_x + show_w / 2;
		int cat_top_idx_y = car_top_y * to_map_scale;
		cat_top_idx_y = -cat_top_idx_y + show_h / 2;

		if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
		{
			cv::circle(points_show, cv::Point(idx_x, idx_y), 5, cv::Scalar(0, 0, 255));
			cv::line(points_show, cv::Point(idx_x, idx_y), cv::Point(cat_top_idx_x, cat_top_idx_y), cv::Scalar(0, 0, 255), 1);
		}

		for (int i = 0; i < new_frame.data.size(); i++)
		{
			int idx_x = (new_frame.data[i].x * cos(car_a) + new_frame.data[i].y * sin(car_a) + car_x) * to_map_scale;
			idx_x = idx_x + show_w / 2;
			int idx_y = (new_frame.data[i].y * cos(car_a) - new_frame.data[i].x * sin(car_a) + car_y) * to_map_scale;
			idx_y = -idx_y + show_h / 2;
			if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
			{
				cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(0, 0, 255));
			}

		}

		cv::imshow("robot_pose", points_show);
		cv::waitKey(100);

		count++;
	}
	return 0;
}

