﻿/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-13 16:52:31
 * @LastEditTime: 2023-07-13 18:14:38
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\lidar_data_show\lidar_data_show.cpp
 * 
 */
// lidar_data_show.cpp : 此文件包含 "main" 函数。程序执行将在此处开始并结束。
//

#include <iostream>
#include "opencv2/opencv.hpp"
#include "../lidar_data_common/lidar_data_common.h"

int main()
{
	LidarDataFrameList frame_data_test;
	frame_data_test.ReadDataFromFile("../data/lidar_data007.txt");
	std::cout << "total frame: " << frame_data_test.get_frame_size() << std::endl;

	int count = 0;
	while (count < frame_data_test.get_frame_size())
	{
		LidarDataTransform data_tran_test;
		data_tran_test.set_lidar_data(frame_data_test.data_list[count]);
		data_tran_test.DataTransform();
		data_tran_test.DataGridFilter(0.05);
		//data_tran_test.DataDownSample(5);

		const int show_w = 400;
		const int show_h = 300;
		cv::Mat points_show(show_h, show_w, CV_8UC3);
		memset(&points_show.data[0], 255, show_h * show_w * 3);

		PointDataFrame data_show_frame = data_tran_test.get_point_data();
		for (int i = 0; i < data_show_frame.data.size(); i++)
		{
			int idx_x = data_show_frame.data[i].x *15;
			idx_x = idx_x + show_w / 2;
			int idx_y = data_show_frame.data[i].y *15;
			idx_y = -idx_y + show_h / 2;

			if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
			{
				cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(0, 0, 255));
			}
		}

		cv::imshow("lidar frame", points_show);
		cv::waitKey(200);
		count++;
	}
	return 0;
}
