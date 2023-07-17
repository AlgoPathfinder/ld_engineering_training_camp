﻿/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-13 16:52:31
 * @LastEditTime: 2023-07-17 22:55:32
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\lidar_data_common\lidar_data_common.cpp
 * 
 */
#include "lidar_data_common.h"

int main()
{
	LidarDataFrameList frame_data_test;
	frame_data_test.ReadDataFromFile("../lidar_data007.txt");
	std::cout << "total frame: " << frame_data_test.get_frame_size()<< std::endl;
	LidarDataTransform data_tran_test;
	data_tran_test.set_lidar_data(frame_data_test.data_list[0]);
	data_tran_test.DataTransform();
	data_tran_test.DataGridFilter(0.05);
	data_tran_test.DataDownSample(5);

	std::cin.get();

	return 0;
}
