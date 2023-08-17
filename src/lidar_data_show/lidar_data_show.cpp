/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-13 16:52:31
 * @LastEditTime: 2023-07-19 16:53:20
 * @LastEditors: Ang.Lee.
 * @Description: 
 * 
 */

//此程序从磁盘中读取雷达数据文件并使用OpenCV显示

#include <iostream>
#include "opencv2/opencv.hpp"
#include "lidar_data_common.h"

int main()
{
	LidarDataFrameList frame_data_list;

	//从文件中读取雷达数据
	frame_data_list.ReadDataFromFile("../data/lidar_data007.txt");
	std::cout << "total frame: " << frame_data_list.get_frame_size() << std::endl;

	int count = 0;
	while (count < frame_data_list.get_frame_size())
	{
		//将雷达数据由极坐标系转为直角坐标系
		LidarDataTransform data_trans;
		data_trans.set_lidar_data(frame_data_list.data_list[count]);

		//过滤密集点
		//data_trans.DataGridFilter(0.05);
		//降采样
		//data_trans.DataDownSample(2);

		//定义显示图像长和宽
		const int show_w = 400;
		const int show_h = 300;
		//定义坐标转换系数
		const float to_map_scale = 15.0f;
		
		//新建图像对象，格式为3通道BGR图像，每通道8位，并初始化为0
		cv::Mat points_show(show_h, show_w, CV_8UC3);
		memset(&points_show.data[0], 255, show_h * show_w * 3);

		PointDataFrame data_show_frame = data_trans.get_point_data();
		for (size_t i = 0; i < data_show_frame.data.size(); i++)
		{
			//计算每个像素点的图像坐标
			int idx_x = data_show_frame.data[i].x *to_map_scale;
			idx_x = idx_x + show_w / 2;
			int idx_y = data_show_frame.data[i].y *to_map_scale;
			idx_y = -idx_y + show_h / 2;

			if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
			{
				cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(0, 0, 255));
			}
		}

		cv::imshow("lidar_frame", points_show);
		cv::waitKey(100);
		count++;
	}
	return 0;
}

