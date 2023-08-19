/*
 * @Author: Ang.Lee.
 * @Date: 2023-08-02 10:55:50
 * @LastEditTime: 2023-08-19 13:46:16
 * @LastEditors: Ang.Lee.
 * @Description: 
 * 
 */

#include <iostream>
#include "opencv2/opencv.hpp"
#include "../grid_matching_with_rtree/scan_matcher.h"
#include "lidar_data_process.h"

int main()
{
    LidarDataProcess lidar_data_process;
	lidar_data_process.Start();

	LidarDataFrame new_lidar_frame;

	ScanMatcher scan_matcher;

	int count = 0;  

	PointDataFrame new_point_frame;

	double car_x = 0;
	double car_y = 0;
	double car_a = 0;

	double last_insert_x = 0;
	double last_insert_y = 0;
	double last_insert_a = 0;

	GridMap2DwithRtree  grid_map_with_rtree(1200, 900, 0.05);
	cv::Mat grid_show(grid_map_with_rtree.get_h(), grid_map_with_rtree.get_w(), CV_8UC1);

	const int show_w = 600;
	const int show_h = 400;
	cv::Mat points_show(show_h, show_w, CV_8UC3);
	memset(&points_show.data[0], 255, show_h * show_w * 3);


	while (1)
	{
		if (lidar_data_process.has_new_data)
		{
			lidar_data_process.GetRawDataFrame(new_lidar_frame);
			//std::cout << "new_lidar_frame.time:" << new_lidar_frame.time_interval << "   new_lidar_frame.size:" << new_lidar_frame.data.size() << std::endl;
            if (new_lidar_frame.data.size() <10)
            {
                usleep(5000);
                continue;
            }
		}
        else
        {
            usleep(5000);
            continue; 
        }

		LidarDataTransform data_trans;
		data_trans.set_lidar_data(new_lidar_frame);
        PointDataFrame insert_frame=data_trans.get_point_data();


		if (count == 0)
		{
			count++;
			for (int i = 0; i < 10; i++)
			{
				grid_map_with_rtree.UpdataMap(insert_frame, 0, 0, 0);
			}
			continue;
		}

		data_trans.DataGridFilter(0.05);
		//data_trans.DataDownSample(2);
		//data_trans.CutData(0.7);

		new_point_frame = data_trans.get_point_data();

		scan_matcher.MatchProcess(grid_map_with_rtree, new_point_frame, car_x, car_y, car_a);

		if (((abs(car_x - last_insert_x) + abs(car_y - last_insert_y)) > 0.1) || (abs(car_a - last_insert_a) > 0.1))
		{
			grid_map_with_rtree.UpdataMap(insert_frame, car_x, car_y, car_a);

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

		for (size_t i = 0; i < new_point_frame.data.size(); i++)
		{
			int idx_x = (new_point_frame.data[i].x * cos(car_a) - new_point_frame.data[i].y * sin(car_a) + car_x) * 15;
			idx_x = idx_x + show_w / 2;
			int idx_y = (new_point_frame.data[i].y * cos(car_a) + new_point_frame.data[i].x * sin(car_a) + car_y) * 15;
			idx_y = -idx_y + show_h / 2;
			if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
			{
				cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(0, 0, 255));
			}

		}

		memcpy(&grid_show.data[0], grid_map_with_rtree.get_map(), grid_map_with_rtree.get_w() * grid_map_with_rtree.get_h());

		cv::flip(grid_show, grid_show, 0);
		//cv::Mat small_grid_show;
		//cv::resize(grid_show, small_grid_show, cv::Size(grid_show.cols / 2, grid_show.rows / 2));
		cv::imshow("grid_map", grid_show);

		cv::imshow("trojectory", points_show);
		cv::waitKey(1);
		//usleep(1000); 
		count++;
	}
	return 0;
}
