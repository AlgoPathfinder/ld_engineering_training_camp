/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-29 15:10:04
 * @LastEditTime: 2023-08-02 21:21:00
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\path_planing_with_astar\path_planing_with_astar.cpp
 * 
 */

#include <iostream>
#include <opencv2/opencv.hpp>
#include "astar.h"

int main()
{
	cv::Mat src_map;
	src_map = cv::imread("../map/astar_map.bmp", 0);
	//cv::cvtColor(src_map, src_map, cv::COLOR_BGR2GRAY);

	AStar path_planer;
	path_planer.SetMap(src_map.cols, src_map.rows, &src_map.data[0]);

	path_planer.UpdataMap();

	cv::Mat cost_map(src_map);
	memcpy(&cost_map.data[0], path_planer.GetMapPtr(), path_planer.GetMapH() * path_planer.GetMapW());

	cv::cvtColor(cost_map, cost_map, cv::COLOR_GRAY2BGR);
	
	PathPoint start(100, 100);
	PathPoint end(path_planer.GetMapW() - 100, path_planer.GetMapH() - 100);

	cv::circle(cost_map, cv::Point(start.x,start.y), 5, cv::Scalar(0, 255, 0), 2);
	cv::circle(cost_map, cv::Point(end.x, end.y), 5, cv::Scalar(255, 0, 0), 2);

	std::vector<PathPoint> path_list;
	path_planer.FindPath(start, end, path_list);

	if (path_list.size() > 0)
	{
		for (size_t i = 1; i < path_list.size() - 1; i++)
		{
			cv::circle(cost_map, cv::Point(path_list[i].x, path_list[i].y), 2, cv::Scalar(0, 0, 255), 2);
		}
	}

	cv::resize(cost_map, cost_map, cv::Size(cost_map.cols / 4, cost_map.rows / 4));

	cv::imshow("cost_map", cost_map);
	cv::waitKey(0);

	return 0;
	
}
