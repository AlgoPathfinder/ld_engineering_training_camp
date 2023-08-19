/*
 * @Author: Ang.Lee.
 * @Date: 2023-08-02 10:55:50
 * @LastEditTime: 2023-08-19 13:41:26
 * @LastEditors: Ang.Lee.
 * @Description: 
 * 
 */

#include <iostream>
#include "opencv2/opencv.hpp"

#include "lidar_data_process.h"
#include "iterative_closest_point.h"
#include "grid_map_2d.h"
#include "pose_2d.h"


int main()
{
    IterativeClosestPoint icp;
    PointsMap2D points_map;

    LidarDataProcess lidar_data_process;
	lidar_data_process.Start();

	LidarDataFrame new_lidar_frame;
	
	int count = 0;  

	PointDataFrame new_point_frame;

	double car_x = 0;
	double car_y = 0;
	double car_a = 0;

	double last_x = 0;
	double last_y = 0;
	double last_a = 0;

	double last_insert_x = 0;
	double last_insert_y = 0;
	double last_insert_a = 0;

	GridMap2D grid_map(1200, 800, 0.05);
	cv::Mat grid_show(grid_map.get_h(), grid_map.get_w(), CV_8UC1);

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
            Pose2D insert_pose(0, 0, 0);
            points_map.InsertObservation(&insert_pose,&insert_frame);
			count++;
			continue;
		}
        
        data_trans.DataGridFilter(0.1);
		data_trans.DataDownSample(2);
		//idar_matching.CutData(0.7);

		new_point_frame = data_trans.get_point_data();

		Pose2D robot_pre_pose(car_x + (car_x - last_x), car_y + (car_y - last_y), car_a + (car_a - last_a));
		//Pose2D robot_pre_pose(car_x, car_y, car_a);

        Pose2D slam_pose;
        int good_ness=0;
        int total_good_ness=0;

        icp.Align(&points_map, &new_point_frame,&robot_pre_pose,&slam_pose,&good_ness,&total_good_ness);

		last_x = car_x;
		last_y = car_y;
		last_a = car_a;

        car_x=slam_pose.X;
        car_y=slam_pose.Y;
        car_a=slam_pose.Phi();

		if (((abs(car_x - last_insert_x) + abs(car_y - last_insert_y)) > 0.1) || (abs(car_a - last_insert_a) > 0.1))
		{
            points_map.InsertObservation(&slam_pose,&insert_frame);
	
			grid_map.UpdataMap(insert_frame, car_x, car_y, car_a);

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
