#include <iostream>
#include <opencv2/opencv.hpp>
#include "iterative_closest_point.h"
#include "grid_map_2d.h"
#include "pose_2d.h"

int main()
{
    IterativeClosestPoint icp;
    PointsMap2D points_map;

	LidarDataFrameList frame_data_list;
	frame_data_list.ReadDataFromFile("../data/lidar_data008.txt");
	std::cout << "total frame: " << frame_data_list.get_frame_size() << std::endl;

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

	const int show_w = 400;
	const int show_h = 300;
	cv::Mat points_show(show_h, show_w, CV_8UC3);
	memset(&points_show.data[0], 255, show_h * show_w * 3);


	while (count < frame_data_list.get_frame_size())
	{


        LidarDataTransform data_trans;
		data_trans.set_lidar_data(frame_data_list.data_list[count]);

        PointDataFrame insert_frame=data_trans.get_point_data();

		data_trans.DataGridFilter(0.1);
		data_trans.DataDownSample(2);
		data_trans.CutData(0.7);

		if (count == 0)
		{
			Pose2D insert_pose(0, 0, 0);
            points_map.InsertObservation(&insert_pose,&insert_frame);
			count++;
			continue;
		}

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

		std::cout << "car_x:" << car_x << "   car_y:" << car_y << "   car_a:" << car_a << std::endl;

		if (((abs(car_x - last_insert_x) + abs(car_y - last_insert_y)) > 0.2) || (abs(car_a - last_insert_a) > 0.5))
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
		idx_y = idx_y + show_h / 2;

		if ((idx_x < show_w) && (idx_y < show_h) && (idx_x >= 0) && (idx_y >= 0))
		{
			cv::circle(points_show, cv::Point(idx_x, idx_y), 1, cv::Scalar(255, 0, 0));
		}

		for (size_t i = 0; i < new_point_frame.data.size(); i++)
		{
			int idx_x = (new_point_frame.data[i].x * cos(car_a) - new_point_frame.data[i].y * sin(car_a) + car_x) * 15;
			idx_x = idx_x + show_w / 2;
			int idx_y = (new_point_frame.data[i].y * cos(car_a) + new_point_frame.data[i].x * sin(car_a) + car_y) * 15;
			idx_y = idx_y + show_h / 2;
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

		count++;
	}
	return 0;
}

