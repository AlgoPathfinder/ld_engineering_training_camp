#pragma once

#include <iostream>
#include <vector>
#include <fstream>
#include <cmath>

//雷达点，每个点包含角度、距离、置信度
struct LidarDataPoint
{
	float angle;
	float dis;
	float conf;
};

//实数点
struct PointXY
{
	float x;
	float y;
};

float GetPointDistance(PointXY a, PointXY b)
{
	return sqrt((a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y));
}

float GetPointDistanceSqr(PointXY a, PointXY b)
{
	return (a.x - b.x) * (a.x - b.x) + (a.y - b.y) * (a.y - b.y);
}

//雷达数据帧
struct LidarDataFrame
{
	int64_t time_interval;
	std::vector<LidarDataPoint> data;
};

//转为点云的雷达数据帧
struct PointDataFrame
{
	int64_t time_interval;
	std::vector<PointXY> data;
};

//雷达数据集
class LidarDataFrameList
{
public:
	std::vector<LidarDataFrame> data_list;

public:
	int get_frame_size()
	{
		return data_list.size();
	}

	int ReadDataFromFile(const char* file)
	{
		std::ifstream ifile;
		ifile.open(file);
		while (!ifile.eof())
		{
			LidarDataFrame frame_tmp;
			ifile >> frame_tmp.time_interval;
			int count;
			ifile >> count;
			for (int i = 0; i < count; i++)
			{
				LidarDataPoint point_tmp;
				ifile >> point_tmp.angle >> point_tmp.dis >> point_tmp.conf;
				if (point_tmp.conf > 0)
				{
					frame_tmp.data.push_back(point_tmp);
				}
			}
			data_list.push_back(frame_tmp);
		}
		ifile.close();
		return 0;
	}
	//SaveDataToFile();
};

//雷达数据处理，包含坐标转换，简单滤波，降采样等
class LidarDataTransform
{
private:
	PointDataFrame point_data;
	LidarDataFrame lidar_data;

public:
	int set_lidar_data(LidarDataFrame lidar_data_in)
	{
		lidar_data = lidar_data_in;
		return 0;
	}
	PointDataFrame get_point_data()
	{
		return point_data;
	}

	//极坐标转直角坐标
	int DataTransform()
	{
		point_data.time_interval = lidar_data.time_interval;
		point_data.data.clear();

		for (int i = 0; i < lidar_data.data.size(); i++)
		{
			PointXY point_tmp;
			point_tmp.x = lidar_data.data[i].dis * cos(lidar_data.data[i].angle / 180.0 * 3.14159);
			point_tmp.y = lidar_data.data[i].dis * sin(lidar_data.data[i].angle / 180.0 * 3.14159);
			point_data.data.push_back(point_tmp);
		}
		return 0;
	}

	//去除密集点
	int DataGridFilter(float dis)
	{
		if (point_data.data.size() <= 0)
		{
			return 1;
		}
		PointDataFrame point_data_tmp;
		point_data_tmp.time_interval = point_data.time_interval;
		point_data_tmp.data.push_back(point_data.data[0]);
		for (int i = 1; i < point_data.data.size(); i++)
		{
			if (GetPointDistance(point_data.data[i], point_data_tmp.data[point_data_tmp.data.size() - 1]) > dis)
			{
				point_data_tmp.data.push_back(point_data.data[i]);
			}
		}
		point_data = point_data_tmp;
		return 0;
	}

	//间隔k个点采样
	int DataDownSample(int k)
	{
		if (point_data.data.size() <= 0)
		{
			return 1;
		}
		PointDataFrame point_data_tmp;
		point_data_tmp.time_interval = point_data.time_interval;
		for (int i = 0; i < point_data.data.size(); i += k)
		{
			point_data_tmp.data.push_back(point_data.data[i]);
		}
		point_data = point_data_tmp;
		return 0;
	}

	//遮挡部分雷达角度，k为遮挡比例
	int CutData(float k)
	{
		if (point_data.data.size() <= 0)
		{
			return 1;
		}
		PointDataFrame point_data_tmp;
		point_data_tmp.time_interval = point_data.time_interval;
		for (int i = 0; i < point_data.data.size() * k / 2; i++)
		{
			point_data_tmp.data.push_back(point_data.data[i]);
		}
		for (int i = point_data.data.size() - point_data.data.size() * k / 2; i < point_data.data.size(); i++)
		{
			point_data_tmp.data.push_back(point_data.data[i]);
		}
		point_data = point_data_tmp;
		return 0;
	}

};
