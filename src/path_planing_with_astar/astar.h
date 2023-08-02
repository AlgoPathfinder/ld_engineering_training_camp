/*
 * @Author: Ang.Lee.
 * @Date: 2023-07-28 14:45:46
 * @LastEditTime: 2023-08-02 22:19:15
 * @LastEditors: Ang.Lee.
 * @Description: 
 * @FilePath: \lidar_data_demo_linux\src\path_planing_with_astar\astar.h
 * 
 */
#pragma once
#include <iostream>
#include <queue>
#include <vector>

struct PathPoint
{
	int x = 0;
	int y = 0;

	PathPoint(int a, int b)
	{
		x = a;
		y = b;
	}
};

class AStar
{
	struct AStarPoint
	{
		int F;
		int G;
		int H;
		int idx;
		int idy;
		int father_id;
		
		bool operator < (const AStarPoint& t) const
		{
			if (F < t.F)
			{
				return true;
			}
			return false;
		}

		bool operator > (const AStarPoint& t) const
		{
			if (F > t.F)
			{
				return true;
			}
			return false;
		}
	};

protected:
	uint8_t* mapdata;
	int map_w = 0;
	int map_h = 0;
	float resolution = 0.05;
	uint8_t obstacle_thr = 127;
	
	int expand_level = 50;
	std::priority_queue<AStarPoint, std::vector<AStarPoint>, std::greater<AStarPoint>> open_list;
	
	std::vector<AStarPoint> search_list;
	int* close_map;
	
	int dx[4] = { 1,0,-1,0 };
	int dy[4] = { 0,1,0,-1 };

public:
	AStar()
	{
		mapdata = nullptr;
		close_map = nullptr;
	}

	~AStar()
	{
		if (mapdata)
		{
			delete[] mapdata;
		}

		if (close_map)
		{
			delete[] close_map;
		}
	}

	int SetMap(int w,int h, uint8_t*map, uint8_t obs_thr= (uint8_t)127)
	{
		if (mapdata)
		{
			delete[] mapdata;
		}

		if (close_map)
		{
			delete[] close_map;
		}

		map_w = w;
		map_h = h;
		mapdata = new uint8_t[map_w * map_h];
		memcpy(mapdata, map, map_w * map_h);
		obstacle_thr = obs_thr;
		close_map = new int[map_w * map_h];;

		return 0;
	}

	int  GetMapW()
	{
		return map_w;
	}

	int  GetMapH()
	{
		return map_h;
	}

	uint8_t* GetMapPtr()
	{
		return mapdata;
	}

	int UpdataMap()
	{
		std::vector<PathPoint>expand_point;
		std::vector<PathPoint>next_expand_point;

		for (int i = 0; i < map_h; i++)
		{
			for (int j = 0; j < map_w; j++)
			{
				int index = i * map_w + j;

				if ((i == 0) || (j == 0) || (i == (map_h-1)) || (j == (map_w-1)))
				{
					mapdata[index] = 0;
					continue;
				}

				if ((i == 1) || (j == 1) || (i == (map_h - 2)) || (j == (map_w - 2)))
				{
					mapdata[index] = 0;
					expand_point.push_back(PathPoint(j, i));
					continue;
				}

				if (mapdata[index] < obstacle_thr)
				{
					mapdata[index] = 0;
					expand_point.push_back(PathPoint(j, i));
				}
				else
				{
					mapdata[index] = 255;
				}

			}
		}

		for (int kk = 1; kk <= expand_level; kk++)
		{
			next_expand_point.clear();
			for (size_t i = 0; i < expand_point.size(); i++)
			{
				for (int j = 0; j < 4; j++)
				{
					int newidx = expand_point[i].x + dx[j];
					int newidy = expand_point[i].y + dy[j];
					int newindex = newidy * map_w + newidx;

					if (mapdata[newindex] > uint8_t(kk))
					{
						mapdata[newindex] = kk;
						next_expand_point.push_back(PathPoint(newidx, newidy));
					}
				}
			}
			expand_point = next_expand_point;
		}
		return 0;
	}

	bool FindPath(PathPoint str_point, PathPoint end_point, std::vector<PathPoint>& out_path)
	{
		open_list = std::priority_queue<AStarPoint, std::vector<AStarPoint>, std::greater<AStarPoint>>();
		search_list.clear();
		memset(close_map, 127, map_h * map_w * 4);

		AStarPoint start;
		start.G = 0;
		start.H = abs(end_point.x- str_point.x)+ abs(end_point.y - str_point.y);
		//start.H = 0;
		start.F = start.G + start.H;

		start.idx = str_point.x;
		start.idy = str_point.y;
		start.father_id = -1;

		int search_count = 0;

		open_list.push(start);

		while (!open_list.empty())
		{
			AStarPoint cur_point;
			cur_point = open_list.top();
			search_list.push_back(cur_point);
			close_map[cur_point.idx + cur_point.idy * map_w] = -1;

			open_list.pop();

			for (int i = 0; i < 4; i++)
			{
				int newidx = cur_point.idx + dx[i];
				int newidy = cur_point.idy + dy[i];
				int newindex = newidy * map_w + newidx;

				if (close_map[newindex]>=0)
				{
					if (mapdata[newindex] <= 0)
					{
						close_map[newindex] = -1;
						continue;
					}

					int cost;
					if (mapdata[newindex] > 250)
					{
						cost = 0;
					}
					else
					{
						cost = expand_level - mapdata[newindex] + 1;
					}

					AStarPoint next_point;
					next_point.G = cur_point.G + 1 + cost;
					next_point.H = (abs(end_point.x - newidx) + abs(end_point.y - newidy));
					//next_point.H = 0;
					next_point.F = next_point.G + next_point.H;
					next_point.idx = newidx;
					next_point.idy = newidy;
					next_point.father_id = search_count;

					if (close_map[newindex] > next_point.F)
					{
						close_map[newindex] = next_point.F;
						open_list.push(next_point);
					}
	
					if ((next_point.idx == end_point.x) && (next_point.idy == end_point.y))
					{
						search_list.push_back(next_point);
						search_count++;
						std::vector<PathPoint> tmp_path;
						tmp_path.clear();
						int cur_id = search_count;

						while (cur_id != -1)
						{
							tmp_path.push_back(PathPoint(search_list[cur_id].idx, search_list[cur_id].idy));
							//std::cout << search_list[cur_id].idx << "   " << search_list[cur_id].idy << "   " << search_list[cur_id].H << std::endl;
							cur_id = search_list[cur_id].father_id;
						}
						for (size_t i = 0; i < tmp_path.size(); i++)
						{
							out_path.push_back(tmp_path[tmp_path.size() - i - 1]);
						}
						
						//std::cout << int(mapdata[150 * map_w + 4]) << std::endl;

						return true;
					}

				}

			}

			search_count++;
		}

		return false;
	}

};
