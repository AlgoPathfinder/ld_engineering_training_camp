/*
 * @Author: Ang.Lee.
 * @Date: 2023-08-03 14:54:34
 * @LastEditTime: 2023-08-10 17:53:52
 * @LastEditors: Ang.Lee.
 * @Description: 
 * 
 */
#pragma once
#include <mutex>
#include <thread>
#include "../ldlidar_driver/include/core/ldlidar_driver.h"
#include "lidar_data_common.h"

uint64_t GetSystemTimeStamp(void) {
  std::chrono::time_point<std::chrono::system_clock, std::chrono::nanoseconds> tp = 
    std::chrono::time_point_cast<std::chrono::nanoseconds>(std::chrono::system_clock::now());
  auto tmp = std::chrono::duration_cast<std::chrono::nanoseconds>(tp.time_since_epoch());
  return ((uint64_t)tmp.count());
}

void LidarPowerOn(void) {
  LDS_LOG_DEBUG("Please Lidar Power On","");
}

void LidarPowerOff(void) {
  LDS_LOG_DEBUG("Please Lidar Power Off","");
}

class LidarDataProcess
{
private:
	std::string product_name;
	std::string communication_mode;
	std::string port_name;
	std::string server_ip;
	std::string server_port;
	uint32_t serial_baudrate = 0;
	ldlidar::LDType type_name;
	ldlidar::LDLidarDriver* node = nullptr;

	std::mutex raw_data_cs;
	LidarDataFrame raw_data_frame;

public:
	bool has_new_data = 0;


	LidarDataProcess()
	{
		communication_mode = "serialcom";
		port_name = "/dev/ttyUSB0";
		server_ip = "192.168.1.200";
		server_port = "2000";
		product_name = "STL06P";

		node = new ldlidar::LDLidarDriver();

		LDS_LOG_INFO("LDLiDAR SDK Pack Version is %s", node->GetLidarSdkVersionNumber().c_str());

		node->RegisterGetTimestampFunctional(std::bind(&GetSystemTimeStamp)); //   注册时间戳获取函数

		node->EnableFilterAlgorithnmProcess(true);

		if (product_name == "LD06") {
			serial_baudrate = 230400;
			type_name = ldlidar::LDType::LD_06;
		}
		else if (product_name == "LD19") {
			serial_baudrate = 230400;
			type_name = ldlidar::LDType::LD_19;
		}
		else if (product_name == "STL06P") {
			serial_baudrate = 230400;
			type_name = ldlidar::LDType::STL_06P;
		}
		else if (product_name == "STL27L") {
			serial_baudrate = 921600;
			type_name = ldlidar::LDType::STL_27L;
		}
		else if (product_name == "STL26") {
			serial_baudrate = 230400;
			type_name = ldlidar::LDType::STL_26;
		}
		else {
			LDS_LOG_ERROR("input <product_name> is error!", "");
			exit(EXIT_FAILURE);
		}

		if (communication_mode == "serialcom") {
			if (node->Start(type_name, port_name, serial_baudrate, ldlidar::COMM_SERIAL_MODE)) {
				LDS_LOG_INFO("ldlidar node start is success", "");
				LidarPowerOn();
			}
			else {
				LD_LOG_ERROR("ldlidar node start is fail", "");
				exit(EXIT_FAILURE);
			}
		}
		else if (communication_mode == "networkcom_tcpclient") {
			if (node->Start(type_name, server_ip.c_str(), server_port.c_str(), ldlidar::COMM_TCP_CLIENT_MODE)) {
				LDS_LOG_INFO("ldlidar node start is success", "");
				LidarPowerOn();
			}
			else {
				LD_LOG_ERROR("ldlidar node start is fail", "");
				exit(EXIT_FAILURE);
			}
		}

		if (node->WaitLidarCommConnect(3000)) {
			LDS_LOG_INFO("ldlidar communication is normal.", "");
		}
		else {
			LDS_LOG_ERROR("ldlidar communication is abnormal.", "");
			exit(EXIT_FAILURE);
		}
	}
	~LidarDataProcess()
	{
		node->Stop();
		LidarPowerOff();

		delete node;
		node = nullptr;
	}

	void DataProcess()
	{
		ldlidar::Points2D laser_scan_points;
		double lidar_spin_freq;
		while (ldlidar::LDLidarDriver::IsOk()) {
			LidarDataFrame tmp_data_frame;

			switch (node->GetLaserScanData(laser_scan_points, 1500)) {
			case ldlidar::LidarStatus::NORMAL:
				// get lidar normal data
				if (node->GetLidarSpinFreq(lidar_spin_freq)) {
					//LDS_LOG_INFO("speed(Hz):%f", lidar_spin_freq);
				}

#ifdef __LP64__
				//LDS_LOG_INFO("size:%d,stamp_front:%lu, stamp_back:%lu",
				//	laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#else
				//LDS_LOG_INFO("size:%d,stamp_front:%llu, stamp_back:%llu",
				//	laser_scan_points.size(), laser_scan_points.front().stamp, laser_scan_points.back().stamp);
#endif
				tmp_data_frame.time_interval = laser_scan_points.front().stamp;

				for (auto point : laser_scan_points) {
#ifdef __LP64__
				//	LDS_LOG_INFO("stamp:%lu,angle:%f,distance(mm):%d,intensity:%d",
				//		point.stamp, point.angle, point.distance, point.intensity);
#else
				//	LDS_LOG_INFO("stamp:%llu,angle:%f,distance(mm):%d,intensity:%d",
				//		point.stamp, point.angle, point.distance, point.intensity);
#endif
					LidarDataPoint tmp_data;
					tmp_data.angle = 360.0f-point.angle;
					tmp_data.dis = float(point.distance)/1000.0f;
					tmp_data.conf = point.intensity;
					tmp_data_frame.data.push_back(tmp_data);

				}

				raw_data_cs.lock();
				raw_data_frame = tmp_data_frame;
				has_new_data = 1;
				raw_data_cs.unlock();

				break;
			case ldlidar::LidarStatus::ERROR:
				LDS_LOG_ERROR("ldlidar is error.", "");
				node->Stop();
				break;
			case ldlidar::LidarStatus::DATA_TIME_OUT:
				LDS_LOG_ERROR("ldlidar publish data is time out, please check your lidar device.", "");
				node->Stop();
				break;
			case ldlidar::LidarStatus::DATA_WAIT:
				break;
			default:
				break;
			}

			usleep(1000 * 100);  // sleep 100ms  == 10Hz
		}
	}
	void Start()
	{
		std::thread lidar_process_thread(std::bind(&LidarDataProcess::DataProcess, this));
		lidar_process_thread.detach();
	}

	void GetRawDataFrame(LidarDataFrame& frame)
	{

		raw_data_cs.lock();
		frame = raw_data_frame;
		has_new_data = 0;
		raw_data_cs.unlock();

	}
};
