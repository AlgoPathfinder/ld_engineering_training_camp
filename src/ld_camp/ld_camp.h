#pragma once
#include <iostream>
#include <mutex>
#include <thread>
#include <functional>

#ifdef WINDOWS
	#include <windows.h>
#else
	#include <unistd.h>
	#define Sleep(x) usleep(x*1000)
#endif // WINDOWS


class TestMyProgram
{
public:
	TestMyProgram()
	{

	}
	~TestMyProgram()
	{

	}

	int thread_loop1()
	{
		while (1)
		{
			count_lock.lock();
			count++;
			std::cout << "thread1:build your own thread hear.   " << count << std::endl;
			count_lock.unlock();
			Sleep(1000);
		}
		return 0;
	}

	int thread_loop2()
	{
		while (1)
		{
			count_lock.lock();
			count++;
			std::cout << "thread2:build your own thread hear.   " << count << std::endl;
			count_lock.unlock();
			Sleep(1000);
		}
		return 0;
	}

	void Start()
	{
		test_thread1 = std::thread(std::bind(&TestMyProgram::thread_loop1, this));
		test_thread2 = std::thread(std::bind(&TestMyProgram::thread_loop2, this));
	}

	void Detach()
	{
		test_thread1.detach();
		test_thread2.detach();
	}

	void Join()
	{
		test_thread1.join();
		test_thread2.join();
	}

private:
	std::thread test_thread1;
	std::thread test_thread2;
	std::mutex count_lock;
	int count = 0;

};