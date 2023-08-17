#include "ld_camp.h"

int main()
{
    std::cout << "build your program hear!\n";
	TestMyProgram test_my_program;
	test_my_program.Start();
	//test_my_program.Join();
	//test_my_program.Detach();
	while (1)
	{
		std::cout << "build your program hear!\n";
		Sleep(500);
	}
	return 0;
}

