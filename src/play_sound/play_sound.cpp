#include <iostream>

int main()
{
	int ret = 0;

	ret |= system("play ../sound/front.wav");
	ret |= system("play ../sound/left.wav");
	ret |= system("play ../sound/right.wav");
	ret |= system("play ../sound/leftfront.wav");
	ret |= system("play ../sound/rightfront.wav");
	ret |= system("play ../sound/back.wav");
	ret |= system("play ../sound/failed.wav");
	ret |= system("play ../sound/finished.wav");
	
	std::cout << "ret=" << ret << "	Test end!" << std::endl;
	return 0;
}

