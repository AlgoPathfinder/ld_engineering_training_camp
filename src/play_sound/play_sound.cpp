#include <iostream>

int main()
{
	int ret = 0;

	ret |= system("play ../sound/front.mp3");
	ret |= system("play ../sound/left.mp3");
	ret |= system("play ../sound/right.mp3");
	ret |= system("play ../sound/leftfront.mp3");
	ret |= system("play ../sound/rightfront.mp3");

	std::cout << "ret=" << ret << "	Test end!" << std::endl;
	return 0;
}

