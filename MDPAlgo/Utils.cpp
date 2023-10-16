#include "Utils.hpp"
#include <iostream>

std::ostream& operator<<(std::ostream& os, const std::vector<int>& vec)
{
	std::cout << "[";
	for (auto& i : vec)
		std::cout << i << ",";
	std::cout << "]" << std::endl;
	return os;
}

namespace Utils {
	
}