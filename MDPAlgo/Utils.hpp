#pragma once
#include <ostream>
#include <vector>
#include <sstream>

std::ostream& operator<<(std::ostream& os, const std::vector<int>& vec);

namespace Utils {
	template <typename T>
	std::string to_string(const T& value)
	{
		std::ostringstream ss;
		ss << value;
		return ss.str();
	}
}