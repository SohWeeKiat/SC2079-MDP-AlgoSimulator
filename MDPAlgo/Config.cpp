#include "Config.hpp"

namespace MDP {
	Config& Config::get()
	{
		static Config config;
		return config;
	}
}