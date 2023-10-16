#pragma once

#define	LogTSP 0

#define GetSetIntMacroV(Name, value) private: int _##Name = value; \
public: int Get_##Name(){ return _##Name; }\
void Set_##Name(int Value){ _##Name = Value; }

#define GetSetBoolMacroV(Name, value) private: bool _##Name = value; \
public: bool Is_##Name(){ return _##Name; }\
void Set_##Name(bool Value){ _##Name = Value; }

namespace MDP {
	class Config {

	public:
		static Config& get();

		GetSetIntMacroV(EXPANDED_CELL, 1);
		GetSetIntMacroV(WIDTH_BUFFER, 20);
		GetSetIntMacroV(HEIGHT_BUFFER, 20);

		//cost
		GetSetIntMacroV(SCREENSHOT_COST, 50);
		GetSetIntMacroV(SAFE_COST, 1000);

		GetSetIntMacroV(TURN_RADIUS, 1);
		GetSetIntMacroV(TURN_FACTOR, 1);
		GetSetIntMacroV(ITERATIONS, 2000);

		GetSetIntMacroV(LEFTWHEEL, 3);
		GetSetIntMacroV(RIGHTWHEEL, 2);

		GetSetBoolMacroV(LimitMax90, true);
		GetSetBoolMacroV(OutsideCommand, false);
	};
}