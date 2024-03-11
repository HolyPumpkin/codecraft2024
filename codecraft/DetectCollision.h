#pragma once
#include "MyStruct.h"

class DetectCollision
{
private:

	// 产生冲突的点集


public:
	
	// 通过机器人下一步位置，检测产生冲突的点
	void DetectInNextStep(vector<Robot>& robots);
};

