#pragma once
#include <stdint.h>
#include <cmath>

//目标点坐标的存储方法:
enum MPStorageMethod
{
	MPStorageMethod_ABSOLUTE = 0, //相对于原点坐标
	MPStorageMethod_RELATIVE = 1  //相对于上一个坐标点
};

//目标点坐标的存储方法:相对于原点/相对于上一个坐标点
enum MPCalibMethod
{
	MPCalibMethod_SQUARE = 0, //方形(xy分别判断误差)
	MPCalibMethod_RADIUS = 1  //圆形(判断到目标点的距离)
};

class MissionPoint
{
private:
public:
	//任务标记号
	uint8_t MissionPayload;

	//目标点坐标
	MPStorageMethod StorgMeth;
	double xCoordinate; //T265坐标系x
	double yCoordinate; //T265坐标系y

	//飞行控制限定值
	double RestrainSpeed;	  //限制速度 m/s
	uint16_t MaxCtrlStep;	  //最大控制周期数(根据任务执行频率调整)
	double IntervalSleepTime; //控制周期(的最小值)s

	//对齐控制限定值
	MPCalibMethod CalibMeth;
	double Caliblimtx; //当使用半径对齐时兼做半径
	double Caliblimty; //当使用半径对齐时无效果

	//刹车控制限定值
	uint16_t BreakStep;			   //刹车控制周期数
	double BreakIntervalSleepTime; //刹车控制周期长度s

	//控制参数(一般不修改)
	double P_FACTOR;

	void DefaultDataInit();

	////////////////////////////////////////////////////////

	MissionPoint();
	MissionPoint(uint8_t Payload, MPStorageMethod StMeth, double destx, double desty);
	MissionPoint(uint8_t Payload, MPStorageMethod StMeth, double destx, double desty, uint16_t Breakstp);
	MissionPoint(uint8_t Payload, MPStorageMethod StMeth, double destx, double desty, double calibx, double caliby,double facp);
	bool inRange(double dx, double dy);
	// bool inRange(double curr_x, double curr_y, double dest_x, double dest_y);
};

void MissionPoint::DefaultDataInit()
{
	this->MissionPayload = 0;
	this->xCoordinate = 0;
	this->yCoordinate = 0;
	this->RestrainSpeed = 0.55;
	this->MaxCtrlStep = 400;
	this->IntervalSleepTime = 0.05;
	this->CalibMeth = MPCalibMethod_SQUARE;
	this->Caliblimtx = 0.05;
	this->Caliblimty = 0.05;
	this->BreakStep = 40;
	this->BreakIntervalSleepTime = 0.01;
	this->P_FACTOR = 1.0;
	return;
}

MissionPoint::MissionPoint()
{
	this->DefaultDataInit();
}

MissionPoint::MissionPoint(uint8_t Payload, MPStorageMethod StMeth, double destx, double desty)
{
	this->DefaultDataInit();
	this->MissionPayload = Payload;
	this->StorgMeth = StMeth;
	this->xCoordinate = destx;
	this->yCoordinate = desty;
}

MissionPoint::MissionPoint(uint8_t Payload, MPStorageMethod StMeth, double destx, double desty, double calibx, double caliby,double facp)
{
	this->DefaultDataInit();
	this->MissionPayload = Payload;
	this->StorgMeth = StMeth;
	this->xCoordinate = destx;
	this->yCoordinate = desty;
	this->Caliblimtx = calibx;
	this->Caliblimty = caliby;
	this->P_FACTOR = facp;
}

MissionPoint::MissionPoint(uint8_t Payload, MPStorageMethod StMeth, double destx, double desty, uint16_t Breakstp)
{
	this->DefaultDataInit();
	this->MissionPayload = Payload;
	this->StorgMeth = StMeth;
	this->xCoordinate = destx;
	this->yCoordinate = desty;
	this->BreakStep = Breakstp;
}

bool MissionPoint::inRange(double dx, double dy)
{
	double dxabs, dyabs;
	dxabs = fabs(dx);
	dyabs = fabs(dy);
	switch (this->CalibMeth)
	{
	//正方形对齐
	case MPCalibMethod_SQUARE:
	{
		return dxabs < Caliblimtx && dyabs < Caliblimty;
	}
	break;
	//圆形对齐
	case MPCalibMethod_RADIUS:
	{
		return dxabs * dxabs + dyabs * dyabs < Caliblimtx * Caliblimtx;
	}
	break;
	default:
		break;
	}
	return false;
}

// bool MissionPoint::inRange(double curr_x, double curr_y, double dest_x, double dest_y)
// {
// 	double dxabs, dyabs;
// 	dxabs = fabs(dest_x - curr_x);
// 	dyabs = fabs(dest_y - curr_y);
// 	switch (this->CalibMeth)
// 	{
// 	//正方形对齐
// 	case MPCalibMethod_SQUARE:
// 	{
// 		return dxabs < Caliblimtx && dyabs < Caliblimty;
// 	}
// 	break;
// 	//圆形对齐
// 	case MPCalibMethod_RADIUS:
// 	{
// 		return dxabs * dxabs + dyabs * dyabs < Caliblimtx * Caliblimtx;
// 	}
// 	break;
// 	default:
// 		break;
// 	}
// 	return false;
// }