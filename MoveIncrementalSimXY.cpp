#include <iostream>
#include <unistd.h>
#include "../Include/FAS_EziMOTIONPlusE.h"

using namespace PE;

#define TCP 0
#define UDP 1

bool Connect(int nCommType, unsigned char* byIP, int nBdID);
bool CheckDriveErr(int nBdID);
bool SetServoOn(int nBdID);
bool MoveLinearIncPos(int* nBdID);
bool MoveLinearAbsPos(int* nBdID);

#define SLAVE_CNT 3

int main()
{
	unsigned char byIP[SLAVE_CNT][4] =
	{
		{ 192, 168, 0, 2 },
        { 192, 168, 0, 3 },
		{ 192, 168, 0, 4 }
	};
	int nBdID[SLAVE_CNT] = { 0, 1, 2 };

	for (int nID = 0; nID < SLAVE_CNT; nID++)
	{
		// Device Connect
		if (Connect(TCP, byIP[nID], nBdID[nID]) == false)
		{
			getchar();
			exit(1);
		}

		// Drive Error Check
		if (CheckDriveErr(nBdID[nID]) == false)
		{
			getchar();
			exit(1);
		}

		// ServoOn 
		if (SetServoOn(nBdID[nID]) == false)
		{
			getchar();
			exit(1);
		}
	}

	// Move LinearIncPos
	if (MoveLinearIncPos(nBdID) == false)
	{
		getchar();
		exit(1);
	}

	// Move LinearAbsPos
	// if (MoveLinearAbsPos(nBdID) == false)
	// {
	// 	getchar();
	// 	exit(1);
	// }

	// Connection Close
	for (int nID = 0; nID < SLAVE_CNT; nID++) FAS_Close(nBdID[nID]);

	getchar();
}

bool Connect(int nCommType, unsigned char* byIP, int nBdID)
{
	bool bSuccess = true;

	// Connection
	switch (nCommType)
	{
	case TCP:
		// TCP Connection
		if (FAS_ConnectTCP(byIP[0], byIP[1], byIP[2], byIP[3], nBdID) == 0)
		{
			printf("[nBdID : %d ] TCP Connection Fail! \n", nBdID);
			bSuccess = false;
		}
		break;

	case UDP:
		// UDP Connection
		if (FAS_Connect(byIP[0], byIP[1], byIP[2], byIP[3], nBdID) == 0)
		{
			printf("[nBdID : %d ] UDP Connection Fail! \n", nBdID);
			bSuccess = false;
		}
		break;

	default:
		printf("[nBdID : %d ] Wrong communication type. \n", nBdID);
		bSuccess = false;

		break;
	}

	if (bSuccess)
		printf("[nBdID : %d ] Connected successfully. \n", nBdID);

	return bSuccess;
}

bool CheckDriveErr(int nBdID)
{
	// Check Drive's Error
	EZISERVO2_AXISSTATUS AxisStatus;

	if (FAS_GetAxisStatus(nBdID, &(AxisStatus.dwValue)) != FMM_OK)
	{
		printf("[nBdID : %d ] Function(FAS_GetAxisStatus) was failed.\n", nBdID);
		return false;
	}

	if (AxisStatus.FFLAG_ERRORALL)
	{
		// if Drive's Error was detected, Reset the ServoAlarm
		if (FAS_ServoAlarmReset(nBdID) != FMM_OK)
		{
			printf("[nBdID : %d ] Function(FAS_ServoAlarmReset) was failed.\n", nBdID);
			return false;
		}
	}

	return true;
}

bool SetServoOn(int nBdID)
{
	// Check Drive's Servo Status
	EZISERVO2_AXISSTATUS AxisStatus;

	// if ServoOnFlagBit is OFF('0'), switch to ON('1')
	if (FAS_GetAxisStatus(nBdID, &(AxisStatus.dwValue)) != FMM_OK)
	{
		printf("[nBdID : %d ] Function(FAS_GetAxisStatus) was failed.\n", nBdID);
		return false;
	}

	if (AxisStatus.FFLAG_SERVOON == 0)
	{
		if (FAS_ServoEnable(nBdID, 1) != FMM_OK)
		{
			printf("[nBdID : %d ] Function(FAS_ServoEnable) was failed.\n", nBdID);
			return false;
		}

		do
		{
			usleep(1000);

			if (FAS_GetAxisStatus(nBdID, &(AxisStatus.dwValue)) != FMM_OK)
			{
				printf("[nBdID : %d ] Function(FAS_GetAxisStatus) was failed.\n", nBdID);
				return false;
			}

			if (AxisStatus.FFLAG_SERVOON)
				printf("[nBdID : %d ] Servo ON \n", nBdID);
		}
		while (!AxisStatus.FFLAG_SERVOON); // Wait until FFLAG_SERVOON is ON
	}
	else
	{
		printf("[nBdID : %d ] Servo is already ON \n", nBdID);
	}

	return true;
}

bool MoveLinearIncPos(int* nBdID)
{
	// Move Linear IncPos
	EZISERVO2_AXISSTATUS AxisStatus;
	int lIncPos[3] = {1000, -50000, -50000};
	int lVelocity = 0;
	unsigned short wAccelTime = 100;

	printf("---------------------------\n");
	// Increase the motor by 370000 pulse (target position : Relative position)

	lVelocity = 3000;

	printf("[Linear Inc Mode] Move Motor! \n");

	if (FAS_MoveLinearIncPos2(3, nBdID, lIncPos, lVelocity, wAccelTime) != FMM_OK)
	{
		printf("Function(FAS_MoveLinearIncPos2) was failed.\n");
		return false;
	}

	// Check the Axis status until motor stops and the Inposition value is checked
	for (int nID = 0; nID < SLAVE_CNT; nID++)
	{
		do
		{
			usleep(1000);

			if (FAS_GetAxisStatus(nBdID[nID], &(AxisStatus.dwValue)) != FMM_OK)
			{
				printf("[nBdID : %d ] Function(FAS_GetAxisStatus) was failed.\n", nBdID[nID]);
				return false;
			}
		}
		while (AxisStatus.FFLAG_MOTIONING || !(AxisStatus.FFLAG_INPOSITION));
	}

	return true;
}

bool MoveLinearAbsPos(int* nBdID)
{
	// Move Linear AbsPos 
	EZISERVO2_AXISSTATUS AxisStatus;
	int lAbsPos[3] = {0, 0, 0 };
	int lVelocity = 0;
	unsigned short wAccelTime = 100;

	printf("---------------------------\n");

	// Move the motor by 0 pulse (target position : Absolute position)
	lVelocity = 40000;

	printf("[Linear Abs Mode] Move Motor! \n");
	if (FAS_MoveLinearAbsPos2(3, nBdID, lAbsPos, lVelocity, wAccelTime) != FMM_OK)
	{
		printf("Function(FAS_MoveLinearAbsPos2) was failed.\n");
		return false;
	}

	// Check the Axis status until motor stops and the Inposition value is checked
	for (int nID = 0; nID < SLAVE_CNT; nID++)
	{
		do
		{
			usleep(1000);

			if (FAS_GetAxisStatus(nBdID[nID], &(AxisStatus.dwValue)) != FMM_OK)
			{
				printf("[nBdID : %d ] Function(FAS_GetAxisStatus) was failed.\n", nBdID[nID]);
				return false;
			}
		}
		while (AxisStatus.FFLAG_MOTIONING || !(AxisStatus.FFLAG_INPOSITION));
	}

	return true;
}
  
