/*
 * Typedef.h
 *
 *  Created on: Mar 8, 2022
 *      Author: asus
 */

#ifndef INC_TYPEDEF_H_
#define INC_TYPEDEF_H_


typedef enum
{
	PCCMV_BACKUP = 0,
	VCCMV_BACKUP ,
	IdleState
}BackupModes;

typedef enum
{
	Generate_I_Wave = 0 ,
	Generate_E_Wave ,
	NoWaveFormState
}WaveFormState;

typedef enum
{
	Compute_I_Wave = 0,
	Compute_E_Wave ,
	NoComputeState
}ComputationState;


typedef enum
{
	PCCMV = 0x01,
	VCCMV = 0x02,
	SIMVPC = 0x03,
	SIMVVC = 0x04,
	APRV = 0x05,
	PSV = 0x06,
	cPAP = 0x07,
	BiPAP = 0x08,
	HFNC  = 0x0C,
	NoMode = 0xFE,
	DebugMode = 0xFF
}CurrentMode;


typedef struct
{
	int P_Max;
	int P_Min;
	float _Pip_Avg;
	float _Pip_Avg_count;
	float _Pip_Avg_val;
	float _Peep_Avg;
	float _Peep_Avg_trigger;
	float _Peep_Avg_count_trigger;
	float _Peep_Avg_count;
	float _Peep_Avg_val;
	int _Peep_Avg_val_int;
	int _Pip_Avg_val_int;
	int error_count;
	int error_count2;
	float _Set_Peep;
	uint16_t Lock_delay;
	int peep_max;
	int lock;
	int now_check;
	int peep_process_done;

}PC_CMV_Mode_Avg_Pip_Peep;


PC_CMV_Mode_Avg_Pip_Peep S5;


CurrentMode _CurrentMode , _RequestedMode ;
WaveFormState _CurrentWaveFormState;
ComputationState _CurrentComputationState;
BackupModes _CurrentBackupMode;

#endif /* INC_TYPEDEF_H_ */
