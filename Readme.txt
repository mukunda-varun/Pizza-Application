----------COMPILER ENABLE/DISABLES----------

androidParameter - > androidParameter = 0; for default Value & androidParameter = 1; for android Value to be taken from android

bladePWM -> bladePWM = 0; for SSR control & bladePWM = 1; for android Value to be taken for Motor control

doughReleaseEnable -> doughReleaseEnable = 0; for dough Release functionality to disable & doughReleaseEnable = 1; for dough Release functionality to enable

CYTRON_DRV -> Press Motor Driver (2 = VNH7070, 1 = CYTRON, 0 = BTN7960)

#define		MACHINE_1						0				//1 for Alpha M1 & 0 for Alpha M2

KNEADING_TIME_REDUCE -> Kneading Time Reducing Macro(For dispensing flour during acceleration, dispense flour during coating), 1 = Enable reduction ; 0 = Disable Reduction

BOARD_V2_0 -> Specify board version for dc motor's control, 1 for latest(v2.0) pcb & 0 for other version of pcb

FLOUR_PRESENCE_SENSOR -> To enable/disable the flour connector presence

COMPLETE_PROCESS -> Enabling Complete process or only kneading(Was made to test the kneading time separately, but not tested)

INTERLOCK_EN -> Enable/Disable the interlock mechanism (INTERLOCK_EN = 1, Enable & INTERLOCK_EN = 0, Disable)

----------COMPILER ENABLE/DISABLES----------