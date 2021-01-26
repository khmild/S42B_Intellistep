#ifndef __KEY_H
#define __KEY_H

#include "main.h"

#define MENU_NUM 7 // Menu item

#define KEY_Select      PAin(3)
#define KEY_Back        PBin(0)
#define KEY_Confirm     PBin(1)

//#define k3     PAin(2)
//#define k4     PAin(3)

void Key_init(void);

void Key_test(void);

void KeyScan(void);

bool checkSelectKey();

bool checkBackKey();

bool checkConfirmKey();

#endif


