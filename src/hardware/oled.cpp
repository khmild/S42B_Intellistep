#include "oled.h"
#include "cstring"
#include "math.h"

// The index of the current top level menu item
int topLevelMenuIndex = 0;
int lastTopLevelMenuIndex = 0;

// The current value of the cursor
int currentCursorIndex = 0;

// The current menu depth. 0 would be the motor data, 1 the main menu, and 2 any submenus
int menuDepthIndex = 0;
int lastMenuDepthIndex = 0;

// Convience function for writing a specific string to the OLED panel
void writeOLEDString(uint8_t x, uint8_t y, String string) {
    OLED_ShowString(x, y, string.c_str());
}

// Function for displaying relevant data on the OLED panel, such as motor data or menu data
void updateDisplay() {

    // Decide on the current depth of the menu
    switch(menuDepthIndex){
        case 0:
            // Not actually in the menu, just displaying motor data for now
            displayMotorData();
            break;

        case 1:
            // In the top level of the menu. Make sure that the set top level menu index is within the range of the menu length
            // Values must first be determined using the mod function in order to prevent overflow errors
            // Then the strings are converted into character arrays by giving the address of the first character in the string
            clearOLED();
            writeOLEDString(0, 0, "->");
            writeOLEDString(25, 0,  &topLevelMenuItems[(topLevelMenuIndex)     % topLevelMenuLength][0]);
            writeOLEDString(25, 16, &topLevelMenuItems[(topLevelMenuIndex + 1) % topLevelMenuLength][0]);
            writeOLEDString(25, 32, &topLevelMenuItems[(topLevelMenuIndex + 2) % topLevelMenuLength][0]);
            writeOLEDString(25, 48, &topLevelMenuItems[(topLevelMenuIndex + 3) % topLevelMenuLength][0]);
            break;

        case 2:
            // We should be in a sub menu, this is where we have to figure out which submenu that should be
            switch(topLevelMenuIndex) {
                case 0:
                    // In the first menu, the calibration one. No need to do anything here, besides maybe displaying an progress bar or PID values (later?)
                    clearOLED();
                    writeOLEDString(0, 0, "Are you sure?");
                    writeOLEDString(0, 16, "Press select");
                    writeOLEDString(0, 32, "to confirm");
                    break;

                case 1:
                    // In the second menu, the motor mAs. This is dynamically generated and has increments every 100 mA from 0 mA (testing only) to 3500 mA
                    clearOLED();

                    // Constrain the current setting within 0 and the maximum current
                    if (currentCursorIndex > MAX_CURRENT / 100) {

                        // Loop back to the start of the list
                        currentCursorIndex = 0;
                    }

                    // Write the pointer
                    writeOLEDString(0, 0, "->");

                    // Write each of the strings
                    for (int stringIndex = 0; stringIndex <= 3; stringIndex++) {

                        // Check to make sure that the current isn't out of range of the max current
                        if (!((currentCursorIndex + stringIndex) * 100 > MAX_CURRENT)) {

                            // Value is in range, display the current on that line
                            writeOLEDString(25, stringIndex * 16, String((int) ((currentCursorIndex + stringIndex) * 100)) + String("mA"));
                        }
                        // else {
                            // Value is out of range, display a blank line for this line
                        // }
                    }
                    break;

                case 2:
                    // In the microstep menu, this is also dynamically generated. Get the current stepping of the motor, then display all of the values around it
                    clearOLED();
                    writeOLEDString(0, 0, "->");

                    // Loop the currentCursor index back if it's out of range
                    if (currentCursorIndex > log2(MAX_MICROSTEP_DIVISOR)) {
                        currentCursorIndex = 2;
                    }
                    else if (currentCursorIndex < 2) {

                        // Make sure that the cursor index is in valid range
                        currentCursorIndex = 2;
                    }

                    // Write each of the strings
                    for (int stringIndex = 0; stringIndex <= 3; stringIndex++) {

                        // Check to make sure that the current isn't out of range of the max current
                        if (!(pow(2, currentCursorIndex + stringIndex) > MAX_MICROSTEP_DIVISOR)) {

                            // Value is in range, display the current on that line
                            writeOLEDString(25, stringIndex * 16, String("1/") + String((int) pow(2, currentCursorIndex + stringIndex)) + String("th"));
                        }
                        // else {
                            // Value is out of range, display a blank line for this line
                        // }
                    }
                    break;

                case 3:
                    // In the enable logic menu, a very simple menu. Just need to invert the displayed state
                    // Clear the OLED
                    clearOLED();

                    // Title
                    writeOLEDString(0, 0, "Enable logic:");

                    // Write the string to the screen
                    if (currentCursorIndex % 2 == 0) {

                        // The index is even, the logic is inverted
                        writeOLEDString(0, 24, "Inverted");
                    }
                    else {
                        // Index is odd, the logic is normal
                        writeOLEDString(0, 24, "Normal");
                    }
                    break;

                case 4:
                    // Another easy menu, just the direction pin. Once again, just need to invert the state
                    clearOLED();

                    // Title
                    writeOLEDString(0, 0, "Dir logic:");

                    // Write the string to the screen
                    if (currentCursorIndex % 2 == 0) {

                        // The index is even, the logic is inverted
                        writeOLEDString(0, 24, "Inverted");
                    }
                    else {
                        // Index is odd, the logic is normal
                        writeOLEDString(0, 24, "Normal");
                    }
                    break;
            } // Top level menu switch
            break;
    } // Main switch

    // Save the last set value (for display functions that don't need to clear the OLED all of the time)
    lastMenuDepthIndex = menuDepthIndex;
    lastTopLevelMenuIndex = topLevelMenuIndex;

} // Display menu function

// Gets the latest parameters from the motor and puts them on screen
void displayMotorData() {

    // Clear the old menu off of the display
    if (!(lastMenuDepthIndex == 0)) {
        clearOLED();
    }

    // RPM of the motor (RPM is capped at 2 decimal places)
    float currentRPM = motor.getMotorRPM();
    writeOLEDString(0, 0, (String("RPM: ") + String(currentRPM)));

    // PID loop error
    float PIDError = motor.getPIDError();
    writeOLEDString(0, 16, (String("Err: ") + String(PIDError)));

    // Current angle of the motor
    double currentAngle = getEncoderAngle();
    writeOLEDString(0, 32, (String("Deg: ") + String(currentAngle)));

    // Maybe a 4th line later?
}


// Function for moving the cursor up
void selectMenuItem() {

    // Go down in the menu index if we're not at the bottom already
    if (menuDepthIndex < 2) {
        menuDepthIndex++;
    }

    // If we're in certain menus, the cursor should start at their current value
    else if (menuDepthIndex == 2) {
        // Cursor settings are only needed in the submenus

        // Check the submenus available
        switch(topLevelMenuIndex % topLevelMenuLength) {

            case 0:
                // Nothing to see here, just the calibration.
                motor.calibrate();

                // Exit the menu
                menuDepthIndex--;

                break;

            case 1:
                // Motor mAs. Need to get the current motor mAs, then convert that to a cursor value
                if (motor.getCurrent() % 100 == 0) {
                    // Motor current is one of the menu items, we can check it to get the cursor index
                    currentCursorIndex = motor.getCurrent() / 100;
                }
                else {
                    // Non-standard value (probably set with serial or CAN)
                    currentCursorIndex = 0;
                }

                // Exit the menu
                menuDepthIndex--;

                break;

            case 2:
                // Motor microstepping. Need to get the current microstepping setting, then convert it to a cursor value. Needs to be -2 because the lowest index, 1/4 microstepping, would be at index 0
                currentCursorIndex = log2(motor.getMicrostepping()) - 2;

                // Exit the menu
                menuDepthIndex--;

                break;

            case 3:
                // Get if the enable pin is inverted
                if (currentCursorIndex % 2 == 0) {

                    // The index is even, the logic is inverted
                    motor.setEnableInversion(true);
                }
                else {
                    // Index is odd, the logic is normal
                    motor.setEnableInversion(false);
                }

                // Exit the menu
                menuDepthIndex--;

                break;

            case 4:
                // Get if the direction pin is inverted
                if (currentCursorIndex % 2 == 0) {

                    // The index is even, the direction is inverted
                    motor.setReversed(true);
                }
                else {
                    // Index is odd, the direction is normal
                    motor.setReversed(false);
                }

                // Exit the menu
                menuDepthIndex--;

                break;
        }

    }
}


// Function for moving the cursor down
void moveCursor() {

    // If we're on the motor display menu, do nothing for now
    if (menuDepthIndex == 0) {
        // Do nothing (maybe add a feature later?)
    }
    else if (menuDepthIndex == 1) {
        // We're in the top level menu, change the topLevelMenuIndex (as long as we haven't exceeded the length of the list)
        if (topLevelMenuIndex + 3 > topLevelMenuLength) {
            topLevelMenuIndex = 0;
        }
        else {
            topLevelMenuIndex++;
        }
    }
    else {
        // We have to be in the submenu, increment the cursor index (submenus handle the display themselves)
        currentCursorIndex++;
    }
}


// Function for exiting the current menu
void exitCurrentMenu() {

    // Go up in the menu index if we're not already at the motor data screen
    if (menuDepthIndex > 0) {
        menuDepthIndex--;
    }
}


// Returns the depth of the menu (helpful for watching the select button)
int getMenuDepth() {
    return menuDepthIndex;
}


// A list of all of the top level menu items
String topLevelMenuItems[] = {
    "Calibrate",
    "Motor mA",
    "Microstep",
    "En Logic",
    "Dir. Logic",
    ""
};

// Length of the list of top menu items (found by dividing the length of the list by how much space a single element takes up)
const int topLevelMenuLength = sizeof(topLevelMenuItems) / sizeof(topLevelMenuItems[0]);





// Old OLED display commands from BTT (will be replaced later)
#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"
//#include "gpio.h"
//////////////////////////////////////////////////////////////////////////////////

#define u8 uint8_t
#define u32 uint32_t

//
//
//[0]0 1 2 3 ... 127
//[1]0 1 2 3 ... 127
//[2]0 1 2 3 ... 127
//[3]0 1 2 3 ... 127
//[4]0 1 2 3 ... 127
//[5]0 1 2 3 ... 127
//[6]0 1 2 3 ... 127
//[7]0 1 2 3 ... 127
u8 OLED_GRAM[128][8];

//
void OLED_Refresh_Gram(void)//
{
	u8 i,n;
	for(i=0;i<8;i++)
	{
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //
		OLED_WR_Byte (0x00,OLED_CMD);      //
		OLED_WR_Byte (0x10,OLED_CMD);      //
		for(n=0;n<128;n++)
		OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA);
	}
}
#if OLED_MODE==1
//
//dat:
//cmd:/ 0;1;
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	DATAOUT(dat);
 	OLED_RS=cmd;
	OLED_CS=0;
	OLED_WR=0;
	OLED_WR=1;
	OLED_CS=1;
	OLED_RS=1;
}
#else
//
//dat:
//cmd:
void OLED_WR_Byte(u8 dat,u8 cmd)
{
	u8 i;
	OLED_RS=cmd; //
	OLED_CS=0;
	for(i=0;i<8;i++)
	{
		OLED_SCLK=0;
		if(dat&0x80)OLED_SDIN=1;
		else OLED_SDIN=0;
		OLED_SCLK=1;
		dat<<=1;
	}
	OLED_CS=1;
	OLED_RS=1;
}
#endif
//
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
//
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}
//!!!
void clearOLED(void)
{
	u8 i,n;
	for(i=0;i<8;i++)
	for(n=0;n<128;n++)
	OLED_GRAM[n][i]=0X00;
	OLED_Refresh_Gram();//
}
//
//x:0~127
//y:0~63
//t:1  0,
///
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//.
	pos=7-y/8;		//
	bx=y%8;			//
	temp=1<<(7-bx);	//
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;
}
//x1,y1,x2,y2
//<=x2;y1<=y2 0<=x1<=127 0<=y1<=63
//dot:0,;1,
void OLED_Fill(u8 x1,u8 y1,u8 x2,u8 y2,u8 dot)
{
	u8 x,y;
	for(x=x1;x<=x2;x++)
	{
		for(y=y1;y<=y2;y++)OLED_DrawPoint(x,y,dot);
	}
	OLED_Refresh_Gram();//
}
//
//x:0~127
//y:0~63
//mode:0,;1,
//size: 16/12
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';//
    for(t=0;t<size;t++)
    {
		if(size==12)temp=oled_asc2_1206[chr][t];  //
		else temp=oled_asc2_1608[chr][t];		 //
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}
    }
	OLED_Refresh_Gram();//
}
//m^n
uint32_t oled_pow(u8 m,u8 n)
{
	uint32_t result=1;
	while(n--)result*=m;
	return result;
}
//
//x,y :
//len :
//size:
//mode:;1,
//num:(0~4294967295);
void OLED_ShowNum(u8 x,u8 y,u32 num,u8 len,u8 size)
{
	u8 t,temp;
	u8 enshow=0;
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1;
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1);
	}
}
//

void OLED_ShowString(u8 x,u8 y,const char *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58
    while(*p!='\0')
    {
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;clearOLED();}
        OLED_ShowChar(x,y,*p,16,1);
        x+=8;
        p++;
    }
}

//SSD1306
void initOLED(void)
{
    // Set all of the menu values back to their defaults (for if the screen needs to be reinitialized)
    topLevelMenuIndex = 0;

    #if 1
	RCC->APB2ENR|=1<<3;    //
	RCC->APB2ENR |=1<<2;		//

    GPIOA->CRH &=0XFFFffFF0;	//
    GPIOA->CRH |=0X00000003;
    GPIOA->ODR |=1<<8;		    //

  	GPIOB->CRH&=0X0000FFFF;//PB
  	GPIOB->CRH|=0X33330000;
	GPIOB->ODR|=0xF<<12;

	#endif
    #if 0
    	RCC->APB2ENR|=1<<3;    //
        RCC->APB2ENR |=1<<2;		//

        GPIOB->CRL&=0X00FFFFFF;//PB6,7
        GPIOB->CRL|=0X33000000;

        GPIOB->CRH&=0XFFFFFFF0;//PB8
        GPIOB->CRH|=0X00000003;
        GPIOB->ODR|=1<<8;

        GPIOA->CRH &=0XFFF00FFF;	//
        GPIOA->CRH |=0X00033000;
        GPIOA->ODR |=1<<11;		    //

    #endif

	OLED_RST=0;			  		//
	delay(100);
	OLED_RST=1;
	OLED_WR_Byte(0xAE,OLED_CMD);//
	OLED_WR_Byte(0xD5,OLED_CMD);//
	OLED_WR_Byte(80,OLED_CMD);  //[3:0],;[7:4],
	OLED_WR_Byte(0xA8,OLED_CMD);//
	OLED_WR_Byte(0X3F,OLED_CMD);//(1/64)
	OLED_WR_Byte(0xD3,OLED_CMD);//
	OLED_WR_Byte(0X00,OLED_CMD);//

	OLED_WR_Byte(0x40,OLED_CMD);// [5:0],.

	OLED_WR_Byte(0x8D,OLED_CMD);//
	OLED_WR_Byte(0x14,OLED_CMD);///
	OLED_WR_Byte(0x20,OLED_CMD);//
	OLED_WR_Byte(0x02,OLED_CMD);//[1:0],;
	OLED_WR_Byte(0xA1,OLED_CMD);//,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD);//;bit3:0,;1, COM[N-1]->COM0;N:
	OLED_WR_Byte(0xDA,OLED_CMD);//
	OLED_WR_Byte(0x12,OLED_CMD);//[5:4]

	OLED_WR_Byte(0x81,OLED_CMD);//
	OLED_WR_Byte(0xEF,OLED_CMD);//1~255;
	OLED_WR_Byte(0xD9,OLED_CMD);//
	OLED_WR_Byte(0xf1,OLED_CMD);//[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD);//
	OLED_WR_Byte(0x30,OLED_CMD);//[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;
	OLED_WR_Byte(0xA4,OLED_CMD);//;bit0:1,;0,;
	OLED_WR_Byte(0xA6,OLED_CMD);//;bit0:1,;0,
	OLED_WR_Byte(0xAF,OLED_CMD);//
	delay(100);
	clearOLED();
}