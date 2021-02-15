#include "flash.h"


// Check if the stepper is calibrated
bool isCalibrated() {

	// Unlock the flash for reading
	unlockFlash();

	// Get the calibration flag
	uint16_t Calibration_flag = flashReadHalfWord(DATA_STORE_ADDRESS);

	// Lock the flash again
	lockFlash();

	// Return if the unit is calibrated
	return ((Calibration_flag >> 8) == 0xAA);
}


// Check if the flash is busy
uint8_t flashGetStatus(void) {

	// Check if the flash state
	uint32_t state = FLASH -> SR;

	if(state & (1<<0)) 
		// Flash busy
		return 1; 

	else if(state & (1<<2)) 
		// Programming error
		return 2;

	else if(state & (1<<4))
		// ! Fix, unknown
		return 3;

	// No result found
	return 0;     
}


// Function to delay execution until the flash is able to to be read again
bool flashWaitDone(uint16_t time) {

    // ! Function needs fixing, it should check the actual system time instead of waiting

    // Create an accumulator for the result
	uint8_t result;

    // Loop forever, checking the value each time
	do {
        // Get the status of the flash
		result = flashGetStatus();

        // If the flash isn't busy, exit the loop
		if(result != 1) {
			break;
        }

        // Delay for a microsecond
		delayMicroseconds(1);

        // Increment the time counter
		time--;

	} while(time);

    // If we ran out of time, return a fail
	if(time == 0) {
		return false;
    }

    // Return the result
	return true;
}


// Erases a full page of flash
bool flashErasePage(uint32_t address) {

    // Wait until the flash is available,  if so, begin writing
	if(flashWaitDone(0X5FFF)) {
		FLASH -> CR |= 1 << 1; 
		FLASH -> AR = address; 
		FLASH -> CR |= 1 << 6; 

        // If the writing operation succeeded 
		if(flashWaitDone(0X5FFF)) {
			FLASH -> CR &= ~(1 << 1);

            // Operation successful
            return true;
		}

        // Failed to actually write the data
        return false;
	}
    else {
        // Flash wasn't available, couldn't write at all
        return false;
    }
}


// ! Write half 
uint8_t flashWriteHalfWord(uint32_t address, uint16_t data)
{
	// Result of the operation
	uint8_t result;

	// Wait until the flash is available
	result = flashWaitDone(0XFF);

	// If we didn't time out, we can proceed
	if(result == 0) {

		// 
		FLASH -> CR |= 1 << 0;

		// Write the data to the address
		*(volatile uint16_t*) address = data; 

		// Wait until the data is written
		result = flashWaitDone(0XFF);
		if(result != 1){
			FLASH -> CR &= ~(1 << 0); 
		}
	}
	return result;
}


uint16_t flashReadHalfWord(uint32_t address)
{
	return *(volatile uint16_t*) address;
}
	

void flashRead(uint32_t address,uint16_t *pBuffer)   	
{
	uint16_t i;
	uint16_t arrayLength = sizeof(pBuffer) / sizeof(pBuffer[0]);
	for(i = 0; i < arrayLength; i++) {
		pBuffer[i]=flashReadHalfWord(address);
		address +=2;
	}
}


void flashWriteNoCheck(uint32_t WriteAddr, uint16_t *pBuffer)   
{ 			 		 
	uint16_t i;
	uint16_t arrayLength = sizeof(pBuffer) / sizeof(pBuffer[0]);
	for(i = 0; i < arrayLength; i++) {
		// ! FLASH_ProgramHalfWord(WriteAddr, pBuffer[i]);
		WriteAddr += 2;
	}  
} 


#define STM32_FLASH_SIZE STM32_memory_size
#if STM32_FLASH_SIZE < 256
#define STM_SECTOR_SIZE 1024 //
#else 
#define STM_SECTOR_SIZE	2048
#endif		 

uint16_t STMFLASH_BUF[STM_SECTOR_SIZE / 2];


void flashWrite(uint32_t WriteAddr, uint16_t *pBuffer)	
{
	uint32_t secpos;	   // Sector position
	uint16_t secoff;	   // Sector offset
	uint16_t secremain; // 
	 uint16_t i;    
	uint32_t offaddr;   // Offset address
	uint16_t arrayLength = sizeof(pBuffer)/sizeof(pBuffer[0]);
	
	if(WriteAddr < FLASH_BASE||(WriteAddr>=(FLASH_BASE+1024*STM32_FLASH_SIZE)))return;//
	unlockFlash();						//
	offaddr = WriteAddr - FLASH_BASE;		//
	secpos = offaddr / STM_SECTOR_SIZE; //  0~63 for STM32F030
	secoff=(offaddr%STM_SECTOR_SIZE)/2;		//
	secremain = STM_SECTOR_SIZE / 2 - secoff;		//   
	if(arrayLength<=secremain)secremain=arrayLength;//
	while(1) 
	{	
		flashRead(secpos*STM_SECTOR_SIZE+FLASH_BASE,STMFLASH_BUF);//
		for(i=0;i<secremain;i++)//
		{
			if(STMFLASH_BUF[secoff+i]!=0XFFFF)break;//  	  
		}
		if(i<secremain)//
		{
			flashErasePage(secpos*STM_SECTOR_SIZE+FLASH_BASE);//
			for(i=0;i<secremain;i++)//
			{
				STMFLASH_BUF[i+secoff]=pBuffer[i];	  
			}
			flashWriteNoCheck(secpos*STM_SECTOR_SIZE+FLASH_BASE,STMFLASH_BUF);//  
		} else flashWriteNoCheck(WriteAddr, pBuffer);
		if(arrayLength==secremain)break;//
		else//
		{
			secpos++;				//
			secoff=0;				// 	 
				 pBuffer+=secremain;  	
			WriteAddr+=secremain*2;	
				 arrayLength-=secremain;	//
			if(arrayLength>(STM_SECTOR_SIZE/2))secremain=STM_SECTOR_SIZE/2;//
			else secremain=arrayLength;//
		}	 
	};	
	lockFlash();//
}


// Erase all of the data stored
void flashErase32K(void)
{
	flashErasePage(0x08018000);
	flashErasePage(0x08018400);
	flashErasePage(0x08018800);
	flashErasePage(0x08018C00);
	flashErasePage(0x08019000);
	flashErasePage(0x08019400);
	flashErasePage(0x08019800);
	flashErasePage(0x08019C00);
	flashErasePage(0x0801A000);
	flashErasePage(0x0801A400);
	flashErasePage(0x0801A800);
	flashErasePage(0x0801AC00);
	flashErasePage(0x0801B000);
	flashErasePage(0x0801B400);
	flashErasePage(0x0801B800);
	flashErasePage(0x0801BC00);
	flashErasePage(0x0801C000);
	flashErasePage(0x0801C400);
	flashErasePage(0x0801C800);
	flashErasePage(0x0801CC00);
	flashErasePage(0x0801D000);
	flashErasePage(0x0801D400);
	flashErasePage(0x0801D800);
	flashErasePage(0x0801DC00);
	flashErasePage(0x0801E000);
	flashErasePage(0x0801E400);
	flashErasePage(0x0801E800);
	flashErasePage(0x0801EC00);
	flashErasePage(0x0801F000);
	flashErasePage(0x0801F400);
	flashErasePage(0x0801F800); 
	flashErasePage(0x0801FC00);
}


// Function to allow flash to be tampered with
void unlockFlash() {

    // Directly from the STM32 flash library
    FLASH -> KEYR = FLASH_KEY1;
    FLASH -> KEYR = FLASH_KEY2;
}


// Function to lock flash, preventing it from being tampered with accidentally
void lockFlash() {

    // Directly from the STM32 flash library
    FLASH -> CR |= CR_LOCK_SET;

}


// Loads the saved parameters from flash and sets them
void loadSavedValues() {
    // ! Write yet
}