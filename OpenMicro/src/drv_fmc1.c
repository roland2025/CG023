
#include "project.h"
#include "drv_fmc.h"

#define fl_offset 0x08007C00    // last K of 32k


uint8_t fmc_write( uint32_t address , uint32_t data)
{
    FLASH_Unlock();

    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    uint8_t flashstatus=FLASH_ProgramWord(address * 4+fl_offset, data);
    
    FLASH_Lock();

    return 0;
}



uint32_t fmc_read( uint32_t address)
{
    address = address * 4 + fl_offset;
    uint32_t *addressptr = (uint32_t *)address;
    return (*addressptr);
}



uint8_t fmc_erasepage(void)
{
    FLASH_Unlock();
    FLASH_ClearFlag(FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPERR);
    FLASH_ErasePage((uint32_t) fl_offset);
    FLASH_Lock();
    return 0;
}



int fmc_write1( int data1 , int data2)
{

    FLASH_Unlock();
    FLASH_OB_Unlock();


  FLASH_OB_Erase();
//	
//	if ( flashstatus == FLASH_ERROR_PROGRAM || flashstatus == FLASH_ERROR_WRP || flashstatus ==  FLASH_TIMEOUT)
//	{
//	//	handle error
//	flasherror = 1;	
//	}

    FLASH_OB_ProgramData( 0x1FFFF804, data1 );


    FLASH_OB_ProgramData( 0x1FFFF806, data2 );


    FLASH_Lock();
    FLASH_OB_Lock();

    return 0;
}



int readdata( unsigned int data )
{
    // checks that data and ~data are valid
    unsigned int userdata = data ;
    int complement = ((userdata &0x0000FF00)>>8 );
    complement |=0xFFFFFF00;

    userdata&=0x000000FF;

    if ( userdata!=~complement) 
        return 127;

    else return userdata;
}

