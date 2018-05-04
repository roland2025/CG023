
#include "binary.h"
#include "drv_spi.h"

#include "project.h"
#include "xn297.h"


void xn_writereg( int reg , int val)
{
	reg = reg&0x0000003F;
	reg = reg|W_REGISTER;
	spi_cson();
	spi_sendbyte( reg);
	spi_sendbyte( val);
	spi_csoff();
}

void xn_writeregmulti(uint8_t reg, const uint8_t data[], uint8_t length)
{
	reg = reg&REGISTER_MASK;
	reg = reg|W_REGISTER;
	spi_cson();
	spi_sendbyte( reg);
	for (uint8_t i = 0; i < length; i++)
	{
		spi_sendbyte( data[i]);
	}
	spi_csoff();
}

int xn_readreg( int reg)
{
	reg = reg&REGISTER_MASK;
	spi_cson();
	spi_sendrecvbyte( reg);
	int val =spi_sendzerorecvbyte();
	spi_csoff();
	return val;
}

int xn_command( int command)
{
	spi_cson();
	spi_sendbyte(command);
	spi_csoff();
	return 0;
}

int xn_activate( int command)
{
	spi_cson();
	spi_sendbyte(ACTIVATE);
	spi_sendbyte(command);
	spi_csoff();
	return 0;
}


void xn_readpayload( int *data , int size )
{
	int index = 0;
	spi_cson();
	spi_sendrecvbyte( R_RX_PAYLOAD ); // read rx payload
	while( index < size )
	{
	data[index]= spi_sendzerorecvbyte();
	index++;
	}
	spi_csoff();
}



void xn_writerxaddress(  int *addr )	
{
    int index = 0;
    spi_cson();
	while(index<5)
        {
        spi_sendbyte( addr[index] );
        index++;
        }
    spi_sendbyte(W_REGISTER | RX_ADDR_P0);
    spi_csoff();
}


void xn_writetxaddress(  int *addr )	
{
    int index = 0;
    spi_cson();
	while(index<5)
    spi_sendbyte(W_REGISTER|TX_ADDR);
        {
        spi_sendbyte( addr[index] );
        index++;
        }
    spi_csoff();
}


void xn_writepayload( int data[] , int size )
{
	int index = 0;
	spi_cson();
	spi_sendrecvbyte( W_TX_PAYLOAD ); // write tx payload
	while(index<size)
        {
        spi_sendrecvbyte( data[index] );
        index++;
        }
	spi_csoff();
}













