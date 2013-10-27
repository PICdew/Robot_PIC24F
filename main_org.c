/* 
 * File:   main.c
 *
 * Created on February 9, 2010, 10:53 AM
 */

#include <stdio.h>
#include <stdlib.h>
#include <p24Fxxxx.h>

_CONFIG1( JTAGEN_OFF & GCP_OFF & GWRP_OFF & COE_OFF & FWDTEN_OFF & ICS_PGx2)
_CONFIG2( FCKSM_CSDCMD & OSCIOFNC_OFF & POSCMOD_XT & FNOSC_PRI)

/*
 * 
 */
void delay(void) {
    long i = 65535;
    while(i--)
        ;
}

int main(int argc, char** argv) {
    int counter;
    TRISA = 0;
    
    // Setup PortA IOs as digital
    AD1PCFG = 0xffff;

    while(1) {
        LATA = 0x00;
        delay();

        LATA = 0xFF;
        delay();
    }
    return (EXIT_SUCCESS);
}
