
// DSPIC33FJ32MC204 Configuration Bit Settings

// 'C' source line config statements

// FBS

#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)

// FGS
#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)

// FOSCSEL
#pragma config FNOSC = PRIPLL           // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)

// FOSC
#pragma config POSCMD = EC              // Primary Oscillator Source (Primary Oscillator Disabled)
#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)

// FWDT
#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
#pragma config FWDTEN = ON              // Watchdog Timer Enable (Watchdog timer always enabled)

// FPOR
#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)

// FICD
#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)

// #pragma config statements should precede project file includes.
// Use project enums instead of #define for ON and OFF.



#include <xc.h>
#include "xc.h"
#include <string.h>
#include <ctype.h>
#include <stdio.h>
#include <stdbool.h>

#define baud_9600 1041 // Este valor se define por la formula de UxBRG 40 * 106 / 4*1960;
#define led_D1red    LATBbits.LATB7
#define led_D2green  LATAbits.LATA7


void delay_ms(unsigned long time_ms){
    unsigned long u;
    for(u=0; u< time_ms*900; u++){      // Cálculo aproximado para una CPU a 4MHz
        asm("NOP");
    }
}

void uart_config(unsigned int baud){
    
    // Configuracion de pines tx y rx(C0)
    TRISCbits.TRISC0  = 1;   //Pin de recepcion de uart establecido como entrada
    RPINR18bits.U1RXR = 23 ; // El registro de I/O de UART1 Receive tiene el registro 18 y subregistro u1rxr
                             // El pin de recepcion rc0 trabajando con el modulo uart (RP23 lo pone en el pin de entrada de Proteus)
    RPOR12bits.RP24R   = 3;   // Tambien buscando "RP24" en el datasheet (el 17  que es lo que pone en el pin de entrada de Proteus)    
                              // a 3 que es los bits que pone en el datasheet para rp24 y U1TX (UART1 transmit))
    
    
    
    // Configuración de registro U1MODE
    U1MODEbits.UARTEN = 0;  // Deshabilitar Uart. (Se vuelve a habilitar al final)
    U1MODEbits.USIDL  = 0;  // Continuar operación aunque esté en modo IDLE
    U1MODEbits.IREN   = 0;  // IR no usado
    U1MODEbits.RTSMD  = 1;  // Control de flujo desactivado
    U1MODEbits.UEN    = 0;  // Solo usamos pin de Tx(transmision) y Rx(recepcion)
    U1MODEbits.WAKE   = 0;  // No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0;  // Loopback deshabilitado
    U1MODEbits.ABAUD  = 0;  // Automedición de Baudios (bps) deshabilitada
    U1MODEbits.URXINV = 0;  // En estado de reposo, el receptor mantiene un estado alto
    U1MODEbits.BRGH   = 1;  // Modo High-Speed
    U1MODEbits.PDSEL  = 0;  // 8 bits de datos y paridad Nula (8N)
    U1MODEbits.STSEL  = 0;  // 1-bit de stop al final de la trama de datos (8N1)
    
    // Configuración de registro U1STA
    U1STAbits.UTXISEL0 = 0;
    U1STAbits.UTXISEL1 = 0;
    U1STAbits.UTXINV   = 0; // Estado de reposo del pin de transmisión es por defecto 1 _> High
    U1STAbits.UTXBRK   = 0; // No usamos trama de sincronización
    U1STAbits.UTXEN    = 1; // El transmisor en pleno funcionamiento
    U1STAbits.URXISEL  = 0; // Tema interrupciones(no mirar aun)
    U1STAbits.ADDEN    = 0; // No usamos direccionamiento
    //U1STAbits.RIDLE    = 0;
    U1STAbits.OERR     = 0; // Reseteamos buffer de recepcion
    
    // Configuramos la velocidad de transmisión/recepción de los datos
    U1BRG = baud;
    
    U1MODEbits.UARTEN = 1; //Uart habilitada por completo.
}


int main(void) {   
    //Fosc = Fin * (M1/(N1*N2))
    //Fosc = 8MHz * (40/(2*2))   
    PLLFBD = 38;                // M = 40;
    CLKDIVbits.PLLPOST = 0;     //N1 = 2;
    CLKDIVbits.PLLPRE= 0;       //N2 = 2;
    while(OSCCONbits.LOCK != 1); // Wait for PLL to LOCK
    
    AD1PCFGL = 0XFFFF; //Primer paso. Todos los pines configurados como digitales
    
    TRISBbits.TRISB7 = 0;   // Configurar el pin RB3 del puerto B como salida   (D1)
    TRISAbits.TRISA7 = 0;   // Configurar el pin RA0 del puerto A como salida   (D2)
    
    delay_ms(10);   // Pequeña espera para asegurar la configuración anterior antes de continuar
    
    LATBbits.LATB7 = 0;    // Escribir un '0' o estado low, por defecto, en el pin RB3 (D1)
    LATAbits.LATA0 = 0;     // Escribir un '0' o estado low, por defecto, en el pin RA0 (D2)
    
    delay_ms(10);   // Pequeña espera para asegurar la configuración anterior antes de continuar
    
    uart_config(baud_9600);
    //variables necesarias
    int contador = '0';
    char nombre[] = "Rocio";
    bool Hpressed = false;
    bool ms500 = true;
    int j = 0;
    
    while(j < sizeof(nombre)){
        U1TXREG = nombre[j];
        j++;
    }

    while(1){ 
        
        //primer ejercicio -ª Lo hemos intentado con sprintf pero no imprime nada.
        if(ms500 == true){
            U1TXREG = contador;
           (int)contador++;
        }
        ms500 = !ms500;
 
        //segundo ejercicio 
        if(U1STAbits.URXDA == 1){
            if(U1RXREG == 69 || U1RXREG == 101 ){ // Tecla E o e
                led_D1red = 1;        // Encender led D2
            }
            else if(U1RXREG == 65 || U1RXREG == 97){ //Tecla A o a
                led_D1red = 0;        // Apagar led D2
            }
            
            //tercer ejercicio
            if(U1RXREG == 72 || U1RXREG == 104){ // Tecla H o h
                Hpressed = !Hpressed;                   
            }
       
            //cuarto ejercicio
            if(U1RXREG == 32){
                contador = '0';
            }
        }
        //tercer ejercicio
        if(Hpressed == true){
                led_D2green = !PORTAbits.RA7;
            }else if(Hpressed == false){
                led_D2green = 0;
            }     
        delay_ms(250);    
    }
    
    
    return 0;
}

