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
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>          // Defines NULL
#include <stdbool.h>         // Defines true
#include <math.h>




#define baud_9600    51
#define P1_pressed !PORTBbits.RB4
#define P2_pressed !PORTBbits.RB7
#define Led_Red    LATBbits.LATB3






unsigned char received_char = 0;
char txbuffer[200];
unsigned char time_count = 0;







void delay_ms(unsigned long time_ms)
{
    unsigned long u;
    for(u = 0; u < time_ms*90; u++) // C�lculo aproximado para una CPU a 2MHz
    {
        asm("NOP");
    }
}

void  timer1_config(){
    T1CONbits.TON   = 0; //comprobamos que esta apagado
    T1CONbits.TSIDL = 0;
    T1CONbits.TCS   = 0;
    T1CONbits.TGATE = 0;
    T1CONbits.TSYNC = 0;
    T1CONbits.TCKPS = 1; //Preescaler de 8
    PR1 = 24999; //Configuracion del periodo para 0,1 segundo
    T1CONbits.TON = 1;
}

void  timer2_config(){
    //T2CON.T32 = 0;
    T2CONbits.TON   = 0; //comprobamos que esta apagado
    T2CONbits.TSIDL = 0;
    T2CONbits.TCS   = 0;
    T2CONbits.TGATE = 0; 
    T2CONbits.TCKPS = 1;//Preescaler de 8.
    PR2 = 24999; //Configuracion del periodo para 0,1 segundos.
    T2CONbits.TON = 1;
}

void EnviarCaracter(char c)
{    
    while(U1STAbits.UTXBF);   // Mientras el buffer del puerto U1 este lleno, esperar en bucle while
    U1TXREG = c;              // Si no esta lleno, proceder a enviar el byte
}

void EnviarString(char *s)
{    
    while((*s) != '\0') EnviarCaracter(*(s++));  // Mientras no se haya llegado al caracter nulo (final de trama), continuar imprimiendo datos.
}                                                // *s es un puntero que apunta hacia la direcci�n del string de datos que le indiquemos. (*(s++) toma el contenido actual, y posteriormente aumenta el valor de la direcci�n (siguiente caracter))



void uart_config (unsigned int baud)
{    
    // Configuraci�n de pines tx y rx
    TRISCbits.TRISC0  = 1;   // Pin de recepcion de uart establecido como entrada.
    RPINR18bits.U1RXR = 16;  // pin de recepcion rc0 trabajando con el modulo uart (RP16)
    RPOR8bits.RP17R   = 3;   // U1TX conectado con el pin RC1 (RP17)
    
    
    
    // Configuraci�n de registro de U1MODE
    U1MODEbits.UARTEN = 0;     // Deshabilitar Uart.
    U1MODEbits.USIDL  = 0;     // Continuar operaci�n en modo IDLE
    U1MODEbits.IREN   = 0;     // IR no usado
    U1MODEbits.RTSMD  = 1;     // Control de flujo desactivado.
    U1MODEbits.UEN    = 0;     // Solo usamos pin de Tx y pin de Rx
    U1MODEbits.WAKE   = 0;     // No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0;     // Loopback deshabilitado.
    U1MODEbits.ABAUD  = 0;     // Automedici�n de baudios (bps) deshabilidada
    U1MODEbits.URXINV = 0;     // En estado de reposo, el receptor mantiene un estado alto, high
    U1MODEbits.BRGH   = 1;     // Modo High-Speed
    U1MODEbits.PDSEL  = 0;     // 8 Bits de datos y paridad Nula (8N)
    U1MODEbits.STSEL  = 0;     // 1-bit de stop al final de la trama de datos.   (8N1)

    
    // Configuraci�n de registro de U1STA
    U1STAbits.UTXISEL0 = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.UTXISEL1 = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.UTXINV   = 0;    // El estado en reposo del pin de transmisi�n es High
    U1STAbits.UTXBRK   = 0;    // No usamos trama de sincronizaci�n
    U1STAbits.UTXEN    = 1;    // El transmisor a pleno funcionamiento.
    U1STAbits.URXISEL  = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.ADDEN    = 0;    // No usamos direccionamiento.
    //U1STAbits.RIDLE    = 0;
    U1STAbits.OERR     = 0;    // Reseteamos buffer de recepci�n

    
    // Configuramos la velocidad de transmisi�n/recepcci�n de los datos
    U1BRG = baud;

    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
}






int main(void) 
{
    //Configurar el oscilador para hacer funcionar la CPU a 2 MHz a partir de un reloj de entrada de 8MHz
    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    //Fosc = 8M * 2/(2 * 2) = 4 MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2 = 4/2 = 2MHz (Frecuencia CPU)
    PLLFBD = 0;                     // M  = 2
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    CLKDIVbits.PLLPRE  = 0;         // N2 = 2
    while(OSCCONbits.LOCK != 1);    // Esperar a un PLL estable      
    
    
    AD1PCFGL         = 0xFFFF;      // Primer paso. Todos los pines configurados como pines digitales
    TRISBbits.TRISB3 = 0;   //B3 como salida (LED))
    LATBbits.LATB3   = 0; 
    TRISBbits.TRISB4 = 1;   //B4 como entrada (P1)
    TRISBbits.TRISB7 = 1;   //B7 como entrada (P2))
 
    uart_config(baud_9600);
    timer1_config();
    timer2_config();

    char txbuffer[100];
    long aux;
    while(1)
    { 
        while(!IFS0bits.T1IF); //Se pone a 1 cuado se alcanza la temporizacion que tenemos
        IFS0bits.T1IF = 0;
        if(time_count++ == 1)
        {
            memset(txbuffer, '/0', sizeof(txbuffer));
            if(P1_pressed){
                sprintf(txbuffer, "P1: Pulsado. ");
            }else{
                sprintf(txbuffer, "P1: No pulsado. ");
            }
            TMR2 = 0;
            EnviarString(txbuffer);
            aux = TMR2;
            printf("TIMER2: %ull.  \r\n", aux*1000);
            if(P2_pressed){
                sprintf(txbuffer, "P2: Pulsado. ");
            }else{
                sprintf(txbuffer, "P2: No pulsado. ");
            }
            TMR2=0;
            EnviarString(txbuffer);
            aux = TMR2;
            printf("TIMER2: %ull.  \r\n", aux*1000);
            
            time_count = 0;          // Reseteamos variable encargada de medir 2x250ms = 500ms
        }
        Led_Red = !PORTBbits.RB3;
        
    }
       
}