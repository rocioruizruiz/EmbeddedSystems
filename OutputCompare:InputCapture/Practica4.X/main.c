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



//#define CPU_40MHz

#ifdef CPU_40MHz
    #define baud_9600    1041
#else
    #define baud_9600    51     // FCPU = 2MHZ
#endif



char dataCMD_ISR[50];
unsigned int dutyOC2 = 4000;
unsigned int dutyOC1 = 4000;


unsigned int pulso1 = 0;
unsigned int time_pulsoIC1 = 0;
unsigned int rise_pulsoIC1 = 0;

unsigned int flag = 1;
double tiempo_real_IC1 = 0.0;
double tiempo_real_IC2 = 0.0;
char txbuffer[200];
char received_char;

unsigned int pulso2 = 0;
unsigned int time_pulsoIC2 = 0;
unsigned int rise_pulsoIC2 = 0;
// Modulo Uart. Variables empleando ISR
char txbuffer_ISR[200];
unsigned int nextchar = 0;







/*lista de comandos*/
const char cmd1[] = {"set ledgreen 1"}; // Comando para encender led verde
const char cmd2[] = {"set ledgreen 0"}; // Comando para apagar led verde
const char cmd3[] = {"set ledred 1"};   // Comando para encender led rojo
const char cmd4[] = {"set ledred 0"};   // Comando para encender led rojo
const char cmd5[] = {"print MIC2_data"};  // Comando para imprimir datos
const char cmd6[] = {"stop MIC2_data"};   // Comando para parar de imprimir 







void delay_ms(unsigned long time_ms)
{
    unsigned long u;
    for(u = 0; u < time_ms*90; u++) // Cálculo aproximado para una CPU a 2MHz
    {
        asm("NOP");
    }
}




void uart_config (unsigned int baud)
{    
    // Configuración de pines tx y rx
    TRISCbits.TRISC0  = 1;   // Pin de recepcion de uart establecido como entrada.
    RPINR18bits.U1RXR = 16;  // pin de recepcion rc0 trabajando con el modulo uart (RP16)
    RPOR8bits.RP17R   = 3;   // U1TX conectado con el pin RC1 (RP17)
    
    
    
    // Configuración de registro de U1MODE
    U1MODEbits.UARTEN = 0;     // Deshabilitar Uart.
    U1MODEbits.USIDL  = 0;     // Continuar operación en modo IDLE
    U1MODEbits.IREN   = 0;     // IR no usado
    U1MODEbits.RTSMD  = 1;     // Control de flujo desactivado.
    U1MODEbits.UEN    = 0;     // Solo usamos pin de Tx y pin de Rx
    U1MODEbits.WAKE   = 0;     // No quiero que la UART despierte del modo sleep
    U1MODEbits.LPBACK = 0;     // Loopback deshabilitado.
    U1MODEbits.ABAUD  = 0;     // Automedición de baudios (bps) deshabilidada
    U1MODEbits.URXINV = 0;     // En estado de reposo, el receptor mantiene un estado alto, high
    U1MODEbits.BRGH   = 1;     // Modo High-Speed
    U1MODEbits.PDSEL  = 0;     // 8 Bits de datos y paridad Nula (8N)
    U1MODEbits.STSEL  = 0;     // 1-bit de stop al final de la trama de datos.   (8N1)

    
    // Configuración de registro de U1STA
    
    
    U1STAbits.UTXISEL0 = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.UTXISEL1 = 0;    // Tema interrupciones (no mirar aun)
     
    
    U1STAbits.UTXINV   = 0;    // El estado en reposo del pin de transmisión es High
    U1STAbits.UTXBRK   = 0;    // No usamos trama de sincronización
    U1STAbits.UTXEN    = 1;    // El transmisor a pleno funcionamiento.
    U1STAbits.URXISEL  = 0;    // Tema interrupciones (no mirar aun)
    U1STAbits.ADDEN    = 0;    // No usamos direccionamiento.
    //U1STAbits.RIDLE    = 0;
    U1STAbits.OERR     = 0;    // Reseteamos buffer de recepción

    
    // Configuramos la velocidad de transmisión/recepcción de los datos
    U1BRG = baud;
    
    
    // Prioridades, flags e interrupciones correspondientes a la Uart
    IPC2bits.U1RXIP = 6;    // U1RX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1RXIF = 0;    // Reset Rx Interrupt flag
    IEC0bits.U1RXIE = 1;    // Enable Rx interrupts
    
    
    IPC3bits.U1TXIP = 5;    // U1TX con nivel de prioridad 6 (7 es el maximo)
    IFS0bits.U1TXIF = 0;    // Reset Tx Interrupt flag
    IEC0bits.U1TXIE = 0;    // Enable Tx interrupts
    
    
    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
}


void output_compare_config(void)
{
    //Configurar pines con los que vamos a trabajar (RP:de remapeable en proteus)
    TRISCbits.TRISC2 = 0;   //Pin configurado como salida
    TRISCbits.TRISC3 = 0;   //Pin configurado como salida
    RPOR9bits.RP18R = 0x12; //Pin RC2 conectado al modulo OC1 subreg:RP18 
    RPOR9bits.RP19R = 0x13; //Pin RC3 conectado al modulo OC2 subreg:RP19   
    
    //Configurar PWM OC1
    OC1CONbits.OCM    = 0;
    OC1CONbits.OCSIDL = 0;  // el OC se detiene si la CPU entra en modo reposo
    OC1CONbits.OCFLT  = 0;   
    OC1CONbits.OCTSEL = 0;  // Trabajamos el OC con Timer2
    OC1CONbits.OCM    = 6;  // Modo PWM sin pin de fallo
    
    //Configuro duty OC1  
    //Duty OC1 = (Ton/T) = (1/5)*100 = 20% de trabajo ------------ Ton = el duty
    OC1R    = 2000;     //Ciclo de trabajo8(duty) 1ms con prescaler=1 y Fcpu= 2MHz. //Este es de solo lectura
    OC1RS   = 2000;     //Igual porque es un segundo buffer para evitar glitch. Este es el que se modifica
    
    //Configurar PWM OC2
    OC2CONbits.OCM    = 0;
    OC2CONbits.OCSIDL = 0;  // el OC se detiene si la CPU entra en modo reposo
    OC2CONbits.OCFLT  = 0;   
    OC2CONbits.OCTSEL = 0;  // Trabajamos el OC con Timer2
    OC2CONbits.OCM    = 6;  // Modo PWM sin pin de fallo
    
    //Configuro duty OC2  
    //Duty OC2 = (Ton/T) = (2/5)*100 = 40% de trabajo ------------ Ton = el duty
    OC2R    = 4000;     //Ciclo de trabajo8(duty) 1ms con prescaler=1 y Fcpu= 2MHz
    OC2RS   = 4000;     //Igual porque es un segundo buffer para evitar glitch.
    
    //Configuramos Timer2
    T2CONbits.TON     = 0;
    T2CONbits.T32     = 0;
    T2CONbits.TGATE   = 0;
    T2CONbits.TCKPS   = 0; //Prescaler mas bajo = 1.
    T2CONbits.TCS     = 0;
    
    //Configuro periodo
    PR2 = 10000;        //Periodo = 5ms con Precaler=1 y Fcpu = 2MHz
    
    // Activar los modulos OC1, OC2 y Timer
    T2CONbits.TON     = 1;
    OC1CONbits.OCM    = 6;  //Activar modulo OC1 por pin RC2
    OC2CONbits.OCM    = 6;  //Activar modulo OC2 por pin RC3
    
}

void input_capture_config(void)
{
   //Configurar pines con los que vamos a trabajar (RP:de remapeable en proteus)
    TRISCbits.TRISC5 = 1;   //Pin configurado como entrada
    TRISCbits.TRISC6 = 1;   //Pin configurado como entrada
    RPINR7bits.IC1R = 21;   // R7 por IC1 y 21 por RP21 (remapping))
    RPINR7bits.IC2R = 22;   // R7 por IC2 y 22 por RP22 (remapping))
    
    //Configurar IC1
    IC1CONbits.ICM    = 0;
    IC1CONbits.ICSIDL = 0;  // el IC se detiene si la CPU entra en modo reposo
    IC1CONbits.ICTMR  = 0;  // Trabajamos con Timer 3
    IC1CONbits.ICI    = 0;  // salta en cada evento
    IFS0bits.IC1IF = 0;
    IEC0bits.IC1IE = 1;  // Enable IC1 interrupts
    
    //Configurar IC2
    IC2CONbits.ICM    = 0;
    IC2CONbits.ICSIDL = 0;  // el IC se detiene si la CPU entra en modo reposo
    IC2CONbits.ICTMR  = 0;  // Trabajamos con Timer 3
    IC2CONbits.ICI    = 0;  // salta en cada evento
    IFS0bits.IC2IF = 0;
    IEC0bits.IC2IE = 1;  // Enable IC2 interrupts
    
   
    //Configuramos Timer3
    T3CONbits.TSIDL   = 0;
    T3CONbits.TON     = 0;
    T3CONbits.TGATE   = 0;
    T3CONbits.TCKPS   = 0; //Prescaler mas bajo = 1.
    T3CONbits.TCS     = 0;
    PR3 = 10000;
    
    
    // Activar los modulos IC1, IC2 y Timer3
    T3CONbits.TON     = 1;  //Activar Timer3
    IC1CONbits.ICM    = 3;  //Activar modulo OC1 por pin RC2
    IC2CONbits.ICM    = 3;  //Activar modulo OC2 por pin RC3 
    
}
void EnviarCaracter(char c)
{    
    while(U1STAbits.UTXBF);   // Mientras el buffer del puerto U1 este lleno, esperar en bucle while
    U1TXREG = c;              // Si no esta lleno, proceder a enviar el byte
}

void EnviarString(char *s)
{    
    while((*s) != '\0') EnviarCaracter(*(s++));  // Mientras no se haya llegado al caracter nulo (final de trama), continuar imprimiendo datos.
} 



int main(void) 
{
    
#ifdef CPU_40MHz
    
    //Configurar el oscilador para hacer funcionar la CPU a 40 MHz a partir de un reloj de entrada de 8MHz
    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    //Fosc = 8M * 40/(2 * 2) = 80 MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2 = 80/2 = 40MHz (Frecuencia CPU)
    PLLFBD = 38;                    // M  = 40
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    CLKDIVbits.PLLPRE  = 0;         // N2 = 2
    while(OSCCONbits.LOCK != 1);    // Esperar a un PLL estable
        
#else
    
    //Configurar el oscilador para hacer funcionar la CPU a 2 MHz a partir de un reloj de entrada de 8MHz
    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
    //Fosc = 8M * 2/(2 * 2) = 4 MHz para un reloj de 8MHz de entrada
    //Fcy = Fosc/2 = 4/2 = 2MHz (Frecuencia CPU)
    PLLFBD = 0;                     // M  = 2
    CLKDIVbits.PLLPOST = 0;         // N1 = 2
    CLKDIVbits.PLLPRE  = 0;         // N2 = 2
    while(OSCCONbits.LOCK != 1);    // Esperar a un PLL estable   
#endif    
    
    
    AD1PCFGL         = 0xFFFF;      // Primer paso. Todos los pines configurados como pines digitales
    uart_config(baud_9600); 
    delay_ms(10);
    output_compare_config();
    delay_ms(10);
    input_capture_config();
    delay_ms(10);
    
    
    while(1){
         
        //if(U1STAbits.OERR) U1STAbits.OERR = 0;

        tiempo_real_IC1 = 1.0*((double)time_pulsoIC1)/2000.0;
        tiempo_real_IC2 = 1.0*((double)time_pulsoIC2)/2000.0;

        sprintf(txbuffer,"TIME_IC1: %05.3fms    TIME_IC2: %05.3fms \r\n",tiempo_real_IC1, tiempo_real_IC2); 
        EnviarString(txbuffer);


        delay_ms(194);

    }
        
    


}

void __attribute__((__interrupt__, no_auto_psv)) _IC1Interrupt(void)
{ 
    
    
    if(pulso1 == 0){
        rise_pulsoIC1 = IC1BUF;
        IC1CONbits.ICM = 2; // Capture next falling edge
        pulso1 = 1;

    }else{
        time_pulsoIC1 = IC1BUF - rise_pulsoIC1;
        IC1CONbits.ICM = 3; // Capture next rising edge
        pulso1 = 0;

    }
        
    IFS0bits.IC1IF = 0;          // Reset IC1 Interrupt
}

void __attribute__((__interrupt__, no_auto_psv)) _IC2Interrupt(void)
{ 
    
    if(pulso2 == 0){
        rise_pulsoIC2 = IC2BUF;
        IC2CONbits.ICM = 2; // Capture next falling edge
        pulso2 = 1;

    }else{
        time_pulsoIC2 = IC2BUF - rise_pulsoIC2;
        IC2CONbits.ICM = 3; // Capture next rising edge
        pulso2 = 0;
    }
        
    IFS0bits.IC2IF = 0;          // Reset IC2 Interrupt
}


void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
{    
    IFS0bits.T1IF = 0;          // Reset Timer1 Interrupt
}


void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
{ 
    
    dataCMD_ISR[0] = U1RXREG;
    if(dataCMD_ISR[0] == '+') dutyOC2 = dutyOC2 + 100;
    if(dataCMD_ISR[0] == '-') dutyOC2 = dutyOC2 - 100;
    if(dataCMD_ISR[0] == 'p') dutyOC1 = dutyOC1 + 100;
    if(dataCMD_ISR[0] == 'm') dutyOC1 = dutyOC1 - 100;
    
    OC2RS = dutyOC2;
    OC1RS = dutyOC1;
    
    IFS0bits.U1RXIF = 0;        // Reset Rx Interrupt
}
   
void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)    // Actualmente configurado para U1STAbits.UTXISEL = 0 (Solo se debe resetear el flag cuando la condicion de UTXISEL no sea cierta.)
{  
    IEC0bits.U1TXIE = 0;                  // Disable UART1 Tx Interrupt  
    
    if(!U1STAbits.UTXBF)                  // Mientras el buffer de transmisión NO se encuentre completo, continuar cargando el buffer con más datos.
    {
        U1TXREG = txbuffer_ISR[nextchar++];  // Cargar el buffer con un nuevo dato.
        asm ("nop");
        if(U1STAbits.UTXBF)               // Si el buffer de transmision se completó con el último dato incorporado al buffer, procedemos a resetear el flag. (Este método se usa para UTXISEL = 0)
        {
            IFS0bits.U1TXIF = 0;          // Clear UART1 Tx Interrupt Flag  
        }
    }
    else IFS0bits.U1TXIF = 0;             // Clear UART1 Tx Interrupt Flag
          
    if(!(nextchar == strlen(txbuffer_ISR)))  // Si se ha finalizado la transmision de todos los caracteres --> Deshabilitar interrupcion y activar flags. strlen cuenta el numero de caracteres hasta encontrar NULL.
    {   
        IEC0bits.U1TXIE = 1;             // Enable UART1 Tx Interrupt   
    }   
}






