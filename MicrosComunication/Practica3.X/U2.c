//// DSPIC33FJ32MC204 Configuration Bit Settings
//
//// 'C' source line config statements
//
//// FBS
//#pragma config BWRP = WRPROTECT_OFF     // Boot Segment Write Protect (Boot Segment may be written)
//#pragma config BSS = NO_FLASH           // Boot Segment Program Flash Code Protection (No Boot program Flash segment)
//
//// FGS
//#pragma config GWRP = OFF               // General Code Segment Write Protect (User program memory is not write-protected)
//#pragma config GSS = OFF                // General Segment Code Protection (User program memory is not code-protected)
//
//// FOSCSEL
//#pragma config FNOSC = PRIPLL           // Oscillator Mode (Internal Fast RC (FRC) with divide by N)
//#pragma config IESO = ON                // Internal External Switch Over Mode (Start-up device with FRC, then automatically switch to user-selected oscillator source when ready)
//
//// FOSC
//#pragma config POSCMD = EC              // Primary Oscillator Source (Primary Oscillator Disabled)
//#pragma config OSCIOFNC = OFF           // OSC2 Pin Function (OSC2 pin has clock out function)
//#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow Only One Re-configuration)
//#pragma config FCKSM = CSDCMD           // Clock Switching and Monitor (Both Clock Switching and Fail-Safe Clock Monitor are disabled)
//
//// FWDT
//#pragma config WDTPOST = PS32768        // Watchdog Timer Postscaler (1:32,768)
//#pragma config WDTPRE = PR128           // WDT Prescaler (1:128)
//#pragma config WINDIS = OFF             // Watchdog Timer Window (Watchdog Timer in Non-Window mode)
//#pragma config FWDTEN = ON              // Watchdog Timer Enable (Watchdog timer always enabled)
//
//// FPOR
//#pragma config FPWRT = PWR128           // POR Timer Value (128ms)
//#pragma config ALTI2C = OFF             // Alternate I2C  pins (I2C mapped to SDA1/SCL1 pins)
//#pragma config LPOL = ON                // Motor Control PWM Low Side Polarity bit (PWM module low side output pins have active-high output polarity)
//#pragma config HPOL = ON                // Motor Control PWM High Side Polarity bit (PWM module high side output pins have active-high output polarity)
//#pragma config PWMPIN = ON              // Motor Control PWM Module Pin Mode bit (PWM module pins controlled by PORT register at device Reset)
//
//// FICD
//#pragma config ICS = PGD1               // Comm Channel Select (Communicate on PGC2/EMUC2 and PGD2/EMUD2)
//#pragma config JTAGEN = OFF             // JTAG Port Enable (JTAG is Disabled)
//
//// #pragma config statements should precede project file includes.
//// Use project enums instead of #define for ON and OFF.
//
//#include <xc.h>
//#include <stdio.h>
//#include <string.h>
//#include <stdlib.h>
//#include <stddef.h>          // Defines NULL
//#include <stdbool.h>         // Defines true
//#include <math.h>
//
//
//
////#define CPU_40MHz
//
//#ifdef CPU_40MHz
//    #define baud_9600    1041
//#else
//    #define baud_9600    51     // FCPU = 2MHZ
//#endif
//
//
//
//char dataCMD_ISR[50];
//
//// Modulo Uart. Variables empleando ISR
//char txbuffer_ISR[200];
//unsigned int nextchar = 0;
//unsigned char BufferLoadDone = 1;
//
//unsigned int U1_PrintRate_ISR = 0;
//unsigned int contador = 0;
//
//volatile unsigned int data_count = 0;
//volatile unsigned char comando_detectado = 0;
//
//unsigned char Allowprint = 0;
//unsigned char dummy;
//
//
///*lista de comandos*/
//const char cmd1[50] = {0x50, 0x01, 0xAA}; // Comando para encender led verde
//const char cmd2[50] = {0x50, 0x00, 0xAA}; // Comando para apagar led verde
//const char cmd3[50] = {0x51, 0x01, 0xAA}; // Comando para encender led rojo
//const char cmd4[50] = {0x51, 0x00, 0xAA}; // Comando para apagar led rojo
//const char cmd5[50] = {0x52, 0x01, 0xAA}; // Comando para imprimir datos
//const char cmd6[50] = {0x52, 0x00, 0xAA}; // Comando para parar de imprimir datos
//
//
//void delay_ms(unsigned long time_ms)
//{
//    unsigned long u;
//    for(u = 0; u < time_ms*90; u++) // Cálculo aproximado para una CPU a 2MHz
//    {
//        asm("NOP");
//    }
//}
//
//
//void uart_config (unsigned int baud)
//{    
//    // Configuración de pines tx y rx
//    TRISCbits.TRISC0  = 1;   // Pin de recepcion de uart establecido como entrada.
//    RPINR18bits.U1RXR = 16;  // pin de recepcion rc0 trabajando con el modulo uart (RP16)
//    RPOR8bits.RP17R   = 3;   // U1TX conectado con el pin RC1 (RP17)
//    
//    
//    
//    // Configuración de registro de U1MODE
//    U1MODEbits.UARTEN = 0;     // Deshabilitar Uart.
//    U1MODEbits.USIDL  = 0;     // Continuar operación en modo IDLE
//    U1MODEbits.IREN   = 0;     // IR no usado
//    U1MODEbits.RTSMD  = 1;     // Control de flujo desactivado.
//    U1MODEbits.UEN    = 0;     // Solo usamos pin de Tx y pin de Rx
//    U1MODEbits.WAKE   = 0;     // No quiero que la UART despierte del modo sleep
//    U1MODEbits.LPBACK = 0;     // Loopback deshabilitado.
//    U1MODEbits.ABAUD  = 0;     // Automedición de baudios (bps) deshabilidada
//    U1MODEbits.URXINV = 0;     // En estado de reposo, el receptor mantiene un estado alto, high
//    U1MODEbits.BRGH   = 1;     // Modo High-Speed
//    U1MODEbits.PDSEL  = 0;     // 8 Bits de datos y paridad Nula (8N)
//    U1MODEbits.STSEL  = 0;     // 1-bit de stop al final de la trama de datos.   (8N1)
//
//    
//    // Configuración de registro de U1STA
//    
//    
//    U1STAbits.UTXISEL0 = 0;    // Tema interrupciones (no mirar aun)
//    U1STAbits.UTXISEL1 = 0;    // Tema interrupciones (no mirar aun)
//     
//    
//    U1STAbits.UTXINV   = 0;    // El estado en reposo del pin de transmisión es High
//    U1STAbits.UTXBRK   = 0;    // No usamos trama de sincronización
//    U1STAbits.UTXEN    = 1;    // El transmisor a pleno funcionamiento.
//    U1STAbits.URXISEL  = 0;    // Tema interrupciones (no mirar aun)
//    U1STAbits.ADDEN    = 0;    // No usamos direccionamiento.
//    //U1STAbits.RIDLE    = 0;
//    U1STAbits.OERR     = 0;    // Reseteamos buffer de recepción
//
//    
//    // Configuramos la velocidad de transmisión/recepcción de los datos
//    U1BRG = baud;
//    
//    
//    // Prioridades, flags e interrupciones correspondientes a la Uart
//    IPC2bits.U1RXIP = 6;    // U1RX con nivel de prioridad 6 (7 es el maximo)
//    IFS0bits.U1RXIF = 0;    // Reset Rx Interrupt flag
//    IEC0bits.U1RXIE = 1;    // Enable Rx interrupts
//    
//    
//    IPC3bits.U1TXIP = 5;    // U1TX con nivel de prioridad 6 (7 es el maximo)
//    IFS0bits.U1TXIF = 0;    // Reset Tx Interrupt flag
//    IEC0bits.U1TXIE = 0;    // Enable Tx interrupts
//    
//    
//    U1MODEbits.UARTEN = 1;     // Uart habilitada por completo
//}
//
//
//
//
//
//
//int main(void) 
//{
//    
//#ifdef CPU_40MHz
//    
//    //Configurar el oscilador para hacer funcionar la CPU a 40 MHz a partir de un reloj de entrada de 8MHz
//    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
//    //Fosc = 8M * 40/(2 * 2) = 80 MHz para un reloj de 8MHz de entrada
//    //Fcy = Fosc/2 = 80/2 = 40MHz (Frecuencia CPU)
//    PLLFBD = 38;                    // M  = 40
//    CLKDIVbits.PLLPOST = 0;         // N1 = 2
//    CLKDIVbits.PLLPRE  = 0;         // N2 = 2
//    while(OSCCONbits.LOCK != 1);    // Esperar a un PLL estable
//        
//#else
//    
//    //Configurar el oscilador para hacer funcionar la CPU a 2 MHz a partir de un reloj de entrada de 8MHz
//    //Fosc = Fin * M/(N1 * N2), Fcy = Fosc/2
//    //Fosc = 8M * 2/(2 * 2) = 4 MHz para un reloj de 8MHz de entrada
//    //Fcy = Fosc/2 = 4/2 = 2MHz (Frecuencia CPU)
//    PLLFBD = 0;                     // M  = 2
//    CLKDIVbits.PLLPOST = 0;         // N1 = 2
//    CLKDIVbits.PLLPRE  = 0;         // N2 = 2
//    while(OSCCONbits.LOCK != 1);    // Esperar a un PLL estable   
//#endif    
//    
//    
//    AD1PCFGL         = 0xFFFF;      // Primer paso. Todos los pines configurados como pines digitales
//    TRISAbits.TRISA0 = 0;
//    TRISBbits.TRISB3 = 0;
//    LATAbits.LATA0   = 0;
//    LATBbits.LATB3   = 0;
// 
//    uart_config(baud_9600); 
//    
//    PR1 = 24999;
//
//    while(1)   
//    {   
//        if ( !strcmp(((const char*)dataCMD_ISR), cmd1) ) LATAbits.LATA0  = 1; 
//        if ( !strcmp(((const char*)dataCMD_ISR), cmd2) ) LATAbits.LATA0  = 0;
//        if ( !strcmp(((const char*)dataCMD_ISR), cmd3) ) LATBbits.LATB3  = 1;
//        if ( !strcmp(((const char*)dataCMD_ISR), cmd4) ) LATBbits.LATB3  = 0;
//        if ( !strcmp(((const char*)dataCMD_ISR), cmd5) ) Allowprint = 1;
//        if ( !strcmp(((const char*)dataCMD_ISR), cmd6) ) Allowprint = 0;
//
//        else
//        {
//            /****code here***/
//        }
//        memset(dataCMD_ISR,'\0',sizeof(dataCMD_ISR));   // Resetear buffer con NULL
//        data_count = 0;
//        
//        
//        if((Allowprint)&&(BufferLoadDone)&&(U1STAbits.TRMT))   // Si se cargaron todos los datos al buffer U1TXREG y finalizó la última transmisión...
//        {
//            if(U1_PrintRate_ISR++ >= 20)         // Determinar el ritmo de transmision de datos de la Uart1 Tx. 20 veces el delay de 10 ms
//            {
//                if(contador == 100){
//                    contador = 0;
//                }
//                  
//                memset(txbuffer_ISR,'\0',sizeof(txbuffer_ISR));   // Clear Buffer and fill it with NULL
//                sprintf(txbuffer_ISR,"IMPRIMIENDO DATOS: %d \r\n",contador++); 
//               
//                nextchar = 0;
//                BufferLoadDone = 0;
//                U1_PrintRate_ISR = 0;
//                
//                if(U1STAbits.UTXBF) IFS0bits.U1TXIF = 0;          // Reseteo el flag de transmision ISR
//                asm("nop");
//                IEC0bits.U1TXIE = 1;                              // Iniciamos una nueva transmisión   
//            }
//        }
//        
//        delay_ms(10);  
//    }
//}
//
//
//
//void __attribute__((__interrupt__, no_auto_psv)) _T1Interrupt(void)
//{    
//    IFS0bits.T1IF = 0;          // Reset Timer1 Interrupt
//}
//
//
//
//void __attribute__((__interrupt__, no_auto_psv)) _U1RXInterrupt(void)
//{   
//    
//    dataCMD_ISR[data_count] = U1RXREG;// Obtener caracter recibido en el buffer  
//    data_count++;
//
//    IFS0bits.U1RXIF = 0;        // Reset Rx Interrupt
//}
//
//
//    
//void __attribute__((__interrupt__, no_auto_psv)) _U1TXInterrupt(void)    // Actualmente configurado para U1STAbits.UTXISEL = 0 (Solo se debe resetear el flag cuando la condicion de UTXISEL no sea cierta.)
//{  
//    IEC0bits.U1TXIE = 0;                  // Disable UART1 Tx Interrupt  
//    
//    if(!U1STAbits.UTXBF)                  // Mientras el buffer de transmisión NO se encuentre completo, continuar cargando el buffer con más datos.
//    {
//        U1TXREG = txbuffer_ISR[nextchar++];  // Cargar el buffer con un nuevo dato.
//        asm ("nop");
//        if(U1STAbits.UTXBF)               // Si el buffer de transmision se completó con el último dato incorporado al buffer, procedemos a resetear el flag. (Este método se usa para UTXISEL = 0)
//        {
//            IFS0bits.U1TXIF = 0;          // Clear UART1 Tx Interrupt Flag  
//        }
//    }
//    else IFS0bits.U1TXIF = 0;             // Clear UART1 Tx Interrupt Flag
//          
//    if(nextchar == strlen(txbuffer_ISR))  // Si se ha finalizado la transmision de todos los caracteres --> Deshabilitar interrupcion y activar flags. strlen cuenta el numero de caracteres hasta encontrar NULL.
//    {   
//        BufferLoadDone = 1;               // Informamos de que se ha terminado de cargar la cadena de texto de 'U1_TxBuffer_ISR' en U1TXREG. No implica que haya finalizado la transmisión. 
//    }   
//    else IEC0bits.U1TXIE = 1;             // Enable UART1 Tx Interrupt   
//}
//
//
//
//
//
