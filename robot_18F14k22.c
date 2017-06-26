/*  PIC18F14K22  mikroC PRO for PIC v6.4
Oscillateur interne 16MHZ


/ 21/06/17 Test sur plateforme sur et generateur pour simuler le capteur, 14308 impulsions ----> 0 erreurs


Hardware:
  18F14K22  DIP20,SOIC
  
  pin 1     VDD Alim +5V
  pin 2     OSC1/RA5
  pin 3     OSC2/RA4
  pin 4     RA3/MCR ---> sortie de l'opto HOA0901 SORTIE B (avec PULL UP)
  pin 5     RC5 ---> sortie PWM P1A, relié à l'entrée IN1 du pont L6203
  pin 6     RC4 ---> sortie PWM P1B (complémenté à P1A), relié à l'entrée IN1 du pont L6203
  pin 7     RC3 ---> tension de la batterie 4
  pin 8     RC6 ---> sortie relié à l'entrée ENABLE du pont L6203
  pin 9     RC7 ---> LED 2
  pin 10    RB7
  pin 11    RB6 ---> SCL I2C clock input
  pin 12    RB5 ---> LED 1
  pin 13    RB4 ---> SDA I2C data output
  pin 14    RC2 ---> tension de la batterie 3
  pin 15    RC1 ---> tension de la batterie 2
  pin 16    RC0 ---> tension de la batterie 1
  pin 17    RA2/INT2/T0CKI---> sortie de l'opto HOA0901 SORTIE A sur entrée de comptage du TIMER0
  pin 18    RA1/INT1 ---> interrupteur de butée de rentrée sur entrée INT1 (avec PULL UP)
  pin 19    RA0/INT0 ---> interrupteur de butée de sortie sur entrée INT0 (avec PULL UP)
  pin 20    VSS Alim 0V
*/

sbit SA at RA2_bit;
sbit SB at RA3_bit;

sbit LED1 at RA4_bit;
//sbit LED1 at RB5_bit;
sbit LED2 at RC7_bit;

int NB_Impulsion = 0;  // Nombre d'impulsions de la sortie de l'opto OPB461T11
int ab = 0;
int val = 0;
int Consigne_position = 0;
int zero = 0;

volatile signed int NB_Impulsion_old = 0;
int Nb_Tour = 0;
int Lecture_PORTA=0;
int Sens = 0;

int Consigne_vitesse = 0;
int Valeur_Consigne_vitesse = 0;
int On = 0;
int Consigne_Sortie = 0;
int Consigne_Rentre = 0;
int Consigne = 0;
int Vitesse = 0;
int Position = 0;


int Egal=0;

enum robot_state {STATE0,STATE1,STATE2,STATE3,STATE4,STATE5};
char state = STATE0;

unsigned int ADC;

unsigned char etat;
unsigned char ancien_etat;
unsigned char oldstate;

const Adresse_I2C = 0x70; //adresse I2C du circuit
unsigned short rxbuffer_I2C_Octet1 = 0;
unsigned short rxbuffer_I2C_Octet2 = 0;
unsigned short rxbuffer_I2C_Octet3 = 0;
unsigned short octet_I2C = 0;
unsigned short capteur_I2C = 0;
unsigned short j = 0;
unsigned compteur = 0;
unsigned compteur1 = 0;
unsigned short octet = 0;
unsigned short NB_Impulsions_octet1 = 0;
unsigned short NB_Impulsions_octet2 = 0;
unsigned short butee_sortie = 0;
unsigned short butee_rentree = 0;
unsigned short Flag_butee = 0;
unsigned short Moteur_ON = 0;
unsigned short Moteur_OFF = 0;
unsigned short tension1_octet1 = 0;
unsigned short tension1_octet2 = 0;
unsigned short tension2_octet1 = 0;
unsigned short tension2_octet2 = 0;
unsigned short tension3_octet1 = 0;
unsigned short tension3_octet2 = 0;
unsigned short tension4_octet1 = 0;
unsigned short tension4_octet2 = 0;
unsigned Toogle = 0;
unsigned Flag = 0;
unsigned compteur_etat3 = 0;

char i = 0;

char Tableau[30];

int Flag_state3 = 0;


/**************************************************************************************************
* Fonction pour convertir les données I2C transmises du maître
* Le maître transmet deux octets à suivre:
* octet1 = 0xFE --> correspond à une consigne de position, suivit de la valeur de la position à atteindre
                    sur deux octets
                    
* octet1 = 0xAB --> correspond à une consigne de vitesse, suivit de la valeur de la vitesse
                    sur deux octets
                    
* octet1 = 0xEE --> correspond à une consigne de marche,arret,mise en butee (sur 1 octets)
* valeur 0xAA 0x00: arret
* valeur 0xAA 0x01: marche
* valeur 0xAA 0x02: mise en butee butee de sortie
* valeur 0xAA 0x03: mise en butee butee de rentree
* valeur 0xAA 0x04: mise en marche du moteur
* valeur 0xAA 0x05: arret du moteur

* octet1 = 0x00 --> correspond à une demande, suivit de la valeur de la correspondance de la consigne
* valeur 0xAA 0x0A: valeur de la tension de batterie 1 sur 2 octets
* valeur 0xAA 0x0C: valeur de la tension de batterie 2 sur 2 octets
* valeur 0xAA 0x0E: valeur de la tension de batterie 3 sur 2 octets
* valeur 0xAA 0x10: valeur de la tension de batterie 4 sur 2 octets
* valeur 0xAA 0x04: valeur NB impulsions  sur 2 octets
* valeur 0xAA 0x06: valeur de la butée de sortie  sur 2 octets
* valeur 0xAA 0x08: valeur de la butée de rentrée sur 2 octets
*
*
* exemple: le maitre transmet les trames suivantes

Consigne d'arret:
I2C1_Start();
I2C1_Write(0x70);   // send byte via I2C  (device address + W) adresse 0x42
I2C1_Write(0xEE);  //  consigne
I2C1_Write(0xAA);
I2C1_Write(0x00);  // on arrete le systeme
I2C1_Stop();

Consigne de position:
I2C1_Start();
I2C1_Write(0x70);   // send byte via I2C  (device address + W) adresse 0x42
I2C1_Write(0xFE);   // consigne de deplacement
I2C1_Write(0x01);
I2C1_Write(0x90);  // 0x0190 --> 400 --> on va à la position 171
I2C1_Stop();

Demande de position:
I2C1_Start();
I2C1_Write(0x70);   // send byte via I2C  (device address + W) adresse 0x42
I2C1_Write(0x00);   // demande d'info
I2C1_Write(0xAA);
I2C1_Write(0x08);  // on retourne la valeur du nombre d'impulsions
I2C1_Stop();

**************************************************************************************************/

void Convertion_I2C(){

    if (rxbuffer_I2C_Octet1 == 0xFE){ // consigne de postion

        //consigne_position = rxbuffer_I2C_Octet2 << 8 + rxbuffer_I2C_Octet3;
        Consigne_position = 0;
        Toogle = 0;
        Consigne = 1;

    }
    else if (rxbuffer_I2C_Octet1 == 0xAB){ // consigne de vitesse
    
        Consigne_vitesse = Consigne_vitesse + rxbuffer_I2C_Octet2;
        Consigne_vitesse = Consigne_vitesse << 8;
        Consigne_vitesse = Consigne_vitesse + rxbuffer_I2C_Octet3;
        Vitesse = 1;

    }
    else if (rxbuffer_I2C_Octet1 == 0xEE){ // consigne de: marche,arret,mise en butee
      switch (rxbuffer_I2C_Octet3){
        case 0: On = 0;
                break; // on arrete le systeme
        case 1: On = 1;
                break; // on met en marche le systeme
        case 2: Consigne_Sortie = 1;
                Consigne_Rentre = 0;
                Consigne = 0;
                break;
        case 3: Consigne_Sortie = 0;
                Consigne_Rentre = 1;
                Consigne = 0;
                break;
        case 4: Moteur_ON = 1;
                Moteur_OFF = 0;
                break;
        case 5: Moteur_OFF = 1;
                Moteur_ON = 0;
                break;
        default:break;
      }
    }
    else if (rxbuffer_I2C_Octet1 == 0x00){ // demande de: valeur des butees, du nombre d'impulsions, de tension des batteries
      
      octet_I2C = rxbuffer_I2C_Octet3;

      switch (octet_I2C){
        case 4:  Tableau[4] = NB_Impulsions_octet1; // valeur NB impulsions
                 Tableau[5] = NB_Impulsions_octet2; // valeur NB impulsions
                 break;
        case 6:  Tableau[6] = butee_sortie; // valeur de la butée de sortie
                 Tableau[7] = 0x00;
                 break;
        case 8:  Tableau[8] = butee_rentree; // valeur de la butée de rentrée
                 Tableau[9] = 0x00;
                 break;
        default:break;
      }
    }
    
}


/**************************************************************************************************
* Fonction qui donne le format de la trame I2C, qui transmet le nombre d'impulsions des codeurs droit,
* gauche et le sens des moteurs
* Entête 1 octet  | Valeur codeur1 2 octets | sens codeur1 | Valeur codeur2 2 octets | sens codeur2
*     FF          |codeur_Low |codeur__high |     1 ou 0   |codeur_Low |codeur__high |     1 ou 0
* la trame fait 7 octets
**************************************************************************************************/

void Trame_I2C(int num){

  switch (num){
    case 0:  SSPBUF = 0xFF;
             octet++;
             break;
    case 1:  SSPBUF = Tableau[octet_I2C];
             octet++;
             break;
    case 2:  SSPBUF = Tableau[octet_I2C + 1];;
             octet = 0;
             break;
  }
  
}


/**************************************************************************************************
* Pour lire la tension des 4 batteries
**************************************************************************************************/

void Read_Tension(){

  ADC = ADC_Read(4);   // Get 10-bit results of AD conversion AN4 batterie 1

  Tableau[10] = ADC;  // lower 8 bits
  Tableau[11] = ADC >> 8; // 2 most significant bits
  delay_ms(5);
  
  ADC = ADC_Read(5);   // Get 10-bit results of AD conversion AN5 batterie 2

  Tableau[12] = ADC;  // lower 8 bits
  Tableau[13] = ADC >> 8; // 2 most significant bits
  delay_ms(5);
  
  ADC = ADC_Read(6);   // Get 10-bit results of AD conversion AN6 batterie 3

  Tableau[14] = ADC;  // lower 8 bits
  Tableau[15] = ADC >> 8; // 2 most significant bits
  delay_ms(5);
  
  ADC = ADC_Read(7);   // Get 10-bit results of AD conversion AN7 batterie 4

  Tableau[16] = ADC;  // lower 8 bits
  Tableau[17] = ADC >> 8; // 2 most significant bits

}


/**************************************************************************************************
* Pour lire la Valeur des butees
**************************************************************************************************/

void Read_butee(){

  if (RA0_bit == 0) butee_sortie = 1;
  else butee_sortie = 0;
  if (RA1_bit == 0) butee_rentree = 1;
  else butee_rentree = 0;

}


/**************************************************************************************************
* Pour commander le moteur avec une rampe
**************************************************************************************************/

void Commande_motor_rampe(int i){

if (compteur >= i){
T1CON.TMR1ON = 0;
//compteur = 0;
}
else
T1CON.TMR1ON = 1;

PWM1_Set_Duty(compteur);  // 0 --> 127 sens sortie

CCP1CON=0b10001100; // Half-bridge output: P1A, P1B modulated with dead-band control
PSTRCON.STRB = 1 ;  // Pour activer la sortie P1B
PWM1_Start();
RC6_bit = 1;  //Enable L6203
 
}


/**************************************************************************************************
* Pour commander le moteur
**************************************************************************************************/

void Commande_motor(int i){

PWM1_Set_Duty(i);
CCP1CON=0b10001100; // Half-bridge output: P1A, P1B modulated with dead-band control
PSTRCON.STRB = 1 ;  // Pour activer la sortie P1B
PWM1_Start();
RC6_bit = 1;  //Enable L6203

}


/**************************************************************************************************
* Arret du Moteur
**************************************************************************************************/

void Off_motor(){

  T1CON.TMR1ON = 0;
  compteur = 0;
  CCPR1 = 50; // Rapport cyclique à 50% pour stopper le moteur et garder du couple
  /*Delay_ms(100);
  RC6_bit = 0;  //Disable L6203*/

}


/**************************************************************************************************
* Arret du Moteur
**************************************************************************************************/

void On_motor(){

  PWM1_Start();
  RC6_bit = 1;
  
}


/**************************************************************************************************
* Pour commander le moteur
**************************************************************************************************/

void Led(int i){

 LED1 = i;
 LED2 = i >> 1;

}


/**************************************************************************************************
* Fonction d'initialisation de l'I2C in slave mode
**************************************************************************************************/

void InitI2C(){

  SSPADD = Adresse_I2C; // Address Register, Get address (7bit). Lsb is read/write flag
  SSPCON1 = 0x3E; // SYNC SERIAL PORT CONTROL REGISTER
                  // bit 3-0 SSPM3:SSPM0: I2C Firmware Controlled Master mode,
                  // 7-bit address with START and STOP bit interrupts enabled
                  // bit 4 CKP: 1 = Enable clock
                  // bit 5 SSPEN: Enables the serial port and configures the SDA and SCL
                  // pins as the source of the serial port pins

  SSPCON2 = 0x00;
  SSPSTAT=0x00;

  SSPSTAT.SMP = 1; // 1 = Slew rate control disabled for standard speed mode
                   // (100 kHz and 1 MHz)
  SSPSTAT.CKE = 1; // 1 = Input levels conform to SMBus spec
  
  PIE1.SSPIE = 1; // Synchronous Serial Port Interrupt Enable bit
  PIR1.SSPIF = 0; // Synchronous Serial Port (SSP) Interrupt Flag, I2C Slave
                 // a transmission/reception has taken place.

}


/**************************************************************************************************
* Mise en marche du TIMER3
**************************************************************************************************/

void On_TIMER3(){

  T3CON.TMR3ON = 1;
  TMR3H = 0x3C;
  TMR3L = 0xB0;
  compteur1 = 0;

}


/**************************************************************************************************
* Initialisation des entrées sorties du PIC
**************************************************************************************************/

void Init(){

 ANSEL = 0xF0;  // Set RC0,RC1,RC2,RC3 to analog (AN4,AN5,AN6,AN7)
 ANSELH = 0x00;

 CM1CON0 = 0x00; // Not using the comparators
 CM2CON0 = 0x00; //

 TRISA0_bit = 1; // RA0 en entrée
 TRISA1_bit = 1; // RA1 en entrée
 TRISA2_bit = 1; // RA2 en entrée
// TRISA3_bit = 1; // RA3 en entrée , RA3/MCLR est toujours en entrée
 //TRISA4_bit = 1; // RA4 en entrée
 TRISA4_bit = 0; // RA4 en sortie
 TRISA5_bit = 0; // RA5 en sortie

 INTCON2.RABPU = 0; // PORTA and PORTB Pull-up Enable bit
 WPUA.WPUA0 = 1; // Pull-up enabled sur RA0, sur inter de butée haute
 WPUA.WPUA1 = 1; // Pull-up enabled sur RA1, sur inter de butée basse
 //WPUA.WPUA2 = 1; // Pull-up enabled sur RA2, sur sortie de l'opto OPB461T11 (sortie collecteur ouvert)
 //WPUA.WPUA3 = 1; // Pull-up enabled sur RA3, sur sortie de l'opto HOA0901 (sortie collecteur ouvert)

 TRISB4_bit = 1; // RB4 en entrée
 TRISB5_bit = 1; // RB5 en entrée
 TRISB6_bit = 1; // RB6 en entrée
 TRISB7_bit = 0; // RB7 en sortie

 TRISC = 0xFF;
 TRISC0_bit = 1; // RC0 en entree voie AN4
 TRISC1_bit = 1; // RC1 en entree voie AN5
 TRISC2_bit = 1; // RC2 en entree voie AN6
 TRISC3_bit = 1; // RC3 en entree voie AN7
 TRISC4_bit = 0; // RC4 en sortie
 TRISC5_bit = 0; // RC5 en sortie
 TRISC6_bit = 0; // RC6 en sortie
 TRISC7_bit = 0; // RC7 en sortie

 RC6_bit = 0;

}


/*void Init(){

 ANSEL = 0xF0;  // Set RC0,RC1,RC2,RC3 to analog (AN4,AN5,AN6,AN7)
 ANSELH = 0x00;

 CM1CON0 = 0x00; // Not using the comparators
 CM2CON0 = 0x00; //

 TRISA2_bit = 1; // RA5 en entrée
 //TRISA5_bit = 1; // RA5 en entrée
 //TRISA3_bit = 1; // RA3 en entrée RA3 est toujours en entrée

 TRISA4_bit = 0; // RA4 en sortie
 //TRISA5_bit = 0; // RA5 en sortie

 INTCON2.RABPU = 0; // PORTA and PORTB Pull-up Enable bit

 TRISB4_bit = 1; // RB4 en entrée
 TRISB5_bit = 0; // RB5 en sortie
 TRISB6_bit = 1; // RB6 en entrée
 TRISB7_bit = 0; // RB7 en sortie

 TRISC = 0xFF;

 TRISC2_bit = 1; // RC2 en entree voie AN6
 TRISC3_bit = 1; // RC3 en entree voie AN7
 TRISC5_bit = 1; // RC5 en entree voie AN7
 TRISC7_bit = 0; // RC7 en sortie

}*/


/**************************************************************************************************
* Programme principal
**************************************************************************************************/

void Main(){

 // Oscillateur interne de 16Mhz
 OSCCON = 0b01110010;   // 0=4xPLL OFF, 111=IRCF<2:0>=16Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block
 
 // Oscillateur interne de 8Mhz
 //OSCCON = 0b01100010;   // 0=4xPLL OFF, 111=IRCF<2:0>=8Mhz  OSTS=0  SCS<1:0>10 1x = Internal oscillator block


 Init(); // Initialisation des I/O
 InitI2C(); // Initialisation de l'I2C en esclave

// Initialisation de l'entrée d'interruption INT0 pour la butée haute

 INTCON2.INTEDG0 = 0; // Interrupt on falling edge
 INTCON.INT0IE = 1; //Enables the INT0 external interrupt
 INTCON.INT0IF = 0; // INT0 External Interrupt Flag bit

// Initialisation de l'entrée d'interruption INT1 pour la butée basse

 INTCON2.INTEDG1 = 0; // Interrupt on falling edge
 INTCON3.INT1IE = 1; //Enables the INT1 external interrupt
 INTCON3.INT1IF = 0; // INT0 External Interrupt Flag bit
 
// Initialisation du TIMER0 en interruption sur comptage d'impulsion

 TMR0H = 0xFF;
 TMR0L = 0xFF;
 T0CON = 0x00;
 T0CON.T08BIT = 1; // Timer0 is configured as an 8-bit timer/counter
 T0CON.T0CS = 1;   // Transition on T0CKI pin
 //T0CON.T0SE = 1;   // Increment on high-to-low transition on T0CKI pin
 T0CON.T0SE = 0;   // Increment on low-to-high transition on T0CKI pin
 T0CON.PSA = 1;    // TImer0 prescaler is NOT assigned. Timer0 clock input bypasses prescaler
 T0CON.TMR0ON = 0; // Enables Timer0
 //INTCON.TMR0IE = 1; // Enables the TMR0 overflow interrupt
 INTCON.TMR0IE = 0; // Disables the TMR0 overflow interrupt

// Initialisation du TIMER1
//Prescaler 1:1; TMR1 Preload = 61536; Actual Interrupt Time : 1 ms

 T1CON = 0x00;
 TMR1IF_bit = 0;
 TMR1H = 0xF0;
 TMR1L = 0x60;
 T1CON.TMR1ON = 0;
 TMR1IE_bit = 0;

// Initialisation du TIMER3 FOSC/4 donc 16Mhz/4
//Prescaler 1:8; TMR3 Preload = 15536; Actual Interrupt Time : 100 ms

 T3CON = 0x30;
 TMR3IF_bit = 0;
 TMR3H = 0x3C;
 TMR3L = 0xB0;
 T3CON.TMR3ON = 1;
 TMR3IE_bit = 1;


// Initialisation d'interruption sur PORTA change sur les entrées RA2 et RA3
// sorties A et B du codeur HOA901-12 sur les entrées RA2 et RA3

 INTCON.RABIE = 1; // RA and RB Port Change Interrupt Enable bit
 IOCA.IOCA2 = 1; // Interrupt-on-change enabled sur RA5
 IOCA.IOCA3 = 1; // Interrupt-on-change enabled sur RA3
 IOCB = 0x00;    // Interrupt-on-change disabled sur le PORTB

 INTCON2.RABIP = 1; //RA and RB Port Change Interrupt Priority bit, high priority
 INTCON3.INT1IP = 1; //INT1 External Interrupt Priority bit, INT0 always a high
                     //priority interrupt source
 IPR1.SSPIP = 0; //Master Synchronous Serial Port Interrupt Priority bit, low priority
 RCON.IPEN = 1;  //Enable priority levels on interrupts
 
 INTCON.GIEH = 1; //enable all high-priority interrupts
 INTCON.GIEL = 1; //enable all low-priority interrupts
 //INTCON.GIE = 1; //enable all high-priority interrupts

 INTCON.PEIE = 1; // Peripheral Interrupt Enable bit

 ADC_Init();

 PWM1_Init(10000);  // Fréquence du PWM à 10Khz

 RC6_bit = 0;  //Enable L6203

 UART1_Init(115200);
 delay_ms(3000);
 UART1_Write_Text("essai");
 Led(0);
 
 Consigne_vitesse = 80;
 //Consigne_vitesse = 100;

 T3CON.TMR3ON = 1;

/*while(1){ // pour test
 Led(3);
 UART1_Write(10);
 delay_ms(1000);
 Led(0);
 delay_ms(1000);
}*/
 
//On = 1;   // pour test

while(1){
         delay_ms(1);
         if(state <3 || state == 4)
                  Read_Tension();
         Read_butee();
switch (state){
         case STATE0:
           UART1_Write(0);

           if (compteur1 == 5){
             LED1 =~ LED1;
             compteur1 = 0;
           }

           if (On == 1){
             state = STATE1;
             T3CON.TMR3ON = 0;
             LED1 = 0;
             Commande_motor(255-Consigne_vitesse);
           }
           
           if (compteur_etat3 == 0){
              delay_us(200);
              oldstate = SB<<1 | SA;  //  ou logique de RA3 et RA2, lecture du capteur pour initialiser la machine d'état
              compteur_etat3 = 1;
           }
           break;

         case STATE1:
           UART1_Write(1);
           Led(1);

           if (Vitesse == 1){
            Vitesse = 0;
           }

           if (butee_sortie == 1){
             state = STATE2;
             Flag_butee = 0;
             Consigne_Sortie = 0;
             NB_Impulsion = 0;
             NB_Impulsion_old = 0;
             compteur_etat3 = 0;
           }
           else if (Moteur_OFF == 1){
             state = STATE0;
             Off_motor();
             Consigne_Sortie = 0;
           }
           else if (On == 0){
             state = STATE0;
             Led(0);
             Off_motor();
             On_TIMER3();
             Consigne_Sortie = 0;
           }
           break;
           
         case STATE2:

//           UART1_Write(2);
//           UART1_Write(NB_Impulsion);

           Moteur_OFF = 0;
           Led(2);
           Off_motor();
           //PWM1_Stop();
           //RC6_bit = 0;  //Disable L6203

           if (Vitesse == 1){
            Commande_motor(Consigne_vitesse);
            Vitesse = 0;
           }

           if (Consigne == 1){
             state = STATE3;
             //Led(3);
             Egal=0;
             if (butee_sortie == 1) NB_Impulsion = 0;
           }
           else if (Consigne_Sortie == 1){
             state = STATE1;
           }
           else if (Consigne_Rentre == 1){
             state = STATE4;
           }
           else if (On == 0){
             state = STATE0;
             Led(0);
             On_TIMER3();
             Consigne_Sortie = 0;
           }
           break;

         case STATE3:
         {
           //UART1_Write(3);

           if (Toogle == 0){
             Consigne_position = Consigne_position + rxbuffer_I2C_Octet2;
             Consigne_position = Consigne_position << 8;
             Consigne_position = Consigne_position + rxbuffer_I2C_Octet3;
             Consigne_position = Consigne_position*4;
             Consigne = 0;

//             if (butee_rentree != 1){
//               if (Valeur < 0) Commande_motor(255-Consigne_vitesse);  // 0 --> 127 sens sortie
//               else if (Valeur > 0) Commande_motor(Consigne_vitesse); // 128 --> 255 sens rentree
//             }
//             else if (butee_rentree == 1){
//               if (Valeur > 0) Commande_motor(Consigne_vitesse);
//             }
             Toogle++;
           }
           
           ab = Consigne_position - NB_Impulsion;
           val = Consigne_position - NB_Impulsion;
           UART1_Write(val);
           UART1_Write(zero < val);
           UART1_Write(zero > val);
           UART1_Write(zero == val);
           UART1_Write(zero <= val);
           if(val == zero){   // Valeur == 0 ==> Etat 2
             state = STATE2;
             Off_motor();
             Toogle = 0;
             Flag_state3 = 1;
           }
           else if(val < 0){// /*&& butee_sortie != 0*/){   // Valeur > 0  ==> 0-127
                Commande_motor(Consigne_vitesse);
                Flag_state3 = 2;
           }
           else if(val > 0){// /*&& butee_rentree != 0*/){   // Valeur < 0 ==> 0-127
                Commande_motor(255-Consigne_vitesse);
                Flag_state3 = 3;
           }
           else{
                state = STATE2;
                Off_motor();
                Flag_state3 = 4;
           }


           // Changement de la vitesse
//           if (Vitesse == 1){
//             if (butee_rentree != 1){
//                 if (Valeur < 0) Commande_motor(Consigne_vitesse);
//                 else Commande_motor(255 - Consigne_vitesse);
//                 Vitesse = 0;
//             }
//           }

//           if (Moteur_OFF == 1){
//             Off_motor();
//             Moteur_OFF = 0;
//           }
//
//           if (Moteur_ON == 1){
//             On_motor();
//             Moteur_OFF = 0;
//             Moteur_ON = 0;
//           }

           // COMM - DEBUG
           if (0 && Flag==1){   // pour test
           UART1_Write(0xFF);
          UART1_Write(Consigne_vitesse);
           UART1_Write(Valeur_Consigne_vitesse);
           UART1_Write(compteur);
           UART1_Write(rxbuffer_I2C_Octet2);
           UART1_Write(rxbuffer_I2C_Octet3);
           UART1_Write(Consigne_position);
           UART1_Write(ancien_etat);
           UART1_Write(etat);
           UART1_Write(NB_Impulsion_old);
           UART1_Write(NB_Impulsion);
           UART1_Write(ab);
           UART1_Write(Flag_state3);
           UART1_Write(NB_Impulsions_octet2);
           UART1_Write(NB_Impulsions_octet1);
           Flag = 0;
           }
           
          /*delay_ms(3000);
           NB_Impulsion = Consigne_position;*/

//           if (NB_Impulsion >= Consigne_position){
//             state = STATE2;
//             Toogle = 0;
//             Consigne_position = 0;
//           }
//           else if ((Valeur < 0)&&(butee_rentree == 1)){
//             state = STATE2;
//             Toogle = 0;
//             Consigne_position = 0;
//           }
//           else if (On == 0){
//             state = STATE0;
//             Led(0);
//             Off_motor();
//             On_TIMER3();
//             Consigne_Sortie = 0;
//             Toogle = 0;
//             Consigne_position = 0;
//           }
//           else if (Flag_butee == 1){
//            state = STATE5;
//            Flag_butee = 0;
//            Off_motor();
//           }
         }
         break;

         case STATE4:

           UART1_Write(4);
           Led(0);
           Commande_motor(Consigne_vitesse);

           if (Vitesse == 1){
            Vitesse = 0;
           }

           if (butee_rentree == 1){
              state = STATE2;
              Flag_butee = 0;
              Consigne_Rentre = 0;
              NB_Impulsion = 0;
           }
           else if (Moteur_OFF == 0){
             state = STATE2;
             Off_motor();
             Consigne_Rentre = 0;
           }
           else if (On == 0){
             state = STATE0;
             Led(0);
             Off_motor();
             On_TIMER3();
             Consigne_Sortie = 0;
             Consigne_Rentre = 0;
           }
           break;

         case STATE5:

           UART1_Write(5);

           if ((butee_rentree == 1)||(butee_sortie == 1)){
             state = STATE2;
             Toogle = 0;
             Consigne_position = 0;
           }
           break;

   }//fin del switch

}//While(1)

}// Main


/**************************************************************************************************
* Fonction de gestion des interruptions:
* interruption sur l'entrée INT0/RA0
* interruption sur l'entrée INT1/RA1
* interruption sur TIMER0 interruptions sur front descendant sur l'entrée INT0
* interruption sur TIMER1 interruptions tous les 1ms pour commande moteur avec rampe
* interruption sur TIMER3 interruptions tous les 100ms pour visu STATE0(tous les 500ms)
* interruption sur le bus I2C
**************************************************************************************************/

void interrupt(){

// Interruption sur l'entrée INT0/RA0, detection de butée de sortie

  if(INTCON.INT0IF == 1) {

    /*butee_sortie = 1;
    Flag_butee = 1;
    RC6_bit = 0;  //Disable L6203*/

    if (!RA0_bit){
      delay_ms(2);

      if (!RA0_bit){
        butee_sortie = 1;
        Flag_butee = 1;
        CCPR1 = 50;  // Rapport cyclique à 50% pour stopper le moteur et garder du couple
        //RC6_bit = 0;  //Disable L6203
      }
    }


    INTCON.INT0IF = 0;
    }

// Interruption sur l'entrée INT1/RA1, detection de butée de rentree

  if(INTCON3.INT1IF == 1) {

   /*butee_rentree = 1;
    Flag_butee = 1;
    RC6_bit = 0;  //Disable L6203*/

    if (!RA1_bit){
      delay_ms(2);

      if (!RA1_bit){
        butee_rentree = 1;
        Flag_butee = 1;
        CCPR1 = 50;  // Rapport cyclique à 50% pour stopper le moteur et garder du couple
        //RC6_bit = 0;  //Disable L6203
      }
    }

    INTCON3.INT1IF = 0;
    }


// Interruption sur PORTA change, sur les entrées RA2 et RA3
// sorties du codeur HOA0901 SA et SB
// 00,10,11,01 dans un sens et 00,01,11,10 dans l'autre sens
// http://www.circuitvalley.com/2012/10/rotary-encoders-interfacing-pic18-avr.html
//
 
  if(INTCON.RABIF == 1){  // RA and RB Port Change Interrupt Flag bit
   
    //unsigned char etat;
    //static unsigned char oldstate;
    delay_us(10);
    //delay_ms(1);
    etat = SB <<1 | SA;  //  ou logique de RC5 et RC3
    
    NB_Impulsion_old = NB_Impulsion;
    ancien_etat = oldstate;

    if (etat == 0x00){
    //if((etat == 0x00)|(etat == 0x3)){
      NB_Impulsion++;
      Flag = 1;
    }

   /*if(oldstate==0x0){

    Flag = 1;

      if(etat == 0x1){
        Sens = 1;  // sortie voie B du capteur HOA0901
        NB_Impulsion--;
      }
      else if(etat == 0x2){
        Sens = 0;
        NB_Impulsion++;
      }
    }

    if(oldstate==0x1){

    Flag = 1;

      if(etat == 0x3){
        Sens = 0;  // sortie voie B du capteur HOA0901
        NB_Impulsion--;
      }
      else if(etat == 0x0){
        Sens = 1;
        NB_Impulsion++;
      }
    }

    if(oldstate==0x3){

    Flag = 1;

      if(etat == 0x1){
        Sens = 1;  // sortie voie B du capteur HOA0901
        NB_Impulsion++;
      }
      else if(etat == 0x2){
        Sens = 0;
        NB_Impulsion--;
      }
    }

    if(oldstate==0x2){

    Flag = 1;

      if(etat == 0x0){
        Sens = 1;  // sortie voie B du capteur HOA0901
        NB_Impulsion--;
      }
      else if(etat == 0x3){
        Sens = 0;
        NB_Impulsion++;
      }
    }*/

    //if (NB_Impulsion == Consigne_position) Egal = 1;

    NB_Impulsions_octet1 = NB_Impulsion;
    NB_Impulsions_octet2 = NB_Impulsion >> 8;
    
    oldstate = etat;  // store the current state value to oldstate value this value will be used in next call
    
    Lecture_PORTA = PORTA;
    INTCON.RABIF = 0;

  }
 
 // Interruption sur le TIMER1

  if (TMR1IF_bit){
    compteur++;
    TMR1H = 0xF0;
    TMR1L = 0x60;

    TMR1IF_bit = 0;
  }
  
  
  // Interruption sur le TIMER3

  if (TMR3IF_bit){
    compteur1++;
    TMR3H = 0x3C;
    TMR3L = 0xB0;

    TMR3IF_bit = 0;
  }

}



void interrupt_low(){

// Interruption sur le bus I2C, le bus est en esclave

if (PIR1.SSPIF){  // I2C Interrupt

  PIR1.SSPIF = 0; // reset SSP interrupt flag

// transmit data to master
// Demande de lecture des coduers par le maître, la trame est sous la forme
// Entête 1 octet  | Valeur codeur1 2 octets | codeur1 | Valeur codeur2 2 octets | codeur2
//     FF          |codeur_Low |codeur__high |     1   |codeur_Low |codeur__high |     2   |
// la trame fait 7 octets

  if (SSPSTAT.R_W == 1){   // Read/Write bit Information, Read request from master

    Trame_I2C(octet);

    SSPCON1.CKP = 1;        // Release SCL line
    j = SSPBUF;
    return;
  }

  if (SSPSTAT.BF == 0){ // Buffer Full Status bit, Data transmit complete, SSPBUF is empty

    j = SSPBUF;        // Nothing in buffer so exit
    return;

  }

  //recieve data from master
  if (SSPSTAT.R_W == 0){

    if (SSPSTAT.D_A == 0){
      j = SSPBUF;
      i=0;
    }

    if (SSPSTAT.D_A == 1){    //1 = Indicates that the last byte received or transmitted was data

      if(i==0){
      rxbuffer_I2C_Octet1 = SSPBUF;  // get data octet1
      i++;
      }
      else if(i==1){
      rxbuffer_I2C_Octet2 = SSPBUF;  // get data octet2
      i++;
      }
      else if(i==2){
      rxbuffer_I2C_Octet3 = SSPBUF;  // get data octet3
      i++;
      }

      Convertion_I2C();

      j = SSPBUF;       // read buffer to clear flag [address]
      return;
     }
  }

  j = SSPBUF;     // read buffer to clear flag [address]

  }

 }