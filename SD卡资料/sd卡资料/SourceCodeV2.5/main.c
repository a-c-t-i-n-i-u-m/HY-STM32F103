/*
 Copyright:      Radig Ulrich  mailto: mail@ulrichradig.de
 Author:         Radig Ulrich
 Remarks:        
 known Problems: none
 Version:        28.05.2004
 Description:    Dieses Programm dient als Beispiel zur Ansteuerung einer MMC/SD-Memory-Card.
				 Zum Zugriff auf eine MMC/SD-Karte, muß man nur die Datei mmc.c
				 in sein eigenes Programm einfügen.
*/

#include <main.h>

//Anpassen der Register wenn ein ATMega128 benutzt wird
#if defined (__AVR_ATmega128__)
#	define USR UCSR0A
#	define UCR UCSR0B
#	define UDR UDR0
#	define UBRR UBRR0L
#	define EICR EICRB
#endif

//Anpassen der Register wenn ein ATMega32 benutzt wird
#if defined (__AVR_ATmega32__)
#	define USR UCSRA
#	define UCR UCSRB
#	define UBRR UBRRL
#	define EICR EICRB
#endif

//Der Quarz auf dem Experimentierboard hat eine Frequenz von 14.318MHz
#define SYSCLK	11059200	//Quarz Frequenz in Hz

//Die Baud_Rate der Seriellen Schnittstelle ist 9600 Baud
#define BAUD_RATE 9600		//Baud Rate für die Serielle Schnittstelle	

//Installation der Seriellen Schnittstelle
void IOInit (void)
{
	//Enable TXEN im Register UCR TX-Data Enable
	UCR=(1 << TXEN);
	//Teiler wird gesetzt 
	UBRR=(SYSCLK / (BAUD_RATE * 16L) - 1);
}


//Routine für printf
int uart_putchar (char c)
{
	if (c == '\n')
		uart_putchar('\r');
	//Warten solange bis Zeichen gesendet wurde
	loop_until_bit_is_set(USR, UDRE);
	UDR = c;
	return (0);
}

//Hauptprogramm
int main (void)
{
	//Initzialisierung der seriellen Schnittstelle
	IOInit();
	
	//öffnet einen kanal für printf
	fdevopen (uart_putchar, NULL, 0);

	//Initialisierung der MMC/SD-Karte
	printf ("System OK\n\n");	
	while ( mmc_init() !=0) //ist der Rückgabewert ungleich NULL ist ein Fehler aufgetreten
		{
		printf("** Keine MMC/SD Karte gefunden!! **\n");	
		}
	printf("Karte gefunden!!\n");
	
	fat_cluster_data_store();//laden Cluster OFFSET und Size ins EEPROM
	//Initialisierung der MMC/SD-Karte ENDE!

	unsigned char Buffer[512];
	unsigned int tmp;
	
	mmc_read_csd (Buffer);
	
	for (tmp = 0;tmp<16;tmp++)
		{
		printf("%x ",Buffer[tmp]);
		};


	//Ausgabe des Root Directory
	unsigned int Clustervar;
	unsigned char Dir_Attrib = 0;
	unsigned long Size = 0;
	printf("\nDirectory\n\n");
	for (char a = 1;a < 240;a++)
	{
		Clustervar = fat_read_dir_ent(0,a,&Size,&Dir_Attrib,Buffer);
			if (Clustervar == 0xffff)
			{
				break;
			}
		tmp = (Size & 0x0000FFFF);
		printf("Cluster = %4x DirA = %2x Size= %8d FileName = ",Clustervar,Dir_Attrib,tmp);
		printf(Buffer);
		printf("\n");
	}
	printf("\nDirectory Ende\n\n");

	//Lade Cluster für das index.htm File in den Speicher 
	Clustervar = 0;//suche im Root Verzeichnis
	if (fat_search_file("mmc.txt",&Clustervar,&Size,&Dir_Attrib,Buffer) == 1)
		{
		printf("\nFile Found!!\n\n");
		//Lese File und gibt es auf der seriellen Schnittstelle aus
		for (int b = 0;b<52;b++)
			{
			fat_read_file (Clustervar,Buffer,b);
			for (int a = 0;a<512;a++)
				{
				printf ("%c",Buffer[a]);
				}
			}
		}

	printf("FERTIG!!\n");
	//Hauptprogramm läuft ständig in einer schleife und macht nichts
	while (1)
		{
		}
return (1);
}

