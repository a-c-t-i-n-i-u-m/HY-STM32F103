/*#######################################################################################
FAT for ARM (MMC/SD) 

Copyright (C) 2004 Ulrich Radig

Bei Fragen und Verbesserungen wendet euch per EMail an

mail@ulrichradig.de

oder im Forum meiner Web Page : www.ulrichradig.de


Dieses Programm ist freie Software. Sie können es unter den Bedingungen der 
GNU General Public License, wie von der Free Software Foundation veröffentlicht, 
weitergeben und/oder modifizieren, entweder gemäß Version 2 der Lizenz oder 
(nach Ihrer Option) jeder späteren Version. 

Die Veröffentlichung dieses Programms erfolgt in der Hoffnung, 
daß es Ihnen von Nutzen sein wird, aber OHNE IRGENDEINE GARANTIE, 
sogar ohne die implizite Garantie der MARKTREIFE oder der VERWENDBARKEIT 
FÜR EINEN BESTIMMTEN ZWECK. Details finden Sie in der GNU General Public License. 

Sie sollten eine Kopie der GNU General Public License zusammen mit diesem 
Programm erhalten haben. 
Falls nicht, schreiben Sie an die Free Software Foundation, 
Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307, USA. 
#######################################################################################*/

#include "fat.h"

U08 cluster_size;
U16 fat_offset;
U16 cluster_offset;
U16 volume_boot_record_addr;

//############################################################################
//Auslesen der Adresse des Volume Boot Record von MBR
U16 fat_addr (U08 *Buffer)
//############################################################################
{
	U16 volume_boot_record_addr;
    
	//auslesen des Master Boot Record von der MMC/SD Karte (addr = 0)
	mmc_read_sector (MASTER_BOOT_RECORD,Buffer); //Read Master Boot Record
    volume_boot_record_addr = Buffer[VBR_ADDR] + (Buffer[VBR_ADDR+1] << 8);
	//Berechnet Volume Boot Record 
	mmc_read_sector (volume_boot_record_addr,Buffer); //Read Master Boot Record
    return (volume_boot_record_addr);
}
//############################################################################
//Auslesen der Adresse des First Root Directory von Volume Boot Record
U16 fat_root_dir_addr (U08 *Buffer) 
//############################################################################
{
	struct BootSec *bootp; //Zeiger auf Bootsektor Struktur
	U16 FirstRootDirSecNum;
	//auslesen des Volume Boot Record von der MMC/SD Karte 
	mmc_read_sector (volume_boot_record_addr,Buffer);
	bootp=(struct BootSec *)Buffer;

	//berechnet den ersten Sector des Root Directory
	FirstRootDirSecNum = ( bootp->BPB_RsvdSecCnt +
	                       (bootp->BPB_NumFATs * bootp->BPB_FATSz16));

	FirstRootDirSecNum+= volume_boot_record_addr;
	
	return(FirstRootDirSecNum);
}

//############################################################################
//	Ausgabe des angegebenen Directory Eintrag in Entry_Count
//	ist kein Eintrag vorhanden, ist der Eintrag im 
//	Rückgabe Cluster 0xFFFF. Es wird immer nur ein Eintrag ausgegeben
//	um Speicherplatz zu sparen um es auch für kleine Atmels zu benutzen
U16 fat_read_dir_ent (U16 dir_cluster, //Angabe Dir Cluster
					U08 Entry_Count,   //Angabe welcher Direintrag
					U32 *Size, 		   //Rückgabe der File Größe
					U08 *Dir_Attrib,   //Rückgabe des Dir Attributs
					U08 *Buffer) 	   //Working Buffer
//############################################################################
{
	U08 *pointer;
	U16 TMP_Entry_Count = 0;
	U32 Block = 0;
	struct DirEntry *dir; //Zeiger auf einen Verzeichniseintrag

	pointer = Buffer;

	if (dir_cluster == 0)
		{
		Block = fat_root_dir_addr(Buffer);
		}
	else
		{
		//Berechnung des Blocks aus BlockCount und Cluster aus FATTabelle
		//Berechnung welcher Cluster zu laden ist
		//Auslesen der FAT - Tabelle
		fat_load (dir_cluster,&Block,Buffer);			 
		Block = ((Block-2) * cluster_size) + cluster_offset;
		}

	//auslesen des gesamten Root Directory
	for (U16 blk = Block;;blk++)
	{
		mmc_read_sector (blk,Buffer);	//Lesen eines Blocks des Root Directory
		for (U16 a=0;a<BlockSize; a = a + 32)
		{
		 dir=(struct DirEntry *)&Buffer[a]; //Zeiger auf aktuellen Verzeichniseintrag holen
		 
			if (dir->DIR_Name[0] == 0) //Kein weiterer Eintrag wenn erstes Zeichen des Namens 0 ist
			{
			return (0xFFFF);
			}
			
			//Prüfen ob es ein 8.3 Eintrag ist
			//Das ist der Fall wenn es sich nicht um einen Eintrag für lange Dateinamen
			//oder um einen als gelöscht markierten Eintrag handelt.
   			if ((dir->DIR_Attr != ATTR_LONG_NAME) &&
				(dir->DIR_Name[0] != DIR_ENTRY_IS_FREE)) 
			{
				//Ist es der gewünschte Verzeichniseintrag
				if (TMP_Entry_Count == Entry_Count) 
				{
					//Speichern des Verzeichnis Eintrages in den Rückgabe Buffer
					for(U08 b=0;b<11;b++)
					{
					if (dir->DIR_Name[b] != SPACE)
						{
						if (b == 8)
							{
							*pointer++= '.';
							}
						*pointer++=dir->DIR_Name[b];
						}
					}						
					*pointer++='\0';
					*Dir_Attrib = dir->DIR_Attr;

					//Speichern der Filegröße
					*Size=dir->DIR_FileSize;
					
					//Speichern des Clusters des Verzeichniseintrages
					dir_cluster = dir->DIR_FstClusLO;

					//Eintrag gefunden Rücksprung mit Cluster File Start
					return(dir_cluster);
				}
			TMP_Entry_Count++;
			}
		}
	}
	return (0xFFFF); //Kein Eintrag mehr gefunden Rücksprung mit 0xFFFF
}

//############################################################################
//	Auslesen der Cluster für ein File aus der FAT
//	in den Buffer(512Byte). Bei einer 128MB MMC/SD 
//	Karte ist die Cluster größe normalerweise 16KB groß
//	das bedeutet das File kann max. 4MByte groß sein.
//	Bei größeren Files muß der Buffer größer definiert
//	werden! (Ready)
//	Cluster = Start Clusterangabe aus dem Directory	
void fat_load (	U16 Cluster, 		//Angabe Startcluster
				U32 *Block,
				U08 *TMP_Buffer) 	//Workingbuffer
//############################################################################
{
	//Zum Überprüfen ob der FAT Block schon geladen wurde
	U16 FAT_Block_Store = 0;	

	//Byte Adresse innerhalb des Fat Blocks
	U16 FAT_Byte_Addresse;	

	//FAT Block Adresse
	U16 FAT_Block_Addresse;
	
	//Berechnung für den ersten FAT Block (FAT Start Addresse)
	for (U16 a = 0;;a++)
	{	
		if (a == *Block)
			{
			*Block = (0x0000FFFF & Cluster);
			return;
			}
		
		if (Cluster == 0xFFFF)
			{
			break; //Ist das Ende des Files erreicht Schleife beenden
			}
		//Berechnung des Bytes innerhalb des FAT Block´s
		FAT_Byte_Addresse = (Cluster*2) % BlockSize;
			
		//Berechnung des Blocks der gelesen werden muß
		FAT_Block_Addresse = ((Cluster*2) / BlockSize) + 
								volume_boot_record_addr + fat_offset;	
		//Lesen des FAT Blocks
		//Überprüfung ob dieser Block schon gelesen wurde
		if (FAT_Block_Addresse != FAT_Block_Store)
			{
			FAT_Block_Store = FAT_Block_Addresse;
			//Lesen des FAT Blocks
			mmc_read_sector (FAT_Block_Addresse,TMP_Buffer);	
			}

		//Lesen der nächsten Clusternummer
		Cluster = (TMP_Buffer[FAT_Byte_Addresse + 1] << 8) + 
					TMP_Buffer[FAT_Byte_Addresse];		
	}
	return;
}

//############################################################################
//Auslesen Cluster Size der MMC/SD Karte und Speichern der größe ins EEprom
//Auslesen Cluster Offset der MMC/SD Karte und Speichern der größe ins EEprom
void fat_cluster_data_store (void)
//############################################################################
{
	struct BootSec *bootp; //Zeiger auf Bootsektor Struktur

	U08 Buffer[BlockSize];

	volume_boot_record_addr = fat_addr (Buffer);	
   
	mmc_read_sector (volume_boot_record_addr,Buffer);

    bootp=(struct BootSec *)Buffer;

	cluster_size = bootp->BPB_SecPerClus;

	fat_offset = bootp->BPB_RsvdSecCnt;

	cluster_offset = ((bootp->BPB_BytesPerSec * 32)/BlockSize);	
	cluster_offset += fat_root_dir_addr(Buffer);
}

//############################################################################
//Lesen eines 512Bytes Blocks von einem File
void fat_read_file (U16 Cluster,//Angabe des Startclusters vom File
				 U08 *Buffer,	  //Workingbuffer
				 U32 BlockCount)	  //Angabe welcher Bock vom File geladen 
										      //werden soll a 512 Bytes
//############################################################################
{
	//Berechnung des Blocks aus BlockCount und Cluster aus FATTabelle
	//Berechnung welcher Cluster zu laden ist
	
	U32 Block = (BlockCount/cluster_size);
	
	//Auslesen der FAT - Tabelle
	fat_load (Cluster,&Block,Buffer);			 
	Block = ((Block-2) * cluster_size) + cluster_offset;
	//Berechnung des Blocks innerhalb des Cluster
	Block += (BlockCount % cluster_size);
	//Read Data Block from Device
	mmc_read_sector (Block,Buffer);	
	return;
}

//############################################################################
//Lesen eines 512Bytes Blocks von einem File
void fat_write_file (U16 cluster,//Angabe des Startclusters vom File
					U08 *buffer,	  //Workingbuffer
					U32 blockCount)	  //Angabe welcher Bock vom File gespeichert 
									  //werden soll a 512 Bytes
//############################################################################
{
	//Berechnung des Blocks aus BlockCount und Cluster aus FATTabelle
	//Berechnung welcher Cluster zu speichern ist
	U08 tmp_buffer[513];	
	U32 block = (blockCount/cluster_size);
	
	//Auslesen der FAT - Tabelle
	fat_load (cluster,&block,tmp_buffer);			 
	block = ((block-2) * cluster_size) + cluster_offset;
	//Berechnung des Blocks innerhalb des Cluster
	block += (blockCount % cluster_size);
	//Write Data Block to Device
	mmc_write_sector (block,buffer);	
	return;
}

//####################################################################################
//Sucht ein File im Directory
U08 fat_search_file (U08 *File_Name,		//Name des zu suchenden Files
							U16 *Cluster, 	//Angabe Dir Cluster welches
											//durchsucht werden soll
											//und Rückgabe des clusters
											//vom File welches gefunden
											//wurde
							U32 *Size, 		//Rückgabe der File Größe
							U08 *Dir_Attrib,//Rückgabe des Dir Attributs
							U08 *Buffer) 	//Working Buffer
//####################################################################################
{
	U16 Dir_Cluster_Store = *Cluster;
	for (U08 a = 0;a < 100;a++)
	{
		*Cluster = fat_read_dir_ent(Dir_Cluster_Store,a,Size,Dir_Attrib,Buffer);
		if (*Cluster == 0xffff)
			{
			return(0); //File not Found
			}
		if(strcasecmp(File_Name,Buffer) == 0)
			{
			return(1); //File Found
			}
	}
	return(2); //Error
}
