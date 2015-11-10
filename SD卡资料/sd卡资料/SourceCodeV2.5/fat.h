/*#######################################################################################
Connect ARM to MMC/SD 

Copyright (C) 2004 Ulrich Radig
#######################################################################################*/

#ifndef _FAT_H_
 #define _FAT_H_

#include <string.h>
#include "mmc.h"
#include "typedefs.h"

//Prototypes
extern U16 fat_root_dir_addr (U08 *);
extern U16 fat_read_dir_ent (U16,U08,U32*,U08 *,U08 *);
extern U16 fat_addr (U08 *);
extern void fat_load (U16,U32 *,U08 *);
extern void fat_read_file (U16,U08 *,U32);
extern void fat_write_file (U16,U08 *,U32);
extern void fat_cluster_data_store (void);
extern U08 fat_search_file (U08 *,U16 *,U32 *,U08 *,U08 *);

//Block Size in Bytes
#define BlockSize			512

//Master Boot Record
#define MASTER_BOOT_RECORD	0

//Volume Boot Record location in Master Boot Record
#define VBR_ADDR 			0x1C6

//define ASCII
#define SPACE 				0x20
#define DIR_ENTRY_IS_FREE   0xE5
#define FIRST_LONG_ENTRY	0x01
#define SECOND_LONG_ENTRY	0x42

//define DIR_Attr
#define ATTR_LONG_NAME		0x0F
#define ATTR_READ_ONLY		0x01
#define ATTR_HIDDEN			0x02
#define ATTR_SYSTEM			0x04
#define ATTR_VOLUME_ID		0x08
#define ATTR_DIRECTORY		0x10
#define ATTR_ARCHIVE		0x20

struct BootSec 
{
	U08 BS_jmpBoot[3];
	U08 BS_OEMName[8];
	U16 BPB_BytesPerSec; //2 bytes
	U08	BPB_SecPerClus;
	U16	BPB_RsvdSecCnt; //2 bytes
	U08	BPB_NumFATs;
	U16	BPB_RootEntCnt; //2 bytes
	U16	BPB_TotSec16; //2 bytes
	U08	BPB_Media;
	U16	BPB_FATSz16; //2 bytes
	U16	BPB_SecPerTrk; //2 bytes
	U16	BPB_NumHeads; //2 bytes
	U32	BPB_HiddSec; //4 bytes
	U32	BPB_TotSec32; //4 bytes
};

//FAT12 and FAT16 Structure Starting at Offset 36
#define BS_DRVNUM			36
#define BS_RESERVED1		37
#define BS_BOOTSIG			38
#define BS_VOLID			39
#define BS_VOLLAB			43
#define BS_FILSYSTYPE		54

//FAT32 Structure Starting at Offset 36
#define BPB_FATSZ32			36
#define BPB_EXTFLAGS		40
#define BPB_FSVER			42
#define BPB_ROOTCLUS		44
#define BPB_FSINFO			48
#define BPB_BKBOOTSEC		50
#define BPB_RESERVED		52

#define FAT32_BS_DRVNUM		64
#define FAT32_BS_RESERVED1	65
#define FAT32_BS_BOOTSIG	66
#define FAT32_BS_VOLID		67
#define FAT32_BS_VOLLAB		71
#define FAT32_BS_FILSYSTYPE	82
//End of Boot Sctor and BPB Structure

struct DirEntry {
	U08	DIR_Name[11];     //8 chars filename
	U08	DIR_Attr;         //file attributes RSHA, Longname, Drive Label, Directory
	U08	DIR_NTRes;        //set to zero
	U08	DIR_CrtTimeTenth; //creation time part in milliseconds
	U16	DIR_CrtTime;      //creation time
	U16	DIR_CrtDate;      //creation date
	U16	DIR_LastAccDate;  //last access date
	U16	DIR_FstClusHI;    //first cluster high word                 
	U16	DIR_WrtTime;      //last write time
	U16	DIR_WrtDate;      //last write date
	U16	DIR_FstClusLO;    //first cluster low word                 
	U32	DIR_FileSize;     
	};

#endif //_FAT_H_
