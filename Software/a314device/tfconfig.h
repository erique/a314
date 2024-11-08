#ifndef TFCONFIG_H
#define TFCONFIG_H

#include <exec/types.h>

#define arraylen(arr) (int)(sizeof(arr)/sizeof(arr[0]))

// terriblefire config port

struct TFConfig
{  
  UBYTE TF_Reserved[0x40];

  UBYTE TF_Blinkenlights; // 0x40 
  UBYTE TF_Reserved00;
  UWORD TF_Reserved01;

  UBYTE TF_Status;  // 0x44
  UBYTE TF_Reserved02;
  UWORD TF_Reserved03;
  
  UBYTE TF_Speed; // 0x48
  UBYTE TF_Reserved04;
  UWORD TF_Reserved05;
  
  UBYTE TF_Fan; // 0x4C
  UBYTE TF_Reserved06;
  UWORD TF_Reserved07;
  
  volatile UBYTE TF_SpiData; // 0x50
  UBYTE TF_Pad0[3];
  volatile UBYTE TF_SpiCtrl; // 0x54
  UBYTE TF_Pad1[3];
  volatile UBYTE TF_SpiRxReg; // 0x58
  UBYTE TF_Pad2[3];
  ULONG TF_Reserved11; // 0x5C

  ULONG TF_Reserved12; // 0x60

  UBYTE TF_MapRom; // 0x64
  UBYTE TF_Reserved13;
  UWORD TF_Reserved14;
};

#define TF_MANU_ID 0x13D8

#ifndef PRODUCTID 
#define PRODUCTID 0x83
#endif

#define MAPB_WP (1) 
#define MAPB_EN (0)

#define MAPF_WP (1<<1)
#define MAPF_EN (1<<0)

#define MAP_ROM_EN_ON  (0<<MAPB_EN)
#define MAP_ROM_EN_OFF (1<<MAPB_EN)

#define MAP_ROM_WP_ON  (0<<MAPB_WP)
#define MAP_ROM_WP_OFF (1<<MAPB_WP)

#define MAPROM_LOWER 0xFF00000
#define MAPROM_UPPER 0xFF80000
#define MAPROM_SIZE 0x80000

#endif
