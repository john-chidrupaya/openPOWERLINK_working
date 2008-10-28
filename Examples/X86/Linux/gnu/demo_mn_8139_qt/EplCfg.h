/****************************************************************************

  (c) SYSTEC electronic GmbH, D-07973 Greiz, August-Bebel-Str. 29
      www.systec-electronic.de

  Project:      EPL Stack

  Description:  configuration file

  -------------------------------------------------------------------------

                $RCSfile$

                $Author$

                $Revision$  $Date$

                $State$

                Build Environment:
                    ...

  -------------------------------------------------------------------------

  Revision History:

  2006/06/06    k.t.: Start of Implementation

****************************************************************************/

#ifndef _EPLCFG_H_
#define _EPLCFG_H_




// =========================================================================
// generic defines which for whole EPL Stack
// =========================================================================
#define EPL_USE_DELETEINST_FUNC TRUE

// needed to support datatypes over 32 bit by global.h
#define USE_VAR64

// EPL_MAX_INSTANCES specifies count of instances of all EPL modules.
// If it is greater than 1 the first parameter of all
// functions is the instance number.
#define EPL_MAX_INSTANCES               1

// This defines the target hardware. Here is encoded wich CPU and wich external
// peripherals are connected. For possible values refere to target.h. If
// necessary value is not available EPL stack has to
// be adapted and tested.
#define TARGET_HARDWARE                 TGTHW_PC_WRAPP

// use no FIFOs, make direct calls
//#define EPL_NO_FIFO

// use no IPC between user- and kernelspace modules, make direct calls
#define EPL_NO_USER_KERNEL

#ifndef BENCHMARK_MODULES
#define BENCHMARK_MODULES       0 //0xEE800042L
#endif

// Default defug level:
// Only debug traces of these modules will be compiled which flags are set in define DEF_DEBUG_LVL.
#ifndef DEF_DEBUG_LVL
#define DEF_DEBUG_LVL           0xEC000000L
#endif
//   EPL_DBGLVL_OBD         =   0x00000004L
// * EPL_DBGLVL_ASSERT      =   0x20000000L
// * EPL_DBGLVL_ERROR       =   0x40000000L
// * EPL_DBGLVL_ALWAYS      =   0x80000000L


// EPL_MODULE_INTEGRATION defines all modules which are included in
// EPL application. Please add or delete modules for your application.
#define EPL_MODULE_INTEGRATION EPL_MODULE_OBDK \
                               | EPL_MODULE_PDOK \
                               | EPL_MODULE_NMT_MN \
                               | EPL_MODULE_SDOS \
                               | EPL_MODULE_SDOC \
                               | EPL_MODULE_SDO_ASND \
                               | EPL_MODULE_SDO_UDP \
                               | EPL_MODULE_NMT_CN \
                               | EPL_MODULE_NMTU \
                               | EPL_MODULE_NMTK \
                               | EPL_MODULE_DLLK \
                               | EPL_MODULE_DLLU \
                               | EPL_MODULE_VETH
//                               | EPL_MODULE_OBDU

// =========================================================================
// EPL ethernet driver (Edrv) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
#define EDRV_FAST_TXFRAMES              FALSE
//#define EDRV_FAST_TXFRAMES              TRUE

// switch this define to TRUE if Edrv supports early receive interrupts
#define EDRV_EARLY_RX_INT               FALSE
//#define EDRV_EARLY_RX_INT               TRUE

// enables setting of several port pins for benchmarking purposes
#define EDRV_BENCHMARK                  FALSE
//#define EDRV_BENCHMARK                  TRUE // MCF_GPIO_PODR_PCIBR

// Call Tx handler (i.e. EplDllCbFrameTransmitted()) already if DMA has finished,
// otherwise call the Tx handler if frame was actually transmitted over ethernet.
#define EDRV_DMA_TX_HANDLER             FALSE
//#define EDRV_DMA_TX_HANDLER             TRUE

// number of used ethernet controller
//#define EDRV_USED_ETH_CTRL              1

// increase the number of Tx buffers, because we are master
// and need one Tx buffer for each PReq and CN
// + SoC + SoA + MN PRes + NmtCmd + ASnd + IdentRes + StatusRes.
#define EDRV_MAX_TX_BUFFERS             80


// =========================================================================
// Data Link Layer (DLL) specific defines
// =========================================================================

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoC
#define EPL_DLL_PRES_READY_AFTER_SOC    FALSE
//#define EPL_DLL_PRES_READY_AFTER_SOC    TRUE

// switch this define to TRUE if Edrv supports fast tx frames
// and DLL shall pass PRes as ready to Edrv after SoA
#define EPL_DLL_PRES_READY_AFTER_SOA    FALSE
//#define EPL_DLL_PRES_READY_AFTER_SOA    TRUE


// =========================================================================
// OBD specific defines
// =========================================================================

// switch this define to TRUE if Epl should compare object range
// automaticly
#define EPL_OBD_CHECK_OBJECT_RANGE          FALSE
//#define EPL_OBD_CHECK_OBJECT_RANGE          TRUE

// set this define to TRUE if there are strings or domains in OD, which
// may be changed in object size and/or object data pointer by its object
// callback function (called event kObdEvWrStringDomain)
//#define EPL_OBD_USE_STRING_DOMAIN_IN_RAM    FALSE
#define EPL_OBD_USE_STRING_DOMAIN_IN_RAM    TRUE

#define EPL_OBD_USE_VARIABLE_SUBINDEX_TAB TRUE


// =========================================================================
// Timer module specific defines
// =========================================================================

// if TRUE it uses the Timer module implementation of EPL user also in EPL kernel
#define EPL_TIMER_USE_USER              TRUE

// if TRUE the high resolution timer module will be used
#define EPL_TIMER_USE_HIGHRES              TRUE
//#define EPL_TIMER_USE_HIGHRES              FALSE

// =========================================================================
// EPL API layer specific defines
// =========================================================================

// size of static input process image
// set to 0 to disable static process image
// it shall not exceed 252 and be a multiple of 4
#define EPL_API_PROCESS_IMAGE_SIZE_IN       16

// size of static output process image
// set to 0 to disable static process image
// it shall not exceed 252 and be a multiple of 4
#define EPL_API_PROCESS_IMAGE_SIZE_OUT      16


// configure whether OD access events shall be forwarded
// to user callback function.
// Because of reentrancy for local OD accesses, this has to be disabled
#define EPL_API_OBD_FORWARD_EVENT       FALSE


// =========================================================================
// SDO module specific defines
// =========================================================================

// increase the number of SDO channels, because we are master
#define EPL_SDO_MAX_CONNECTION_ASND 100
#define EPL_MAX_SDO_SEQ_CON         100
#define EPL_MAX_SDO_COM_CON         100
#define EPL_SDO_MAX_CONNECTION_UDP  50


#endif //_EPLCFG_H_


