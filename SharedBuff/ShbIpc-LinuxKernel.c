/****************************************************************************

  (c) SYSTEC electronic GmbH, D-08468 Heinsdorfergrund, Am Windrad 2
      www.systec-electronic.com

  Project:      Project independend shared buffer (linear + circular)

  Description:  Implementation of platform specific part for the
                shared buffer
                (Implementation for Linux KernelSpace)

  License:

    Redistribution and use in source and binary forms, with or without
    modification, are permitted provided that the following conditions
    are met:

    1. Redistributions of source code must retain the above copyright
       notice, this list of conditions and the following disclaimer.

    2. Redistributions in binary form must reproduce the above copyright
       notice, this list of conditions and the following disclaimer in the
       documentation and/or other materials provided with the distribution.

    3. Neither the name of SYSTEC electronic GmbH nor the names of its
       contributors may be used to endorse or promote products derived
       from this software without prior written permission. For written
       permission, please contact info@systec-electronic.com.

    THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
    "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
    LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
    FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
    COPYRIGHT HOLDERS OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
    INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
    BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
    LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
    CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
    LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
    ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
    POSSIBILITY OF SUCH DAMAGE.

    Severability Clause:

        If a provision of this License is or becomes illegal, invalid or
        unenforceable in any jurisdiction, that shall not affect:
        1. the validity or enforceability in that jurisdiction of any other
           provision of this License; or
        2. the validity or enforceability in other jurisdictions of that or
           any other provision of this License.

  -------------------------------------------------------------------------

  2006/06/28 -rs:   V 1.00 (initial version)

****************************************************************************/


#include "global.h"
#include "SharedBuff.h"
#include "ShbIpc.h"
#include "Debug.h"

#include <linux/version.h>
#include <linux/string.h>
#include <linux/module.h>
#include <asm/processor.h>
#include <linux/crc32.h>
#include <linux/sched.h>
#include <linux/param.h>
#include <linux/spinlock.h>
#include <linux/wait.h>
#include <linux/kthread.h>
#include <linux/slab.h>
#if LINUX_VERSION_CODE >= KERNEL_VERSION(2, 6, 26)
#include <linux/semaphore.h>
#endif


/***************************************************************************/
/*                                                                         */
/*                                                                         */
/*          G L O B A L   D E F I N I T I O N S                            */
/*                                                                         */
/*                                                                         */
/***************************************************************************/

#if (!defined(SHBIPC_INLINED)) || defined(SHBIPC_INLINE_ENABLED)

//---------------------------------------------------------------------------
//  Configuration
//---------------------------------------------------------------------------



//---------------------------------------------------------------------------
//  Constant definitions
//---------------------------------------------------------------------------

#define MAX_LEN_BUFFER_ID       256

#define TIMEOUT_ENTER_ATOMIC    1000        // (ms) for debgging: INFINITE
#define TIMEOUT_TERM_THREAD     1000
#define INFINITE                3600

#define SBI_MAGIC_ID            0x5342492B  // magic ID ("SBI+")
#define SBH_MAGIC_ID            0x5342482A  // magic ID ("SBH*")

#define INVALID_ID              NULL

#define TABLE_SIZE              10


//---------------------------------------------------------------------------
//  Local types
//---------------------------------------------------------------------------

// This structure is the common header for the shared memory region used
// by all processes attached this shared memory. It includes common
// information to administrate/manage the shared buffer from a couple of
// separated processes (e.g. the refernce counter). This structure is
// located at the start of the shared memory region itself and exists
// consequently only one times per shared memory instance.
typedef struct
{

    unsigned long       m_ulShMemSize;
    unsigned long       m_ulRefCount;
    int                 m_iBufferId;
//    int                 m_iUserSpaceMem;           //0 for userspace mem   !=0 kernelspace mem
    spinlock_t          m_SpinlockBuffAccess;
    BOOL                m_fNewData;
    BOOL                m_fJobReady;
    wait_queue_head_t   m_WaitQueueNewData;
    wait_queue_head_t   m_WaitQueueJobReady;
    tShbInstance*       m_pShbInstMaster;

    #ifndef NDEBUG
        unsigned long   m_ulOwnerProcID;
    #endif

} tShbMemHeader;



// This structure is the "external entry point" from a separate process
// to get access to a shared buffer. This structure includes all platform
// resp. target specific information to administrate/manage the shared
// buffer from a separate process. Every process attached to the shared
// buffer has its own runtime instance of this structure with its individual
// runtime data (e.g. the scope of an event handle is limitted to the
// owner process only). The structure member <m_pShbMemHeader> points
// to the (process specific) start address of the shared memory region
// itself.
typedef struct
{
    unsigned long       m_SbiMagicID;           // magic ID ("SBI+")
//    void*               m_pSharedMem;
    struct task_struct* m_tThreadNewDataId;
    long                m_lThreadNewDataNice;   // nice value of the new data thread
    struct task_struct* m_tThreadJobReadyId;
    unsigned long       m_ulFlagsBuffAccess;    // d.k. moved from tShbMemHeader, because each
                                                // process needs to store the interrupt flags separately
    tSigHndlrNewData    m_pfnSigHndlrNewData;
    unsigned long       m_ulTimeOutMsJobReady;
    tSigHndlrJobReady   m_pfnSigHndlrJobReady;
    struct semaphore    m_SemaphoreStopThreadJobReady;
    tShbMemHeader*      m_pShbMemHeader;
    #ifndef NDEBUG
        unsigned long   m_ulThreadIDNewData;
        unsigned long   m_ulThreadIDJobReady;
    #endif
} tShbMemInst;


//---------------------------------------------------------------------------
//  Prototypes of internal functions
//---------------------------------------------------------------------------

//tShbMemInst*            ShbIpcGetShbMemInst         (tShbInstance pShbInstance_p);
//tShbMemHeader*          ShbIpcGetShbMemHeader       (tShbMemInst* pShbMemInst_p);

//---------------------------------------------------------------------------
//  Get pointer to process local information structure
//---------------------------------------------------------------------------

static inline tShbMemInst*  ShbIpcGetShbMemInst (
    tShbInstance pShbInstance_p)
{

tShbMemInst*  pShbMemInst;


    pShbMemInst = (tShbMemInst*)pShbInstance_p;


    return (pShbMemInst);

}



//---------------------------------------------------------------------------
//  Get pointer to shared memory header
//---------------------------------------------------------------------------

static inline tShbMemHeader*  ShbIpcGetShbMemHeader (
    tShbMemInst* pShbMemInst_p)
{

tShbMemHeader*  pShbMemHeader;


    pShbMemHeader = pShbMemInst_p->m_pShbMemHeader;

    return (pShbMemHeader);

}

//  Get pointer to process local information structure
//#define ShbIpcGetShbMemInst(pShbInstance_p) ((tShbMemInst*)pShbInstance_p)

//  Get pointer to shared memory header
//#define ShbIpcGetShbMemHeader(pShbMemInst_p) (pShbMemInst_p->m_pShbMemHeader)

// not inlined internal functions
int                     ShbIpcThreadSignalNewData   (void* pvThreadParam_p);
int                     ShbIpcThreadSignalJobReady  (void* pvThreadParam_p);
#endif

//---------------------------------------------------------------------------
// module global vars
//---------------------------------------------------------------------------

#if !defined(SHBIPC_INLINE_ENABLED)
struct sShbMemTable *psMemTableElementFirst_g;

static void*            ShbIpcAllocPrivateMem       (unsigned long ulMemSize_p);
static int              ShbIpcFindListElement       (int iBufferId, struct sShbMemTable **ppsReturnMemTableElement);
static void             ShbIpcAppendListElement     (struct sShbMemTable *sNewMemTableElement);
static void             ShbIpcDeleteListElement     (int iBufferId);

#ifndef CONFIG_CRC32
static void             ShbIpcCrc32GenTable         (unsigned long aulCrcTable[256]);
static unsigned long    ShbIpcCrc32GetCrc           (const char *pcString, unsigned long aulCrcTable[256]);
#endif

#endif


//=========================================================================//
//                                                                         //
//          P U B L I C   F U N C T I O N S                                //
//                                                                         //
//=========================================================================//

#if !defined(SHBIPC_INLINE_ENABLED)
// not inlined external functions

//---------------------------------------------------------------------------
//  Initialize IPC for Shared Buffer Module
//---------------------------------------------------------------------------

tShbError  ShbIpcInit (void)
{
    psMemTableElementFirst_g = NULL;
    return (kShbOk);

}



//---------------------------------------------------------------------------
//  Deinitialize IPC for Shared Buffer Module
//---------------------------------------------------------------------------

tShbError  ShbIpcExit (void)
{

    return (kShbOk);

}



//---------------------------------------------------------------------------
//  Allocate Shared Buffer
//---------------------------------------------------------------------------

tShbError  ShbIpcAllocBuffer (
    unsigned long ulBufferSize_p,
    const char* pszBufferID_p,
    tShbInstance* ppShbInstance_p,
    unsigned int* pfShbNewCreated_p)
{
tShbError               ShbError;
int                     iBufferId=0;
unsigned long           ulCrc32=0;
unsigned int            uiFirstProcess=0;
unsigned long           ulShMemSize;
tShbMemHeader*          pShbMemHeader;
tShbMemInst*            pShbMemInst=NULL;
tShbInstance            pShbInstance;
unsigned int            fShMemNewCreated=FALSE;
void                    *pSharedMem=NULL;
struct sShbMemTable     *psMemTableElement;


    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer \n");
    ulShMemSize      = ulBufferSize_p + sizeof(tShbMemHeader);

    //create Buffer ID
#ifndef CONFIG_CRC32
    {
        unsigned long           aulCrcTable[256];
        ShbIpcCrc32GenTable(aulCrcTable);
        ulCrc32 = ShbIpcCrc32GetCrc(pszBufferID_p, aulCrcTable);
    }
#else
    ulCrc32 = crc32(0xFFFFFFFF, pszBufferID_p, strlen(pszBufferID_p));
#endif

    iBufferId=ulCrc32;
    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer BufferSize:%d sizeof(tShb..):%d\n",ulBufferSize_p,sizeof(tShbMemHeader));
    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer BufferId:%d MemSize:%d\n",iBufferId,ulShMemSize);
    //---------------------------------------------------------------
    // (1) open an existing or create a new shared memory
    //---------------------------------------------------------------
    //test if buffer already exists
    if (ShbIpcFindListElement(iBufferId, &psMemTableElement) == 0)
    {
        //Buffer already exists
        fShMemNewCreated=FALSE;
        pSharedMem = psMemTableElement->m_pBuffer;
        DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer attach Buffer at:%p Id:%d\n",pSharedMem);
        uiFirstProcess=1;
    }
    else
    {
        //create new Buffer
        fShMemNewCreated = TRUE;
        uiFirstProcess=0;
        pSharedMem = kmalloc(ulShMemSize,GFP_KERNEL);
        DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer Create New Buffer at:%p Id:%d\n",pSharedMem,iBufferId);
        if (pSharedMem == NULL)
        {
            //unable to create mem
            ShbError = kShbOutOfMem;
            goto Exit;
        }
        DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer create semas\n");
        // append Element to Mem Table
        psMemTableElement = kmalloc(sizeof(struct sShbMemTable),GFP_KERNEL);
        psMemTableElement->m_iBufferId = iBufferId;
        psMemTableElement->m_pBuffer = pSharedMem;
        psMemTableElement->m_psNextMemTableElement = NULL;
        ShbIpcAppendListElement (psMemTableElement);
    }

    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer update header\n");
    //update header
    pShbMemHeader = (tShbMemHeader*)pSharedMem;
    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer 0 pShbMemHeader->m_ulShMemSize: %d\n",pShbMemHeader->m_ulShMemSize);
    // allocate a memory block from process specific mempool to save
    // process local information to administrate/manage the shared buffer
    DEBUG_LVL_29_TRACE("ShbIpcAllocBuffer alloc private mem\n");
    pShbMemInst = (tShbMemInst*) ShbIpcAllocPrivateMem (sizeof(tShbMemInst));
    if (pShbMemInst == NULL)
    {
        ShbError = kShbOutOfMem;
        goto Exit;
    }

    // reset complete header to default values
    //pShbMemInst->m_SbiMagicID                             = SBI_MAGIC_ID;
//    pShbMemInst->m_pSharedMem                               = pSharedMem;
    pShbMemInst->m_tThreadNewDataId                         = INVALID_ID;
    pShbMemInst->m_tThreadJobReadyId                        = INVALID_ID;
    pShbMemInst->m_pfnSigHndlrNewData                       = NULL;
    pShbMemInst->m_ulTimeOutMsJobReady                      = 0;
    pShbMemInst->m_pfnSigHndlrJobReady                      = NULL;
    sema_init(&pShbMemInst->m_SemaphoreStopThreadJobReady, 1);
    pShbMemInst->m_pShbMemHeader                            = pShbMemHeader;

    ShbError         = kShbOk;
    if ( fShMemNewCreated )
    {
        // this process was the first who wanted to use the shared memory,
        // so a new shared memory was created
        // -> setup new header information inside the shared memory region
        //    itself
        pShbMemHeader->m_ulShMemSize = ulShMemSize;
        pShbMemHeader->m_ulRefCount  = 1;
        pShbMemHeader->m_iBufferId=iBufferId;
        pShbMemHeader->m_pShbInstMaster = NULL;
        // initialize spinlock
        spin_lock_init(&pShbMemHeader->m_SpinlockBuffAccess);
        // initialize wait queues
        init_waitqueue_head(&pShbMemHeader->m_WaitQueueNewData);
        init_waitqueue_head(&pShbMemHeader->m_WaitQueueJobReady);
    }
    else
    {
        // any other process has created the shared memory and this
        // process only has to attach to it
        // -> check and update existing header information inside the
        //    shared memory region itself
        if (pShbMemHeader->m_ulShMemSize != ulShMemSize)
        {
            ShbError = kShbOpenMismatch;
            goto Exit;
        }
        pShbMemHeader->m_ulRefCount++;
    }

    Exit:
    pShbInstance = (tShbInstance*)pShbMemInst;
    *pfShbNewCreated_p = fShMemNewCreated;
    *ppShbInstance_p   = pShbInstance;
    return (ShbError);

}



//---------------------------------------------------------------------------
//  Release Shared Buffer
//---------------------------------------------------------------------------

tShbError  ShbIpcReleaseBuffer (tShbInstance pShbInstance_p)
{
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;
tShbError       ShbError2;

    DEBUG_LVL_26_TRACE("ShbIpcReleaseBuffer(%p)\n", pShbInstance_p);
    if (pShbInstance_p == NULL)
    {
        return (kShbOk);
    }
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    // stop threads in any case, because they are bound to that specific instance
    ShbError2 = ShbIpcStopSignalingNewData (pShbInstance_p);

    down(&pShbMemInst->m_SemaphoreStopThreadJobReady);
    if (pShbMemInst->m_tThreadJobReadyId != INVALID_ID)
    {
        kthread_stop(pShbMemInst->m_tThreadJobReadyId);
    }
    up(&pShbMemInst->m_SemaphoreStopThreadJobReady);


    if ( !--pShbMemHeader->m_ulRefCount )
    {
        ShbError = kShbOk;
        // delete mem table element
        ShbIpcDeleteListElement(pShbMemHeader->m_iBufferId);
        // delete shared mem
        kfree(pShbMemInst->m_pShbMemHeader);
    }
    else
    {
        ShbError = kShbMemUsedByOtherProcs;
    }
    //delete privat mem
    kfree(pShbMemInst);
    return (ShbError);
}


//---------------------------------------------------------------------------
//  Signal new data (called from writing process)
//---------------------------------------------------------------------------

tShbError  ShbIpcSignalNewData (
    tShbInstance pShbInstance_p)
{
tShbMemHeader*  pShbMemHeader;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    pShbMemHeader = ShbIpcGetShbMemHeader (ShbIpcGetShbMemInst (pShbInstance_p));
    //set semaphore
    pShbMemHeader->m_fNewData = TRUE;
    DEBUG_LVL_29_TRACE("ShbIpcSignalNewData set Sem -> New Data\n");

    wake_up(&pShbMemHeader->m_WaitQueueNewData);

    if (pShbMemHeader->m_pShbInstMaster != NULL)
    {
        return ShbIpcSignalNewData(pShbMemHeader->m_pShbInstMaster);
    }

    return (kShbOk);
}



#endif  // !defined(SHBIPC_INLINE_ENABLED)

#if (!defined(SHBIPC_INLINED)) || defined(SHBIPC_INLINE_ENABLED)


//---------------------------------------------------------------------------
//  Enter atomic section for Shared Buffer access
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcEnterAtomicSection (
    tShbInstance pShbInstance_p)
{

tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError = kShbOk;

    if (pShbInstance_p == NULL)
    {
        ShbError = kShbInvalidArg;
        goto Exit;
    }
    DEBUG_LVL_29_TRACE("enter atomic\n");
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    // lock interrupts
    spin_lock_irqsave(&pShbMemHeader->m_SpinlockBuffAccess, pShbMemInst->m_ulFlagsBuffAccess);

Exit:
    return ShbError;

}



//---------------------------------------------------------------------------
//  Leave atomic section for Shared Buffer access
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcLeaveAtomicSection (
    tShbInstance pShbInstance_p)
{

tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError = kShbOk;

    if (pShbInstance_p == NULL)
    {
        ShbError = kShbInvalidArg;
        goto Exit;
    }
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);
    // unlock interrupts
    spin_unlock_irqrestore(&pShbMemHeader->m_SpinlockBuffAccess, pShbMemInst->m_ulFlagsBuffAccess);

Exit:
    DEBUG_LVL_29_TRACE("Leave Atomic \n");
    return ShbError;

}



//---------------------------------------------------------------------------
//  Set master instance of this slave instance
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcSetMaster (
    tShbInstance pShbInstance_p,
    tShbInstance pShbInstanceMaster_p)
{

tShbMemHeader*  pShbMemHeader;

    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }

    pShbMemHeader = ShbIpcGetShbMemHeader (pShbInstance_p);

    pShbMemHeader->m_pShbInstMaster = pShbInstanceMaster_p;

    return (kShbOk);

}



//---------------------------------------------------------------------------
//  Start signaling of new data (called from reading process)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcStartSignalingNewData (
    tShbInstance pShbInstance_p,
    tSigHndlrNewData pfnSignalHandlerNewData_p,
    tShbPriority ShbPriority_p)
{
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    DEBUG_LVL_29_TRACE("------->ShbIpcStartSignalingNewData\n");
    if ((pShbInstance_p == NULL) || (pfnSignalHandlerNewData_p == NULL))
    {
        return (kShbInvalidArg);
    }

    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);
    ShbError = kShbOk;

    if ((pShbMemInst->m_tThreadNewDataId != INVALID_ID)
        || (pShbMemInst->m_pfnSigHndlrNewData != NULL))
    {
        ShbError = kShbAlreadySignaling;
        goto Exit;
    }
    DEBUG_LVL_26_TRACE("ShbIpcStartSignalingNewData(%p) m_pfnSigHndlrNewData = %p\n", pShbInstance_p, pfnSignalHandlerNewData_p);
    pShbMemInst->m_pfnSigHndlrNewData = pfnSignalHandlerNewData_p;
    pShbMemHeader->m_fNewData = FALSE;

    switch (ShbPriority_p)
    {
        case kShbPriorityLow:
            pShbMemInst->m_lThreadNewDataNice = -2;
            break;

        case kShbPriorityNormal:
            pShbMemInst->m_lThreadNewDataNice = -9;
            break;

        case kShbPriorityHigh:
            pShbMemInst->m_lThreadNewDataNice = -20;
            break;

    }

    //create thread for signalling new data
    pShbMemInst->m_tThreadNewDataId = kthread_run(ShbIpcThreadSignalNewData,
                                                  pShbInstance_p,
                                                  "ShbND%p", pShbInstance_p);

    set_cpus_allowed(pShbMemInst->m_tThreadNewDataId, cpumask_of_cpu(1));

Exit:
    return ShbError;

}



//---------------------------------------------------------------------------
//  Stop signaling of new data (called from reading process)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcStopSignalingNewData (
    tShbInstance pShbInstance_p)
{
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    DEBUG_LVL_29_TRACE("------->ShbIpcStopSignalingNewData\n");
    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    ShbError = kShbOk;
    pShbMemInst = ShbIpcGetShbMemInst (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    DEBUG_LVL_26_TRACE("ShbIpcStopSignalingNewData(%p) pfnSignHndlrNewData=%p\n", pShbInstance_p, pShbMemInst->m_pfnSigHndlrNewData);
    if (pShbMemInst->m_pfnSigHndlrNewData != NULL)
    {   // signal handler was set before
        kthread_stop(pShbMemInst->m_tThreadNewDataId);

        pShbMemInst->m_pfnSigHndlrNewData = NULL;
        pShbMemInst->m_tThreadNewDataId = INVALID_ID;
    }

    return ShbError;

}



//---------------------------------------------------------------------------
//  Start signaling for job ready (called from waiting process)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcStartSignalingJobReady (
    tShbInstance pShbInstance_p,
    unsigned long ulTimeOutMs_p,
    tSigHndlrJobReady pfnSignalHandlerJobReady_p)
{
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
tShbError       ShbError;

    if ((pShbInstance_p == NULL) || (pfnSignalHandlerJobReady_p == NULL))
    {
        return (kShbInvalidArg);
    }
    pShbMemInst   = ShbIpcGetShbMemInst   (pShbInstance_p);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    ShbError = kShbOk;
    if ((pShbMemInst->m_tThreadJobReadyId != INVALID_ID)
        || (pShbMemInst->m_pfnSigHndlrJobReady != NULL))
    {
        ShbError = kShbAlreadySignaling;
        goto Exit;
    }
    pShbMemInst->m_ulTimeOutMsJobReady = ulTimeOutMs_p;
    pShbMemInst->m_pfnSigHndlrJobReady = pfnSignalHandlerJobReady_p;
    pShbMemHeader->m_fJobReady = FALSE;

    //create thread for signalling new data
    pShbMemInst->m_tThreadJobReadyId = kthread_run(ShbIpcThreadSignalJobReady,
                                                   pShbInstance_p,
                                                   "ShbJR%p", pShbInstance_p);
Exit:
    return ShbError;
}



//---------------------------------------------------------------------------
//  Signal job ready (called from executing process)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcSignalJobReady (
    tShbInstance pShbInstance_p)
{
tShbMemHeader*  pShbMemHeader;


    DEBUG_LVL_29_TRACE("ShbIpcSignalJobReady\n");
    if (pShbInstance_p == NULL)
    {
        return (kShbInvalidArg);
    }
    pShbMemHeader = ShbIpcGetShbMemHeader (ShbIpcGetShbMemInst (pShbInstance_p));
    //set semaphore
    pShbMemHeader->m_fJobReady = TRUE;
    DEBUG_LVL_29_TRACE("ShbIpcSignalJobReady set Sem -> Job Ready \n");

    wake_up(&pShbMemHeader->m_WaitQueueJobReady);
    return (kShbOk);
}



//---------------------------------------------------------------------------
//  Get pointer to common used share memory area
//---------------------------------------------------------------------------

INLINE_FUNCTION void*  ShbIpcGetShMemPtr (tShbInstance pShbInstance_p)
{

tShbMemHeader*  pShbMemHeader;
void*  pShbShMemPtr;


    pShbMemHeader = ShbIpcGetShbMemHeader (ShbIpcGetShbMemInst (pShbInstance_p));
    if (pShbMemHeader != NULL)
    {
        pShbShMemPtr = (BYTE*)pShbMemHeader + sizeof(tShbMemHeader);
    }
    else
    {
        pShbShMemPtr = NULL;
    }

    return (pShbShMemPtr);

}


//---------------------------------------------------------------------------
//  Process function (only used for implementations without threads)
//---------------------------------------------------------------------------

INLINE_FUNCTION tShbError  ShbIpcProcess (void)
{
tShbError       ShbError = kShbOk;

    return ShbError;
}


#endif



//=========================================================================//
//                                                                         //
//          P R I V A T E   F U N C T I O N S                              //
//                                                                         //
//=========================================================================//

#if !defined(SHBIPC_INLINE_ENABLED)

//---------------------------------------------------------------------------
//  Get pointer to process local information structure
//---------------------------------------------------------------------------

/*tShbMemInst*  ShbIpcGetShbMemInst (
    tShbInstance pShbInstance_p)
{

tShbMemInst*  pShbMemInst;


    pShbMemInst = (tShbMemInst*)pShbInstance_p;


    return (pShbMemInst);

}
*/


//---------------------------------------------------------------------------
//  Get pointer to shared memory header
//---------------------------------------------------------------------------

/*tShbMemHeader*  ShbIpcGetShbMemHeader (
    tShbMemInst* pShbMemInst_p)
{

tShbMemHeader*  pShbMemHeader;


    pShbMemHeader = pShbMemInst_p->m_pShbMemHeader;

    return (pShbMemHeader);

}
*/


//---------------------------------------------------------------------------
//  Allocate a memory block from process specific mempool
//---------------------------------------------------------------------------

static void*  ShbIpcAllocPrivateMem (unsigned long ulMemSize_p)
{
tShbError       ShbError;
void*           pMem;

    DEBUG_LVL_29_TRACE("ShbIpcAllocPrivateMem \n");
    //get private mem
    pMem = kmalloc(ulMemSize_p, GFP_KERNEL);
    if (pMem == NULL)
    {
        //unable to create mem
        ShbError = kShbOutOfMem;
        goto Exit;
    }
Exit:
    return (pMem);

}


//---------------------------------------------------------------------------
//  Thread for new data signaling
//---------------------------------------------------------------------------

int ShbIpcThreadSignalNewData (void *pvThreadParam_p)
{
tShbInstance    pShbInstance;
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
int             fCallAgain;

    pShbInstance = (tShbMemInst*)pvThreadParam_p;
    pShbMemInst  = ShbIpcGetShbMemInst (pShbInstance);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    DEBUG_LVL_26_TRACE("ShbIpcThreadSignalNewData(%p)\n",pvThreadParam_p);

    set_user_nice(current, pShbMemInst->m_lThreadNewDataNice);

#if defined(CONFIG_PREEMPT_RT_FULL) || defined(CONFIG_PREEMPT_RT)
    if (pShbMemInst->m_lThreadNewDataNice == -20) // highest priority
    {
    struct sched_param  rt_prio;
        rt_prio.sched_priority = 79;
        sched_setscheduler(current, SCHED_FIFO, &rt_prio);
    }
#endif

//            DEBUG_LVL_29_TRACE("ShbIpcThreadSignalNewData wait for New Data Sem %p\n",pShbMemInst->m_pSemNewData);
    while (!kthread_should_stop())
    {
        wait_event_interruptible(pShbMemHeader->m_WaitQueueNewData,
            kthread_should_stop()
            || (pShbMemHeader->m_fNewData != FALSE));

        if (pShbMemHeader->m_fNewData != FALSE)
        {
            pShbMemHeader->m_fNewData = FALSE;
            do
            {
                fCallAgain = pShbMemInst->m_pfnSigHndlrNewData(pShbInstance);
                // call scheduler, which will execute any task with higher priority
                schedule();
            } while (fCallAgain != FALSE);
        }
    }

    DEBUG_LVL_29_TRACE("ShbIpcThreadSignalNewData terminated \n");

    return 0;
}



//---------------------------------------------------------------------------
//  Thread for new data Job Ready signaling
//---------------------------------------------------------------------------

int ShbIpcThreadSignalJobReady (void *pvThreadParam_p)
{
tShbInstance    pShbInstance;
tShbMemInst*    pShbMemInst;
tShbMemHeader*  pShbMemHeader;
long            lTimeOutJiffies;
int             iRetVal=-1;

    pShbInstance = (tShbMemInst*)pvThreadParam_p;
    pShbMemInst  = ShbIpcGetShbMemInst (pShbInstance);
    pShbMemHeader = ShbIpcGetShbMemHeader (pShbMemInst);

    DEBUG_LVL_29_TRACE("ShbIpcThreadSignalJobReady wait for job ready Sem\n");
    if (pShbMemInst->m_ulTimeOutMsJobReady != 0)
    {
        lTimeOutJiffies = (long) pShbMemInst->m_ulTimeOutMsJobReady / (1000 / HZ);
        if (lTimeOutJiffies <= 0)
        {   // wait at least 1 jiffy
            lTimeOutJiffies = 1;
        }
        //wait for job ready semaphore
        iRetVal = wait_event_timeout(pShbMemHeader->m_WaitQueueJobReady,
            kthread_should_stop()
            || (pShbMemHeader->m_fJobReady != FALSE),
            lTimeOutJiffies);
    }
    else
    {
        //wait for job ready semaphore
        wait_event_interruptible(pShbMemHeader->m_WaitQueueJobReady,
            kthread_should_stop()
            || (pShbMemHeader->m_fJobReady != FALSE));
    }

    if (pShbMemInst->m_pfnSigHndlrJobReady != NULL)
    {
        //call Handler
        pShbMemInst->m_pfnSigHndlrJobReady(pShbInstance, !pShbMemHeader->m_fJobReady);
    }

    pShbMemInst->m_pfnSigHndlrJobReady = NULL;

    if (down_trylock(&pShbMemInst->m_SemaphoreStopThreadJobReady))
    {   // lock failed
        wait_event_interruptible(pShbMemHeader->m_WaitQueueJobReady,
                                 kthread_should_stop());

        pShbMemInst->m_tThreadJobReadyId = INVALID_ID;
    }
    else
    {
        pShbMemInst->m_tThreadJobReadyId = INVALID_ID;
        up(&pShbMemInst->m_SemaphoreStopThreadJobReady);
    }

    DEBUG_LVL_29_TRACE("ShbIpcThreadSignalJobReady terminated\n");

    return 0;
}



#ifndef CONFIG_CRC32
//Build the crc table
static void ShbIpcCrc32GenTable(unsigned long aulCrcTable[256])
{
    unsigned long       ulCrc,ulPoly;
    int                 iIndexI,iIndexJ;

    ulPoly = 0xEDB88320L;
    for (iIndexI = 0; iIndexI < 256; iIndexI++)
    {
        ulCrc = iIndexI;
        for (iIndexJ = 8; iIndexJ > 0; iIndexJ--)
        {
            if (ulCrc & 1)
            {
                ulCrc = (ulCrc >> 1) ^ ulPoly;
            }
            else
            {
                ulCrc >>= 1;
            }
        }
        aulCrcTable[iIndexI] = ulCrc;
    }
}

//Calculate the crc value
static unsigned long ShbIpcCrc32GetCrc(const char *pcString,unsigned long aulCrcTable[256])
{
    unsigned long   ulCrc;
    int             iIndex;

    ulCrc = 0xFFFFFFFF;
    for (iIndex=0;iIndex<strlen(pcString);iIndex++)
    {
        ulCrc = ((ulCrc>>8) & 0x00FFFFFF) ^ aulCrcTable[ (ulCrc^pcString[iIndex]) & 0xFF ];
    }
    return( ulCrc^0xFFFFFFFF );

}
#endif

static void ShbIpcAppendListElement (struct sShbMemTable *psNewMemTableElement)
{
    struct sShbMemTable *psMemTableElement=psMemTableElementFirst_g;
    psNewMemTableElement->m_psNextMemTableElement=NULL;

    if (psMemTableElementFirst_g!= NULL )
    {      /* sind Elemente vorhanden */
       while (psMemTableElement->m_psNextMemTableElement != NULL )
       {    /* suche das letzte Element */
           psMemTableElement=psMemTableElement->m_psNextMemTableElement;
       }
       psMemTableElement->m_psNextMemTableElement=psNewMemTableElement;              /*  Haenge das Element hinten an */
    }
    else
    {                           /* wenn die liste leer ist, bin ich das erste Element */
        psMemTableElementFirst_g=psNewMemTableElement;
    }
}




static int ShbIpcFindListElement (int iBufferId, struct sShbMemTable **ppsReturnMemTableElement)
{
    struct sShbMemTable *psMemTableElement=psMemTableElementFirst_g;
    while (psMemTableElement!=NULL)
    {
        if(psMemTableElement->m_iBufferId==iBufferId)
        {
//printk("ShbIpcFindListElement Buffer at:%p Id:%d\n",psMemTableElement->m_pBuffer,psMemTableElement->m_iBufferId);
             *ppsReturnMemTableElement=psMemTableElement;
//printk("ShbIpcFindListElement Buffer at:%p Id:%d\n",(*ppsReturnMemTableElement)->m_pBuffer,(*ppsReturnMemTableElement)->m_iBufferId);
             return 0;
        }
        psMemTableElement=psMemTableElement->m_psNextMemTableElement;
    }
    return -1;
}

static void ShbIpcDeleteListElement(int iBufferId)
{
   struct sShbMemTable *psMemTableElement=psMemTableElementFirst_g;
   struct sShbMemTable *psMemTableElementOld=psMemTableElementFirst_g;
   if (psMemTableElement!=NULL)
   {
        while((psMemTableElement!=NULL)&&(psMemTableElement->m_iBufferId!=iBufferId))
        {
            psMemTableElementOld=psMemTableElement;
            psMemTableElement=psMemTableElement->m_psNextMemTableElement;
        }
        if (psMemTableElement!=NULL)
        {
            if (psMemTableElement!=psMemTableElementFirst_g)
            {
                psMemTableElementOld->m_psNextMemTableElement=psMemTableElement->m_psNextMemTableElement;
                kfree(psMemTableElement);
            }
            else
            {
                kfree(psMemTableElement);
                psMemTableElementFirst_g=NULL;
            }

        }
   }

}

#endif
