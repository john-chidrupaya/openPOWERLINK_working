/**
********************************************************************************
\file   errsigk.c

\brief  Implementation of kernel error signaling module

This module implements the kernel part of the error signaling module.
It is responsible for storing and sending the errors on CN to MN.

\ingroup module_errhndk
*******************************************************************************/

/*------------------------------------------------------------------------------
Copyright (c) 2012, SYSTEC electronic GmbH
Copyright (c) 2012, Bernecker+Rainer Industrie-Elektronik Ges.m.b.H. (B&R)
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
    * Redistributions of source code must retain the above copyright
      notice, this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright
      notice, this list of conditions and the following disclaimer in the
      documentation and/or other materials provided with the distribution.
    * Neither the name of the copyright holders nor the
      names of its contributors may be used to endorse or promote products
      derived from this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL COPYRIGHT HOLDERS BE LIABLE FOR ANY
DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
------------------------------------------------------------------------------*/

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------
#include <nmt.h>
#include <Benchmark.h>
#include <obd.h>
#include <kernel/eventk.h>
#include <kernel/dllk.h>

#include <errhnd.h>
#include <kernel/errhndk.h>

#include "errhndkcal.h"

#include "kernel/errsigk.h"
//============================================================================//
//            G L O B A L   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// module global vars
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// global function prototypes
//------------------------------------------------------------------------------
tEplKernel errsigk_allocateErrStatusBuffers(void);
tEplKernel errsigk_deAllocateErrStatusBuffers(void);
tEplKernel errsigk_deallocateStatusEntryQueue(tErrSigkBuffer* errSigkBuffer);
tEplKernel errsigk_addStatusEntrytoQueue(
                                            UINT8* m_uiNumberOfHistoryEntries,
                                            tEplErrHistoryEntry** dstStatusQueue,
                                            tEplErrHistoryEntry* historyEntry
                                           );
//============================================================================//
//          P R I V A T E   D E F I N I T I O N S                             //
//============================================================================//

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------

#define MAX_STATUS_FRAMES            3
#define MAX_STATUS_ENTRY_PER_BUFFER  5
//------------------------------------------------------------------------------
// local types
//------------------------------------------------------------------------------

typedef enum
{
    kNoBuffer               = 0,    ///< No ErrorSignaling buffer is initialised
    kBuffersEmpty,                  ///< No ErrorSignaling buffer is used
    kBuffersAvailable,              ///< Some buffers excluding the reserved one,
                                        ///with ErrorSignaling module are occupied
    kBuffersFull,                   ///< All buffers excluding the reserved one,
                                        ///with ErrorSignaling module are occupied
    kReservedBufferFull             ///< All buffers including the reserved one,
                                        ///with ErrorSignaling module are occupied
}tErrSigkBufferStatus;

/**
\brief  instance of kernel error signaller

The structure defines the instance variables of the kernel error signaller
*/
typedef struct
{
    tErrSigkBufferStatus    m_Status;               ///< current status of the Error Status Buffer
    tErrSigkBuffer*         m_ErrorBufferHead;      ///< The first Error Status Buffer
    tErrSigkBuffer*         m_pCurrentErrorBuffer;  ///< Current Error Status Buffer used for storing status entries
    tErrSigkBuffer*         m_pReservedErrorBuffer; ///< Reserved Error Status Buffer for storing any sudden error status changes
}tErrSigkInstance;

//------------------------------------------------------------------------------
// module local vars
//------------------------------------------------------------------------------
static tErrSigkInstance instance_l;
//------------------------------------------------------------------------------
// local function prototypes
//------------------------------------------------------------------------------



//============================================================================//
//            P U B L I C   F U N C T I O N S                                 //
//============================================================================//

//------------------------------------------------------------------------------
/**
\brief    Initialize kernel error signaller module

The function initializes the kernel error signaller module.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_init(void)
{
    tEplKernel      ret;

    ret = kEplSuccessful;
    //TODO: Reset Error queue and error objects: requires posting event to user side.
    instance_l.m_Status = kNoBuffer;
    instance_l.m_ErrorBufferHead = NULL;
    instance_l.m_pCurrentErrorBuffer = NULL;
    instance_l.m_pReservedErrorBuffer = NULL;

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    Stops kernel error signaller module

The function stops the kernel error signaller module.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_exit(void)
{
    tEplKernel      ret;

    ret = kEplSuccessful;
    //TODO: Reset Error queue and error objects: requires posting event to user side.
    if (instance_l.m_Status != kNoBuffer)
    {
        if((ret = errsigk_deAllocateErrStatusBuffers()) != kEplSuccessful)
        {
            goto Exit;
        }
    }
    instance_l.m_Status = kNoBuffer;
    instance_l.m_ErrorBufferHead = NULL;
    instance_l.m_pCurrentErrorBuffer = NULL;
    instance_l.m_pReservedErrorBuffer = NULL;

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    Resets kernel error signaller module

The function resets the kernel error signaller module to the status
after initialisation.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_reset(void)
{
    tEplKernel      ret;
    UINT8           loopCount;
    tErrSigkBuffer* currentErrSigkBuffer;
    //printf("Entered %s\n", __func__);
    ret = kEplSuccessful;
    //TODO: Reset Error queue and error objects: requires posting event to user side.

    instance_l.m_Status = kBuffersEmpty;

    if (instance_l.m_pReservedErrorBuffer == NULL)
    {
        ret = kEplInvalidEvent;
        goto Exit;
    }

    instance_l.m_pReservedErrorBuffer->m_fDataValid = FALSE;
    instance_l.m_pReservedErrorBuffer->m_qwStaticError = 0x0;
    ret = errsigk_deallocateStatusEntryQueue(instance_l.m_pReservedErrorBuffer);


    currentErrSigkBuffer = (tErrSigkBuffer*) instance_l.m_ErrorBufferHead;

    for (loopCount = 1; loopCount < (MAX_STATUS_FRAMES - 1); loopCount++)
    {

        if (currentErrSigkBuffer == NULL)
        {
            ret = kEplInvalidEvent;
            break;
        }

        currentErrSigkBuffer->m_fDataValid = FALSE;
        currentErrSigkBuffer->m_qwStaticError = 0x0;
        ret = errsigk_deallocateStatusEntryQueue(currentErrSigkBuffer);

        currentErrSigkBuffer = currentErrSigkBuffer->m_pNextErrorBuffer;

    }
    //printf("Exit %s\n", __func__);
Exit:
    return ret;
}

#define ERROR_TEST
//------------------------------------------------------------------------------
/**
\brief    Returns the current status of kernel error signaller module

The function returns the current status and the error status buffer
when polled by dllk.

\param      dllErrStatusBuffer      pointer to the current error status
                                    buffer owned by dllk, This will be
                                    exchanged if new status entries are available
\param      fErrFlag                flag to indicate to dllk if new status entries
                                    are available

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

tEplKernel errsigk_getErrStatusBuffer(tErrSigkBuffer** dllErrStatusBuffer, BOOL* fErrFlag)
{
    tEplKernel ret;
    tErrSigkBuffer* nextErrorBuffer;
    tErrSigkBuffer* currentErrorBuffer;

#ifdef ERROR_TEST
    static UINT8 Count = 0;
    tEplErrHistoryEntry         historyEntry;
    tEplNetTime netTime;
    Count++;
    if(Count%10 == 0)
    {
        Count = 0;
        netTime.m_dwNanoSec = 0;
        netTime.m_dwSec = 0;
        historyEntry.m_wEntryType = EPL_ERR_ENTRYTYPE_MODE_OCCURRED |
                                    EPL_ERR_ENTRYTYPE_PROF_EPL |
                                    EPL_ERR_ENTRYTYPE_HISTORY;

        historyEntry.m_wErrorCode = EPL_E_DLL_CRC_TH;
        historyEntry.m_TimeStamp = netTime;
        memset (historyEntry.m_abAddInfo, 0, sizeof(historyEntry.m_abAddInfo));


        //printf("Set Err\n");
        ret = errsigk_addStatusEntry(&historyEntry);
        //printf("Err Set\n");
    }

#endif

    ret = kEplSuccessful;


    if ((instance_l.m_Status <= kBuffersEmpty) || (*dllErrStatusBuffer == NULL))
    {
        *fErrFlag = FALSE;
        goto Exit;
    }
    nextErrorBuffer = (*dllErrStatusBuffer)->m_pNextErrorBuffer;
    if (nextErrorBuffer->m_fDataValid == FALSE)
    {
        //TODO Check at this point if Reserved Buffer data is valid, that would be awkward
        *fErrFlag = FALSE;
        goto Exit;
    }

    currentErrorBuffer = instance_l.m_pCurrentErrorBuffer;

    //Check if dll has updated the previous data in StatusRes
    if ((*dllErrStatusBuffer)->m_fDataValid == TRUE)
    {
        *fErrFlag = FALSE;
        ret = kEplInvalidOperation;
        goto Exit;
    }
    (*dllErrStatusBuffer)->m_uiOwner = kOwnerErrSigk;
    ret = errsigk_deallocateStatusEntryQueue(*dllErrStatusBuffer);
    (*dllErrStatusBuffer)->m_qwStaticError = 0;

    nextErrorBuffer->m_uiOwner = kOwnerDll;
    *dllErrStatusBuffer = nextErrorBuffer;
    if (currentErrorBuffer->m_uiOwner == kOwnerDll)
    {
        //XXX Should I check the owner here!!
        currentErrorBuffer = currentErrorBuffer->m_pNextErrorBuffer;
        instance_l.m_Status = kBuffersAvailable;
        //XXX Reinitialization not necessary
        if ((instance_l.m_Status >= kBuffersFull) && (instance_l.m_pReservedErrorBuffer->m_fDataValid == TRUE))
        {
            currentErrorBuffer->m_pErrHistoryEntry = instance_l.m_pReservedErrorBuffer->m_pErrHistoryEntry;
            currentErrorBuffer->m_pErrStatusEntry = instance_l.m_pReservedErrorBuffer->m_pErrStatusEntry;
            currentErrorBuffer->m_qwStaticError = instance_l.m_pReservedErrorBuffer->m_qwStaticError;
            currentErrorBuffer->m_uiNumberOfHistoryEntries = instance_l.m_pReservedErrorBuffer->m_uiNumberOfHistoryEntries;
            currentErrorBuffer->m_uiNumberOfStatusEntries = instance_l.m_pReservedErrorBuffer->m_uiNumberOfStatusEntries;
            currentErrorBuffer->m_fDataValid = TRUE;

            instance_l.m_pReservedErrorBuffer->m_qwStaticError = 0x0;
            instance_l.m_pReservedErrorBuffer->m_pErrHistoryEntry = NULL;
            instance_l.m_pReservedErrorBuffer->m_pErrStatusEntry = NULL;
            instance_l.m_pReservedErrorBuffer->m_uiNumberOfHistoryEntries = 0;
            instance_l.m_pReservedErrorBuffer->m_uiNumberOfStatusEntries = 0;
            if (currentErrorBuffer->m_uiNumberOfHistoryEntries == MAX_STATUS_ENTRY_PER_BUFFER)
            {
                instance_l.m_Status = kBuffersFull;
            }
        }
    }

    instance_l.m_pCurrentErrorBuffer = currentErrorBuffer;
    *fErrFlag = TRUE;




Exit:
    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    Appends a new error status to kernel error signaller module

The function appends a new error status entry to the current error
status buffer owned by kernel error signaller module.

\param      historyEntry        pointer to the status entry to be appended

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

tEplKernel errsigk_addStatusEntry(tEplErrHistoryEntry*  historyEntry)
{
    tEplKernel ret;
    tErrSigkBuffer** currentErrorBuffer;
    tErrSigkBuffer* nextErrorBuffer;
    tErrSigkBufferStatus    nextProbableStatus;

    ret = kEplSuccessful;
//TODO:
    /* 1. The entry type check is to be done error handler 2. Handle the Error Status Bit field
     * 2. Status and History entry order i.e. History entries from Emergency queue are to be kept after status entries
     * 3. CN supported items: case 1/ case 2/ case 3/ case 4
     * 4. Cases for which different types of status entry validation is used (M=0/ M=1, C=0/ M=1, C>0)/ variable frame length
     *
     */
    switch(instance_l.m_Status)
    {
        case kNoBuffer:
            //entry Dropped
            goto Exit;

        case kReservedBufferFull:
            //The reserved buffer has to be updated with the most recent status
            currentErrorBuffer = &instance_l.m_pReservedErrorBuffer;
            instance_l.m_Status = kBuffersFull;
            nextProbableStatus = kReservedBufferFull;
            ret = errsigk_deallocateStatusEntryQueue(*currentErrorBuffer);
            break;

        case kBuffersFull:
            currentErrorBuffer = &instance_l.m_pReservedErrorBuffer;
            nextProbableStatus = kReservedBufferFull;
            break;

        default: //kBuffersAvailable || kBuffersEmpty
            currentErrorBuffer = &instance_l.m_pCurrentErrorBuffer;
            instance_l.m_Status = kBuffersAvailable;
            nextProbableStatus = kBuffersFull;
            break;
    }

    if (historyEntry->m_wEntryType & EPL_ERR_ENTRYTYPE_EMCY)
    {
        ret = errsigk_addStatusEntrytoQueue(&(*currentErrorBuffer)->m_uiNumberOfHistoryEntries,
                                                &(*currentErrorBuffer)->m_pErrHistoryEntry, historyEntry);
    }
    else if (historyEntry->m_wEntryType & EPL_ERR_ENTRYTYPE_STATUS)
    {
        ret = errsigk_addStatusEntrytoQueue(&(*currentErrorBuffer)->m_uiNumberOfStatusEntries,
                                                &(*currentErrorBuffer)->m_pErrStatusEntry, historyEntry);
    }

    if (ret == kEplSuccessful)
    {
        (*currentErrorBuffer)->m_fDataValid = TRUE;
        (*currentErrorBuffer)->m_qwStaticError |= EPL_ERR_GENERIC;
    }

    if (((*currentErrorBuffer)->m_uiNumberOfHistoryEntries +
            (*currentErrorBuffer)->m_uiNumberOfStatusEntries) == MAX_STATUS_ENTRY_PER_BUFFER)
    {
        nextErrorBuffer = (*currentErrorBuffer)->m_pNextErrorBuffer;
        if ((nextErrorBuffer == NULL) || (nextErrorBuffer->m_uiOwner == kOwnerDll))
        {
            instance_l.m_Status = nextProbableStatus;
        }
        else //if (nextErrorBuffer->m_uiOwner == kOwnerErrSigk)
        {
            (*currentErrorBuffer) = nextErrorBuffer;
        }
    }

Exit:
    return ret;
}

//------------------------------------------------------------------------------
/**
\brief    updates the static error bit field in kernel error signaller module

The function updates the static error bit field in the error status buffer
when triggered by a change in OBD. If the current buffer already holds an
invalidated bit field, this data  is updated in the Emergency Buffer

\param      pEvent_p                pointer to the event structure
                                    holding the new data

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

tEplKernel errsigk_updateStaticErrorBitField(tEplEvent* pEvent_p)
{
    tEplKernel      ret;
    tErrSigkBuffer* currentErrorBuffer;
    UINT8           staticErrorBitField;

    ret = kEplSuccessful;


    if (instance_l.m_Status == kNoBuffer)
    {
        //Entry dropped
        goto Exit;
    }

    currentErrorBuffer = instance_l.m_pCurrentErrorBuffer;

    //Check if the current buffer holds any error bit field
    if (currentErrorBuffer->m_qwStaticError & ~EPL_ERR_GENERIC)
    {
        currentErrorBuffer = instance_l.m_pReservedErrorBuffer;
    }
    staticErrorBitField = *((UINT8*)pEvent_p->m_pArg);  //XXX: Unused pEvent_p->m_uiSize

    //Check if the passed error bit field is valid
    if (staticErrorBitField & EPL_ERR_GENERIC)
    {
        currentErrorBuffer->m_qwStaticError = staticErrorBitField;
    }

Exit:
    return ret;
}
//------------------------------------------------------------------------------
/**
\brief    creates buffers kernel error signaller module

The function initialises error status buffers for the kernel
error signaller module when called by dllk and decides the ownership of the
buffers

\param      dllErrStatusBuffer      pointer to where the dllk-owned error status
                                    buffer shall be stored

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_createErrStatusBuffers(tErrSigkBuffer** dllErrStatusBuffer)
{
    tEplKernel      ret;

    ret = kEplSuccessful;
    //printf("Entered %s\n", __func__);
    if (instance_l.m_Status != kNoBuffer)
    {
        ret = kEplInvalidEvent;
        goto Exit;
    }
    ret = errsigk_allocateErrStatusBuffers();

    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
    instance_l.m_Status = kBuffersEmpty;
    (*dllErrStatusBuffer) = instance_l.m_ErrorBufferHead;
    (*dllErrStatusBuffer)->m_uiOwner = kOwnerDll;
    instance_l.m_pCurrentErrorBuffer = (*dllErrStatusBuffer)->m_pNextErrorBuffer;
    //printf("Exit %s\n", __func__);

Exit:
    return ret;


}


//------------------------------------------------------------------------------
/**
\brief    Deallocates the buffers in kernel error signaller module

The function deallocates error status buffers and resets the resets the
error signalling module structure

\param      dllErrStatusBuffer      pointer to where the dllk-owned error status
                                    buffer

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------
tEplKernel errsigk_cleanErrStatusBuffers(tErrSigkBuffer** dllErrStatusBuffer)
{
    tEplKernel      ret;

    ret = kEplSuccessful;
    //printf("Entered %s\n", __func__);
    if (instance_l.m_Status == kNoBuffer)
    {
        ret = kEplInvalidEvent;
        goto Exit;
    }
    ret = errsigk_deAllocateErrStatusBuffers();

    if (ret != kEplSuccessful)
    {
        goto Exit;
    }
    instance_l.m_Status = kNoBuffer;
    (*dllErrStatusBuffer) = NULL;
    instance_l.m_ErrorBufferHead = NULL;
    instance_l.m_pCurrentErrorBuffer = NULL;
    //printf("Exit %s\n", __func__);

Exit:
    return ret;


}





//============================================================================//
//            P R I V A T E   F U N C T I O N S                               //
//============================================================================//
/// \name    private functions
/// \{

//------------------------------------------------------------------------------
/**
\brief    allocates the buffers for error signaller module

The function allocates the error status buffers.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

tEplKernel errsigk_allocateErrStatusBuffers(void)
{
    tEplKernel      ret;
    UINT8           loopCount;
    tErrSigkBuffer* currentErrSigkBuffer;
    tErrSigkBuffer* previousErrSigkBuffer;

    ret = kEplSuccessful;

    currentErrSigkBuffer = (tErrSigkBuffer*) EPL_MALLOC(sizeof (tErrSigkBuffer));

    if (currentErrSigkBuffer == NULL)
    {
        ret = kEplNoResource;
        goto Exit;
    }

    currentErrSigkBuffer->m_fDataValid = FALSE;
    currentErrSigkBuffer->m_qwStaticError = 0x0;
    currentErrSigkBuffer->m_uiNumberOfHistoryEntries = 0;
    currentErrSigkBuffer->m_uiNumberOfStatusEntries = 0;
    currentErrSigkBuffer->m_uiOwner = kOwnerReserved;
    currentErrSigkBuffer->m_pNextErrorBuffer = NULL;
    currentErrSigkBuffer->m_pErrHistoryEntry = NULL;
    currentErrSigkBuffer->m_pErrStatusEntry = NULL;
    instance_l.m_pReservedErrorBuffer = currentErrSigkBuffer;

    for (loopCount = 1; loopCount < MAX_STATUS_FRAMES; loopCount++)
    {
        currentErrSigkBuffer = (tErrSigkBuffer*) EPL_MALLOC(sizeof (tErrSigkBuffer));

        if (currentErrSigkBuffer == NULL)
        {
            ret = kEplNoResource;
            break;
        }

        currentErrSigkBuffer->m_fDataValid = FALSE;
        currentErrSigkBuffer->m_qwStaticError = 0x0;
        currentErrSigkBuffer->m_uiNumberOfHistoryEntries = 0;
        currentErrSigkBuffer->m_uiNumberOfStatusEntries = 0;
        currentErrSigkBuffer->m_uiOwner = kOwnerErrSigk;
        currentErrSigkBuffer->m_pErrHistoryEntry = NULL;
        currentErrSigkBuffer->m_pErrStatusEntry = NULL;
        if (loopCount == 1)
        {
            instance_l.m_ErrorBufferHead = currentErrSigkBuffer;
        }
        else
        {
            if (loopCount == (MAX_STATUS_FRAMES - 1))
            {
                currentErrSigkBuffer->m_pNextErrorBuffer = instance_l.m_ErrorBufferHead;
            }
            previousErrSigkBuffer->m_pNextErrorBuffer = currentErrSigkBuffer;
        }
        previousErrSigkBuffer = currentErrSigkBuffer;
        printf("ErrBufs: #%d- %x\n", loopCount, currentErrSigkBuffer);
    }

Exit:

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    deallocates the buffers for error signaller module

The function deallocates the error status buffers.

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

tEplKernel errsigk_deAllocateErrStatusBuffers(void)
{
    tEplKernel      ret;
    UINT8           loopCount;
    tErrSigkBuffer* currentErrSigkBuffer;
    tErrSigkBuffer* nextErrSigkBuffer;

    ret = kEplSuccessful;

    currentErrSigkBuffer = (tErrSigkBuffer*) instance_l.m_pReservedErrorBuffer;

    if (currentErrSigkBuffer != NULL)
    {
       ret = errsigk_deallocateStatusEntryQueue(currentErrSigkBuffer);
       EPL_FREE(currentErrSigkBuffer);
    }
    instance_l.m_pReservedErrorBuffer = NULL;

    for (loopCount = 1; loopCount < (MAX_STATUS_FRAMES - 1); loopCount++)
    {


        if (loopCount == 1)
        {
            currentErrSigkBuffer = (tErrSigkBuffer*) instance_l.m_ErrorBufferHead;
        }

        nextErrSigkBuffer = currentErrSigkBuffer->m_pNextErrorBuffer;

        if (currentErrSigkBuffer != NULL)
        {
           ret = errsigk_deallocateStatusEntryQueue(currentErrSigkBuffer);
           EPL_FREE(currentErrSigkBuffer);
        }
        currentErrSigkBuffer = nextErrSigkBuffer;
    }

    return ret;
}


//------------------------------------------------------------------------------
/**
\brief    allocates status entry queues and adds status entry

The function allocates the error status entry queues in the error signaller
module buffers and updates the status entries

\param      m_uiNumberOfHistoryEntries      pointer to the number of error status
                                            entries in the error signal buffer
\param      dstStatusQueue                  pointer to the status entry queue
                                            in the error signal buffer which has
                                            to be updated
\param      historyEntry                    pointer to the source history
                                            event structure

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

tEplKernel errsigk_addStatusEntrytoQueue(
                                            UINT8* m_uiNumberOfHistoryEntries,
                                            tEplErrHistoryEntry** dstStatusQueue,
                                            tEplErrHistoryEntry* historyEntry
                                            )
{
        tEplKernel      ret;
        tEplErrHistoryEntry* tempStatusQueue;
        UINT8 currentNumberOfHistoryEntries;

        ret = kEplSuccessful;

        currentNumberOfHistoryEntries = (*m_uiNumberOfHistoryEntries);

        if ((*dstStatusQueue) == NULL)
        {
            currentNumberOfHistoryEntries = 0;
            tempStatusQueue = (tEplErrHistoryEntry*)EPL_MALLOC((currentNumberOfHistoryEntries + 1)
                                                                                            * sizeof(tEplErrHistoryEntry));
        }
        else
        {
           //TODO: Use a generic function
            tempStatusQueue = (tEplErrHistoryEntry*)realloc((*dstStatusQueue),
                                                              (currentNumberOfHistoryEntries + 1) * sizeof(tEplErrHistoryEntry));
        }

        //TODO: Handle realloc return: if new mem is allocated or not
        if (tempStatusQueue == NULL)
        {
            ret = kEplNoResource;
            goto Exit;
        }
        (*dstStatusQueue) = tempStatusQueue;
        EPL_MEMCPY((void*)((unsigned int)(*dstStatusQueue) + currentNumberOfHistoryEntries * sizeof(tEplErrHistoryEntry)),
                                  historyEntry, sizeof(tEplErrHistoryEntry));

        currentNumberOfHistoryEntries += 1;
        (*m_uiNumberOfHistoryEntries) = currentNumberOfHistoryEntries;

Exit:
         return ret;

}


//------------------------------------------------------------------------------
/**
\brief    deallocates the buffers for status entries in error signaller module

The function deallocates the error status entry queues in the error signaller
module buffers

\param      errSigkBuffer       pointer to the error sigalling module buffer
                                of which, the queues would be deallocated

\return Returns a tEplKernel error code.

\ingroup module_errsigk
*/
//------------------------------------------------------------------------------

tEplKernel errsigk_deallocateStatusEntryQueue(tErrSigkBuffer* errSigkBuffer)
{
        tEplKernel      ret;

        ret = kEplSuccessful;

        if (errSigkBuffer->m_pErrHistoryEntry != NULL)
         {
             EPL_FREE(errSigkBuffer->m_pErrHistoryEntry);
         }
        errSigkBuffer->m_pErrHistoryEntry = NULL;
        errSigkBuffer->m_uiNumberOfHistoryEntries = 0;

         if (errSigkBuffer->m_pErrStatusEntry != NULL)
         {
             EPL_FREE(errSigkBuffer->m_pErrStatusEntry);
         }
         errSigkBuffer->m_pErrStatusEntry = NULL;
         errSigkBuffer->m_uiNumberOfStatusEntries = 0;

         return ret;

}
/// \}
