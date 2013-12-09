/**
********************************************************************************
\file   errsigk.h

\brief  External interface of the error handler kernel module

This header provides the external interface of the error handler kernel module

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

#ifndef _INC_errsigk_H_
#define _INC_errsigk_H_

//------------------------------------------------------------------------------
// includes
//------------------------------------------------------------------------------

//------------------------------------------------------------------------------
// const defines
//------------------------------------------------------------------------------


//------------------------------------------------------------------------------
// typedef
//------------------------------------------------------------------------------

typedef enum
{
    kOwnerDll               = 0,    ///< Dll owns the current buffer
    kOwnerErrSigk,                  ///< Error Signaling module owns the current buffer
    kOwnerReserved                  ///< Current frame's ownership is non-changeable
}tErrBufOwner;



/**
\brief  status entry buffer

The structure defines the status entry fields of error signaling module and status response frame
*/
typedef struct ErrSigkBuffer
{
    BOOL                    m_fDataValid;                       ///< Data with in the buffer is valid or invalid
    tErrBufOwner            m_uiOwner;                          ///< Owner of the buffer
    UINT8                   m_uiNumberOfHistoryEntries;         ///< Number of Error Entries in the current buffer
    struct ErrSigkBuffer*   m_pNextErrorBuffer;                 ///< pointer to Next StatusEntry Buffer structure
    QWORD                   m_qwStaticError;                    ///< static error bit field
    tEplErrHistoryEntry*    m_pErrHistoryEntry;                 ///< History entry
}PACK_STRUCT tErrSigkBuffer;

//------------------------------------------------------------------------------
// function prototypes
//------------------------------------------------------------------------------

#ifdef __cplusplus
extern "C" {
#endif

// init function
tEplKernel errsigk_init(void);
//reset status
tEplKernel errsigk_reset(void);
//TODO Remove functions un necessary for MN from MN codespace
// allocate the buffers from dll
tEplKernel errsigk_createErrStatusBuffers(tErrSigkBuffer** dllErrStatusBuffer);
//deallocate buffers from dll
tEplKernel errsigk_cleanErrStatusBuffers(tErrSigkBuffer** dllErrStatusBuffer);

//Set Error Status
tEplKernel errsigk_addStatusEntry(tEplErrHistoryEntry*  historyEntry);

//Get Error Status
tEplKernel errsigk_getErrStatusBuffer(tErrSigkBuffer** dllErrStatusBuffer, BOOL* fErrFlag);

// delete instance
tEplKernel errsigk_exit(void);
/*

// processes error events
tEplKernel errsigk_process(tEplEvent* pEvent_p);

// posts error events
tEplKernel errsigk_postError(tErrSigkEvent* pDllEvent_p);

// cycle finished (decrement threshold counters)
tEplKernel errsigk_decrementCounters(BOOL fMN_p) SECTION_ERRHNDK_DECRCNTERS;

// reset error flag for the specified CN
tEplKernel errsigk_resetCnError(UINT nodeId_p);
*/

#ifdef __cplusplus
}
#endif

#endif /* _INC_errsigk_H_ */
