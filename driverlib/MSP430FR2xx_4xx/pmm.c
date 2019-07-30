/* --COPYRIGHT--,BSD
 * Copyright (c) 2017, Texas Instruments Incorporated
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * *  Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *
 * *  Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * *  Neither the name of Texas Instruments Incorporated nor the names of
 *    its contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR
 * PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS;
 * OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 * WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR
 * OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * --/COPYRIGHT--*/
//*****************************************************************************
//
// pmm.c - Driver for the pmm Module.
//
//*****************************************************************************
//*****************************************************************************
//
//! \addtogroup pmm_api pmm
//! @{
//
//*****************************************************************************
#include "inc/hw_memmap.h"

#ifdef __MSP430_HAS_PMM_FRAM__
#include "pmm.h"

#include <assert.h>

void PMM_enableSVSH(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0_L) |= SVSHE;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_disableSVSH(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0_L) &= ~SVSHE;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_turnOnRegulator(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) &= ~PMMREGOFF;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_turnOffRegulator(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) |= PMMREGOFF;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_trigPOR(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) |= PMMSWPOR;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_trigBOR(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG8(PMM_BASE + OFS_PMMCTL0) |= PMMSWBOR;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_clearInterrupt(uint16_t mask)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;
    HWREG16(PMM_BASE + OFS_PMMIFG) &= ~mask;
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

uint16_t PMM_getInterruptStatus(uint16_t mask)
{
    return ((HWREG16(PMM_BASE + OFS_PMMIFG)) & mask);
}

void PMM_unlockLPM5(void)
{
    //Direct register access to avoid compiler warning - #10420-D  
    //For FRAM devices, at start up, the GPO power-on default 
    //high-impedance mode needs to be disabled to activate previously 
    //configured port settings. This can be done by clearing the LOCKLPM5
    //bit in PM5CTL0 register

    PM5CTL0 &= ~LOCKLPM5;
}

uint16_t PMM_getBandgapMode(void)
{
    return (HWREG16((PMM_BASE) + OFS_PMMCTL2) & BGMODE);
}

uint16_t PMM_isBandgapActive(void)
{
    return (HWREG16((PMM_BASE) + OFS_PMMCTL2) & REFBGACT);
}

uint16_t PMM_isRefGenActive(void)
{
    return (HWREG16((PMM_BASE) + OFS_PMMCTL2) & REFGENACT);
}

uint16_t PMM_getBufferedBandgapVoltageStatus(void)
{
    return (HWREG16((PMM_BASE) + OFS_PMMCTL2) & REFBGRDY);
}

uint16_t PMM_getVariableReferenceVoltageStatus(void)
{
    return (HWREG16((PMM_BASE) + OFS_PMMCTL2) & REFGENRDY);
}

void PMM_disableTempSensor(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;

    HWREG8(PMM_BASE + OFS_PMMCTL2) &= ~TSENSOREN;

    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_enableTempSensor(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;

    HWREG8(PMM_BASE + OFS_PMMCTL2) |= TSENSOREN;

    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_disableExternalReference(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;

    HWREG8(PMM_BASE + OFS_PMMCTL2) &= ~EXTREFEN;

    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_enableExternalReference(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;

    HWREG8(PMM_BASE + OFS_PMMCTL2) |= EXTREFEN;

    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_disableInternalReference(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;

    HWREG8(PMM_BASE + OFS_PMMCTL2) &= ~INTREFEN;

    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_enableInternalReference(void)
{
    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = PMMPW_H;

    HWREG8(PMM_BASE + OFS_PMMCTL2) |= INTREFEN;

    HWREG8(PMM_BASE + OFS_PMMCTL0_H) = 0x00;
}

void PMM_selectVoltageReference(uint16_t refV)
{
#ifdef REFVSEL
    HWREG16(PMM_BASE + OFS_PMMCTL2) &= ~REFVSEL;
    HWREG16(PMM_BASE + OFS_PMMCTL2) |= refV;
#endif
}

void PMM_setPowerMode(uint8_t mode)
{
#ifdef PWRMODE
    HWREG16(PMM_BASE + OFS_PMMCTL2) &= ~PWRMODE;
    HWREG16(PMM_BASE + OFS_PMMCTL2) |= (mode << 14);
#endif
}

#endif
//*****************************************************************************
//
//! Close the doxygen group for pmm_api
//! @}
//
//*****************************************************************************
