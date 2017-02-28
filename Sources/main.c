/* Copyright 2017, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright 2017, Diego Alegrechi
 * Copyright 2017, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

// SDK Included Files
#include "fsl_sim_hal.h"
#include "fsl_mcg_hal.h"
#include "fsl_tpm_hal.h"

// Project Included Files
#include "board.h"

/*==================[macros and definitions]=================================*/

/*==================[internal data declaration]==============================*/
static int32_t cuenta = 0;

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/
void TPM_Init(void)
{
	uint32_t channel = 0;

	// Habilito clock del periferico
	SIM_HAL_EnableClock(SIM, kSimClockGateTpm0);

	// Detengo cuenta del Timer
	TPM_HAL_SetClockMode(TPM0, kTpmClockSourceNoneClk);

	// Reseteo el timer
	TPM_HAL_Reset(TPM0, 0);

	// Clock prescaler = 1
	TPM_HAL_SetClockDiv(TPM0, kTpmDividedBy1);

	// Borro bandera de overflow
	TPM_HAL_ClearTimerOverflowFlag(TPM0);

	// Configuro valor MOD a TPM_MOD_MOD_MASK (0xFFFF)
	TPM_HAL_SetMod(TPM0, TPM_MOD_MOD_MASK);

	// Uso TPM solo como timer -- Deshabilito todos los canales
	for (channel = 0; channel < FSL_FEATURE_TPM_CHANNEL_COUNT ; channel++)
	{
		TPM_HAL_DisableChn(TPM0, channel);
	}

	// Cuenta ascendente
	TPM_HAL_SetCpwms(TPM0, 0);

	// Activo interupcion ante bandera de OverFlow
	TPM_HAL_EnableTimerOverflowInt(TPM0);

	// Elijo la fuente de clock para el TPM (3.8.1.1 reference manual)
	CLOCK_HAL_SetTpmSrc(SIM, 0, kClockTpmSrcMcgIrClk);

	// Habilito el Timer -- Fuente de clock interna
	TPM_HAL_SetClockMode(TPM0, kTpmClockSourceModuleClk);
}

/*==================[external functions definition]==========================*/

int main(void)
{
	// Se habilita IRC y se elige Frec de 32KHz
	CLOCK_HAL_SetInternalRefClkEnableCmd(MCG, true);
	CLOCK_HAL_SetInternalRefClkMode(MCG, kMcgIrcSlow);

	// Se inicializan funciones de la placa
	board_init();

	ledRojo_on();

    // Se inicializa el timer
	TPM_Init();

    // Se habilitan interrupciones de TPM
	NVIC_ClearPendingIRQ(TPM0_IRQn);
    NVIC_EnableIRQ(TPM0_IRQn);

    while(1)
    {

    }
}

///////////////////////////////////////////////////////////////////////////////
// TPM0 IRQ
///////////////////////////////////////////////////////////////////////////////

void TPM0_IRQHandler(void)
{
	if (cuenta < 9)
	{
		cuenta ++;
	}
	else
	{
		cuenta = 0;
		ledRojo_toggle();

	}

	ledVerde_toggle();

	// Borro bandera de overflow
	TPM_HAL_ClearTimerOverflowFlag(TPM0);
}

/*==================[end of file]============================================*/
