/*
 * Local standalone override for the archive experiment slave.
 * Reuse the continuously armed repo slave implementation so the
 * archive experiment follows the same callback-driven flow, but seed
 * the default transmit buffer with the roundtrip pattern expected by
 * the local experiment.
 */

#include <stddef.h>

void *experiment_memset(void *ptr, int value, size_t num);

#define APP_ENABLE_SEMIHOST 1
#define EXPERIMENT_SLAVE_ECHO_COUNT_PLUS_ONE 1
#define memset experiment_memset

#include "../../../src/slave/i3c_interrupt_b2b_transfer_slave.c"

#undef memset

void *experiment_memset(void *ptr, int value, size_t num)
{
	void *result = __builtin_memset(ptr, value, num);

	if ((ptr == g_slave_txBuff) && (value == 0) && (num == sizeof(g_slave_txBuff)))
	{
		for (size_t index = 0U; index < num; index++)
		{
			g_slave_txBuff[index] = (uint8_t)(index + 1U);
		}
	}

	return result;
}