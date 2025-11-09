/*******************************************************************************
 * Copyright (C) 2021 Timon Reich
 *
 * NXT C driver code.
 *
 * The module provides support for audio output. Two modes are supported,
 * tone generation and the playback of PCM based audio samples. Both use
 * pulse density modulation to actually produce the output.
 * To produce a tone a single pdm encoded cycle is created (having the
 * requested amplitude), this single cycle is then played repeatedly to
 * generate the tone. The bit rate used to output the sample defines the
 * frequency of the tone and the number of repeats represents then length.
 * To play an encoded sample (only 8 bit PCM is currently supported),
 * each PCM sample is turned into a 256 bit pdm block, which is then output
 * (at the sample rate), to create the output. Again the amplitude of the
 * samples may be controlled.
 * The actual output of the bits is performed using the built in Synchronous
 * Serial Controller (SSC). This is capable of outputting a series of bits
 * to port at fixed intervals and is used to output the pdm audio.
 *
 * License notes see LICENSE.txt
 *******************************************************************************/

#include "drivers/nxt_sound.h"

#include "drivers/nxt_avr.h"

#include "platform/aic.h"
#include "platform/systick.h"

#include "platform/at91/at91sam7.h"

#include <string.h>

#pragma GCC diagnostic ignored "-Wdiscarded-qualifiers"

/* Buffer length must be a multiple of 8 and at most 64 (preferably as long as
 * possible) */
#define PDM_BUFFER_LENGTH 64
/* Main clock frequency */
#define OSC CLOCK_FREQUENCY
/* Size of a sample block */
#define SAMPBITS 256
#define SAMPBYTES SAMPBITS / 8
#define SAMPWORDS SAMPBITS / 32

#define MAXRATE 22050
#define MINRATE 2000
#define DEFRATE 8000

enum
{
    SOUND_MODE_NONE,
    SOUND_MODE_SILENCE,
    SOUND_MODE_TONE,
    SOUND_MODE_PCM
};

/* Numbers with 0-32 evenly spaced bits set */
const uint32 sample_pattern[33] = {
    0x00000000, 0x80000000, 0x80008000, 0x80200400, 0x80808080, 0x82081040,
    0x84208420, 0x88442210, 0x88888888, 0x91224488, 0x92489248, 0xa4924924,
    0xa4a4a4a4, 0xa94a5294, 0xaa54aa54, 0xaaaa5554, 0xaaaaaaaa, 0xd555aaaa,
    0xd5aad5aa, 0xd6b5ad6a, 0xdadadada, 0xdb6db6da, 0xedb6edb6, 0xeeddbb76,
    0xeeeeeeee, 0xf7bbddee, 0xfbdefbde, 0xfdf7efbe, 0xfefefefe, 0xffdffbfe,
    0xfffefffe, 0xfffffffe, 0xffffffff};

volatile uint8 sound_mode = SOUND_MODE_NONE;

// add volatile because GCC does over-optimization
struct
{
    // pointer to the next sample
    volatile uint8* ptr;
    // The number of samples ahead
    volatile sint32 count;
    // 0 or 1, identifies the current buffer
    volatile uint8 buf_id;
    // Double buffer
    volatile uint32 buf[2][PDM_BUFFER_LENGTH];
    // Amplification LUT
    volatile uint8 amp[257];
    // Volume val used to create the amp LUT
    volatile sint32 cur_vol;
    // Clock divisor to use to play the sample
    volatile uint32 clock_div;
    // Size of the sample
    volatile uint32 len;
} sample;

/* The following tables provide input to the wave generation code. This
 * code takes a set of points describing one half cycle of a symmetric waveform
 * and from this creates a pdm encoded version of a single full cycle of the
 * wave.
 *
 * A number of sample wave shapes have been tried, an accurate sine wave, a
 * square wave, triangular wave and a rough approximation of a sine wave.
 * Currently, the rough sine wave is used, the square wave also works well.
 * The purer shapes do not seem to work very well at frequencies below
 * about 800Hz. It seems that a combination of the sounder and the Lego
 * amplifier electronics mean that the response below this is very poor.
 * However, by using a waveform like a square wave that has a lot of harmonics
 * the ear can be fooled into hearing the lower frequencies. Using a pure wave
 * form like the sine wave results in a very low volume. This rather surprising
 * result has to some extent been validated using an audio spectrum analyzer
 * which shows virtually no fundamental below 700Hz but lots of harmonics.
 *
 * The square wave also produces a higher volume than other wave shapes.
 * Higher volumes can be achieved when using the rough sine wave by allowing
 * the volume setting to push the shape into distortion and effectively
 * becoming a square wave!
 */
const uint8 sine[] = {0xc0, 0xc8, 0xd0, 0xd8, 0xe0, 0xea, 0xf4, 0xff,
                      0xff, 0xf0, 0xe5, 0xdc, 0xd4, 0xcc, 0xc4, 0xbc};

// Time required to generate the tone and volume lookup table...
#define TONE_OVERHEAD 1

/* To minimise the number of cracks and pops when playing a series of tones
 * and/or samples we output a short period of "silence" at the end of each
 * sample. A small click is generated when turning off the sound system so
 * by filling small gaps with explicit silence we can avoid this additional
 * noise.
 */

const uint32 silence[16] = {0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa,
                            0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa,
                            0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa,
                            0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa, 0xaaaaaaaa};
#define SILENCE_CLK (OSC / (16 * 32 * 2) + 250 / 2) / 250
#define SILENCE_CNT 50

/* We use a volume law that approximates a log function. The values in the
 * table below have been hand tuned to try and provide a smooth change in
 * the loudness of both tones and samples. */
const uint8 logvol[] = {0, 8, 24, 40, 56, 80, 104, 128, 162, 196, 255, 255};

static void sound_interrupt_enable(uint32 typ)
{
    // Enable interrupt notification of either the end of the next buffer
    // or the end of all output. Having both enabled does not seem to work
    // the end notification seems to get lost.
    *AT91C_SSC_IDR = AT91C_SSC_TXBUFE;
    *AT91C_SSC_IDR = AT91C_SSC_ENDTX;
    *AT91C_SSC_IDR = AT91C_SSC_TXEMPTY;
    *AT91C_SSC_IER = typ;
}

static void sound_interrupt_disable(void)
{
    *AT91C_SSC_IDR = AT91C_SSC_TXBUFE;
    *AT91C_SSC_IDR = AT91C_SSC_ENDTX;
    *AT91C_SSC_IDR = AT91C_SSC_TXEMPTY;
}

void nxt_sound_init(void)
{
    // Initialise the hardware. We make use of the SSC module.
    sound_interrupt_disable();

    nxt_sound_disable();

    *AT91C_PMC_PCER = (1 << AT91C_ID_SSC);

    *AT91C_PIOA_ODR = AT91C_PA17_TD;
    *AT91C_PIOA_OWDR = AT91C_PA17_TD;
    *AT91C_PIOA_MDDR = AT91C_PA17_TD;
    *AT91C_PIOA_PPUDR = AT91C_PA17_TD;
    *AT91C_PIOA_IFDR = AT91C_PA17_TD;
    *AT91C_PIOA_CODR = AT91C_PA17_TD;
    *AT91C_PIOA_IDR = AT91C_PA17_TD;

    *AT91C_SSC_CR = AT91C_SSC_SWRST;
    *AT91C_SSC_TCMR =
        AT91C_SSC_CKS_DIV + AT91C_SSC_CKO_CONTINOUS + AT91C_SSC_START_CONTINOUS;
    *AT91C_SSC_TFMR = 31 + (7 << 8) + AT91C_SSC_MSBF; // 8 32-bit words
    *AT91C_SSC_CR = AT91C_SSC_TXEN;

    aic_mask_on(AT91C_ID_SSC);
    aic_clear(AT91C_ID_SSC);
    aic_set_vector(AT91C_ID_SSC,
                   AT91C_AIC_PRIOR_LOWEST |
                       AT91C_AIC_SRCTYPE_INT_EDGE_TRIGGERED,
                   (uint32)nxt_sound_isr_handler); /*PG*/
    sample.buf_id = 0;
    sample.cur_vol = -1;
}

static void create_tone(const uint8* lookup, sint32 lookup_len, uint32* pattern,
                        sint32 len)
{
    // Fill the supplied buffer with len longs representing a pdm encoded
    // wave. We use a pre-generated lookup table for the wave shape.
    // The shape will be symmetric. The original code used interpolation as part
    // of the lookup but this was too slow.
    sint32 num_samples = len * 32 / 2;
    sint32 step = num_samples / lookup_len;
    sint32 word = 0;

    uint32 bit = 0x80000000;
    uint32 bit2 = 0x00000001;
    sint32 i = num_samples / step;
    sint32 error = 0;
    sint32 error2 = 0;
    sint32 out = 0;
    uint32 bits = 0;
    uint32 bits2 = 0;
    sint32 entry = 0;

    while (i-- > 0)
    {
        int res = lookup[entry++];
        res = sample.amp[res] - 128;
        int j = step;
        while (j-- > 0)
        {
            // Perform pdm conversion
            error = res - out + error;
            error2 = error - out + error2;
            if (error2 > 0)
            {
                out = 127;
                bits |= bit;
            }
            else
            {
                out = -127;
                bits2 |= bit2;
            }
            bit2 <<= 1;
            bit >>= 1;
            if (bit == 0)
            {
                bit = 0x80000000;
                bit2 = 0x00000001;
                pattern[word++] = bits;
                pattern[len - word] = bits2;
                bits2 = bits = 0;
            }
        }
    }
}

static void set_vol(uint8 vol)
{
    // Create the amplification control lookup table. We use a logarithmic
    // volume system mapped into a range of 0 to 120. 0 is muted, 100 is
    // full volume, 120 is driving into overload.
    sint32 i;
    sint32 output;

    // Get into range and use log conversion
    if (vol < 0)
    {
        vol = 0;
    }
    if (vol > MAXVOL)
    {
        vol = MAXVOL;
    }

    // Do we need to create a new LUT?
    if (sample.cur_vol == vol)
    {
        return;
    }
    output = logvol[vol / 10];
    output = output + ((logvol[vol / 10 + 1] - output) * (vol % 10)) / 10;

    // Create the symmetric lookup table
    for (i = 0; i <= 128; i++)
    {
        sint32 a = (i * output) / 128;
        sint32 b = a;
        if (a > 127)
        {
            a = 127;
            b = 128;
        }
        sample.amp[128 - i] = 128 - b;
        sample.amp[i + 128] = a + 128;
    }
    sample.cur_vol = vol;
}

void nxt_sound_freq_vol(uint32 freq, uint32 ms, uint8 vol)
{
    // Set things up ready to go, note we avoid using anything that may
    // be used by the interrupt routine because ints may still be enabled
    // at this point
    sint32 len;

    // we use longer samples for lower frequencies
    if (freq > 1000)
    {
        len = 16;
    }
    else if (freq < 500)
    {
        len = 64;
    }
    else
    {
        len = 32;
    }
    sound_mode = SOUND_MODE_TONE;

    // Update the volume lookup table if we need to
    set_vol(vol);
    sint32 buf = sample.buf_id ^ 1;
    create_tone(sine, sizeof(sine), sample.buf[buf], len);

    // The note generation takes approx 1ms, to ensure that we do not get gaps
    // when playing a series of tones we extend the requested period to cover
    // this 1ms cost.
    ms += TONE_OVERHEAD;

    // Turn of ints while we update shared values
    sound_interrupt_disable();

    /* Generate the pdm wave of the correct amplitude */
    sample.clock_div = (OSC / (len * 32 * 2) + freq / 2) / freq;

    // Calculate actual frequency and use this for length calc
    freq = (OSC / (2 * sample.clock_div)) / (len * 32);
    if (ms <= TONE_OVERHEAD)
    {
        sample.count = 0;
    }
    else
    {
        sample.count = (freq * ms + 1000 - 1) / 1000;
    }
    sample.len = len;
    sample.ptr = (uint8*)sample.buf[buf];
    sample.buf_id = buf;

    *AT91C_SSC_PTCR = AT91C_PDC_TXTEN;
    sound_mode = SOUND_MODE_TONE;

    sound_interrupt_enable(AT91C_SSC_TXBUFE);
}

void nxt_sound_freq(uint32 freq, uint32 ms)
{
    nxt_sound_freq_vol(freq, ms, 70);
}

void nxt_sound_enable(void)
{
    *AT91C_PIOA_PDR = AT91C_PA17_TD;
}

void nxt_sound_disable(void)
{
    *AT91C_PIOA_PER = AT91C_PA17_TD;
}

void nxt_sound_fill_sample_buffer(void)
{
    sample.buf_id ^= 1;
    uint32* sbuf = sample.buf[sample.buf_id];
    uint8 i;

    /* Each 8-bit sample is turned into 8 32-bit numbers, i.e. 256 bits
     * altogether */
    for (i = 0; i < PDM_BUFFER_LENGTH >> 3; i++)
    {
        uint8 smp = (sample.count > 0 ? sample.amp[*sample.ptr] : 128);
        uint8 msk = "\x00\x10\x22\x4a\x55\x6d\x77\x7f"[smp & 7];
        uint8 s3 = smp >> 3;

        *sbuf++ = sample_pattern[s3 + (msk & 1)];
        msk >>= 1;
        *sbuf++ = sample_pattern[s3 + (msk & 1)];
        msk >>= 1;
        *sbuf++ = sample_pattern[s3 + (msk & 1)];
        msk >>= 1;
        *sbuf++ = sample_pattern[s3 + (msk & 1)];
        msk >>= 1;
        *sbuf++ = sample_pattern[s3 + (msk & 1)];
        msk >>= 1;
        *sbuf++ = sample_pattern[s3 + (msk & 1)];
        msk >>= 1;
        *sbuf++ = sample_pattern[s3 + (msk & 1)];
        *sbuf++ = sample_pattern[s3];
        /*
         //An alternative that doesn't need a sample_pattern array:
         U32 msb = 0xffffffff << (32 - (smp >> 3));
         *sbuf++ = msb | ((msk & 1) ? msb >> 1 : 0); msk >>= 1;
         *sbuf++ = msb | ((msk & 1) ? msb >> 1 : 0); msk >>= 1;
         *sbuf++ = msb | ((msk & 1) ? msb >> 1 : 0); msk >>= 1;
         *sbuf++ = msb | ((msk & 1) ? msb >> 1 : 0); msk >>= 1;
         *sbuf++ = msb | ((msk & 1) ? msb >> 1 : 0); msk >>= 1;
         *sbuf++ = msb | ((msk & 1) ? msb >> 1 : 0); msk >>= 1;
         *sbuf++ = msb | ((msk & 1) ? msb >> 1 : 0);
         *sbuf++ = msb;
         */
        sample.ptr++;
        sample.count--;
    }
}

void nxt_sound_play_sample(uint8* data, uint32 length, uint32 freq, uint8 vol)
{
    if (data == (uint8*)0 || length == 0)
    {
        return;
    }

    /* Calculate the clock divisor based upon the recorded sample frequency. */
    if (freq == 0)
    {
        freq = DEFRATE;
    }
    if (freq > MAXRATE)
    {
        freq = MAXRATE;
    }
    if (freq < MINRATE)
    {
        freq = MINRATE;
    }
    uint32 cdiv = (OSC / (2 * SAMPBITS) + freq / 2) / freq;
    set_vol(vol);

    /* Turn off ints while we update shared values. */
    sound_interrupt_disable();
    sound_mode = SOUND_MODE_PCM;
    sample.count = length;
    sample.ptr = data;
    sample.len = PDM_BUFFER_LENGTH;
    sample.clock_div = cdiv;

    /* Re-enable and wait for the current sample to complete. */
    sound_interrupt_enable(AT91C_SSC_TXBUFE);
    *AT91C_SSC_PTCR = AT91C_PDC_TXTEN;
}

sint32 nxt_sound_get_time()
{
    // Return the amount of time still to play for the current tone/sample
    if (sound_mode > SOUND_MODE_SILENCE)
    {
        // long long int is needed to avoid overflow (this is a bug in leJOS
        // original code)
        sint32 time_ms = (int)(((long long int)sample.count * 1000 * sample.len * 32) /
                       (OSC / (2 * sample.clock_div)));
        // remove the extra time we added
        if (sound_mode == SOUND_MODE_TONE && time_ms > 0)
        {
            time_ms -= TONE_OVERHEAD;
        }
        return time_ms;
    }

    return 0;
}

void nxt_sound_isr_handler()
{
    if (sample.count > 0)
    {
        // Refill the buffer, and adjust any clocks
        *AT91C_SSC_CMR = sample.clock_div;
        nxt_sound_enable();

        if (*AT91C_SSC_TCR == 0)
        {
            if (sound_mode == SOUND_MODE_PCM)
            {
                nxt_sound_fill_sample_buffer();
                *AT91C_SSC_TPR = (unsigned int)sample.buf[sample.buf_id];
            }
            else
            {
                *AT91C_SSC_TPR = (unsigned int)sample.ptr;
            }
            *AT91C_SSC_TCR = sample.len;
            sample.count--;
        }
        if (sound_mode == SOUND_MODE_PCM)
        {
            nxt_sound_fill_sample_buffer();
            *AT91C_SSC_TNPR = (unsigned int)sample.buf[sample.buf_id];
        }
        else
        {
            *AT91C_SSC_TNPR = (unsigned int)sample.ptr;
        }

        *AT91C_SSC_TNCR = sample.len;
        sample.count--;

        // If this is the last sample wait for it to complete, otherwise wait
        // to switch buffers
        sound_interrupt_enable(sample.count <= 0
                                   ? (sound_mode == SOUND_MODE_SILENCE
                                          ? AT91C_SSC_TXEMPTY
                                          : AT91C_SSC_TXBUFE)
                                   : AT91C_SSC_ENDTX);
    }
    else
    {
        sound_mode = SOUND_MODE_NONE;
        nxt_sound_disable();
        sound_interrupt_disable();
    }
}
