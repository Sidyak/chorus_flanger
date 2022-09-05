/*
Author: Kim Radmacher

Date: 05.09.2022

Description:
  Chorus Flanger effect
  Project is deployed on x86
*/

#include <math.h>
#include <stdio.h>
#include <stdint.h>
#include <string.h>
#include <time.h>
#include <stdlib.h>

//#define DEBUG

#if defined(_MSC_VER)
#include <getopt.h>
#else
#include <unistd.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif
#include "wavreader.h"
#include "wavwriter.h"
#ifdef __cplusplus
}
#endif

#define BUFFER_SIZE (16384)
#define MAX_DELAY_TIME 2

void cleanup(void);

// set the frequency of the oscillators
int16_t gInputBuffer[BUFFER_SIZE];
int gInputBufferPointer = 0;
int16_t gOutputBuffer[BUFFER_SIZE];
int32_t inL, inR, outL, outR;
int gOutputBufferWritePointer = 0;
int gOutputBufferReadPointer = 0;
int gSampleCount = 0;

// algorithm vars
float* circBuff;
int idxCircBuff;
int circBuffLen;

float feedbackParameter;
float feedback = 0.f;
float dryWetParameter = 0;
float LFOPhase = 0;
float depthParameter;
float rateParameter;

int setup(float Fs)
{
    gOutputBufferWritePointer = 0;
    gOutputBufferReadPointer = 0;

    // Parameter set
    feedbackParameter = 0.75f; // 0...0,98
    dryWetParameter = 0.5f; // 0...1
    depthParameter = 0.5f; // 0...1
    rateParameter = 5.f; // 0,1...20

    circBuffLen = Fs * MAX_DELAY_TIME;
    circBuff = (float*) malloc (circBuffLen * sizeof (float));
    if(circBuff == NULL)
    {
        return 0;
    }
  
    idxCircBuff = 0;
    
    memset(gOutputBuffer, 0, BUFFER_SIZE * sizeof(int16_t));
    memset(gInputBuffer, 0, BUFFER_SIZE * sizeof(int16_t));
    
    return 1;

}

float linearInterpolation(float curr, float next, float diff)
{
    return (1.f - diff) * curr + diff * next;
}

int getDelayToRead(float lfo, float Fs)
{
    float delayTimeSmp = Fs * lfo;
    
    int delayToRead = idxCircBuff - delayTimeSmp;

    if (delayToRead < 0)
        delayToRead += circBuffLen;

    return delayToRead;
}

float translate(float value, float leftMin, float leftMax, float rightMin, float rightMax)
{
    float leftSpan = leftMax - leftMin;
    float rightSpan = rightMax - rightMin;

    // Convert the left range into a 0-1 range
    float valueScaled = (value - leftMin) / leftSpan;

    // Convert the 0-1 range into a value in the right range
    return rightMin + (valueScaled * rightSpan);
}

// Perform processing for chorus effect
float processChorus(float lfo)
{
    // Map LFO values to ms for chorus
    float lfoOutMapped = translate(lfo, -1.f, 1.f, 0.005f, 0.03f);
    
    return lfoOutMapped;
}

// Perform processing for flanger effect
float processFlanger(float lfo)
{
    // Map LFO values to ms for chorus
    float lfoOutMapped = translate(lfo, -1.f, 1.f, 0.001f, 0.005f);
        
    return lfoOutMapped;
}

// This function handles the FFT based chorus_flanger
void process_chorus_flanger(int16_t *inBuffer, int inWritePointer, int16_t *outBuffer, int outWritePointer, float Fs, int doChorus)
{
    circBuff[idxCircBuff++] = inBuffer[inWritePointer] + feedback;
    
    if (idxCircBuff >= circBuffLen)
        idxCircBuff = 0;

    float lfoOut = sinf(2.f * M_PI * LFOPhase);

#if 0 // if stereo is processed, use different phases for both channel
    float LFOPhase_ch2 = LFOPhase + phaseOff;
    lfoOut_ch2 = sinf(2 * M_PI * LFOPhase_ch2);
#endif

    LFOPhase += rateParameter / Fs;

    if (LFOPhase >= 1)
        LFOPhase -= 1;

    // Depth 
    lfoOut *= depthParameter;

    float lfoMapped = (doChorus == 1) ? processChorus(lfoOut) : processFlanger(lfoOut);
    int delayToRead = getDelayToRead(lfoMapped, Fs);
    
    int current = delayToRead;
    int next = current + 1;
    float diff = delayToRead - current;

    if (next >= circBuffLen)
        next -= circBuffLen;

    // Linear interpolation 
    float delayed_sample = linearInterpolation(circBuff[current], circBuff[next], diff);

    // Manage feedback
    feedback = delayed_sample * feedbackParameter;

    outBuffer[outWritePointer] = inBuffer[inWritePointer] * (1 - dryWetParameter) + delayed_sample * dryWetParameter;
}

void usage(const char* name)
{
    fprintf(stderr, "%s in.wav out.wav\n", name);
}

int main(int argc, char *argv[])
{
    const char *infile, *outfile;
    FILE *out;
    void *wavIn;
    void *wavOut;
    int format, sample_rate, channels, bits_per_sample;
    uint32_t data_length;
    int input_size;
    uint8_t* input_buf;
    int16_t* convert_buf;
    int doChorus = 0;
    
    if (argc - optind < 2)
    {
        fprintf(stderr, "Error: not enough parameter provided\n");
        usage(argv[0]);
        return 1;
    }
    
    infile = argv[optind];
    outfile = argv[optind + 1];

    if (argc - optind > 2)
    {
        doChorus = atoi(argv[optind + 2]);
    }

    wavIn = wav_read_open(infile);
    if (!wavIn)
    {
        fprintf(stderr, "Unable to open wav file %s\n", infile);
        return 1;
    }
    if (!wav_get_header(wavIn, &format, &channels, &sample_rate, &bits_per_sample, &data_length))
    {
        fprintf(stderr, "Bad wav file %s\n", infile);
        return 1;
    }
    if (format != 1)
    {
        fprintf(stderr, "Unsupported WAV format %d\n", format);
        return 1;
    }

    if(!setup(sample_rate))
    {
        fprintf(stderr, "setup failed\n");
    }

    wavOut = wav_write_open(outfile, sample_rate, bits_per_sample, channels);

    if (!wavOut)
    {
        fprintf(stderr, "Unable to open wav file for writing %s\n", infile);
        return 1;
    }

    input_size = data_length;
    input_buf = (uint8_t*) malloc(input_size);
    convert_buf = (int16_t*) malloc(input_size);

    if (input_buf == NULL || convert_buf == NULL)
    {
        fprintf(stderr, "Unable to allocate memory for buffer\n");
        return 1;
    }

    int read = wav_read_data(wavIn, input_buf, input_size);

    printf("using doChorus = %d (1 for Chorus, 0 for Flanger)\n", doChorus);
    
    printf("data_length = %d\tread = %d\tinput_size = %d \n", data_length, read, input_size);
    printf("sample_rate = %d\tbits_per_sample = %d\tchannels = %d \n", sample_rate, bits_per_sample, channels);

    int numSamples = read/2;
    for(unsigned int n = 0; n < numSamples; n++)
    {
        const uint8_t* in = &input_buf[2*n];
        convert_buf[n] = in[0] | (in[1] << 8);
    }
    
    // iterate over the audio frames and create three oscillators, seperated in phase by PI/2
    for(unsigned int n = 0; n < numSamples; n+=channels)
    {
        // Read audio inputs
        if(channels == 1)
        {
            inL = (int32_t)convert_buf[n];
            inR = inL;
        }
        else if(channels == 2)
        {
            // interleaved left right channel
            inL = (int32_t)convert_buf[n];
            inR = (int32_t)convert_buf[n+1];
        }
        else
        {
            fprintf(stderr, "channel = %d\n", channels);
            return -1;
        }

        gInputBuffer[0] = (int16_t)((inR+inL)/2);

        process_chorus_flanger(gInputBuffer, gInputBufferPointer, gOutputBuffer, gOutputBufferWritePointer, sample_rate, doChorus);

        outL = gOutputBuffer[0];
        outR = outL;

        int16_t oL = (int16_t)outL;
        int16_t oR = (int16_t)outR;
        wav_write_data(wavOut, (unsigned char*)&oL, 2);
        if(channels > 1)
        {
            wav_write_data(wavOut, (unsigned char*)&oR, 2);
        }
    }    

    free(convert_buf);
    free(input_buf);
    
    cleanup();

    wav_write_close(wavOut);
    wav_read_close(wavIn);

    return 0;
}
// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup()
{

    free(circBuff);
}
