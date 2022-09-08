/*

Chorus Flanger effect

Project is deployed on 
 ____  _____ _        _    
| __ )| ____| |      / \   
|  _ \|  _| | |     / _ \  
| |_) | |___| |___ / ___ \ 
|____/|_____|_____/_/   \_\

The platform for ultra-low latency audio and sensor processing

http://bela.io

The Bela software is distributed under the GNU Lesser General Public License
(LGPL 3.0), available here: https://www.gnu.org/licenses/lgpl-3.0.txt
*/

#include <Bela.h>
#include <libraries/ne10/NE10.h>
//#include <libraries/Midi/Midi.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#define BUFFER_SIZE (16384)
#define MAX_DELAY_TIME 2

// set the frequency of the oscillators
float gInputBuffer[BUFFER_SIZE];
int gInputBufferPointer = 0;
float gOutputBuffer[BUFFER_SIZE];
float inL, inR, outL, outR;
int gOutputBufferWritePointer = 0;
int gOutputBufferReadPointer = 0;
int gSampleCount = 0;

// algorithm vars
float* circBuff;
int idxCircBuff;
int circBuffLen;

float feedbackParameter;
float modFeedbackParameter = 0.f;
float feedback = 0.f;
float dryWetParameter = 0;
float modDryWetParameter = 0.f;
float depthParameter;
float modDepthParameter = 0.f;
float rateParameter;
float modRateParameter = 0.f;
float LFOPhase = 0;

void process_chorus_flanger_background(void *);

// Set the analog channels to read from
const int gAnalogIn = 0;
int gAudioFramesPerAnalogFrame = 0;

int gReadPtr = 0;        // Position of last read sample from file
AuxiliaryTask gFFTTask;
int gFFTInputBufferPointer = 0;
int gFFTOutputBufferPointer = 0;
float gFs = 0.f;

// cpu cycle read
static inline uint32_t ccnt_read (void)
{
  uint32_t cc = 0;
  __asm__ volatile ("mrc p15, 0, %0, c9, c13, 0":"=r" (cc));
  return cc;
}

// instantiate the scope
//Scope scope;

// userData holds an opaque pointer to a data structure that was passed
// in from the call to initAudio().
//
// Return true on success; returning false halts the program.
bool setup(BelaContext* context, void* userData)
{
    printf("go setup\n");
    printf("context->audioFrames = %d\n", context->audioFrames);
    printf("context->audioSampleRate = %f\n", context->audioSampleRate);
    printf("context->audioInChannels = %d\n", context->audioInChannels);
    printf("context->audioOutChannels = %d\n", context->audioOutChannels);
    // Check that we have the same number of inputs and outputs.
    if(context->audioInChannels != context->audioOutChannels ||
            context->analogInChannels != context-> analogOutChannels){
        printf("Error: for this project, you need the same number of input and output channels.\n");
        return false;
    }

    gOutputBufferWritePointer = 0;
    gOutputBufferReadPointer = 0;

    // Parameter set
    feedbackParameter = 0.1f; // 0...0,98
    dryWetParameter = 0.5f; // 0...1
    depthParameter = 0.85f; // 0...1
    rateParameter = 5.f; // 0,1...20

    gFs = context->audioSampleRate;
    
    circBuffLen = (int)(gFs * MAX_DELAY_TIME);
    circBuff = (float*) malloc (circBuffLen * sizeof (float));
    if(circBuff == NULL)
    {
        return 0;
    }
  
    idxCircBuff = 0;
    
    memset(gOutputBuffer, 0, BUFFER_SIZE * sizeof(int16_t));
    memset(gInputBuffer, 0, BUFFER_SIZE * sizeof(int16_t));

    // Initialise auxiliary tasks
    if((gFFTTask = Bela_createAuxiliaryTask(&process_chorus_flanger_background, 90, "flanger-calculation")) == 0)
        return false;

    // Check if analog channels are enabled
    if(context->analogFrames == 0 || context->analogFrames > context->audioFrames) {
        rt_printf("Error: this example needs analog enabled, with 4 or 8 channels\n");
        return false;
    }
    // Useful calculations
    if(context->analogFrames)
        gAudioFramesPerAnalogFrame = context->audioFrames / context->analogFrames;
        
    printf("bye setup\n");
    return true;
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

// Processing
void process_chorus_flanger(float *inBuffer, int inWritePointer, float *outBuffer, int outWritePointer, float Fs)
{
    uint32_t t0 = ccnt_read();
    uint32_t t1 = t0;//ccnt_read();       
    //rt_printf("%u\n", t1-t0);

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

    float lfoMapped = processChorus(lfoOut);//(doChorus == 1) ? processChorus(lfoOut) : processFlanger(lfoOut);
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

    t1 = ccnt_read();
    rt_printf("\r\r\r[%u cycles] ", t1-t0);
    rt_printf("rate = %f | ", rateParameter);
    rt_printf("dryWet = %f | ", dryWetParameter);
    rt_printf("feedback = %f | ", feedbackParameter);
    rt_printf("depth = %f ", depthParameter);
}

// Function to process the FFT in a thread at lower priority
void process_chorus_flanger_background(void*) {
    process_chorus_flanger(gInputBuffer, gFFTInputBufferPointer, gOutputBuffer, gFFTOutputBufferPointer, gFs);
}

int modParamInt(int param, int* pParam, int range)
{
    if(param < 0)
    {
        if(param <= *pParam-range)
        {
            *pParam -= range;
        }
        else if(param >= *pParam+range)
        {
            *pParam += range;
        }
    }
    else
    {
        if(param >= *pParam+range)
        {
            *pParam += range;
        }
        else if(param <= *pParam-range)
        {
            *pParam -= range;
        }
    }

    param = *pParam;
    return param;
}

float modParamFloat(float param, float* pParam, float range)
{
    if(param < 0)
    {
        if(param <= *pParam)
        {
            *pParam -= range;
        }
        else if(param >= *pParam)
        {
            *pParam += range;
        }
    }
    else
    {
        if(param > *pParam)
        {
            *pParam += range;
        }
        else if(param <= *pParam)
        {
            *pParam -= range;
        }
    }

    param = *pParam;
    return param;
}

void render(BelaContext *context, void *userData)
{
    // iterate over the audio frames and create three oscillators, seperated in phase by PI/2
    for(unsigned int n = 0; n < context->audioFrames; n++) {
        if(gAudioFramesPerAnalogFrame && !(n % gAudioFramesPerAnalogFrame)) {
            if(n==0)
            {
                // read analog inputs and update parameter
#if 0
                rateParameter = (float)map(analogRead(context, n/gAudioFramesPerAnalogFrame, gAnalogIn), 0, 1, 1, 200)/10.f;
                rateParameter = modParamFloat(rateParameter, &modRateParameter, 1.f);

                depthParameter = (float)map(analogRead(context, n/gAudioFramesPerAnalogFrame, gAnalogIn), 0, 1, 0, 100)/100.f;
                depthParameter = modParamFloat(depthParameter, &modDepthParameter, 0.1f);
                
                feedbackParameter = (float)map(analogRead(context, n/gAudioFramesPerAnalogFrame, gAnalogIn), 0, 1, 0, 98)/100.f;
                feedbackParameter = modParamFloat(feedbackParameter, &modFeedbackParameter, 0.01f);
#endif
                dryWetParameter = (float)map(analogRead(context, n/gAudioFramesPerAnalogFrame, gAnalogIn), 0, 1, 0, 100)/100.f;
                dryWetParameter = modParamFloat(dryWetParameter, &modDryWetParameter, 0.1f);
            }
        }

        // Read audio inputs
        inL = audioRead(context,n,0);
        inR = audioRead(context,n,1);

        gInputBuffer[0] = (inR+inL) * 0.5f;
        
#if 1 // direct processing for single sample processing. Scheduling for block processing! 
        /* do not use scheduling here! */
        process_chorus_flanger(gInputBuffer, gInputBufferPointer, gOutputBuffer, gOutputBufferWritePointer, gFs);
#else
        #error "Scheduling only for larger block processings!"
        gFFTInputBufferPointer = 0;
        gFFTOutputBufferPointer = 0;
        Bela_scheduleAuxiliaryTask(gFFTTask);
#endif
 
        outL = gOutputBuffer[0];
        outR = outL;

        audioWrite(context, n, 0, outL);
        audioWrite(context, n, 1, outR);
    }
}

// cleanup_render() is called once at the end, after the audio has stopped.
// Release any resources that were allocated in initialise_render().

void cleanup(BelaContext* context, void* userData)
{
    free(circBuff);
}
