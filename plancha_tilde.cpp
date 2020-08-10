#include "c74_msp.h"
       
#include "Plateau/Dattorro.hpp"
#include "Common/DSP/NonLinear.hpp"
#include "Common/DSP/LinearEnvelope.hpp"
#include <iostream>

using namespace c74::max;

static t_class* this_class = nullptr;

#define DEFAULT_SAMPLE_RATE 48000.0f


inline float rescale(float x, float xMin, float xMax, float yMin, float yMax) {
	return yMin + (x - xMin) / (xMax - xMin) * (yMax - yMin);
}

inline float min(float a, float b) {
	return (a < b) ? a : b;
}

inline float max(float a, float b) {
	return (a > b) ? a : b;
}

inline float clamp(float x, float a, float b) {
	return min(max(x, a), b);
}

struct t_plancha {
    t_pxobject m_obj;

    // CV scaling
    const float dryMin = 0.f;
    const float dryMax = 1.f;
    const float wetMin = 0.f;
    const float wetMax = 1.f;
    const float preDelayNormSens = 0.1f;
    const float preDelayLowSens = 0.05f;
    const float sizeMin = 0.0025f;
    const float sizeMax = 4.0f;
    const float diffMin = 0.f;
    const float diffMax = 1.f;
    const float decayMin = 0.1f;
    const float decayMax = 0.9999f;
    const float reverbLowDampMin = 0.f;
    const float reverbLowDampMax = 10.f;
    const float reverbHighDampMin = 0.f;
    const float reverbHIghDampMax = 10.f;
    const float modSpeedMin = 0.f;
    const float modSpeedMax = 1.f;
    const float modDepthMin = 0.f;
    const float modDepthMax = 16.f;
    const float modShapeMin = 0.001f;
    const float modShapeMax = 0.999f;

    float wet;
    float dry;
    float preDelay;
    float preDelayCVSens;
    float size;
    float newsize;
    float diffusion;
    float decay;
    float inputSensitivity;
    float inputDampLow;
    float inputDampHigh;
    float reverbDampLow;
    float reverbDampHigh;
    float modSpeed;
    float modShape;
    float modDepth;

    bool freeze;
    bool frozen;
    int preDelayCVSensState;
    int inputSensitivityState;
    int outputSaturationState;

    bool clear;
    bool cleared;
    bool fadeOut, fadeIn;
    double leftInput, rightInput;
 
    int tuned;
    float diffuseInput;

    bool printy = true;
    int printLength = 512;
    int printCount = 0;

};

LinearEnvelope envelope;
Dattorro reverb;

void plancha_perform64(t_plancha* self, t_object* dsp64, double** ins, long numins, double** outs, long numouts, long sampleframes, long flags, void* userparam) {
    double    *in = ins[0];     // first inlet
    double    *in2 = ins[1];     // first inlet
    double    *out = outs[0];   // first outlet
    double    *out2 = outs[1];   // first outlet

    reverb.processLFO();
    for (int i=0; i<sampleframes; ++i){

        if (std::abs(self->size - self->newsize) > 0.01f)
        {
            self->size += (self->newsize - self->size) * 0.01f;
            reverb.setTimeScale(self->size);
        }else if(self->size != self->newsize){
            self->size = self->newsize;
            reverb.setTimeScale(self->size);
        }

        self->leftInput = *in++;
        self->rightInput = *in2++;

        envelope.process();

        reverb.process(self->leftInput * envelope._value, self->rightInput * envelope._value);

        *out++ = SoftClip((self->leftInput * self->dry + reverb.leftOut * self->wet * envelope._value) * 0.19f);
        *out2++ = SoftClip((self->rightInput * self->dry + reverb.rightOut * self->wet * envelope._value) * 0.19f);
    }

}

void* plancha_new(void) {
	t_plancha* self = (t_plancha*) object_alloc(this_class);

    dsp_setup((t_pxobject*)self, 2);

    outlet_new(self, "signal");
    outlet_new(self, "signal");
    inlet_new(self, NULL);


    reverb.setSampleRate(DEFAULT_SAMPLE_RATE);
    envelope.setSampleRate(DEFAULT_SAMPLE_RATE);
    envelope.setTime(0.004f);
    envelope._value = 1.f;

    self->wet = 1.f;
    self->dry = 0.f;
    self->preDelay = 0.f;
    self->size = 1.f;
    self->diffusion = 1.f;
    self->decay = 0.f;
    self->inputDampLow = 0.f;
    self->inputDampHigh = 10.f;
    self->reverbDampLow = 0.f;
    self->reverbDampHigh = 10.f;
    self->modSpeed = 0.1f;
    self->modShape = 0.5f;
    self->modDepth = 0.0f;

    self->freeze = false;
    self->frozen = false;

    self->clear = false;
    self->cleared = true;
    self->fadeOut = false;
    self->fadeIn = false;
    self->tuned = 0;
    self->diffuseInput = 1.0f;

    if(self->freeze && !self->frozen) {
        self->frozen = true;
        reverb.freeze();
    }
    else if(!self->freeze && self->frozen){
        self->frozen = false;
        reverb.unFreeze();
    }
    
    reverb.setTimeScale(self->size);
    reverb.setPreDelay(self->preDelay);

    reverb.decay = self->decay;
    reverb.plateDiffusion1 = self->diffusion * 0.9f;
    reverb.plateDiffusion2 = self->diffusion * 0.99f;

    reverb.diffuseInput = self->diffuseInput;

    reverb.inputLowCut = 440.f * powf(2.f, self->inputDampLow - 5.f);
    reverb.inputHighCut = 440.f * powf(2.f, self->inputDampHigh - 5.f);
    reverb.reverbLowCut = 440.f * powf(2.f, self->reverbDampLow - 5.f);
    reverb.reverbHighCut = 440.f * powf(2.f, self->reverbDampHigh - 5.f);

    reverb.modSpeed = self->modSpeed;
    reverb.modDepth = self->modDepth;
    reverb.setModShape(self->modShape);



	return (void *)self;
}

/*

Params

*/

void plancha_samplerate(t_plancha *x, double f)
{
    reverb.setSampleRate(f);
    envelope.setSampleRate(f);
}

void plancha_freeze(t_plancha *x, double f)
{
  	x->freeze = f > 0.5f ? true : false;
    if(x->freeze && !x->frozen) {
        x->frozen = true;
        reverb.freeze();
    }
    else if(!x->freeze && x->frozen){
        x->frozen = false;
        reverb.unFreeze();
    }
}

void plancha_tuned(t_plancha *x, double f)
{
	x->tuned = f > 0.5f ? true : false;
}

void plancha_diffuseInput(t_plancha *x, double f)
{
	x->diffuseInput = (1.0f - f);
    reverb.diffuseInput = x->diffuseInput;
}

void plancha_diffusion(t_plancha *x, double f)
{
	x->diffusion = (1.0f - f);
    reverb.plateDiffusion1 = x->diffusion * 0.9f;
    reverb.plateDiffusion2 = x->diffusion * 0.99f;
}

void plancha_clear(t_plancha *x, double f)
{
    x->clear = f > 0.5f;
}

void plancha_predelay(t_plancha *x, double f)
{	    
	x->preDelay = f;
    reverb.setPreDelay(x->preDelay);
}

void plancha_size(t_plancha *x, double f)
{	    

        float size = f;
        size *= size;
        x->newsize = rescale(size, 0.f, 1.f, 0.01f, x->sizeMax) * 40.0f;
}	    

void plancha_decay(t_plancha *x, double f)
{	    
    x->decay = f;
    x->decay = 1.f - x->decay;
    x->decay = 1.f - x->decay * x->decay;
    reverb.decay = x->decay;
}

void plancha_inputDampLow(t_plancha *x, double f)
{
    x->inputDampLow = clamp(f * 10.0f, 0.f, 10.f);
    x->inputDampLow = 10.f - x->inputDampLow;
    reverb.inputLowCut = 440.f * powf(2.f, x->inputDampLow - 5.f);
    reverb._inputHpf.setCutoffFreq(reverb.inputLowCut);
}

void plancha_inputDampHigh(t_plancha *x, double f)
{
    x->inputDampHigh = clamp(f * 10 , 0.f, 10.f);
    reverb.inputHighCut = 440.f * powf(2.f, x->inputDampHigh - 5.f);
    reverb._inputLpf.setCutoffFreq(reverb.inputHighCut);
    /*object_post((t_object *) &x->x_obj, "plancha_inputDampHigh");
    std::string s = std::to_string(x->inputDampHigh);
    object_post((t_object *) &x->x_obj, s.c_str());*/
}

void plancha_reverbDampLow(t_plancha *x, double f)
{
    x->reverbDampLow = clamp(f * 10.0f, 0.f, 10.f);
    x->reverbDampLow = 10.f - x->reverbDampLow;
    reverb.reverbLowCut = 440.f * powf(2.f, x->reverbDampLow - 5.f);
    reverb._leftHpf.setCutoffFreq(reverb.reverbLowCut);
    reverb._rightHpf.setCutoffFreq(reverb.reverbLowCut);
}

void plancha_reverbDampHigh(t_plancha *x, double f)
{
    x->reverbDampHigh = clamp(f * 10.0f, 0.f, 10.f);
    reverb.reverbHighCut = 440.f * powf(2.f, x->reverbDampHigh - 5.f);
    reverb._leftFilter.setCutoffFreq(reverb.reverbHighCut);
    reverb._rightFilter.setCutoffFreq(reverb.reverbHighCut);
}

void plancha_modSpeed(t_plancha *x, double f)
{
    x->modSpeed = f * 99 + 1;
    reverb.modSpeed = x->modSpeed;

    reverb._lfo1.setFrequency(reverb._lfo1Freq * reverb.modSpeed);
    reverb._lfo2.setFrequency(reverb._lfo2Freq * reverb.modSpeed);
    reverb._lfo3.setFrequency(reverb._lfo3Freq * reverb.modSpeed);
    reverb._lfo4.setFrequency(reverb._lfo4Freq * reverb.modSpeed);
}

void plancha_modShape(t_plancha *x, double f)
{
    x->modShape = f;
    reverb.setModShape(x->modShape);
}

void plancha_modDepth(t_plancha *x, double f)
{
    x->modDepth = f * 16;
    reverb.modDepth = x->modDepth;
}

void plancha_dry(t_plancha *x, double f)
{
    x->dry = f;
}

void plancha_wet(t_plancha *x, double f)
{
    x->wet = f;
}

void plancha_free(t_plancha* self) {
	dsp_free((t_pxobject*)self);
}

void plancha_dsp64(t_plancha* self, t_object* dsp64, short* count, double samplerate, long maxvectorsize, long flags) {
	object_method_direct(void, (t_object*, t_object*, t_perfroutine64, long, void*),
						 dsp64, gensym("dsp_add64"), (t_object*)self, (t_perfroutine64)plancha_perform64, 0, NULL);
}

void plancha_assist(t_plancha* self, void* unused, t_assist_function io, long index, char* string_dest) {
	if (io == ASSIST_INLET) {
		switch (index) {
			case 1: 
				strncpy(string_dest,"(signal) L IN", ASSIST_STRING_MAXSIZE); 
				break;
			case 2: 
				strncpy(string_dest,"(signal) R IN", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
	else if (io == ASSIST_OUTLET) {
		switch (index) {
			case 0: 
				strncpy(string_dest,"(signal) L Output", ASSIST_STRING_MAXSIZE); 
				break;
			case 1: 
				strncpy(string_dest,"(signal) R Output", ASSIST_STRING_MAXSIZE); 
				break;
		}
	}
}

void ext_main(void* r) {
	this_class = class_new("plancha~", (method)plancha_new, (method)plancha_free, sizeof(t_plancha), NULL, A_GIMME, 0);

	class_addmethod(this_class,(method) plancha_assist, "assist",	A_CANT,		0);
	class_addmethod(this_class,(method) plancha_dsp64, "dsp64",	A_CANT,		0);
	
	class_addmethod(this_class,(method) plancha_freeze, "freeze", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_samplerate, "samplerate", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_tuned, "tuned", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_clear, "clear", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_diffuseInput, "diffuseInput", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_predelay, "predelay", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_diffusion, "diffusion", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_size, "size", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_decay, "decay", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_dry, "dry", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_wet, "wet", A_DEFFLOAT, 0);

	class_addmethod(this_class,(method) plancha_modSpeed, "modSpeed", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_modShape, "modShape", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_modDepth, "modDepth", A_DEFFLOAT, 0);


	class_addmethod(this_class,(method) plancha_inputDampLow, "inputDampLow", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_inputDampHigh, "inputDampHigh", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_reverbDampLow, "reverbDampLow", A_DEFFLOAT, 0);
	class_addmethod(this_class,(method) plancha_reverbDampHigh, "reverbDampHigh", A_DEFFLOAT, 0);

	class_dspinit(this_class);
	class_register(CLASS_BOX, this_class);
}