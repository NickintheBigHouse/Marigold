#include "daisy_petal.h"
#include "daisysp.h"
#include "funbox.h"
#include "varSpeedLooper.h"

//
// This is a template for creating a pedal on the GuitarML Funbox_v3/Daisy Seed platform.
// You can start from here to fill out your effects processing and controls.
// Allows for Stereo In/Out, 6 knobs, 3 3-way switches, 4 dipswitches, 2 SPST Footswitches, 2 LEDs.
//
// Keith Bloemer 6/12/2024
//

using namespace daisy;
using namespace daisysp;
using namespace funbox;  // This is important for mapping the correct controls to the Daisy Seed on Funbox PCB

// Declare a local daisy_petal for hardware access
DaisyPetal hw;
Parameter trim1, trim2, level, loopspeed, filter, reverb;


bool force_reset = false;

bool            bypass;
bool            hold;

bool            pswitch1[2], pswitch2[2], pswitch3[2], pdip[4];
int             switch1[2], switch2[2], switch3[2], dip[4];


// Looper Parameters
#define MAX_SIZE (48000 * 60) // 1 minute
float DSY_SDRAM_BSS bufA[MAX_SIZE];
varSpeedLooper looperA;
Oscillator      led_oscA; // For pulsing the led when recording / paused playback
float           ledBrightnessA;
int             doubleTapCounterA;
bool            checkDoubleTapA;
bool            pausePlaybackA;
float           currentSpeedA;
bool isPlaybackA;

SmoothRandomGenerator smoothRandA;

#define MAX_SAMPLE static_cast<int>(48000.0 * 20.0) // 20 second sample
#define MAX_SAMPLE_SIZET static_cast<size_t>(MAX_SAMPLE) 
float DSY_SDRAM_BSS audioSample[3][MAX_SAMPLE_SIZET];  // three sample banks selected with right toggle

//float audioSample[MAX_SAMPLE_SIZET];  

bool recording = false;

int current_sample_size[3]{};
int sample_mode = 0;
int current_sample_bank = 0;

int recording_sample_index = 0;
bool trigger;
int fade_length;  // fade in/fade out audio sample by this many individual samples

int switch1_action = 0;

float middleC = 261.6256;

int trim1_index = 0;
int trim2_index = 0;
float global_voice_level = 0.5;

// global midi key values for granular synth
//float note_ = 0.0;
//float velocity_ = 0.0;

// Reverb
ReverbSc        verb;  // Reverb

//Tone filterLPHP;       // Low Pass
Svf filterLPHP;       // Low Pass and high pass

bool first_start=true;

Led led1, led2;

// Midi
bool midi_control[6]; // knobs 0-5
float pknobValues[6];
float knobValues[6];



int counter = 0;


class Voice {
  public:
    Voice() {}
    ~Voice() {}
    void Init(float samplerate) {
        active_ = false;

        env_.Init(samplerate);
        env_.SetSustainLevel(1.0f);
        env_.SetTime(ADSR_SEG_ATTACK, 0.5f);
        env_.SetTime(ADSR_SEG_DECAY, 0.05f);
        env_.SetTime(ADSR_SEG_RELEASE, 0.5f);

        playhead = static_cast<float>(trim1_index);  // set initial playhead to trim1 location
        play_speed = 1.0;
        is_first_sample = true;
    }

    // Process a single sample
    float getNextSample() {
        if (current_sample_size[current_sample_bank] == 0) return 0.0f;

        if (is_first_sample) {
            is_first_sample = false;
            playhead = static_cast<float>(trim1_index);
        }

        bool isReverse = false;
        if (trim1_index > trim2_index) {  // User could trigger reverse mid sample, so put here and not in initialization (TODO see if this works as expected)
            isReverse = true;
        }


        // If we get to the end of the sample, reset
        if (!isReverse && playhead >= static_cast<float>(trim2_index)) {
            //playhead = 0.0;
            playhead = static_cast<float>(trim1_index);
            if (sample_mode == 0 ) {
                active_ = false; // for now, end sample playback at end of sample in sample_mode= 0 (active remains true and repeats sample in other modes)
            } 
        }

        if (isReverse && playhead <= static_cast<float>(trim2_index)) {
            //playhead = 0.0;
            playhead = static_cast<float>(trim1_index);
            if (sample_mode == 0 ) {
                active_ = false; // for now, end sample playback at end of sample in sample_mode= 0 (active remains true and repeats sample in other modes)
            }
        }


        // Separate integer and fractional parts
        size_t index = static_cast<size_t>(playhead);
        float fraction = playhead - index;

        if (isReverse && index < 1) {
            return 0.0;
        }

        // Get current and next samples (handle boundary protection)
        float sample1 = audioSample[current_sample_bank][index];

        float sample2;
        if (!isReverse) {
            sample2 = (index + 1 < static_cast<float>(trim2_index)) ? audioSample[current_sample_bank][index + 1] : 0.0f;
        } else {
            sample2 = (index - 1 > static_cast<float>(trim2_index)) ? audioSample[current_sample_bank][index - 1] : 0.0f;
        }

        // Linear Interpolation
        float output = sample1 + static_cast<float>(fraction) * (sample2 - sample1);

        // Advance the playhead
        if (!isReverse) {
            playhead += play_speed;
        } else {
            playhead -= play_speed;
        }

        return output;
    }

    float Process() {
        if (active_) {
            float sig, amp;
            amp = env_.Process(env_gate_);
            if (!env_.IsRunning()) {
                active_ = false;
                playhead = 0.0;
            }

            return getNextSample() * (velocity_ / 127.f) * amp * global_voice_level;
        }
        return 0.f;
    }

    void OnNoteOn(float note, float velocity) {
        note_ = note;
        velocity_ = velocity;

        active_ = true;
        env_gate_ = true;
        play_speed = mtof(note_) / middleC;
        is_first_sample = true;
    }

    void OnNoteOff() { env_gate_ = false; }


    inline bool IsActive() const { return active_; }
    inline float GetNote() const { return note_; }

  private:

    Adsr env_;
    float note_, velocity_;
    bool active_;
    bool env_gate_;
    float play_speed;
    float playhead;
    bool is_first_sample;
};

template <size_t max_voices> class VoiceManager {
  public:
    VoiceManager() {}
    ~VoiceManager() {}

    void Init(float samplerate) {
        for (size_t i = 0; i < max_voices; i++) {
            voices[i].Init(samplerate);
        }
    }

    float Process() {
        float sum;
        sum = 0.f;
        for (size_t i = 0; i < max_voices; i++) {
            sum += voices[i].Process();
        }
        return sum;
    }

    void OnNoteOn(float notenumber, float velocity) {
        Voice *v = FindFreeVoice();
        if (v == NULL)
            return;
        v->OnNoteOn(notenumber, velocity);
    }

    void OnNoteOff(float notenumber, float velocity) {
        for (size_t i = 0; i < max_voices; i++) {
            Voice *v = &voices[i];
            if (v->IsActive() && v->GetNote() == notenumber) {
                v->OnNoteOff();
            }
        }
    }

    void FreeAllVoices() {
        for (size_t i = 0; i < max_voices; i++) {
            voices[i].OnNoteOff();
        }
    }



  private:
    Voice voices[max_voices];
    Voice *FindFreeVoice() {
        Voice *v = NULL;
        for (size_t i = 0; i < max_voices; i++) {
            if (!voices[i].IsActive()) {
                v = &voices[i];
                break;
            }
        }
        return v;
    }
};


// Sampler
static VoiceManager<12> voice_handler;


bool knobMoved(float old_value, float new_value)
{
    float tolerance = 0.005;
    if (new_value > (old_value + tolerance) || new_value < (old_value - tolerance)) {
        return true;
    } else {
        return false;
    }
}

// Switches1 
void updateSwitch1() 
{
    if (pswitch1[0] == true) { 
        switch1_action = 0;

    } else if (pswitch1[1] == true) {  
        switch1_action = 2;

    } else {  
        switch1_action = 1;

    }   
}

void updateSwitch2()
{
    if (pswitch2[0] == true) {    // One shot playback samples
        sample_mode = 0;
    } else if (pswitch2[1] == true) {    // TODO figure out something to put here, rand? granular?
        sample_mode = 2;
    } else {                      // Looped playback samples
        sample_mode = 1;
    }


}


void updateSwitch3() // left=, center=, right=
{
    if (pswitch3[0] == true) {  // left
        current_sample_bank = 0;

    } else if (pswitch3[1] == true) {  // right
        current_sample_bank = 2;

    } else {   // center
        current_sample_bank = 1;
    } 

    force_reset = true;   
}


void UpdateButtons()
{

    if(hw.switches[Funbox::FOOTSWITCH_2].RisingEdge())
    {

        recording = true;
        led2.Set(1.0f);

    } 

    if(hw.switches[Funbox::FOOTSWITCH_2].FallingEdge())
    {
        recording = false;
        recording_sample_index = 0;
        led2.Set(0.0f);
    }



    // LOOPER A //
    // Looper footswitch pressed (start/stop recording, doubletap to pause/unpause playback)
    if (hw.switches[Funbox::FOOTSWITCH_1].RisingEdge())
    {
        if (!pausePlaybackA) {
            looperA.TrigRecord();
            isPlaybackA = false;
            if (!looperA.Recording()) {  // Turn on LED if not recording and in playback
                led1.Set(1.0f);
                isPlaybackA = true;
            }
         
        }

        // Start or end double tap timer
        if (checkDoubleTapA) {
            // if second press comes before 1.0 seconds, pause playback
            if (doubleTapCounterA <= 1000) {
                if (looperA.Recording()) {  // Ensure looper is not recording when double tapped (in case it gets double tapped while recording)
                    looperA.TrigRecord();
                }
                pausePlaybackA = !pausePlaybackA;
                if (pausePlaybackA) {        // Blink LED if paused, otherwise set to triangle wave for pulsing while recording
                    led_oscA.SetWaveform(4); // WAVE_SIN = 0, WAVE_TRI = 1, WAVE_SAW = 2, WAVE_RAMP = 3, WAVE_SQUARE = 4
                } else {
                    led_oscA.SetWaveform(1); 
                }
                doubleTapCounterA = 0;    // reset double tap here also to prevent weird behaviour when triple clicked
                checkDoubleTapA = false;
                led1.Set(1.0f);
            }
        } else {
            checkDoubleTapA = true;
        }
    }

    if (checkDoubleTapA) {
        doubleTapCounterA += 1;          // Increment by 1 (48000 * 0.75)/blocksize = 1000   (blocksize is 48)
        if (doubleTapCounterA > 1000) {  // If timer goes beyond 1.0 seconds, stop double tap checking
            doubleTapCounterA = 0;
            checkDoubleTapA = false;
        }
    }

    // If switch1 is held, clear the looper and turn off LED
    if(hw.switches[Funbox::FOOTSWITCH_1].TimeHeldMs() >= 1000)
    {
        pausePlaybackA = false;
        led_oscA.SetWaveform(1); 
        looperA.Clear();
        led1.Set(0.0f);
    } 




  
    led1.Update();
    led2.Update();
}


void UpdateSwitches()
{
    // Detect any changes in switch positions (3 On-Off-On switches and Dip switches)

    // 3-way Switch 1
    bool changed1 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch1[i]].Pressed() != pswitch1[i]) {
            pswitch1[i] = hw.switches[switch1[i]].Pressed();
            changed1 = true;
        }
    }
    if (changed1 || first_start) 
        updateSwitch1();
    

    // 3-way Switch 2
    bool changed2 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch2[i]].Pressed() != pswitch2[i]) {
            pswitch2[i] = hw.switches[switch2[i]].Pressed();
            changed2 = true;
        }
    }
    if (changed2 || first_start) 
        updateSwitch2();

    // 3-way Switch 3
    bool changed3 = false;
    for(int i=0; i<2; i++) {
        if (hw.switches[switch3[i]].Pressed() != pswitch3[i]) {
            pswitch3[i] = hw.switches[switch3[i]].Pressed();
            changed3 = true;
        }
    }
    if (changed3 || first_start) 
        updateSwitch3();

    // Dip switches
    for(int i=0; i<2; i++) {
        if (hw.switches[dip[i]].Pressed() != pdip[i]) {
            pdip[i] = hw.switches[dip[i]].Pressed();
            // Action for dipswitches handled in audio callback
        }
    }


}



// This runs at a fixed rate, to prepare audio samples
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    hw.ProcessAnalogControls();
    hw.ProcessDigitalControls();

    UpdateButtons();
    UpdateSwitches();

//Parameter trim1, trim2, level, loopspeed, filter, reverb;

    float vtrim1 = trim1.Process();
    float vtrim2 = trim2.Process();
    float vlevel = level.Process();
    float vspeedA = loopspeed.Process();
    float vfilter = filter.Process();
    float vreverb = reverb.Process();



    // Handle Knob Changes Here
    float current_sample_size_float = static_cast<float>(current_sample_size[current_sample_bank]);
    trim1_index = static_cast<int>(vtrim1 * current_sample_size_float);
    trim2_index = static_cast<int>(vtrim2 * current_sample_size_float);
  
    global_voice_level = vlevel * 2.5;


    verb.SetFeedback(vreverb);

    float cutoff_filter;
    if (vfilter <= 0.5) {

        cutoff_filter = vfilter * vfilter * 2 * 18000.0 + 150.0; // exponential range 150 to 18150 as knob goes from 0 to 0.5
        filterLPHP.SetFreq(cutoff_filter);

    } else {
        cutoff_filter = ((vfilter - 0.5) * 2.0) * ((vfilter - 0.5) * 2.0) * 4000.0;
        filterLPHP.SetFreq(cutoff_filter); // exponential range 0 to 18150 as knob goes from 0.5 to 1.0

    }



    //////////////////////



    // LOOPER A
    float speed_inputA = 0.0;
    float speed_inputAabs = 0.0;


    // If switch1 in left position and not under midi control, or under midi control and set accordingly
    if (switch1_action == 0) { // Switch1 left = smooth

        if (vspeedA <= 0.5) {
            speed_inputA = vspeedA * 6.0 - 2.0; // maps 0 to 0.5 control to -2x to 1x speed

        } else {
            speed_inputA = vspeedA * 2.0;  // maps 0.5 to 1.0 control to 1x to 2x speed

        }


    } else if (switch1_action == 2) { // Switch1 right = random TODO

    } else {                        // Switch1 center = stepped  // TODO Verify that I don't need some kind of midi check here
        if (vspeedA < 0.05) {
            speed_inputA = -2.0;
        } else if (vspeedA >= 0.05 && vspeedA <= 0.15) {
            speed_inputA = -1.5;
        } else if (vspeedA >= 0.15 && vspeedA <= 0.25) {
            speed_inputA = -1.0;
        } else if (vspeedA >= 0.25 && vspeedA <= 0.35) {
            speed_inputA = -0.5;
        } else if (vspeedA >= 0.35 && vspeedA <= 0.45) {
            speed_inputA = 0.5;
        } else if (vspeedA >= 0.45 && vspeedA <= 0.55) {  // Noon is 1x speed
            speed_inputA = 1.0;
        } else if (vspeedA >= 0.55 && vspeedA <= 0.7) {
            speed_inputA = 1.5;
        } else if (vspeedA >= 0.7 && vspeedA <= 0.8) {
            speed_inputA = 2.0;
        } else if (vspeedA >= 0.8 && vspeedA <= 0.9) {
            speed_inputA = 2.5;
        } else {
            speed_inputA = 3.0;
        }

        if (speed_inputA < 0.0) {
            looperA.SetReverse(true);
        } else {
            looperA.SetReverse(false);
        }
        speed_inputAabs = abs(speed_inputA);

        looperA.SetIncrementSize(speed_inputAabs);
    }



//////////////////////////////////
    

    first_start=false;
    force_reset = false;

    float sendl, sendr, wetl, wetr;

    // Process the Audio Buffer //
    for(size_t i = 0; i < size; i++)
    {
        // Process your signal here
        if(bypass)
        {
            
            out[0][i] = out[1][i] = in[0][i];  // MISO when in bypass

        } else {   


            // Handle smooth speed knob transitions
            if (switch1_action == 0) { // Switch1 left = smooth

                // Smooth out Looper A transitions
                fonepole(currentSpeedA, speed_inputA, .00006f); 

                if (currentSpeedA < 0.0) {
                    looperA.SetReverse(true);
                } else {
                    looperA.SetReverse(false);
                }
                speed_inputAabs = abs(currentSpeedA);
                looperA.SetIncrementSize(speed_inputAabs);
            }

            
            float input = in[0][i];

            
            // Record input audio while footswitch is held, or until max buffer is reached
            if (recording) {
                audioSample[current_sample_bank][recording_sample_index] = input;
                current_sample_size[current_sample_bank] = recording_sample_index;
                recording_sample_index++;
                
                if (recording_sample_index >= MAX_SAMPLE) {
                    recording = false;
                    recording_sample_index = 0;
                    current_sample_size[current_sample_bank] = MAX_SAMPLE;
                    led1.Set(0.0f);
                }

            }


            float sum        = 0.f;
            sum        = voice_handler.Process();
    
            float filtered_sum = 0.0;
            filterLPHP.Process(sum);

            //sum += filterLPHP.Low();
            //sum += filterLPHP.High();

            if (vfilter <= 0.5) {  // is lowpass
                filtered_sum = filterLPHP.Low();
            } else {  // is highpass
                filtered_sum = filterLPHP.High();
            }


            sendl = sendr = filtered_sum;


            verb.Process(sendl, sendr, &wetl, &wetr);

            float audio_out = filtered_sum + wetl;

            /// Process Looper A // 
            float loop_outA = 0.0;
            if (!pausePlaybackA) {
                loop_outA = looperA.Process(audio_out);
            }

            
            out[0][i] = input + audio_out + loop_outA;
            out[1][i] = input + audio_out + loop_outA;

        }
    }

    // Handle Pulsing LEDs
    if (looperA.Recording()) {
        led1.Set(ledBrightnessA * 0.5 + 0.5);       // Pulse the LED when recording
    } 

    if (pausePlaybackA) {
        led1.Set(ledBrightnessA * 2.0);             // Blink the LED when paused
    }

    led1.Update();

}

void OnNoteOff(float notenumber, float velocity)
{

    voice_handler.OnNoteOff(notenumber, velocity);

}

void OnNoteOn(float notenumber, float velocity)
{
    // Note Off can come in as Note On w/ 0 Velocity
    if(velocity == 0.f)
    {
        OnNoteOff(notenumber, velocity);

    }
    else
    {
        voice_handler.OnNoteOn(notenumber, velocity);

    }
}


// Typical Switch case for Message Type.
void HandleMidiMessage(MidiEvent m)
{
    switch(m.type)
    {
        case NoteOn:
        {
            NoteOnEvent p = m.AsNoteOn();
            OnNoteOn(p.note, p.velocity);
        }
        break;

        case NoteOff:
        {
            NoteOffEvent p = m.AsNoteOff();
            OnNoteOff(p.note, p.velocity);
        }
        break;

        case ControlChange:
        {

            ControlChangeEvent p = m.AsControlChange();
            float val = (float)p.value / 127.0f;
            switch(p.control_number)
            {   
                
                // Knob Controls //////////////////////////////
                case 14: {
                    midi_control[0] = true;
                    knobValues[0] = ((float)p.value / 127.0f);
                    break;
                }
                case 15: {
                    midi_control[1] = true;
                    knobValues[1] = ((float)p.value / 127.0f);
                    break;
                }
                case 16: {
                    midi_control[2] = true;
                    knobValues[2] = ((float)p.value / 127.0f);
                    break;
                }
                case 17: {
                    midi_control[3] = true;
                    knobValues[3] = ((float)p.value / 127.0f);
                    break;
                }
                case 18: {
                    midi_control[4] = true;
                    knobValues[4] = ((float)p.value / 127.0f);
                    break;
                }
                case 19: {
                    midi_control[5] = true;
                    knobValues[5] = ((float)p.value / 127.0f);
                    break;
                }



                default: { break; }
            }
            break;
        }
        default: { break; }
    }
}

          

int main(void)
{
    float samplerate;

    hw.Init(true);
    samplerate = hw.AudioSampleRate();

    hw.SetAudioBlockSize(48); 


    switch1[0]= Funbox::SWITCH_1_LEFT;
    switch1[1]= Funbox::SWITCH_1_RIGHT;
    switch2[0]= Funbox::SWITCH_2_LEFT;
    switch2[1]= Funbox::SWITCH_2_RIGHT;
    switch3[0]= Funbox::SWITCH_3_LEFT;
    switch3[1]= Funbox::SWITCH_3_RIGHT;
    dip[0]= Funbox::SWITCH_DIP_1;
    dip[1]= Funbox::SWITCH_DIP_2;
    dip[2]= Funbox::SWITCH_DIP_3;
    dip[3]= Funbox::SWITCH_DIP_4;

    pswitch1[0]= false;
    pswitch1[1]= false;
    pswitch2[0]= false;
    pswitch2[1]= false;
    pswitch3[0]= false;
    pswitch3[1]= false;
    pdip[0]= false;
    pdip[1]= false;
    pdip[2]= false;
    pdip[3]= false;



    counter = 0;

    // I dont think this is needed after moving buffer from SDRAM to SRAM TODO Verify that
    for(int i = 0; i < MAX_SAMPLE; i++) { // hard coding sample length for now
        audioSample[0][i] = 0.;
        audioSample[1][i] = 0.;
        audioSample[2][i] = 0.;
    }


    looperA.Init(bufA, MAX_SIZE);
    looperA.SetMode(varSpeedLooper::Mode::NORMAL);
    led_oscA.Init(samplerate);
    led_oscA.SetFreq(1.0);
    led_oscA.SetWaveform(1); // WAVE_SIN = 0, WAVE_TRI = 1, WAVE_SAW = 2, WAVE_RAMP = 3, WAVE_SQUARE = 4
    ledBrightnessA = 0.0;
    pausePlaybackA = false;
    currentSpeedA = 1.0;

    led_oscA.Init(samplerate);
    led_oscA.SetFreq(1.5);
    led_oscA.SetWaveform(1);

    smoothRandA.Init(samplerate);

    verb.Init(samplerate); 
    verb.SetLpFreq(100000);
    verb.SetFeedback(0.0);

    filterLPHP.Init(samplerate);
    filterLPHP.SetFreq(20000.0);

    voice_handler.Init(samplerate);

    trim1.Init(hw.knob[Funbox::KNOB_1], 0.0f, 1.0f, Parameter::LINEAR);
    trim2.Init(hw.knob[Funbox::KNOB_2], 0.0f, 1.0f, Parameter::LINEAR);
    level.Init(hw.knob[Funbox::KNOB_3], 0.0f, 1.0f, Parameter::LINEAR); 
    loopspeed.Init(hw.knob[Funbox::KNOB_4], 0.01f, 1.0f, Parameter::LINEAR);
    filter.Init(hw.knob[Funbox::KNOB_5], 0.0f, 1.0f, Parameter::LINEAR); 
    reverb.Init(hw.knob[Funbox::KNOB_6], 0.0f, 1.0f, Parameter::LINEAR);


    // Init the LEDs and set activate bypass
    led1.Init(hw.seed.GetPin(Funbox::LED_1),false);
    led1.Update();
    bypass = false;

    led2.Init(hw.seed.GetPin(Funbox::LED_2),false);
    led2.Update();




    // Midi
    for( int i = 0; i < 6; ++i ) 
        midi_control[i] = false;  

    hw.InitMidi();
    hw.midi.StartReceive();

    hw.StartAdc();
    hw.StartAudio(AudioCallback);
    while(1)
    {
        hw.midi.Listen();
        // Handle MIDI Events
        while(hw.midi.HasEvents())
        {
            HandleMidiMessage(hw.midi.PopEvent());
        }

    }
}