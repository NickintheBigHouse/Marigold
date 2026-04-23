// Marigold.cpp
// Venus Spectral Reverb ported from Marigold (Funbox/DaisyPetal) hardware
// to Emerald (custom 125B Daisy Seed) hardware.
//
// STEP 1: Pin remapping, relay/mute GPIO init, boot into bypass.
//         Expression, DIP switches, toggle switches, and MIDI are
//         stripped in later steps. They are still present but commented
//         out or left inert so the DSP compiles and runs immediately.
//
// PIN CHANGES vs. original venus.cpp (Marigold hardware):
//   Knobs:       ADC_0-5 -> same logical order, physical shift by 1
//                (D15=A0 -> now D15=A0, but physical pin 22 not 23)
//                Emerald: D15-D20 = physical pins 22-27
//   FOOTSWITCH_1 (Bypass): was D25/physical 32 -> now D6/physical 7
//   FOOTSWITCH_2 (Freeze): was D26/physical 33 -> now D5/physical 6
//   NOTE: Emerald swaps L/R vs Marigold. On Emerald:
//         D6 (phys 7) = right footswitch = BYPASS
//         D5 (phys 6) = left footswitch  = FREEZE
//   LEDs: D22/D23 = physical pins 29/30 (unchanged)
//
// RELAY / MUTE (Emerald-specific, not present on Marigold):
//   DISABLE_AUDIO_BYPASS relay: D1  (physical pin 2)  active-low
//   ENABLE_AUDIO_MUTE    mute:  D12 (physical pin 13) active-high
//
// HARDCODED (hardware not present on Emerald, stripped in Step 3):
//   shimmer_mode = 1  (center: octave up only)
//   reverb_mode  = 1  (center: normal, no lofi)
//   drift_mode   = 1  (center: no drift)

#include <string.h>
#include "daisy_seed.h"
#include "daisysp.h"

#include <cmath>
#include <complex>
#include "shy_fft.h"

#include "fourier.h"
#include "wave.h"

#define PI 3.1415926535897932384626433832795

using namespace daisy;
using namespace daisysp;
using namespace soundmath;

// ============================================================
// Hardware
// ============================================================
DaisySeed hw;

// Relay and mute GPIOs (Emerald-specific)
// DISABLE_AUDIO_BYPASS: D1, active-low
//   Write LOW  -> relay energized -> effect in signal path
//   Write HIGH -> relay bypassed  -> dry signal passes
// ENABLE_AUDIO_MUTE: D12, active-high
//   Write HIGH -> mute on  (use briefly during relay switching)
//   Write LOW  -> mute off
GPIO relay_pin;   // D1  (physical pin 2)
GPIO mute_pin;    // D12 (physical pin 13)

// Helper: set relay to effect path (bypass=false) or bypass path (bypass=true)
inline void SetRelay(bool bypass_active) {
    // relay is active-low: LOW = effect in path, HIGH = bypass
    relay_pin.Write(bypass_active ? true : false);
}

// Helper: enable/disable output mute
inline void SetMute(bool muted) {
    // mute is active-high: HIGH = muted
    mute_pin.Write(muted);
}

// ============================================================
// Parameters
// ============================================================
Parameter decay, mix, damp, shimmer, shimmer_tone, detune;

float samplerate = 32000.0f;

bool bypass = true;

// Footswitch state (polled directly from GPIO)
bool fs1_prev = false;  // D6 = right = bypass
bool fs2_prev = false;  // D5 = left  = freeze

// Dedicated GPIO inputs for Emerald footswitches.
// NOTE:
// - We intentionally use explicit GPIO objects instead of DaisySeed::GetPin(...) so we
//   have concrete input instances that support .Init(...) and .Read(...).
// - Both switches are active-low on Emerald 125b hardware:
//   * Electrical LOW  = physically pressed
//   * Electrical HIGH = physically released (via pull-up)
GPIO fs_bypass_pin;   // D6 (physical pin 7)  = BYPASS toggle switch
GPIO fs_freeze_pin;   // D5 (physical pin 6)  = FREEZE hold switch

// LED pins (D22, D23)
Led led1, led2;

// ============================================================
// DSP state (unchanged from venus.cpp)
// ============================================================
SampleRateReducer samplerateReducer;
Tone lowpass;
int reverb_mode = 1;   // HARDCODED: center = normal

Wave<float> hann([] (float phase) -> float { return 0.5 * (1 - cos(2 * PI * phase)); });
Wave<float> halfhann([] (float phase) -> float { return sin(PI * phase); });

const size_t order = 12;
const size_t N = (1 << order);
const float  sqrtN = sqrt(N);
const size_t laps = 4;
const size_t buffsize = 2 * laps * N;

float in_buf[buffsize];
float middle[buffsize];
float out_buf[buffsize];

float reverb_energy[N/2];

ShyFFT<float, N, RotationPhasor>* fft;
Fourier<float, N>* stft;

float fft_size = N / 2;

float vdecay, vmix, vdamp, vshimmer, vshimmer_tone, vdetune;
float octave_up_rate_persecond, octave_up_rate_perinterval, shimmer_double, shimmer_triple, shimmer_remainder;
float detune_rate_persecond, detune_rate_perinterval, detune_double, detune_remainder;

float window_samples   = 32768.0f;
float interval_samples = ceil(window_samples / laps);

bool freeze       = false;
int  shimmer_mode = 1;   // HARDCODED: center = octave up
int  detune_mode  = 1;
int  detune_multiplier = 1;

bool first_start = true;

int  drift_mode = 1;     // HARDCODED: center = no drift
Oscillator drift_osc, drift_osc2, drift_osc3, drift_osc4;
float drift_multiplier  = 1.0f;
float drift_multiplier2 = 1.0f;
float drift_multiplier3 = 1.0f;
float drift_multiplier4 = 1.0f;

// ============================================================
// Knob values array (kept for future MIDI, currently direct)
// ============================================================
float knobValues[6];

// Analog controls backing each parameter.
// IMPORTANT:
// - `daisy::Parameter::Init(...)` takes an AnalogControl object, not a raw ADC pointer.
// - Each AnalogControl is initialized from the ADC pointer + callback rate first.
// - Parameter then reads/scales that control each callback in ProcessControls().
AnalogControl knob_ctrl_decay;
AnalogControl knob_ctrl_mix;
AnalogControl knob_ctrl_damp;
AnalogControl knob_ctrl_shimmer;
AnalogControl knob_ctrl_shimmer_tone;
AnalogControl knob_ctrl_detune;

// ============================================================
// ProcessControls
// ============================================================
static void ProcessControls()
{
    // Read knobs directly (no expression remapping in Step 1)
    vdecay        = decay.Process()        * 99.0f + 1.0f;
    vmix          = mix.Process();
    vdamp         = damp.Process();
    vshimmer      = shimmer.Process();
    vshimmer_tone = shimmer_tone.Process();
    float vdetune_temp = detune.Process();

    vdetune = fabsf(vdetune_temp);

    // Drift automation: drift_mode == 1 (no drift), so this block is inert
    // but kept structurally intact for Step 3 cleanup.
    if (drift_mode == 0 || drift_mode == 2) {
        vdamp         = vdamp         * fabsf(drift_multiplier)  * 0.7f + 0.3f;
        vshimmer     *= fabsf(drift_multiplier2);
        vshimmer_tone *= fabsf(drift_multiplier3);
        vdetune      *= fabsf(drift_multiplier4);
    }

    if (vdetune > 0.03f) {
        vdetune = vdetune - 0.029f;
        if (vdetune_temp >= 0) {
            detune_mode       = 2;
            detune_multiplier = 1;
        } else {
            detune_mode       = 0;
            detune_multiplier = -1;
        }
    } else {
        detune_mode = 1;
    }

    octave_up_rate_persecond  = std::pow(8.0f, vshimmer) - 1.0f;
    octave_up_rate_perinterval = std::min(0.75f, octave_up_rate_persecond / samplerate * interval_samples);

    float octave_up_rate_persecond2  = std::pow(8.0f, vshimmer_tone) - 1.0f;
    float octave_up_rate_perinterval2 = std::min(0.75f, octave_up_rate_persecond2 / samplerate * interval_samples);

    shimmer_double    = octave_up_rate_perinterval  * (1.0f - vshimmer_tone / 1.58f);
    shimmer_triple    = (octave_up_rate_perinterval2 / 1.58f) * vshimmer_tone;
    shimmer_remainder = 1.0f - shimmer_double - shimmer_triple;

    detune_rate_persecond  = std::pow(8.0f, vdetune) - 1.0f;
    detune_rate_perinterval = std::min(0.75f, detune_rate_persecond / samplerate * interval_samples);
    detune_double    = detune_rate_perinterval;
    detune_remainder = 1.0f - detune_double;
}

// ============================================================
// UpdateButtons
// Emerald footswitch mapping:
//   D6 (phys 7) = right footswitch = BYPASS toggle
//   D5 (phys 6) = left footswitch  = FREEZE (hold)
// Both footswitches are active-low on Emerald 125b hardware
// (.Pressed() returns false when physically pressed).
// We read the GPIO directly here since we're not using DaisyPetal.
// ============================================================
static void UpdateButtons()
{
    // Read raw GPIO state from dedicated switch inputs.
    // Active-low behavior:
    // - .Read() == false means pin is LOW  -> physically pressed
    // - .Read() == true  means pin is HIGH -> physically released
    // We invert to convert into the more readable boolean "pressed" semantics.
    bool fs1_raw = !fs_bypass_pin.Read();  // true when BYPASS switch is physically pressed
    bool fs2_raw = !fs_freeze_pin.Read();  // true when FREEZE switch is physically pressed

    // Bypass: toggle on rising edge (press)
    if (fs1_raw && !fs1_prev) {
        bypass = !bypass;

        // Mute briefly during relay transition to prevent pop
        SetMute(true);
        System::DelayUs(1000);  // 1ms mute window
        SetRelay(bypass);
        System::DelayUs(1000);
        SetMute(false);

        led1.Set(bypass ? 0.0f : 1.0f);
    }

    // Freeze: hold behavior (active while held)
    freeze = fs2_raw;
    led2.Set(freeze ? 1.0f : 0.0f);

    fs1_prev = fs1_raw;
    fs2_prev = fs2_raw;

    led1.Update();
    led2.Update();
}

// ============================================================
// AudioCallback
// ============================================================
static void AudioCallback(AudioHandle::InputBuffer  in,
                          AudioHandle::OutputBuffer out,
                          size_t                    size)
{
    ProcessControls();
    UpdateButtons();

    for (size_t i = 0; i < size; i++)
    {
        // Drift oscillators still processed (harmless at drift_mode==1)
        drift_multiplier  = drift_osc.Process();
        drift_multiplier2 = drift_osc2.Process();
        drift_multiplier3 = drift_osc3.Process();
        drift_multiplier4 = drift_osc4.Process();

        if (bypass)
        {
            out[0][i] = in[0][i];
            out[1][i] = in[1][i];
        }
        else
        {
            stft->write(in[0][i]);

            float wet = 0.0f;
            if (reverb_mode == 0) {
                wet = lowpass.Process(samplerateReducer.Process(stft->read()));
            } else if (reverb_mode == 1) {
                wet = stft->read();
            } else if (reverb_mode == 2) {
                wet = samplerateReducer.Process(stft->read());
            }

            out[0][i] = wet * vmix + in[0][i] * (1.0f - vmix);
            out[1][i] = out[0][i];
        }
    }
}

// ============================================================
// Reverb spectral processing (unchanged from venus.cpp)
// ============================================================
inline void reverb(const float* in, float* out)
{
    static const size_t offset = N / 2;

    float reverb_amp = 0.0f;
    for (size_t i = 0; i < N / 2; i++)
    {
        float fft_bin = i + 1;
        float real    = in[i];
        float imag    = in[i + offset];
        float energy  = real * real + imag * imag;

        reverb_amp = sqrt(reverb_energy[i]);
        if (fft_bin / fft_size > vdamp) {
            reverb_amp *= vdamp * fft_size / fft_bin;
        }

        float random_phase = rand() * 2 * PI;
        real = reverb_amp * cos(random_phase);
        imag = reverb_amp * sin(random_phase);

        if (!freeze) {
            reverb_energy[i] += energy / laps;

            float reverb_decay_factor = 1.0f / vdecay;
            reverb_energy[i] *= 1.0f - reverb_decay_factor;

            float half_fft_size = fft_size / 2;
            float current       = reverb_energy[i];

            if (i > 0 && i < half_fft_size - 2) {
                if (shimmer_mode == 1 || shimmer_mode == 2) {
                    reverb_energy[2*i - 1] += 0.123f * shimmer_double * current;
                    reverb_energy[2*i    ] += 0.25f  * shimmer_double * current;
                    reverb_energy[2*i + 1] += 0.123f * shimmer_double * current;
                } else if ((shimmer_mode == 0 || shimmer_mode == 2) && i > 1 && !(i % 2)) {
                    reverb_energy[i/2 - 1] += 0.75f * shimmer_double * current;
                    reverb_energy[i/2    ] += 1.5f  * shimmer_double * current;
                    reverb_energy[i/2 + 1] += 0.75f * shimmer_double * current;
                }

                if (3*i + 1 < half_fft_size) {
                    reverb_energy[3*i - 2] += 0.055f * shimmer_triple * current;
                    reverb_energy[3*i - 1] += 0.11f  * shimmer_triple * current;
                    reverb_energy[3*i    ] += 0.17f  * shimmer_triple * current;
                    reverb_energy[3*i + 1] += 0.11f  * shimmer_triple * current;
                    reverb_energy[3*i + 2] += 0.105f * shimmer_triple * current;
                }

                if (i > 2 && i < half_fft_size - 2 && detune_mode != 1) {
                    reverb_energy[i + (3*detune_multiplier)] += 0.123f * detune_double * current;
                    reverb_energy[i + (2*detune_multiplier)] += 0.25f  * detune_double * current;
                    reverb_energy[i + (1*detune_multiplier)] += 0.123f * detune_double * current;
                }
            }

            if (detune_mode == 1)
                detune_remainder = 1.0f;
            reverb_energy[i] = detune_remainder * shimmer_remainder * current;
        }

        out[i]          = real;
        out[i + offset] = imag;
    }
}

// ============================================================
// main
// ============================================================
int main(void)
{
    // --- Hardware init ---
    hw.Configure();
    hw.Init();
    hw.SetAudioSampleRate(SaiHandle::Config::SampleRate::SAI_32KHZ);
    samplerate = hw.AudioSampleRate();
    hw.SetAudioBlockSize(256);

    // --- Serial for testing ---
    hw.StartLog(false);
    System::Delay(500);
    hw.PrintLine("Marigold boot: Step 1");

    // --- Relay and mute GPIO init ---
    // D1 = relay (active-low). D12 = mute (active-high).
    relay_pin.Init(seed::D1,  GPIO::Mode::OUTPUT);
    mute_pin.Init(seed::D12, GPIO::Mode::OUTPUT);

    // Boot state: bypass active, audio muted briefly then unmuted
    SetMute(true);
    SetRelay(true);   // bypass path active (HIGH = bypass on active-low relay)
    System::DelayUs(5000);
    SetMute(false);

    hw.PrintLine("Relay init OK. Boot into bypass.");

    // --- Footswitch GPIO init ---
    // D6 = right = bypass, D5 = left = freeze. Active-low, so we enable pull-ups.
    // Pull-up means idle state reads HIGH; pressing the switch pulls to LOW.
    fs_bypass_pin.Init(seed::D6, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);
    fs_freeze_pin.Init(seed::D5, GPIO::Mode::INPUT, GPIO::Pull::PULLUP);

    // --- Knob init (ADC channels A0-A5 = D15-D20) ---
    AdcChannelConfig adc_cfg[6];
    adc_cfg[0].InitSingle(seed::D15);  // Decay
    adc_cfg[1].InitSingle(seed::D16);  // Mix
    adc_cfg[2].InitSingle(seed::D17);  // Damp
    adc_cfg[3].InitSingle(seed::D18);  // Shimmer
    adc_cfg[4].InitSingle(seed::D19);  // Shimmer Tone
    adc_cfg[5].InitSingle(seed::D20);  // Detune
    hw.adc.Init(adc_cfg, 6);

    float cb_rate = hw.AudioCallbackRate();

    // ------------------------------------------------------------
    // Analog control initialization (raw ADC -> normalized controls)
    // ------------------------------------------------------------
    // These six controls map directly to A0-A5 / D15-D20 in order.
    // Keeping this explicit and in-order makes future hardware debugging easier.
    knob_ctrl_decay.Init(        hw.adc.GetPtr(0), cb_rate);  // KNOB 1: Decay
    knob_ctrl_mix.Init(          hw.adc.GetPtr(1), cb_rate);  // KNOB 2: Mix
    knob_ctrl_damp.Init(         hw.adc.GetPtr(2), cb_rate);  // KNOB 3: Damp
    knob_ctrl_shimmer.Init(      hw.adc.GetPtr(3), cb_rate);  // KNOB 4: Shimmer
    knob_ctrl_shimmer_tone.Init( hw.adc.GetPtr(4), cb_rate);  // KNOB 5: Shimmer Tone
    knob_ctrl_detune.Init(       hw.adc.GetPtr(5), cb_rate);  // KNOB 6: Detune

    // ------------------------------------------------------------
    // Parameter mapping initialization (normalized control -> DSP range)
    // ------------------------------------------------------------
    // NOTE:
    // - Parameter::Init signature is (AnalogControl input, min, max, curve)
    // - We keep the same ranges and curves as the original Venus implementation.
    decay.Init(       knob_ctrl_decay,        0.0f,  1.0f,  Parameter::LINEAR);
    mix.Init(         knob_ctrl_mix,          0.0f,  1.0f,  Parameter::LINEAR);
    damp.Init(        knob_ctrl_damp,         0.0f,  1.0f,  Parameter::EXPONENTIAL);
    shimmer.Init(     knob_ctrl_shimmer,      0.0f,  0.1f,  Parameter::LINEAR);
    shimmer_tone.Init(knob_ctrl_shimmer_tone, 0.0f,  0.3f,  Parameter::LINEAR);
    detune.Init(      knob_ctrl_detune,      -0.15f, 0.15f, Parameter::LINEAR);

    // --- LED init ---
    led1.Init(seed::D22, false, cb_rate);
    led2.Init(seed::D23, false, cb_rate);
    led1.Set(0.0f);  // bypass = on, LED off
    led2.Set(0.0f);
    led1.Update();
    led2.Update();

    // --- Reverb energy buffer clear ---
    for (size_t i = 0; i < N / 2; i++)
        reverb_energy[i] = 0.0f;

    // --- FFT / STFT init ---
    fft  = new ShyFFT<float, N, RotationPhasor>();
    fft->Init();
    stft = new Fourier<float, N>(reverb, fft, &hann, laps, in_buf, middle, out_buf);

    // --- LoFi DSP init (reverb_mode hardcoded to 1, but init anyway) ---
    samplerateReducer.Init();
    samplerateReducer.SetFreq(0.3f);
    lowpass.Init(samplerate);
    lowpass.SetFreq(8000.0f);

    // --- Drift oscillators (drift_mode hardcoded to 1 = no drift) ---
    drift_osc.Init(samplerate);   drift_osc.SetAmp(1.0f);
    drift_osc2.Init(samplerate);  drift_osc2.SetAmp(1.0f);
    drift_osc3.Init(samplerate);  drift_osc3.SetAmp(1.0f);
    drift_osc4.Init(samplerate);  drift_osc4.SetAmp(1.0f);

    // --- Initial DSP parameter values ---
    vdecay        = 10.0f;
    vmix          = 0.5f;
    vdamp         = 0.1f;
    vshimmer      = 0.0f;
    vshimmer_tone = 0.0f;
    vdetune       = 0.0f;

    hw.PrintLine("DSP init OK. Starting audio.");

    // --- Start ---
    hw.adc.Start();
    hw.StartAudio(AudioCallback);

    hw.PrintLine("Audio started. Marigold Step 1 running.");

    while (1)
    {
        System::DelayUs(100);
    }

    delete stft;
    delete fft;
}
