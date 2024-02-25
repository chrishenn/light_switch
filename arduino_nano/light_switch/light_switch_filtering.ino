#include <CircularBuffer.h>

#include <Filters.h>
#include <Filters/Notch.hpp>
#include <AH/Timing/MillisMicrosTimer.hpp>

// Hardware pins
const int PIN_LIGHT_SENS = A1;
const int PIN_RELAY_SIG = 2;

// constants for smoothing, debounce
const float SMOOTH_FACTOR = 0.75;
const int DEBOUNCE_INTERVAL = 25;

// init reused refs
float sig_value = 0.0;

float sig_smooth = 0.0;
float sig_smooth_deriv = 0.0;
float prev_sig_smooth_value = 0.0;

float prev_sig_value = 0.0;
float sig_deriv = 0.0;
float sig_deriv_abs = 0.0;
int debounce_timer = 0;
bool relay_state = false;

// 
const float TRIGGER_THRESH = 2.5;
bool trigger_fired = false;

//
const double serial_baud = 1e6;

// Sampling frequency (Hz)
const double f_s = 100;
// Notch frequency (Hz)
const double f_c = 60; 
// Normalized notch frequency
const double f_n = 2 * f_c / f_s;

// Sample timer
Timer<micros> timer = std::round(serial_baud / f_s);

// Very simple Finite Impulse Response notch filter
auto filter1 = simpleNotchFIR(f_n);     // fundamental
auto filter2 = simpleNotchFIR(2 * f_n); // second harmonic
auto filter3 = simpleNotchFIR(4 * f_n); // third harmonic

const float V_REF = 3.3;

void setup()
{
    Serial.begin(serial_baud);
    
    pinMode(PIN_RELAY_SIG, OUTPUT);
    digitalWrite(PIN_RELAY_SIG, LOW);
}

float do_sample()
{
    int sig = analogRead(PIN_LIGHT_SENS);
    // return float(sig) / V_REF;
    return float(sig);
}

float filter_60Hz(float sig) {
   
    auto filtered_1 = filter1(sig);
    auto filtered_2 = filter2(filtered_1);
    auto filtered_3 = filter3(filtered_2);
       
    return filtered_3;      
}

void do_thing() {
    
    // sig_deriv = sig_value - prev_sig_value;
    // prev_sig_value = sig_value;

    // sig_smooth = (SMOOTH_FACTOR * sig_value) + (1 - SMOOTH_FACTOR) * sig_smooth;
    // sig_smooth_deriv = sig_smooth - prev_sig_smooth_value;
    // prev_sig_smooth_value = sig_smooth;
}

void print_plotter_bounds() {
    
    Serial.print(15.0);
    Serial.print(",");
    Serial.print(0.0);
    Serial.print("\n");
}

float smooth = 0;

template <typename T>
void sprint(T val) {
    Serial.print(val);
    Serial.print(",");
}

void loop()
{
    if (!timer) return;

    const float sig = do_sample();    
    const float filtered = filter_60Hz(sig);
    // sprint(sig);
    sprint(filtered);
    
    debounce_timer += 1;    

    smooth = smooth * (SMOOTH_FACTOR) + filtered * (1 - SMOOTH_FACTOR);
    sprint(smooth);
    
    const float diff = fabs(filtered - smooth);
    sprint(diff);
    sprint(TRIGGER_THRESH);

    trigger_fired = (diff > TRIGGER_THRESH);
    
    bool do_toggle = trigger_fired && (debounce_timer > DEBOUNCE_INTERVAL);
    if (do_toggle)
    {
        trigger_fired = false;
        debounce_timer = 0;
        relay_state = !relay_state;

        if (relay_state)
        {
            digitalWrite(PIN_RELAY_SIG, HIGH);
        }
        else
        {
            digitalWrite(PIN_RELAY_SIG, LOW);
        }
    }

    sprint(do_toggle);
    
    print_plotter_bounds();    
}