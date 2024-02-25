const int PIN_LIGHT_SENS = A1;
const int PIN_RELAY_SIG = 2;

const int n_samples = 50;
const float EXP_SMOOTH_FACTOR = 0.5;
const float DEBOUNCE_INTERVAL = 4 * n_samples;

float exp_av = 0.0;
float samples_av = 0.0;
float debounce_timer = 0.0;
bool relay_state = false;
float trigger_thresh = 1023;
bool trigger_fired = false;

void setup() {
  Serial.begin(9600);

  pinMode(PIN_RELAY_SIG, OUTPUT);
  digitalWrite(PIN_RELAY_SIG, LOW);
}

void loop() {

  float sum = 0.0;
  for (int i = 0; i < n_samples; i++) {

    float light_value = float(analogRead(PIN_LIGHT_SENS));
    exp_av = (EXP_SMOOTH_FACTOR * light_value) + (1 - EXP_SMOOTH_FACTOR) * exp_av;

    sum += exp_av;
    delay(1);
  }
  debounce_timer += n_samples;

  float samples_av_new = float(sum) / float(n_samples);
  float change = abs(samples_av_new - samples_av);
  samples_av = samples_av_new;
  
  // real-world sig range = [0,40] (roughly)
  // signal = 3.0, trigger = 60% of signal
  // signal = 10.0, trigger = 30% 
  // limit signal trigger fraction = [5%, 60%]  
  float trigger_frac = samples_av * (-0.0428571429) + 0.728571429;
  // float trigger_frac = samples_av * (-0.0428571429) + 1.728571429;
  trigger_frac = max(trigger_frac, 0.05);
  trigger_frac = min(trigger_frac, 0.6);

  // when signal=0, min thresh=1 
  trigger_thresh = samples_av * trigger_frac;
  trigger_thresh = max(trigger_thresh, 3.5);
        
  if ( (change > trigger_thresh) && (debounce_timer >= DEBOUNCE_INTERVAL) ) {
    trigger_fired = true;
  }

  Serial.print(samples_av, 3);
  Serial.print(",");
  Serial.print(trigger_thresh, 3);
  Serial.print(",");
  Serial.print(change, 3);
  Serial.print(",");  
  Serial.println(trigger_fired);
  
  if (trigger_fired) {
    trigger_fired = false; 
    debounce_timer = 0.0;
    relay_state = !relay_state;

    if (relay_state) {
      digitalWrite(PIN_RELAY_SIG, HIGH);
    } else {
      digitalWrite(PIN_RELAY_SIG, LOW);
    }    
  }

}