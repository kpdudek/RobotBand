int pul_pin = 11, dir_pin = 10;
float pul_per_rev = 800.0;
unsigned long int t, t_prev = -10000000000;
float pi = 3.1415926;

class MIDIElement{
  public:
    char id[20];
    float step_size, rps;
    int dir = 1;
    int at_goal = 0, stopped = 1;
    int pul_pin = -1, dir_pin = -1;
    unsigned long int t = 0, t_prev = -1000, t_diff;
    unsigned int spd = 50, pul_delay = 0, pulse_min = 40, pulse_max = 3000;
    float prev_setpoint = -1.0, setpoint = 6.28, angle = 0.0, error = 0.0, tolerance = 0.05;

    MIDIElement(int pul_pin,int dir_pin, float step_size, char* id){
      this->pul_pin = pul_pin;
      this->dir_pin = dir_pin;
      this->step_size = step_size;
      strcpy(this->id,id);
      digitalWrite(this->dir_pin,LOW);
    }
    
    void update_position(void){
        // Send pulse to stepper driver
        digitalWrite(this->pul_pin,HIGH);
        delayMicroseconds(this->pul_delay);
        digitalWrite(this->pul_pin,LOW);
        delayMicroseconds(this->pul_delay);
    }

    void set_frequency(float freq){
        this->pul_delay = 1.0/(freq*2.0*pul_per_rev/1000000.0);
    }
    void set_midi_tone(int midi_number){
      float freq = pow(2,(float(midi_number)-69.0)/12.0);
      this->pul_delay = 1.0/(freq*2.0*pul_per_rev/1000000.0);
    }
};

MIDIElement elements[1] = {MIDIElement(pul_pin,dir_pin, (1.0/pul_per_rev)*2.0*pi, "1")};

int note;
int idx = 0;
int scale[] = {60,62,64,65,67,69,71,72,-1,72,71,69,67,65,64,62,60, 9999};
unsigned long int duration = 200000;

void setup() {
  // put your setup code here, to run once:
  pinMode(pul_pin,OUTPUT);
  pinMode(dir_pin,OUTPUT);
}

void loop() {
  t = micros();
  if ((t - t_prev > duration)){
    note = scale[idx];
    if (note == 9999){
      return;
    }
    if (note < 0){
      delayMicroseconds(duration);
    }
    for (int j=0;j<1;j++){
//      elements[j].set_frequency(note);
      elements[j].set_midi_tone(note);
    }
    idx++;
    t_prev = t;
  }
//  if (idx < (sizeof(scale)/sizeof(int))){
//    for (int j=0;j<1;j++){
//      elements[j].update_position();
//    }
//  }
  for (int j=0;j<1;j++){
    elements[j].update_position();
  }
}
