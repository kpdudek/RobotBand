#include <ros.h>
#include <std_msgs/Int32.h>
#include <std_msgs/Float32.h>

int pul_pin = 11, dir_pin = 10;
float pul_per_rev = 800.0;
unsigned long int t, t_prev = -10000000000;
float pi = 3.1415926;

class MIDIElement{
  public:
    char id[20];
    float step_size, rps, freq;
    int at_goal = 0, stopped = 1;
    int pul_pin, dir_pin, dir = 1;
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
      this->freq = pow(2,(float(midi_number)-69.0)/12.0);
      this->pul_delay = 1.0/(this->freq*2.0*pul_per_rev/1000000.0);
    }
};

MIDIElement elements[1] = {MIDIElement(pul_pin,dir_pin, (1.0/pul_per_rev)*2.0*pi, "1")};

///////////////////////////////////////////////////////////////////////////////////////////
// ROS Definitions
///////////////////////////////////////////////////////////////////////////////////////////
ros::NodeHandle  nh;

int note = -1;
std_msgs::Float32 midi_feedback_data;
ros::Publisher midi_feedback_pub("/midi/feedback", &midi_feedback_data);

void midi_stream_callback(const std_msgs::Int32& data){
    note = data.data;
    for (int j=0;j<1;j++){
      elements[j].set_midi_tone(note);
      midi_feedback_data.data = elements[j].freq;
    }
    midi_feedback_pub.publish(&midi_feedback_data);
}
ros::Subscriber<std_msgs::Int32> midi_data_sub("/midi/stream",& midi_stream_callback);

void setup() {
  pinMode(pul_pin,OUTPUT);
  pinMode(dir_pin,OUTPUT);
      
  // Ros subscribers
  nh.initNode();
  nh.subscribe(midi_data_sub);
  nh.advertise(midi_feedback_pub);
}

void loop() {
  t = micros();
  if (note > 0){
    for (int j=0;j<1;j++){
      elements[j].update_position();
    }
  }
//  else if ((t - t_prev > duration)){
//    if (note < 0){
//      delayMicroseconds(duration);
//    }
//    else{
//      for (int j=0;j<1;j++){
//        elements[j].set_midi_tone(note);
//      }
//      idx++;
//      t_prev = t;
//    }
//  }
//  else{
//    for (int j=0;j<1;j++){
//      elements[j].update_position();
//    }
//  }
  nh.spinOnce();
}
