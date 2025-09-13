#define C4 262 //Define the frequency of each note
#define D4 294
#define E4 330
#define F4 349
#define G4 392
#define A4 440
#define B4 494
#define Buzzer 33
int bpm=100;// Beats per minute
int tune[]=    // The frequencies are listed according to the short notation
{
  C4,C4,G4,G4,A4,A4,G4,
  F4,F4,E4,E4,D4,D4,C4,
  G4,G4,F4,F4,E4,E4,D4,
  G4,G4,F4,F4,E4,E4,D4,
};
float durt[]=    //List the beats according to the musical notation
{
  1,1,1,1,1,1,2,
  1,1,1,1,1,1,2,
  1,1,1,1,1,1,2,
  1,1,1,1,1,1,2,
}; 
void setup() {
}
void loop() {
  for(int x=0;x<28;x++) 
  { 
    tone(Buzzer,tune[x]);
    delay(60000/bpm*durt[x]);
    noTone(Buzzer);
  }
}