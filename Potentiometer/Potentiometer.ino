// Noise filtering
  bool EMA = true; // Exponential Movement Average
  bool ignoreLSB = true; // ignore least significant bits

//Global Variables
    
  int potentiometerPin = 3;//potentiometer pin #
  int potValue = 0;    //initialization of potentiometer value, equivalent to EMA Y

  int LSB_TO_IGNORE = 1; // # of least significant bits to ignore

  float EMA_a = 0.6;      //initialization of EMA alpha
  int EMA_S = 0;  //set EMA S for t=1

 
void setup(){
  Serial.begin(9600);

  if(EMA){
    int EMA_S = analogRead(potentiometerPin);  //set EMA S for t=1
  }
}
 
void loop(){
  // printing noise filter configuration
  if(ignoreLSB && EMA){
    Serial.print("EMA + ignoreLSB: ");
  }else if(ignoreLSB){
    Serial.print("ignoreLSB: ");
  }else if(EMA){
    Serial.print("EMA: ");
  }else{
    Serial.print("Unfiltered: ");
  }

  // LSB reduction
  if(ignoreLSB){
    potValue = analogRead(potentiometerPin) >> LSB_TO_IGNORE;
  }else{
    potValue = analogRead(potentiometerPin);
  }

  // EMA filtering
  if(EMA){
    EMA_S = (EMA_a*potValue) + ((1-EMA_a)*EMA_S);    //run the EMA
    Serial.println(EMA_S + 512); //print digital value to serial... add 512 as map input range is 512-1023
  }
  else{
    Serial.println(potValue + 512);
  }
}