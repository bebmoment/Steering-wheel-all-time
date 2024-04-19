#define A 0
#define B 1

volatile byte temp = 0;
volatile byte counter = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Started");

  pinMode(A, INPUT_PULLUP);
  pinMode(B, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(A), A_INTERRUPT, RISING);
  attachInterrupt(digitalPinToInterrupt(B), B_INTERRUPT, RISING);
}

void loop() {
  if (counter != temp) {
    Serial.println(counter);
    temp = counter;
  }

}

void A_INTERRUPT() {
  (LOW==digitalRead(A)) ? counter++ : counter--;
}

void B_INTERRUPT() {
  (LOW==digitalRead(B)) ? counter-- : counter++;
}

