//Tranceiver Module
#include <SPI.h>
#include <nRF24L01.h>
#include <RF24.h>
RF24 radio(7, 8);
String bin101Full = "B01F";
String bin101Done = "B01D";
String bin102Full = "B02F";
String bin102Done = "B02D";
String phrase;
const byte address[6] = "00001";

//Led Strip
#include <FastLED.h>
#define LED_STRIP_ONE 5
#define LED_STRIP_TWO 6
#define NUM_LEDS_ONE 14
#define NUM_LEDS_TWO 29
CRGB ledsStripOne[NUM_LEDS_ONE];
CRGB ledsStripTwo[NUM_LEDS_TWO];
int counterOne;
int counterTwo;

void setup() {
//Tranceiver module setup
Serial.begin(9600);
radio.begin();
radio.openReadingPipe(0, address);
radio.setPALevel(RF24_PA_MIN);
radio.startListening();

//FastLed module setup
FastLED.addLeds<WS2812, LED_STRIP_ONE, GRB>(ledsStripOne, NUM_LEDS_ONE);
FastLED.addLeds<WS2812, LED_STRIP_TWO, GRB>(ledsStripTwo, NUM_LEDS_TWO);
  
}

void loop() {
if(radio.available()){
      
    char text[32] = "";
    radio.read(&text, sizeof(text));    
    phrase = phrase+text;
    Serial.println(phrase);
   
    if(bin101Full.equals(phrase)){
      
    phrase="";    
    
    for(counterTwo=7;counterTwo<14;counterTwo++){
      ledsStripOne[counterTwo] = CRGB(0,255,0);
      FastLED.show();
      delay(25);
      
    }
    
    for(counterOne=13;counterOne<29;counterOne++){
      ledsStripTwo[counterOne] = CRGB(0,255,0);
      FastLED.show();
      delay(25);
      
    }

    counterTwo = 0;
    counterOne = 0;
      
    }else if(bin101Done.equals(phrase)){
    phrase="";

      for(counterTwo=0;counterTwo<14;counterTwo++){
      ledsStripOne[counterTwo] = CRGB(0,0,0);
      FastLED.show();
      delay(25);
      
    }
    
    for(counterOne=13;counterOne<29;counterOne++){
      ledsStripTwo[counterOne] = CRGB(0,0,0);
      FastLED.show();
      delay(25);
      
    }

    counterTwo = 0;
    counterOne = 0;
    
    }else if(bin102Full.equals(phrase)){
    phrase="";

    for(counterOne=0;counterOne<29;counterOne++){
      ledsStripTwo[counterOne] = CRGB(0,0,255);
      FastLED.show();
      delay(25);
      
    }

    counterOne = 0;
      
    }else if(bin102Done.equals(phrase)){
    phrase="";

    for(counterOne=0;counterOne<29;counterOne++){
      ledsStripTwo[counterOne] = CRGB(0,0,0);
      FastLED.show();
      delay(25);
      
    }

    counterOne = 0;
      
    }
    
    
  }  
}
