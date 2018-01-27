// Some really gross and sloppy high-speed analog read test code.
// Largely stolen from http://yaab-arduino.blogspot.co.uk/2015/02/fast-sampling-from-analog-input.html
// Another useful ref: https://meettechniek.info/embedded/arduino-analog.html
#include <SPI.h>
#include "asp_SPIFlash.h"

volatile byte channel = 0;
volatile byte tmpADMUX;

uint32_t t, t0, t2;

struct sample_page
{
  uint16_t sample[128];
};

const uint8_t FLASHMEM_CS_PIN = 6;
const uint16_t SAMPLES_PER_PAGE = 128;
const uint16_t PAGES_TO_WRITE = 128; // maybe make this at most 256, so it fits in the one 64K block erased at the beginning.
const uint8_t PAGES_IN_BUFFER = 2;
const uint8_t CHANNELS_TO_SAMPLE = 2;

//the stuff below is a really terrible circular buffer
sample_page circular_buffer[2];
volatile uint8_t page_for_sampling = 0;
volatile uint8_t sample_index = 0;
uint32_t current_addr = 0;

volatile byte ready_to_write = 0; // yes, eight flags when we only need two, oh well

bool getting_data = true;

SPIFlashChip flashmem;

void setup()
{
  Serial.begin(115200);
  SPI.begin();
  pinMode(FLASHMEM_CS_PIN, OUTPUT);  

  if (flashmem.begin(FLASHMEM_CS_PIN))
  {
    Serial.println("Flash memory set up successfully."); 
  } else {
    Serial.println("Flash memory error.");    
  }

  Serial.println("Erasing previous data...");
  flashmem.erase_64K_block(0);
  while(flashmem.is_busy());
  Serial.println("Done.");

  ADCSRA = 0;             // clear ADCSRA register
  ADCSRB = 0;             // clear ADCSRB register
  ADMUX |= (0 & 0x07);    // set A0 analog input pin
  ADMUX |= (1 << REFS0);  // set reference voltage

  // sampling rate is [ADC clock] / [prescaler] / [conversion clock cycles]
  // for 3.3 V Arduino Pro Mini ADC clock is 8 MHz and a conversion takes 13 ADC clock cycles
  ADCSRA |= (1 << ADPS2) | (1 << ADPS0);    // 32 prescaler for 19.2 KHz
  //ADCSRA |= (1 << ADPS2);                     // 16 prescaler for 38.5 KHz
  //ADCSRA |= (1 << ADPS1) | (1 << ADPS0);    // 8 prescaler for 76.9 KHz

  ADCSRA |= (1 << ADATE); // enable auto trigger
  ADCSRA |= (1 << ADIE);  // enable interrupts when measurement complete
  ADCSRA |= (1 << ADEN);  // enable ADC
  ADCSRA |= (1 << ADSC);  // start ADC measurements

  t0 = micros();
}

// ♫ ...you get nothing if you wait for it, wait for it, wait-!
ISR(ADC_vect)
{
  if (getting_data)
  {
    circular_buffer[page_for_sampling].sample[sample_index] = ADCL; // need to read ADCL first, according to ATmega328 datasheet
    circular_buffer[page_for_sampling].sample[sample_index]+= ADCH << 8;
  }

  // set next sample to come from next channel.
  channel = (channel+1) % CHANNELS_TO_SAMPLE;
  tmpADMUX = ADMUX & 0b11110000;
  tmpADMUX |= channel;
  ADMUX = tmpADMUX;
  
  sample_index++;

  // if the page is full, mark this page as being ready to write (and then move to the next page)
  if (sample_index == SAMPLES_PER_PAGE)
  {
    sample_index = 0;
    bitSet(ready_to_write, page_for_sampling);
    page_for_sampling = (page_for_sampling + 1) % PAGES_IN_BUFFER;
  }
}
  
void loop()
{
  if (current_addr < PAGES_TO_WRITE*SAMPLES_PER_PAGE*2) 
  {
    for (int i = 0; i<PAGES_IN_BUFFER; i++)
    {
      if (bitRead(ready_to_write, i) == 1)
      {
        flashmem.write_some_data(reinterpret_cast<byte*>(&circular_buffer[i]), sizeof(circular_buffer[i]), current_addr); 
        while(flashmem.is_busy()); // "I'm not standing still/I am lying in wait" (for the chip to finish writing)

        current_addr += SAMPLES_PER_PAGE*2;

        bitClear(ready_to_write, i);
      }
    }    
  }
  else
  {
    getting_data = false;
    Serial.print("Written all pages in ");
    Serial.print(micros()-t0);
    Serial.println(" us. Reading back.");
    current_addr = 0;

    for (int i=0; i < PAGES_TO_WRITE; i++)
    {
      flashmem.read_some_data(reinterpret_cast<byte*>(&circular_buffer[0]), sizeof(circular_buffer[0]), current_addr);
    
      for (int j=0; j < SAMPLES_PER_PAGE; j++)
      {
        Serial.print(circular_buffer[0].sample[j]);
        Serial.print(",");
      }
      Serial.println(".");

      current_addr += SAMPLES_PER_PAGE*2;
      delay(50);
    }

    Serial.println("boop.");
    // done, spin forever
    while(1);
  }  
}
