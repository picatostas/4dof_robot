#ifndef AS5040_h
#define AS5040_h

#include <Arduino.h>

class AS5040
{
  public:
    AS5040(uint16_t DataPin, uint16_t ClockPin, uint16_t ChipSelectPin);
    uint32_t encoder_degrees(void);
    uint32_t encoder_value(void);
    uint32_t encoder_error(void);
    struct err_struct{
    	  bool DECn;
      	bool INCn;
      	bool OCF;
      	bool COF;
      	bool LIN; 
    } err_value;
    void read_array_chip(double [],int);
  private:
    uint32_t read_chip(void);

    uint16_t _clock;        // clock pin: output from arduino to as5040
    uint16_t _cs;           // chip select: output
    uint16_t _data;         // data pin: input
    
};

#endif


