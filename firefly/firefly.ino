#define FASTLED_FORCE_SOFTWARE_PINS
#define SerialUSB Serial

#include "FastLED.h"
#include "SAMDTimerInterrupt.h"  
#include "sbus.h"

#define SBUS_M100     (172)
#define SBUS_P100     (1811)
#define SBUS_INC      ((SBUS_P100 - SBUS_M100)/ 200)
#define SBUS_NUTRAL   ((SBUS_P100 - SBUS_M100) / 2 + SBUS_M100)
#define SBUS_M50      (SBUS_M100 + (SBUS_P100 - SBUS_M100) / 4)
#define SBUS_P50      (SBUS_P100 - (SBUS_P100 - SBUS_M100) / 4)

#define sbus2byte(val) ((val - SBUS_M100) *4 ) / 25

bfs::SbusRx sbus(&Serial1);


#define LED_PIN A0

#define REG_PIN A1

//number of LEDS at each tip
#define WING_TIP_LEDS (3)
//number of down facing LEDS in each wing pannel
#define DOWN_LEDS (49)
#define STAB_LEDS (2)
#define VERT_LEDS (1)

#define NUM_LEDS (2*(WING_TIP_LEDS + DOWN_LEDS )+ STAB_LEDS + VERT_LEDS)

#define RIGHT_TIP_START (WING_TIP_LEDS + 2 * DOWN_LEDS)
#define LEFT_TIP_START (0)

#define LEFT_CENTER_IDX   (WING_TIP_LEDS + DOWN_LEDS-1)
#define RIGHT_CENTER_IDX  (WING_TIP_LEDS + DOWN_LEDS)

#define TOTAL_DOWN_LEDS  (2 * DOWN_LEDS )

#define FIRST_DOWN_WING_LED (WING_TIP_LEDS)
#define LAST_DOWN_WING_LED (WING_TIP_LEDS + TOTAL_DOWN_LEDS -1)

#define FIRST_STAB_LED  (RIGHT_TIP_START + WING_TIP_LEDS)

#define RIGHT_STAB_IDX  (FIRST_STAB_LED + 1)
#define LEFT_STAB_IDX   (FIRST_STAB_LED)
#define TOP_TAIL_IDX    (FIRST_STAB_LED + 2)

#define MODE_INVALID    (-1)

#define ZOOM_LED_SPEED  (3)

#define TAIL_SAT        (150)

enum{FLIGHT_MODE_LAUNCH=0, FLIGHT_MODE_ZOOM, FLIGHT_MODE_GLIDE};
enum{LED_MODE_0=0, LED_MODE_1, LED_MODE_2, LED_MODE_3, LED_MODE_4, LED_MODE_5, LED_MODE_OFF};

CRGB leds[NUM_LEDS];

volatile unsigned int startup, start_rep;

enum{TICK_FLAG=0x0001};

volatile unsigned flags = 0;

SAMDTimer ITimer(TIMER_TCC);

void TickHandler()
{
  flags |= TICK_FLAG;
}

class LED_Pattern {
  public :
    uint8_t hue, sat, brt;
    int flightMode = MODE_INVALID, ledMode = MODE_INVALID;
    bool fm_new = 0;
    unsigned int pos, count;

    void begin(void)
    {
      //TODO : startup things
    }
    
    void update_inputs(std::array<int16_t, bfs::SbusRx::NUM_CH()> chans)
    {
      int tmp;
      int old_fm = flightMode;
      //convert channel value to hue
      hue = sbus2byte(chans[7]);
      //convert channel value to brightness value
      tmp = sbus2byte(chans[6]);
      //saturate values
      if(tmp >= 0xFF)
      {
        brt = 0xFF;
      }
      else if(tmp<=0)
      {
        brt = 0;
      }
      else
      {
        brt = tmp;
      }
      //convert channel value to flight mode
      tmp = chans[5];
      if(tmp < SBUS_M50)
      {
        flightMode = FLIGHT_MODE_GLIDE;
      }
      else if(tmp < SBUS_P50)
      {
        
        flightMode = FLIGHT_MODE_ZOOM;
      }
      else
      {
        flightMode = FLIGHT_MODE_LAUNCH;
      }
      fm_new = flightMode != old_fm;
      //convert channel value to saturation value
      tmp = sbus2byte(chans[4]);
      //saturate values
      if(tmp >= 0xFF)
      {
        sat = 0xFF;
      }
      else if(tmp<=0)
      {
        sat = 0;
      }
      else
      {
        sat = tmp;
      }
      //get LED mode selection
      tmp = chans[3];
      //offset nutral point to zero
      //tmp -= SBUS_NUTRAL;
      if( tmp > (SBUS_NUTRAL + 10*SBUS_INC))
      {
        //modes 3, 4, 5
        tmp = tmp - SBUS_NUTRAL;
        if(tmp < 295)
        {
          ledMode = LED_MODE_3;
        }
        else if(tmp < 525)
        {
          ledMode = LED_MODE_4;
        }
        else
        {
          ledMode = LED_MODE_5;
        }
      }
      else if( tmp < (SBUS_NUTRAL - 10*SBUS_INC))
      {
        //modes 0, 1, 2
        tmp = SBUS_NUTRAL - tmp;
        if(tmp < 295)
        {
          ledMode = LED_MODE_0;
        }
        else if(tmp < 525)
        {
          ledMode = LED_MODE_1;
        }
        else
        {
          ledMode = LED_MODE_2;
        }
      }
      else
      {
        //off mode
        ledMode = LED_MODE_OFF;
      }
    }

    void update_LEDs(void)
    {
      CRGB colorR,colorL,colorFwd,colorAft,tailR,tailL;

      //Set max saturation for tail LEDS
      uint8_t tail_sat = min(sat, TAIL_SAT);

      //set brightness based on value
      FastLED.setBrightness( brt );
  
      colorR   = CHSV(hue      ,sat,255);
      colorFwd = CHSV(hue + 64 ,sat,255);
      colorL   = CHSV(hue + 128,sat,255);
      colorAft = CHSV(hue + 192,tail_sat,255);
      tailL    = CHSV(hue + 128,tail_sat,255);
      tailR    = CHSV(hue      ,tail_sat,255);

      //set regulator enable
      digitalWrite(REG_PIN, (ledMode == LED_MODE_OFF)?LOW:HIGH);

      if(ledMode == LED_MODE_OFF)
      {
         fill_solid(leds, NUM_LEDS, CRGB::Black);
      }
      else if(ledMode == LED_MODE_0)
      {
        fill_solid(leds, DOWN_LEDS + WING_TIP_LEDS, colorL);
        fill_solid(&leds[LAST_DOWN_WING_LED - DOWN_LEDS +1], DOWN_LEDS + WING_TIP_LEDS, colorR);
    
        leds[TOP_TAIL_IDX] = colorAft;
  
        leds[LEFT_STAB_IDX] = tailL;
        leds[RIGHT_STAB_IDX] = tailR;
      }
      else if(ledMode == LED_MODE_1)
      {
        fill_solid(&leds[FIRST_DOWN_WING_LED + 5 + 32], 2 * (DOWN_LEDS - 5 - 32), colorFwd);
        
        fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS + 5, colorL);
        fill_solid(&leds[RIGHT_TIP_START - 5], WING_TIP_LEDS + 5, colorR);

        for(uint8_t i=0;i<32;i++)
        {
          leds[FIRST_DOWN_WING_LED + 5 + i ] = blend(colorL, colorFwd, 8 * i);
          leds[LAST_DOWN_WING_LED - 5 - i] = blend(colorR, colorFwd, 8 * i);
        }
    
        leds[TOP_TAIL_IDX] = colorAft;
  
        leds[LEFT_STAB_IDX] = tailL;
        leds[RIGHT_STAB_IDX] = tailR;
      }
      else if(ledMode == LED_MODE_2)
      {
        
        //set left tip to red
        fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, CRGB(255, 0, 0));
        //set right tip to Green
        fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, CRGB(0, 255, 0));
        //fill solid purple in center
        fill_solid(&leds[FIRST_DOWN_WING_LED], 2 * DOWN_LEDS, CHSV(HUE_AQUA, 130, 255));
        //set stab LEDs to yellow
        fill_solid(&leds[FIRST_STAB_LED], STAB_LEDS, CHSV(HUE_YELLOW, 130, 255));
        // set the on board LED
        fill_rainbow(&leds[FIRST_DOWN_WING_LED], DOWN_LEDS, HUE_RED - 4 , -4);
        fill_rainbow(&leds[FIRST_DOWN_WING_LED + DOWN_LEDS], DOWN_LEDS, (uint8_t)(HUE_GREEN + 4*DOWN_LEDS - 4), -4);
        
        //set tail LEDS
        leds[TOP_TAIL_IDX] = CRGB::White;
  
        leds[LEFT_STAB_IDX] = CRGB::Red;
        leds[RIGHT_STAB_IDX] = CRGB::Green;
      }
      else if(ledMode == LED_MODE_3)
      {
        count = 0;
        pos += 1;
        if(pos>=8)
        {
          pos = 0;
        }
        
        fill_solid(leds, DOWN_LEDS + WING_TIP_LEDS, colorL);
        fill_solid(&leds[LAST_DOWN_WING_LED - DOWN_LEDS +1], DOWN_LEDS + WING_TIP_LEDS, colorR);
    
        leds[TOP_TAIL_IDX] = colorAft;
  
        leds[LEFT_STAB_IDX] = tailL;
        leds[RIGHT_STAB_IDX] = tailR;

        for(int i=0;i<DOWN_LEDS;i++)
        {
          uint8_t val = qadd8(cos8(32*((i-pos)%8)), 16);
          leds[FIRST_DOWN_WING_LED + DOWN_LEDS - i].nscale8(val);
          leds[LAST_DOWN_WING_LED - DOWN_LEDS + i].nscale8(val);
        }
      }
      else if(ledMode == LED_MODE_4)
      {
        //if we just switched flight modes, refresh things
        if(fm_new)
        {
          pos = 0;
          count = 0;
        }

        if(flightMode == FLIGHT_MODE_LAUNCH)
        {
          //launch things

          //clear LEDs 
          FastLED.clear();

          //set wing tip color
          fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, colorL);
          fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, colorR);

          if(pos >= DOWN_LEDS)
          {
            pos = 0;
          }

          if(pos < DOWN_LEDS/2)
          {
            leds[LEFT_CENTER_IDX - 2*pos] = colorL;
            leds[RIGHT_CENTER_IDX + 2*pos] = colorR;
          }
          else
          {
            leds[LEFT_CENTER_IDX - DOWN_LEDS + 2*(pos-DOWN_LEDS/2)] = colorL;
            leds[RIGHT_CENTER_IDX + DOWN_LEDS - 2*(pos-DOWN_LEDS/2)] = colorR;
          }
        }
        else if(flightMode == FLIGHT_MODE_ZOOM)
        {
          //zoom things

          if(pos < DOWN_LEDS/ZOOM_LED_SPEED)
          {
            //clear LEDs 
            FastLED.clear();
  
            //set wing tip color
            fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, colorL);
            fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, colorR);

            for(int i=0;i<(ZOOM_LED_SPEED*pos) && i<DOWN_LEDS;i++)
            {
              leds[LEFT_CENTER_IDX - DOWN_LEDS + i] = colorL;
              leds[RIGHT_CENTER_IDX + DOWN_LEDS - i] = colorR;
            }
          }
          else
          {
            fill_solid(leds, DOWN_LEDS + WING_TIP_LEDS, colorL);
            fill_solid(&leds[LAST_DOWN_WING_LED - DOWN_LEDS +1], DOWN_LEDS + WING_TIP_LEDS, colorR);
          }
        }
        else
        {
          //glide things
          
          if(pos < 5)
          {
            //flash white when entering glide
            fill_solid(leds, NUM_LEDS, CRGB::White);
              
            //set wing tip color
            fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, colorL);
            fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, colorR);
            //force max brightness
            FastLED.setBrightness( 255 );
          }
          else
          {
            fill_solid(leds, DOWN_LEDS + WING_TIP_LEDS, colorL);
            fill_solid(&leds[LAST_DOWN_WING_LED - DOWN_LEDS +1], DOWN_LEDS + WING_TIP_LEDS, colorR);
          }
        }
    
        leds[TOP_TAIL_IDX] = colorAft;
  
        leds[LEFT_STAB_IDX] = tailL;
        leds[RIGHT_STAB_IDX] = tailR;
        
        pos += 1;
      }
      else if(ledMode == LED_MODE_5)
      {
        //if we just switched flight modes, refresh things
        if(fm_new)
        {
          pos = 0;
          count = 0;
        }

        if(flightMode == FLIGHT_MODE_LAUNCH)
        {
          //launch things


          //set wing tip color
          fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, colorL);
          fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, colorR);
          //rainbow colors, cause that looked cool in Josh Finn's video

          uint8_t rnbw_hue = HUE_RED;
          for(int i=0;i<DOWN_LEDS;i++,rnbw_hue+=32)
          {
            leds[FIRST_DOWN_WING_LED + i] = CHSV(rnbw_hue, 255, 255);
            leds[FIRST_DOWN_WING_LED + 2*DOWN_LEDS - 1 - i] = CHSV(rnbw_hue, 255, 255);
          }
          
        }
        else if(flightMode == FLIGHT_MODE_ZOOM)
        {
          //zoom things

          if(pos < DOWN_LEDS/ZOOM_LED_SPEED)
          {
            //clear LEDs 
            FastLED.clear();
          }
          //set wing tip color
          fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, colorL);
          fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, colorR);

          uint8_t rnbw_hue = HUE_RED;
          for(int i=0;i<ZOOM_LED_SPEED*pos && i<DOWN_LEDS;i++,rnbw_hue+=32)
          {
            leds[FIRST_DOWN_WING_LED + i] = CHSV(rnbw_hue, 255, 255);
            leds[FIRST_DOWN_WING_LED + 2*DOWN_LEDS - 1 - i] = CHSV(rnbw_hue, 255, 255);
          }
        }
        else
        {
          //glide things
          
          if(pos < 5)
          {
            //flash white when entering glide
            fill_solid(leds, NUM_LEDS, CRGB::White);
              
            //set wing tip color
            fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, colorL);
            fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, colorR);
            //force max brightness
            FastLED.setBrightness( 255 );
          }
          else
          {
            fill_solid(&leds[FIRST_DOWN_WING_LED + 5 + 32], 2 * (DOWN_LEDS - 5 - 32), colorFwd);
            
            fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS + 5, colorL);
            fill_solid(&leds[RIGHT_TIP_START - 5], WING_TIP_LEDS + 5, colorR);
    
            for(uint8_t i=0;i<32;i++)
            {
              leds[FIRST_DOWN_WING_LED + 5 + i ] = blend(colorL, colorFwd, 8 * i);
              leds[LAST_DOWN_WING_LED - 5 - i] = blend(colorR, colorFwd, 8 * i);
            }
          }
        }
    
        leds[TOP_TAIL_IDX] = colorAft;
  
        leds[LEFT_STAB_IDX] = tailL;
        leds[RIGHT_STAB_IDX] = tailR;
        
        pos += 1;
      }
      else
      {
        CRGB color;
        //set left tip to red
        fill_solid(&leds[LEFT_TIP_START], WING_TIP_LEDS, CRGB(255, 0, 0));
        //set right tip to Green
        fill_solid(&leds[RIGHT_TIP_START], WING_TIP_LEDS, CRGB(0, 255, 0));
    
        leds[TOP_TAIL_IDX] = CRGB::White;
  
        leds[LEFT_STAB_IDX] = CRGB::Red;
        leds[RIGHT_STAB_IDX] = CRGB::Green;
        
        //unknown mode
        for(int i=0;i<DOWN_LEDS;i++)
        {
          color = (i%2)?(CRGB::White):(CRGB::Black);
          leds[FIRST_DOWN_WING_LED + DOWN_LEDS - i - 1] = color;
          leds[LAST_DOWN_WING_LED - DOWN_LEDS + i + 1] = color;
        }
        //force max brightness
        FastLED.setBrightness( 255 );
      }

      //Fix color order on discrete LEDs
      color_swap(leds[RIGHT_STAB_IDX]);
      color_swap(leds[LEFT_STAB_IDX]);
      color_swap(leds[TOP_TAIL_IDX ]);
      //write the data
      FastLED.show();
      //flight mode is no longer new
      fm_new = 0;
    }
};

LED_Pattern pat;

void setup() {
  pinMode(REG_PIN, OUTPUT);
  digitalWrite(REG_PIN, HIGH); //turn regulator on
  
  FastLED.addLeds<WS2812, LED_PIN,GRB>(leds, NUM_LEDS);

  fill_solid(leds, NUM_LEDS, CRGB(255, 255, 255));
  
  FastLED.setBrightness( 255 ); // out of 255
  
  // set 1ms timer interrupt
  ITimer.attachInterruptInterval(10 * 1000, TickHandler);   

  sbus.Begin();

  pat.begin();
}

//swap red and green on a color
void color_swap(CRGB &color)
{
  uint8_t red;
  red = color.red;
  color.red = color.green;
  color.green = red;
}

void loop() {
  unsigned int event;
  std::array<int16_t, bfs::SbusRx::NUM_CH()> chans;

  //capture flags
  event = flags;
  //clear the flags we captured
  flags ^= event;

  if(event & TICK_FLAG)
  {
    pat.update_LEDs();
  }
  //check if we have sbus data
  if (sbus.Read()) {
    chans = sbus.ch();
    pat.update_inputs(chans);
  }
  delay(60);
}
