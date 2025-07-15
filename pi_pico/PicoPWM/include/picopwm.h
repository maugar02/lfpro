#include "hardware/clocks.h"
#include "hardware/pwm.h"
#include "pico/stdlib.h"

class PicoPwm {
   protected:
    const int TOP_MAX = 65534;
    uint8_t pin = 0;
    uint8_t slice_num = 0;
    uint8_t channel = 0;
    uint32_t duty = 0;
    uint32_t div = 0;
    uint16_t top = 0;

   public:
    PicoPwm(uint8_t pin);
    ~PicoPwm();
    int setFrequency(uint32_t frequency);
    void setDuty(uint32_t duty);
    void setDutyPercentage(uint8_t percentage);
    void setInverted(bool inverted_a, bool inverted_b);

    uint8_t getPin();
    uint8_t getSlice();
    uint8_t getChannel();
    void stop();
};