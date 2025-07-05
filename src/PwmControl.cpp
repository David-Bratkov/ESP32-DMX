#include "PwmControl.h"

/**
 * @brief Construct a new Pwm Control:: Pwm Control object
 * 
 * @param address defaults to 0x40 if omitted
 * @param frequency default to 500Hz if omitted
 */
PwmControl::PwmControl(uint8_t address, float frequency)
{
    pwm = new Adafruit_PWMServoDriver(address);
    this->frequency = frequency;
}

/**
 * @brief Destroy the Pwm Control:: Pwm Control object.
 * Puts PCA9685 to sleep.
 * 
 */
PwmControl::~PwmControl()
{
    pwm->sleep();
    delete pwm;
}

void PwmControl::begin()
{
    pwm->begin();
    /*
   * In theory the internal oscillator (clock) is 25MHz but it really isn't
   * that precise. You can 'calibrate' this by tweaking this number until
   * you get the PWM update frequency you're expecting!
   * The int.osc. for the PCA9685 chip is a range between about 23-27MHz and
   * is used for calculating things like writeMicroseconds()
   * Analog servos run at ~50 Hz updates, It is important to use an
   * oscilloscope in setting the int.osc frequency for the I2C PCA9685 chip.
   * 1) Attach the oscilloscope to one of the PWM signal pins and ground on
   *    the I2C PCA9685 chip you are setting the value for.
   * 2) Adjust setOscillatorFrequency() until the PWM update frequency is the
   *    expected value (50Hz for most ESCs)
   * Setting the value here is specific to each individual I2C PCA9685 chip and
   * affects the calculations for the PWM update frequency. 
   * Failure to correctly set the int.osc value will cause unexpected PWM results
   */
    pwm->setOscillatorFrequency(27000000);
    pwm->setPWMFreq(frequency);
    delay(50);
}

void PwmControl::SetColor(uint8_t channel, uint8_t red, uint8_t green, uint8_t blue, uint8_t white)
{
    // limit total power
    uint16_t sum = (red + green + blue + white);
    if(sum > MAX_POWER)
    {
        const float ratio = (float)MAX_POWER / (float)sum;
        red *= ratio;
        green *= ratio;
        blue *= ratio;
        white *= ratio;
    }

    // convert to CIE curve levels
    const uint16_t r = cie[red];
    const uint16_t g = cie[green];
    const uint16_t b = cie[blue];
    const uint16_t w = cie[white];

    // store channel data
    const ColorData redChannel = ColorData(RED, r);
    const ColorData greenChannel = ColorData(GREEN, g);
    const ColorData blueChannel = ColorData(BLUE, b);
    const ColorData whiteChannel = ColorData(WHITE, w);

    const ColorData *channels[4];

    channels[0] = &redChannel;
    channels[1] = &greenChannel;
    channels[2] = &blueChannel;
    channels[3] = &whiteChannel;


    // sort channels by value
    uint8_t swaps;
    do
    {
        swaps = 0;
        for (uint8_t i = 0; i < 3; i++)
        {
            if (channels[i]->value > channels[i + 1]->value)
            {
                const ColorData *temp;
                temp = channels[i];
                channels[i] = channels[i + 1];
                channels[i + 1] = temp;
                swaps++;
            }
        }
    } while (swaps != 0);

    // sum of 4 channels  <=    16348
    // max value of uint16_t =  65535

    // const uint16_t sumRGB = r + g + b;
    // const uint16_t sumAll = sumRGB + w;

    uint16_t counter = 0;
    // place 3 of the smallest values starting from the left
    for (uint8_t i = 0; i < 3; i++)
    {
        if (counter + channels[i]->value > 4095)
            counter = 0;
        pwm->setPWM(channel + channels[i]->channel, counter, counter + channels[i]->value);
        // printf("Channel %i, value %i, start %i, stop %i\n", channel + channels[i]->channel, channels[i]->value, counter, counter + channels[i]->value);
        counter += channels[i]->value;
    }
    // place the last (largest) value on the right
    pwm->setPWM(channel + channels[3]->channel, 4095 - channels[3]->value, 4095);
    // printf("Channel %i, value %i, start %i, stop %i\n", channel + channels[3]->channel, channels[3]->value, 4095 - channels[3]->value, 4095);
}

ColorData::ColorData(COLORS channel, uint16_t value)
{
    this->channel = channel;
    this->value = value;
}
