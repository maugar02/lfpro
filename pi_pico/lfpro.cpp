/*  CARRO SEGUIDOR DE LINEA 5 SENSOSRES PID
    FECHA: 11/07/2025
    #MGU2508PICO
    NOTA: ESTE CODIGO SOLO FUNCIONA CON SENSORES INFRAROJOS DIGITALES.
*/

#include "pico/stdlib.h"
#include "picopwm.h"
#include <stdio.h>

using uint = unsigned int;

constexpr uint BOARD_LED(25U);

// PINES SENSORES INFRAROJOS
constexpr uint IR1(17U);
constexpr uint IR2(16U);
constexpr uint IR3(15U);
constexpr uint IR4(14U);
constexpr uint IR5(13U);

// PINES PUENTE H(L298N)
constexpr uint IN1(22U);
constexpr uint IN2(21U);
constexpr uint IN3(20U);
constexpr uint IN4(19U);
constexpr uint ENA(23U);
constexpr uint ENB(18U);

// VELOCIDAD BASE Y MAXIMA
constexpr int BASE_VEL1(120);
constexpr int BASE_VEL2(120);
constexpr int MAX_VEL1(BASE_VEL1+60);
constexpr int MAX_VEL2(BASE_VEL2+60);

// PARAMETROS PROPORCIONAL Y DERIVATIVO
float Kp(0.06f);
float Kd(1.2f);

int P(0);
int D(0);
int error(0);
int last_error(0);

// CONTROLADOR PWM
PicoPwm *pwm_ena(nullptr);
PicoPwm *pwm_enb(nullptr);

inline uint umap(uint x,  uint in_min, uint in_max, uint out_min, uint out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

inline void init_output(uint gpio_num) {
    gpio_init(gpio_num);
    gpio_set_dir(gpio_num, GPIO_OUT);
}

inline void init_input(uint gpio_num) {
    gpio_init(gpio_num);
	gpio_set_dir(gpio_num, GPIO_IN);
}

// MOVER ADELANTE
void MoveForward()
{
	gpio_put(IN1, true);
	gpio_put(IN2, false);
	gpio_put(IN3, true);
	gpio_put(IN4, false);
}

// DETENER
void MoveStop()
{
	gpio_put(IN1, false);
	gpio_put(IN2, false);
	gpio_put(IN3, false);
	gpio_put(IN4, false);
}

void setup()
{

    sleep_ms(100);

    // CONFIGURAR PINES DE LOS IR
    init_input(IR1);
    init_input(IR2);
    init_input(IR3);
    init_input(IR4);
    init_input(IR5);

    // CONFIGURAR PINES DEL PUENTE H
    init_output(IN1);
    init_output(IN2);
    init_output(IN3);
    init_output(IN4);

    // CONGIGURAR PWM
    pwm_ena = new PicoPwm(ENA);
	pwm_enb = new PicoPwm(ENB);

    // Ajusta la frecuencia del PWM a 500Hz
	pwm_ena->setFrequency(500);
	pwm_enb->setFrequency(500);

	pwm_ena->setDuty(0);
	pwm_enb->setDuty(0);

    init_output(BOARD_LED);

    gpio_put(BOARD_LED, true);
    MoveForward();
}


int CalculatePosition(int *values, uint sensor_count)
{
    uint Wsum(0U);
    uint sum(0U);

    for(int i(0); i < sensor_count; i++) {
        Wsum += values[i] * (i + 1)*1000;
        sum += values[i];
    }

    if(sum > 0U && sum != 5U) {
        return static_cast<int> (Wsum / sum);
    }

    return -1;
}

void PIDcontrol() 
{
    int irs[5];
    int irposition(-1);
    int pid(0);
    int ena(0);
    int enb(0);

    irs[0] = static_cast<int> (gpio_get(IR1));
    irs[1] = static_cast<int> (gpio_get(IR2));
    irs[2] = static_cast<int> (gpio_get(IR3));
    irs[3] = static_cast<int> (gpio_get(IR4));
    irs[4] = static_cast<int> (gpio_get(IR5));

    irposition = CalculatePosition(irs, 5U);

    if(irposition != -1) {
        error = 3000 - irposition;
    }

    P = error;
    D = error - last_error;
    last_error = error;
    
    pid = P*Kp + D*Kd;

    ena = BASE_VEL1 + pid;
    enb = BASE_VEL2 - pid;

    ena = (ena > MAX_VEL1) ? MAX_VEL1 : ena;
    enb = (enb > MAX_VEL2) ? MAX_VEL2 : enb;

    ena = (ena < 0) ? 0 : ena;
    enb = (enb < 0) ? 0 : enb;

    pwm_ena->setDuty(umap(ena, 0, 255, 0, 65534));
    pwm_enb->setDuty(umap(enb, 0, 255, 0, 65534));

}

void loop() {
    PIDcontrol();
    sleep_ms(1);
}

int main()
{
    stdio_init_all();
    setup();

    while(true) {
        loop();
    }

    // ESTO NUNCA VA OCURRIR
    return 0;
}
