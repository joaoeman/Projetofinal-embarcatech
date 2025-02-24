#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>
#include "pico/stdlib.h"
#include "pico/binary_info.h"
#include "ssd1306.h"
#include "hardware/i2c.h"
#include "hardware/pio.h"
#include "matriz_leds.h"
#include "hardware/timer.h"
#include <ctype.h> //funcao isdigit
#include <string.h>
#include "hardware/adc.h"
#include "hardware/dma.h"
#include "math.h"

// Pino e canal do microfone no ADC.
#define MIC_CHANNEL 2
#define MIC_PIN (26 + MIC_CHANNEL)
bool repeat = false;
// Parâmetros e macros do ADC.
#define ADC_CLOCK_DIV 96.f
#define SAMPLES 200                                   // Número de amostras que serão feitas do ADC.
#define ADC_ADJUST(x) (x * 3.3f / (1 << 12u) - 1.65f) // Ajuste do valor do ADC para Volts.
#define ADC_MAX 3.3f
#define ADC_STEP (3.3f / 5.f) // Intervalos de volume do microfone.

#define abs(x) ((x < 0) ? (-x) : (x))

// Canal e configurações do DMA 
uint dma_channel;
dma_channel_config dma_cfg;

// Buffer de amostras do ADC.
uint16_t adc_buffer[SAMPLES];

void sample_mic();
float mic_power();
uint8_t get_intensity(float v);

const uint I2C_SDA = 14;
const uint I2C_SCL = 15;
const uint LED_B = 12;
const uint LED_G = 11;
const uint LED_R = 13;
const uint button_A = 5;
const uint button_B = 6;
const uint BUZZER_PIN = 10;

char buffer[50];

static volatile int antb = 0;
static volatile int anta = 0;

static volatile int amarelo = 5;
static volatile int vermelho = 20;
static volatile int verde = 15;

bool red = true;
bool green = false;
bool yellow = false;
bool som = false;
bool buzzer_active1 = false; // Variável para controlar o estado do buzzer
bool buzzer_active2 = false;
bool buzzer_active3 = false;
bool buzzer_active4 = false;

int contador = 0;

// Variável para armazenar o estado anterior do LED
enum EstadoLED { VERMELHO, VERDE, AMARELO };
enum EstadoLED estado_anterior = VERMELHO; // Começa com vermelho

struct render_area frame_area = {
    start_column : 0,
    end_column : ssd1306_width - 1,
    start_page : 0,
    end_page : ssd1306_n_pages - 1
};

PIO pio;
uint sm;

Matriz_leds_config matriz = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz0 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz1 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz2 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz3 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz4 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz5 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz6 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz7 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz8 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz9 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz10 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz11 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz12 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz13 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz14 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    
    Matriz_leds_config matriz15 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    
    Matriz_leds_config matriz16 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz17 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz18 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz19 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config matriz20 = {
        //   Coluna 0         Coluna 1         Coluna 2         Coluna 3         Coluna 4
        // R    G    B      R    G    B      R    G    B      R    G    B      R    G    B
        {{0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 0
        {{0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 1
        {{0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 2
        {{0.5, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.0, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 3
        {{0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}, {0.5, 0.0, 0.0}}, // Linha 4
    };
    Matriz_leds_config *matrizes[] = {&matriz0, &matriz1, &matriz2, &matriz3, &matriz4, &matriz5, &matriz6, &matriz7, &matriz8, &matriz9, &matriz10, &matriz11, &matriz12, &matriz13, &matriz14, &matriz15, &matriz16, &matriz17, &matriz18, &matriz19, &matriz20};

    void inicia()
    {
        adc_init();
        adc_gpio_init(MIC_PIN); // Configura o pino do microfone como entrada analógica
        adc_select_input(0);    // Seleciona o canal ADC0 (GP26)
        // inicia buzzer
        gpio_init(BUZZER_PIN);
        gpio_set_dir(BUZZER_PIN, GPIO_OUT);
    
        i2c_init(i2c1, ssd1306_i2c_clock * 1000);
        gpio_set_function(I2C_SDA, GPIO_FUNC_I2C);
        gpio_set_function(I2C_SCL, GPIO_FUNC_I2C);
        gpio_pull_up(I2C_SDA);
        gpio_pull_up(I2C_SCL);
    
        // inicializacao dos leds e botoes
        gpio_init(LED_G);
        gpio_set_dir(LED_G, GPIO_OUT);
    
        gpio_init(LED_B);
        gpio_set_dir(LED_B, GPIO_OUT);
    
        gpio_init(LED_R);
        gpio_set_dir(LED_R, GPIO_OUT);
    
        gpio_init(button_A);
        gpio_set_dir(button_A, GPIO_IN);
        gpio_pull_up(button_A);
    
        gpio_init(button_B);
        gpio_set_dir(button_B, GPIO_IN);
        gpio_pull_up(button_B);
        ssd1306_init();
    }
    
    // Função buzz
    void buzz(uint8_t BUZZER_PIN, uint16_t freq, uint16_t duracao) {
        int periodo = 1000000 / freq; // Período em microssegundos
        int pulso = periodo / 2;      // Meio período (tempo alto e tempo baixo)
        int cycles = freq * duracao / 1000; // Número de ciclos para a duração desejada
    
        for (int i = 0; i < cycles; i++) {
            gpio_put(BUZZER_PIN, 1); // Liga o buzzer
            sleep_us(pulso);         // Espera meio período
            gpio_put(BUZZER_PIN, 0); // Desliga o buzzer
            sleep_us(pulso);         // Espera meio período
        }
    }
    
    // Toque 1: Curto e agudo
    void toque_curto_agudo(uint8_t BUZZER_PIN) {
        buzz(BUZZER_PIN, 2000, 100); // 2000 Hz por 100 ms
    }
    
    // Toque 2: Longo e grave
    void toque_longo_grave(uint8_t BUZZER_PIN) {
        buzz(BUZZER_PIN, 500, 500); // 500 Hz por 500 ms
    }
    
    // Toque 3: Intermitente
    void toque_intermitente(uint8_t BUZZER_PIN) {
        for (int i = 0; i < 3; i++) {
            buzz(BUZZER_PIN, 1000, 50); // 1000 Hz por 50 ms
            sleep_ms(50);               // Pausa de 50 ms entre os toques
        }
    }
    
    // Toque 4: Ascendente
    void toque_ascendente(uint8_t BUZZER_PIN) {
        for (int freq = 500; freq <= 2000; freq += 100) {
            buzz(BUZZER_PIN, freq, 50); // Aumenta a frequência em passos de 100 Hz
        }
    }
    
    void gpio_irq_handler1(uint gpio, uint32_t events)
    {
        uint32_t current = to_ms_since_boot(get_absolute_time());
    
        if (gpio == button_A && current - anta > 200)
        {
            anta = current;
            if (!som)
            {
                contador = (contador + 1) % 4;
                if (contador == 3)
                {
                    red = true;
                    yellow = false;
                    green = false;
                    verde = 15;
                    vermelho = 20;
                    amarelo = 5;
                    contador = 0;
                }
            }
        }
        else if (gpio == button_B && current - antb > 200)
        {
            antb = current;
            if (!som)
            {
                som = !som;
                red = false;
                yellow = false;
                green = true;
                verde = 15;
                vermelho = 20;
                amarelo = 5;
                contador = 0;
            }
            else
                som = !som;
        }
    }
    
    bool repeating_timer_callback(struct repeating_timer *t)
    {
        int char_lido;
        
        if (!som)
        {
            repeat = false;
            if (red)
            {
                if (vermelho > 0)
                {
                    vermelho--;
                    imprimir_desenho(*matrizes[vermelho], pio, sm);
                    memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
                    snprintf(buffer, sizeof(buffer), "sinal fechado");
    
                    gpio_put(LED_R, 1);
                    gpio_put(LED_G, 0);
                    gpio_put(LED_B, 0);
                }
                else
                {
                    vermelho = 20;
                    red = false;
                    green = true;
                    yellow = false;
                    gpio_put(LED_R, 0);
                    gpio_put(LED_G, 1);
                    gpio_put(LED_B, 0);
                        
                    buzzer_active2 = true;
                }
            }
            else if (yellow)
            {
                if (amarelo > 0)
                {
                    amarelo--;
                    imprimir_desenho(*matrizes[amarelo], pio, sm);
    
                    memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
                    snprintf(buffer, sizeof(buffer), "atencao!");
    
                    gpio_put(LED_R, 0);
                    gpio_put(LED_G, 0);
                    gpio_put(LED_B, 1);
                }
                else
                {
                    amarelo = 5;
                    yellow = false;
                    red = true;
                    green = false;
                    gpio_put(LED_R, 1);
                    gpio_put(LED_G, 0);
                    gpio_put(LED_B, 0);
    
                    buzzer_active1 = true;
                }
            }
            else if (green)
            {
                if (verde > 0)
                {
                    verde--;
                    imprimir_desenho(*matrizes[verde], pio, sm);
    
                    memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
                    snprintf(buffer, sizeof(buffer), "sinal aberto");
    
                    gpio_put(LED_R, 0);
                    gpio_put(LED_G, 1);
                    gpio_put(LED_B, 0);
                }
                else
                {
                    verde = 15;
                    green = false;
                    yellow = true;
                    red = false;
                    gpio_put(LED_R, 0);
                    gpio_put(LED_G, 0);
                    gpio_put(LED_B, 1);
    
                    buzzer_active3 = true;
                }
            }
        }
        else
        {
            // Modo "PCD passando"
            imprimir_desenho(matriz, pio, sm);
            memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
            snprintf(buffer, sizeof(buffer), "PCD passando");
    
            gpio_put(LED_R, 1);
            gpio_put(LED_G, 0);
            gpio_put(LED_B, 0);
            buzzer_active4 = true;
        }
        return true;
    }
    
    int main()
    {
        stdio_init_all();
        inicia();
        ssd1306_init();
    
        calculate_render_area_buffer_length(&frame_area);
    
        // Zera o display inteiro
        uint8_t ssd[ssd1306_buffer_length];
        memset(ssd, 0, ssd1306_buffer_length);
        render_on_display(ssd, &frame_area);
    
        pio = pio0;
        sm = configurar_matriz(pio);
    
        struct repeating_timer timer;
        add_repeating_timer_ms(1000, repeating_timer_callback, NULL, &timer);
    
        gpio_set_irq_enabled_with_callback(button_A, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler1);
        gpio_set_irq_enabled_with_callback(button_B, GPIO_IRQ_EDGE_FALL, true, &gpio_irq_handler1);
    
        // configurando adc
        adc_gpio_init(MIC_PIN);
        adc_init();
        adc_select_input(MIC_CHANNEL);
    
        adc_fifo_setup(
            true,  // Habilitar FIFO
            true,  // Habilitar request de dados do DMA
            1,     // Threshold para ativar request DMA é 1 leitura do ADC
            false, // Não usar bit de erro
            false  // Não fazer downscale das amostras para 8-bits, manter 12-bits.
        );
        adc_set_clkdiv(ADC_CLOCK_DIV);
    
        // Tomando posse de canal do DMA.
        dma_channel = dma_claim_unused_channel(true);
    
        // Configurações do DMA.
        dma_cfg = dma_channel_get_default_config(dma_channel);
    
        channel_config_set_transfer_data_size(&dma_cfg, DMA_SIZE_16); // Tamanho da transferência é 16-bits (usamos uint16_t para armazenar valores do ADC)
    
        channel_config_set_read_increment(&dma_cfg, false); // Desabilita incremento do ponteiro de leitura (lemos de um único registrador)
    
        channel_config_set_write_increment(&dma_cfg, true); // Habilita incremento do ponteiro de escrita (escrevemos em um array/buffer)
    
        channel_config_set_dreq(&dma_cfg, DREQ_ADC); // Usamos a requisição de dados do ADC
    
        // Amostragem de teste.
        sample_mic();
    
        while (true)
        {
            ssd1306_draw_string(ssd, 5, 0, buffer);
            render_on_display(ssd, &frame_area);
            // Realiza uma amostragem do microfone.
            sample_mic();
    
            // Pega a potência média da amostragem do microfone.
            float avg = mic_power();
            avg = 2.f * abs(ADC_ADJUST(avg)); // Ajusta para intervalo de 0 a 3.3V. (apenas magnitude, sem sinal)
    
            uint value = get_intensity(avg); // Calcula intensidade a ser mostrada na matriz de LEDs.
            if (value >= 4)
            {
                som = !som;
                if (som)
                {
                    red = false;
                    yellow = false;
                    green = true;
                    verde = 15;
                    vermelho = 20;
                    amarelo = 5;
                    contador = 0;
                }
            }
            if(buzzer_active1){
                toque_curto_agudo(BUZZER_PIN);
                buzzer_active1 = false;
                repeat = false;
            }else if(buzzer_active2){
                toque_longo_grave(BUZZER_PIN);
                buzzer_active2 = false;
                repeat = false;
            }else if(buzzer_active3){
                toque_intermitente(BUZZER_PIN);
                buzzer_active3 = false;
                repeat = false;
            }else if(buzzer_active4){
                if(!repeat){
                    toque_ascendente(BUZZER_PIN);
                    buzzer_active4 = false;
                    repeat = true;
                }
                
            }
        }
    }
    
    /**
     * Realiza as leituras do ADC e armazena os valores no buffer.
     */
    void sample_mic()
    {
        adc_fifo_drain(); // Limpa o FIFO do ADC.
        adc_run(false);   // Desliga o ADC (se estiver ligado) para configurar o DMA.
    
        dma_channel_configure(dma_channel, &dma_cfg,
                              adc_buffer,      // Escreve no buffer.
                              &(adc_hw->fifo), // Lê do ADC.
                              SAMPLES,         // Faz "SAMPLES" amostras.
                              true             // Liga o DMA.
        );
    
        // Liga o ADC e espera acabar a leitura.
        adc_run(true);
        dma_channel_wait_for_finish_blocking(dma_channel);
    
        // Acabou a leitura, desliga o ADC de novo.
        adc_run(false);
    }
    
    /**
     * Calcula a potência média das leituras do ADC. (Valor RMS)
     */
    float mic_power()
    {
        float avg = 0.f;
    
        for (uint i = 0; i < SAMPLES; ++i)
            avg += adc_buffer[i] * adc_buffer[i];
    
        avg /= SAMPLES;
        return sqrt(avg);
    }
    
    /**
     * Calcula a intensidade do volume registrado no microfone, de 0 a 4, usando a tensão.
     */
    uint8_t get_intensity(float v)
    {
        uint count = 0;
    
        while ((v -= ADC_STEP / 20) > 0.f)
            ++count;
    
        return count;
    }