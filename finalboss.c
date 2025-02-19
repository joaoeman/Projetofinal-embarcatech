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
#include "neopixel.c"

const uint I2C_SDA = 14;
const uint I2C_SCL = 15;
const uint LED_B = 12;
const uint LED_G = 11;
const uint LED_R = 13;
const uint button_A = 5;
const uint button_B = 6;
const uint MIC_PIN = 26;

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

int contador = 0;

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

bool repeating_timer_callback(struct repeating_timer *t) {
    int char_lido;

    if (!som) {
        if (red) {
            if (vermelho > 0) {
                vermelho--;
                imprimir_desenho(*matrizes[vermelho], pio, sm);
                memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
                snprintf(buffer, sizeof(buffer), "sinal fechado");

                gpio_put(LED_R, 1);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 0);
            } else {
                vermelho = 20;
                red = false;
                green = true;
                yellow = false;
                gpio_put(LED_R, 0);
                gpio_put(LED_G, 1);
                gpio_put(LED_B, 0);
            }
        } else if (yellow) {
            if (amarelo > 0) {
                amarelo--;
                imprimir_desenho(*matrizes[amarelo], pio, sm);

                memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
                snprintf(buffer, sizeof(buffer), "atencao!");

                gpio_put(LED_R, 0);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 1);
            } else {
                amarelo = 5;
                yellow = false;
                red = true;
                green = false;
                gpio_put(LED_R, 1);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 0);
            }
        } else if (green) {
            if (verde > 0) {
                verde--;
                imprimir_desenho(*matrizes[verde], pio, sm);

                memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
                snprintf(buffer, sizeof(buffer), "sinal aberto");

                gpio_put(LED_R, 0);
                gpio_put(LED_G, 1);
                gpio_put(LED_B, 0);
            } else {
                verde = 15;
                green = false;
                yellow = true;
                red = false;
                gpio_put(LED_R, 0);
                gpio_put(LED_G, 0);
                gpio_put(LED_B, 1);
            }
        }
    } else {
        // Modo "PCD passando"
        imprimir_desenho(matriz, pio, sm);
        memset(buffer, 0, sizeof(buffer)); // Limpa o buffer
        snprintf(buffer, sizeof(buffer), "PCD passando");

        gpio_put(LED_R, 1);
        gpio_put(LED_G, 0);
        gpio_put(LED_B, 0);
    }
    return true;
}

int main() {
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

    while (true) {
        uint16_t value = adc_read();
        ssd1306_draw_string(ssd, 5, 0, buffer);
        render_on_display(ssd, &frame_area);

        if (value > 3000) {
            som = !som;
            if (som) {
                red = false;
                yellow = false;
                green = true;
                verde = 15;
                vermelho = 20;
                amarelo = 5;
                contador = 0;
            }
        }
    }
}