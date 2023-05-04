# 8x8 rgb displej s a/d prevodnikom a pomocnym i2c lcd displejom

# Schéma

![schéma](schema.png)

# Princíp fungovania

Tento kód v skratke funguje nasledovne:

1. Vytvorí sa 3 rozmerná matica `volatile uint8_t matrix_rgb[PWM_SIZE][NUM_OF_ROWS][BYTES_PER_ROW] `, ktorá obsahuje hodnoty každej jednej LED, ktoré sa upravujú funkciami definovanými v súbore `utils.c`, kde sa špecifikuje riadok, stĺpec, farba a aktualizuje sa hodnota LED v tejto matici.

2. Volaním týchto funkcií na zmeny farby jednotlivých LEDiek, ako aj vyššio-úrovňové funkcie, napríklad na vypisovanie písmen vlastne menia hodnoty v tejto globálnej matici, a o vykresľovanie hodnôt v tejto matici sa stará interrupt vyvolaný časovačom pri zhode s hodnotou v makre `OCR_VAL`. Čím je táto hodnota vyššia, tým je nižšia obnovovacia frekvencia. 

3. Táto ISR vyvolaná časovačom zapíše do posuvných registrov cez SPI hodnoty jednotlivých LED, vždy len pre jeden riadok a jednu pwm úroveň.

4. Potom inkrementuje PWM úroveň až kým nedosiahne hodnoty PWM_MAX a pri ďalšom volaní ISR vykreslí hodnotu LED ďalšieho PWM bitu v poradí. Týmto spôsobom vieme dosiahnuť efekt viacerých farieb, než len siedmych, keby sme túto funkcionalitu nemali, ale na úkor rýchlosti obnovovania obrazu, čo môže byť pri veľkých displejoch problém. Pri tomto displeji však nie, pretože je dostatočné malý a kód dostatočne vyladený a rýchly.

5. Ak sa dosiahne maximálna hodnota PWM, vynuluje sa a inkrementuje sa riadok. Ak riadok dosiahne hodnoty ROWS_MAX, vynuluje sa. Týmto sme dosiahli efekt "viacerých riadkov" aj keď v skutočnosti sa vždy zapisuje len jeden riadok, jednej úrovne PWM. Robí sa to ale tak týchlo, že si to ľudské oko nevšimne. Táto technika sa volá *multiplexing*.

6. Taktiež je tu pripojený potenciometer, s 8 bitovým rozlíšením a/d prevodníka, ktorého hodnota je zapisovaná do globálnej premennej potentiometer_value pre ďalšie spracovanie v kóde.

7. Taktiež na pozadí funguje prerušenie pri prijatí dát z UARTU, ktoré slúži na ovládanie toho, čo sa má na tomto 8x8 displeji zobraziť. V skratke čaká, kým príjme reťazec ukončený '\r', ktorý by mal byť vo formáte `"FARBA_PISMENA,PISMENO,FARBA_POZADIA\r"`. Po prijatí tohto reťazca vykreslí požadované písmeno, v požadovaných farbách. Ak farby nepozná, tak nastaví hodnotu `BLACK`

8. Na i2c displeji sa pri každom prijatí dát z USARTU vykreslia aktuálne farby, znak a hodnota potenciometra.



# Použité technológie
- práca s I/O, napríklad pri riadení tranzistorov, ktoré poskytujú prívod prúdu do jednotlivých riadkov.
- A/D prevodník pre zaznamenávanie napätia na potenciometri
- I2C pre komunikáciu s displejom
- SPI pre komunikáciu s posuvnými registrami
- timer/counter pre obnovovanie hodnôt na displeji
- UART pre komunikáciu s počítačom

# Súbor `setup.c`

## Popis
Tento súbor obsahuje implementáciu funkcií potrebných na nastavenie mikrokontroléru a jeho periférií pre riadenie 8x8 RGB LED displeja. 

## Použitie
Funkcie v tomto súbore sú určené na jednorázové inicializovanie mikrokontroléru a jeho periférií pred použitím displeja. Pre použitie displeja sa potom musia zavolať ďalšie funkcie, ktoré budú riadiť jeho správanie, definované v súbore `utils.c`.

## Funkcie
### `inline void setup_timer(void)`
Táto funkcia nastavuje Timer/Counter1 pre obnovenie stĺpcov LED displeja. Používa sa v móde CTC (Clear Timer on Compare) s prescalerom 64, čo znamená, že každý odpočítaný časový interval je 4 us. Po každom odpočítaní sa zavolá prerušenie. Hodnota, na ktorú sa porovnáva Timer/Counter1, sa nastavuje pomocou premennej `OCR_VAL`.

#### Vstupné parametre
- Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `inline void setup_SPI(void)`
Táto funkcia nastavuje perifériu SPI (Serial Peripheral Interface) ako master a nastavuje jej rýchlosť na polovicu frekvencie taktu.

#### Vstupné parametre
- Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `inline void setup_pins(void)`
Táto funkcia nastavuje piny mikrokontroléru, ktoré sú prepojené s LED displejom, ako výstupné piny. Taktiež inicializuje hodnoty pinov, ktoré sú následne využívané pri riadení displeja.

#### Vstupné parametre
- Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `inline void setup_ADC(void)`
Táto funkcia nastavuje perifériu ADC (Analog-to-Digital Converter), ktorá sa používa na čítanie analógových hodnôt z A/D prevodníka. Nastavuje sa referenčné napätie na AVcc, prepiná sa vstup na A0 a nastavuje sa hodnota prescalera pre časovač ADC a povolí sa prerušenie pri konverzii.

#### Vstupné parametre
- Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void setup_USART(uint16_t ubrr)`
Táto funkcia nastavuje UART (Universal Asynchronous Receiver/Transmitter), ktorý slúži na komunikáciu medzi mikrokontrolérom a externými zariadeniami. Nastavuje sa rýchlosť prenosu dát podľa premennej ubrr, formát prenosu a povoluje sa prerušenie pri príjme a odosielaní dát.

#### Vstupné parametre
- uint16_t ubrr - nastavuje rýchlosť prenosu uartu. Vieme ju získať prepočtom z baudov vzorcom `ubrr = ((frekvencia_procesora / (16UL * rychlost_v_baudoch)) - 1)`

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu





# Súbor `utils.c`

## Popis

Súbor `utils.c` obsahuje implementácie niektorých pomocných funkcií pre prácu s RGB LED displejom a niektorými ďalšími perifériami, ktoré sa používajú v mikrokontroléri Atmega328p. Konkrétne sa jedná o funkcie na aktualizáciu farieb na RGB LED displeji podľa HSV farbového modelu, na preklad číselnej hodnoty na signál PWM, na komunikáciu pomocou sériového portu USART a podobne.

## Funkcie

### `static inline uint8_t get_pwm_bits(uint8_t value)`

Funkcia `get_pwm_bits` slúži na výpočet počtu bitov pre PWM signál, ktorý sa používa na riadenie jasu LED diód. Na vstupe očakáva hodnotu jasu od 0 do 255. Výstupom je počet bitov pre PWM signál.

#### Vstupné parametre
- uint8_t value - hodnota jasu od 0-255

#### Návratová hodnota
- uint8_t - číslo, kde počet jednotiek v binárnej reprezentácií je úmerný veľkosti vstupu


### `inline void SPI_transfer(uint8_t data)`

Funkcia `SPI_transfer` slúži na prenos dát cez SPI rozhranie. Na vstupe očakáva dáta na prenos.

#### Vstupné parametre
- uint8_t data - bajt určený pre odoslanie po SPI zbernici.

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void write_row_to_spi_rgb(uint8_t pwm_level, uint8_t row)`

Funkcia `write_row_to_spi_rgb` slúži na zápis riadku RGB LED displeja cez SPI rozhranie. Na vstupe očakáva úroveň PWM signálu a číslo riadku, ktorý sa má zapísať.

#### Vstupné parametre
- uint8_t pwm_level - momentálna hodnota PWM, hodnota od 0-PWM_MAX
- uint8_t row - momentálna hodnota riadku led matice od 0-ROW_MAX

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void update_LED_color_r(uint8_t row, uint8_t column, uint8_t red)`

Funkcia `update_LED_color_r` slúži na aktualizáciu farby červenej pre konkrétnu LED diódu na zadanom riadku a stĺpci. Na vstupe očakáva číslo riadku, číslo stĺpca a hodnotu farby červenej.

#### Vstupné parametre
- uint8_t row - riadok LED diódy (od 0)
- uint8_t column - stĺpec LED diódy (od 0)
- uint8_t red - hodnota červenej od 0-255

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu


### `void update_LED_color_g(uint8_t row, uint8_t column, uint8_t green)`

Funkcia `update_LED_color_g` slúži na aktualizáciu farby zelenej pre konkrétnu LED diódu na zadanom riadku a stĺpci. Na vstupe očakáva číslo riadku, číslo stĺpca a hodnotu farby zelenej.

#### Vstupné parametre
- uint8_t row - riadok LED diódy (od 0)
- uint8_t column - stĺpec LED diódy (od 0)
- uint8_t green - hodnota zelenej od 0-255

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void update_LED_color_b(uint8_t row, uint8_t column, uint8_t blue)`

Funkcia `update_LED_color_b` slúži na aktualizáciu farby modrej pre konkrétnu LED diódu na zadanom riadku a stĺpci. Na vstupe očakáva číslo riadku, číslo stĺpca a hodnotu farby modrej.

#### Vstupné parametre
Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `inline void update_row_rgb(uint8_t row, uint8_t red, uint8_t green, uint8_t blue)`

Funkcia `update_row_rgb` slúži na aktualizáciu farieb pre všetky LED diódy v zadanom riadku RGB LED displeja. Na vstupe očakáva číslo riadku, hodnotu farby červenej, zelenej a modrej.

#### Vstupné parametre
- uint8_t row - riadok LED diódy (od 0)
- uint8_t red - hodnota červenej od 0-255
- uint8_t green - hodnota zelenej od 0-255
- uint8_t blue - hodnota modrej od 0-255

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `inline void update_col_rgb(uint8_t col, uint8_t red, uint8_t green, uint8_t blue)`

Funkcia `update_col_rgb` slúži na aktualizáciu farieb pre všetky LED diódy v zadanom stĺpci RGB LED displeja. Na vstupe očakáva číslo stĺpca, hodnotu farby červenej, zelenej a modrej.

#### Vstupné parametre
- uint8_t col - stĺpec LED diódy (od 0)
- uint8_t red - hodnota červenej od 0-255
- uint8_t green - hodnota zelenej od 0-255
- uint8_t blue - hodnota modrej od 0-255

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `inline void update_matrix_rgb(uint8_t red, uint8_t green, uint8_t blue)`

Funkcia `update_matrix_rgb` slúži na aktualizáciu farieb pre všetky LED diódy na RGB LED displeji. Na vstupe očakáva hodnotu farby červenej, zelenej a modrej.

#### Vstupné parametre
Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
Funkcia nemá návratovú hodnotu

### `inline void update_LED_color_rgb(uint8_t row, uint8_t column, uint8_t red, uint8_t green, uint8_t blue)`

Funkcia `update_LED_color_rgb` slúži na aktualizáciu farby pre konkrétnu LED diódu na zadanom riadku a stĺpci RGB LED displeja. Na vstupe očakáva číslo riadku, číslo stĺpca a hodnotu farby červenej, zelenej a modrej.

#### Vstupné parametre
Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
Funkcia nemá návratovú hodnotu

### `void update_LED_color_hsv(uint8_t row, uint8_t column, uint8_t hue, uint8_t saturation, uint8_t value)`

Funkcia `update_LED_color_hsv` slúži na aktualizáciu farby pre konkrétnu LED diódu na zadanom riadku a stĺpci RGB LED displeja. Na vstupe očakáva číslo riadku, číslo stĺpca a hodnotu farebného tónu (hue), nasýtenia (saturation) a jasu (value) podľa HSV farbového modelu.

#### Vstupné parametre
- uint8_t row - riadok LED diódy (od 0)
- uint8_t column - stĺpec LED diódy (od 0)
- uint8_t hue - farebný tón (0-255)
- uint8_t saturation - saturácia (0-255)
- uint8_t value - jas (0-255)

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void update_row_hsv(uint8_t row, uint8_t hue, uint8_t saturation, uint8_t value)`

Funkcia `update_row_hsv` slúži na aktualizáciu farieb pre všetky LED diódy v zadanom riadku RGB LED displeja podľa HSV farbového modelu. Na vstupe očakáva číslo riadku a hodnotu farebného tónu, nasýtenia a jasu.

#### Vstupné parametre
- uint8_t row - riadok LED diódy (od 0)
- uint8_t hue - farebný tón (0-255)
- uint8_t saturation - saturácia (0-255)
- uint8_t value - jas (0-255)

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void update_col_hsv(uint8_t col, uint8_t hue, uint8_t saturation, uint8_t value)`

Funkcia `update_col_hsv` slúži na aktualizáciu farieb pre všetky LED diódy v zadanom stĺpci RGB LED displeja podľa HSV farbového modelu. Na vstupe očakáva číslo stĺpca a hodnotu farebného tónu, nasýtenia a jasu.

#### Vstupné parametre
- uint8_t col - stĺpec LED diódy (od 0)
- uint8_t hue - farebný tón (0-255)
- uint8_t saturation - saturácia (0-255)
- uint8_t value - jas (0-255)

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void update_matrix_hsv(uint8_t hue, uint8_t saturation, uint8_t value)`

Funkcia `update_matrix_hsv` slúži na aktualizáciu farieb pre všetky LED diódy na RGB LED displeji podľa HSV farbového modelu. Na vstupe očakáva hodnotu farebného tónu, nasýtenia a jasu.

#### Vstupné parametre
- uint8_t hue - farebný tón (0-255)
- uint8_t saturation - saturácia (0-255)
- uint8_t value - jas (0-255)

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

## Súbor `bitmasks.c`

## Popis

Súbor `bitmasks.c` obsahuje bitmasky pre jednotlivé písmená, ktoré vie vypísať na RGB displej, ako aj funkciu na napísanie týchto písmen.

## Funkcie

### `uint8_t* get_letter_bits(char letter)`

Funkcia vráti bitmasku pre požadovaný znak.

#### Vstupné parametre
- Funkcia nemá žiadne vstupné parametre

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

### `void matrix_write_letter(char letter,const char* background_colour,const char* letter_colour)`

Funkcia najprv získa bitmasku pre požadované písmeno pomocou funkcie `get_letter_bits()`. Následne sa pre každý riadok a stĺpec tejto bitmasky preiteruje a zobrazia sa príslušné pixely na LED matici. Ak je daný bit nastavený, pixel sa zobrazí farbou písmena. Ak nie, zobrazí sa farbou pozadia. Farby sa prevádzajú z reťazca (napr. RED, BLUE, PURPLE...) na RGB pomocou funkcie `get_colour()`, definovanej v `colours.h`.

# colours.c

## Popis
Tento súbor obsahuje deklaráciu statického poľa štruktúr "Colour", ktoré obsahuje názvy farieb a ich RGB hodnoty. Súbor obsahuje aj funkciu "get_colour", ktorá vyhľadáva farbu v poli podľa názvu a vracia jej RGB hodnoty.

## Štruktúra "Colour"

Štruktúra "Colour" obsahuje nasledujúce položky:

- `name` - názov farby
- `red` - hodnota červenej zložky farby (0 až 255)
- `green` - hodnota zelenej zložky farby (0 až 255)
- `blue` - hodnota modrej zložky farby (0 až 255)

## Funkcia "get_colour"

Funkcia "get_colour" vyhľadáva farbu v statickom poli "colours" na základe zadaného názvu farby. Ak farba existuje v poli, funkcia vráti jej hodnoty RGB. Ak farba neexistuje v poli, funkcia vráti hodnoty RGB pre farbu "BLACK" (0, 0, 0).

#### Vstupné parametre
- `char* colour_name` - názov farby, ktorú chceme vyhľadať, ich zoznam je definovaný v odseku "Dostupné farby v poli "colours""

#### Návratová hodnota
- Funkcia nemá návratovú hodnotu

## Statické pole farieb "colours"

Statické pole "colours" obsahuje 42 položiek štruktúry "Colour", ktoré predstavujú rôzne farby s ich RGB hodnotami. Polia "red", "green" a "blue" obsahujú hodnoty od 0 do 255.

### Dostupné farby v poli "colours"

- RED
- GREEN
- BLUE
- YELLOW
- CYAN
- MAGENTA
- ORANGE
- PURPLE
- LIME
- TEAL
- PINK
- OLIVE
- LAVENDER
- BROWN
- BEIGE
- MAROON
- NAVY
- FOREST_GREEN
- GRAY
- DARK_RED
- DARK_GREEN
- DARK_BLUE
- DARK_CYAN
- DARK_MAGENTA
- DARK_YELLOW
- LIGHT_YELLOW
- LIGHT_CYAN
- LIGHT_MAGENTA
- LIGHT_GREEN
- LIGHT_BLUE
- LIGHT_GRAY
- DARK_GRAY
- BEIGE2
- OLIVE_DRAB
- KHAKI
- SANDY_BROWN
- SIENNA
- CORAL
- SALMON
- TOMATO
- ORCHID
- THISTLE
- PLUM
- DEEP_PINK

Ak potrebujeme pridať alebo upraviť farby v poli, musíme aktualizovať definíciu štruktúry "Colour" a pole "colours".

# súbor `main.c`


## Knižnice

Program využíva štandardné knižnice pre jazyk C, ako sú `avr/io.h`, `avr/interrupt.h`, `math.h`, `util/delay.h`, `stdlib.h`, `string.h` a `stdio.h`, ako aj vlastné knižnice `setup.h`, `i2c_lcd.h`, `utils.h`, `animations.h`, `bitmasks.h` a `colours.h`.

## Globálne premenné

- `potentiometer_value` - premenná typu `uint8_t`, ktorá obsahuje hodnotu prevodníka A/D. Je aktualizovaná v interrupt service routine pre A/D prevodník.

- `matrix_rgb` - globálny 3D array, ktorý obsahuje RGB hodnoty každého LED na matici LED displeja, ako aj hodnoty PWM. Má tri rozmery: veľkosť PWM, počet riadkov a počet bajtov na riadok. Do tohto arrayu nezapisujeme priamo, ale cez funkcie, definované v `utils.c`, ktoré prerátavajú hodnoty riadkov, stĺpcov, jasu a podobne a zapisujú ich do tohto arrayu.

- `receiving_str` - premenná typu `uint8_t`, ktorá sa používa v interrupt service routine pre USART na indikovanie, či sa práve prijíma reťazec.

- `received_str` - retazec, ktoý sa prijal

## Interrupt Service Routines

- `TIMER1_COMPA_vect` - táto funkcia sa spúšťa pri príchode signálu z časovača TIMER1. Aktualizuje sa hodnota premenných `current_row` a `pwm_level`, čím sa ovláda PWM signál pre riadenie jasu LED diód.

- `ADC_vect` - táto funkcia sa spúšťa po dokončení prevodu A/D prevodníka. Premenná `potentiometer_value` sa aktualizuje na hodnotu, ktorú prevodník vráti.

- `USART_RX_vect` - táto funkcia sa spúšťa pri prijatí dát z USART. V tomto prípade slúži na príjem reťazca, ktorý obsahuje farby a znak, ktorý sa má zobraziť na LED matici, vo formáte `"FARBA_PISMENA,PISMENO,FARBA_POZADIA"` a je ukončený znakom '\r'.

## Funkcia `main`

Na začiatku funkcie sa vykonajú inicializačné kroky pre rôzne periférie, ako napríklad nastavenie pinov, SPI rozhrania, časovača, USART rozhrania, I2C rozhrania, LCD displeja a A/D prevodníka. Po inicializácii sa spúšťa cyklus, ktorý sa vykonáva neustále.

V tomto cykle sa neustále kontoluje, či sa práve nepríjma reťazec cez USART rozhranie. Ak áno, tak sa reťazec spracuje a na základe neho sa zobrazí na LED matici a LCD displeji. Reťazec obsahuje informácie o farbe písma, pozadia a písmene, ktoré sa majú zobraziť.

Tieto informácie sa spracúvajú funkciou sscanf, ktorá z reťazca vyextrahuje jednotlivé hodnoty. Potom sa volá funkcia matrix_write_letter, ktorá napíše dané písmeno na maticu LED displeja s farbou písma a pozadia. Okrem toho sa na LCD displeji zobrazia informácie o farbe písma, pozadia a hodnota z A/D prevodníka.



# súbor `i2c_lcd.c`

Tento súbor obsahuje funkcie pre komunikáciu s I2C LCD displejom pomocou PCF8574 I/O expanderu. 

## Funkcie

### `void setup_i2c(void)`
Táto funkcia nastavuje rýchlosť I2C komunikácie.

### `void lcd_send_command(uint8_t cmd)`
Táto funkcia slúži na odosielanie príkazov na LCD displej. Parametrom funkcie je príkaz, ktorý sa má poslať na displej.

### `void lcd_send_byte(uint8_t chr)`
Táto funkcia slúži na odosielanie znakov na LCD displej. Parametrom funkcie je znak, ktorý sa má poslať na displej.

### `void lcd_send_string(uint8_t row, uint8_t col, char *str)`
Táto funkcia slúži na odosielanie reťazcov na LCD displej. Parametrami funkcie sú riadok a stĺpec, kde sa má reťazec zobraziť a samotný reťazec.

### `void lcd_send_char(uint8_t row, uint8_t col, char character);`
Táto funkcia slúži na odosielanie charakterov na LCD displej. Parametrami funkcie sú riadok a stĺpec, kde sa má charakter zobraziť a samotný charakter.

### `void lcd_send_number_at_end(uint8_t row, int number);`
Táto funkcia slúži na odosielanie čísiel na LCD displej. Parametrami funkcie je riadok, kde sa má číslo zobraziť, spolu so samotným číslom.

### `void lcd_clear(void)`
Táto funkcia slúži na vymazanie obsahu displeja.

### `void setup_lcd(void)`
Táto funkcia inicializuje LCD displej. 

## Interné funkcie

### `static void i2c_start(void)`
Táto interná funkcia slúži na odoslanie start bitu pri komunikácii po I2C zbernici.

### `static void i2c_stop(void)`
Táto interná funkcia slúži na odoslanie stop bitu pri komunikácii po I2C zbernici.

### `static void i2c_write(uint8_t data)`
Táto interná funkcia slúži na zápis dát do registra.

### `static uint8_t i2c_receive_ack(void)`
Táto interná funkcia slúži na prijatie ACK po odoslaní dát pri komunikácii po I2C zbernici.

### `static uint8_t i2c_receive_nack(void)`
Táto interná funkcia slúži na prijatie NACK po odoslaní dát pri komunikácii po I2C zbernici.

### `static uint8_t i2c_component (uint8_t address)`
Táto interná funkcia slúži na komunikáciu s konkrétnym komponentom po I2C zbernici. Parametrom funkcie je adresa komponentu.

### `static void i2c_send_byte (char data)`
Táto interná funkcia slúži na odoslanie bajtu po I2C zbernici. Parametrom funkcie je bajt, ktorý sa má odoslať.

### `static void i2c_send_string (uint8_t *datas)`
Táto interná funkcia slúži na odoslanie reťazca po I2C zbernici. Parametrom funkcie je reťazec, ktorý sa má odoslať.
