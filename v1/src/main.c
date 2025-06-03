#define _BV(bit) (1 << (bit))
#define _NBV(bit) ~(1 << (bit))
#define bit_is_set(sfr, bit)   ((sfr) & (1 << (bit)))

#define sbi(reg, bit) (reg |= _BV(bit))
#define cbi(reg, bit) (reg &= _NBV(bit))

#define cli() __asm__ __volatile__ ("cli"); // Disattiva gli interrupt Globali 
#define sei() __asm__ __volatile__ ("sei"); // Attiva gli interrupt Globali 

// COSTANTI
#define END_BUTTON_PIN 2 // Ingresso del bottone per la fine del campionamento
#define EXTERNAL_TRIGGER_PIN 2 // Porta utilizzata per il trigger esterno
#define SAMPLE_PIN 3 // Porta utilizzata per il sampling
#define LED_PIN 4
#define START_BUTTON_PIN 3
#define F_CPU 16000000UL // Frequenza dell'oscillatore della CPU (trovata nei datasheet)
#define TIMER1_SAMPLING_PRESCALER 64 // Valore del prescaler per quanto riguarda il sampling del Timer1
#define ADC_PRESCALER 128 // Valore del prescaler dell'ADC
#define MAX_INTERRUPT_FREQ 9500UL // Calcolata in base alla velocità di conversione dell'ADC con il prescaler a 128
#define MAX_SAMPLES 500 // aleatoria per non occupare tutta la memoria 
#define MAX_VOLTAGE 5.0 // Massimo voltaggio di trigger

// VARIABILI GLOBALI
uint16_t sample_index = 0; // Index per la lista dei campioni
volatile uint16_t samples[MAX_SAMPLES]; // Array che contiene i valori campionati
uint16_t current_sample_value = 0; // Variabile di appoggio
boolean timeout_flag = false; // Flag per indicare se è decorso il tempo specificato di inizio ritardato del campionamento
boolean trigger_active = false; // Flag per indicare se il trigger del voltage è stato raggiunto
volatile boolean button_pressed = false; // Flag per indicare se il bottone è stato premuto per la fine del campionamento anticipato
volatile boolean done_sampling = false; // Flag per indicare se il campionamento è terminato
uint16_t trigger_threshold = 0; // Variabile in cui verrà trasformato il valore del trigger da Volt in "digitale" (valore compreso tra 0 e 1023)


// ######## Modificare queste variabili per modificare i dati in input ########
uint16_t num_samples = 500; // Numero di campioni voluti
uint16_t sampling_freq = 100; // Frequenza in Hz (>0)
float trigger_voltage = 2.5; // Trigger in Volt --> da 0V a 5.0V
uint16_t timeout_ms = 0; // Ritardo nell'avvio del campionamento in ms
uint8_t trigger_source = 0; // Sorgente di trigger (0 = bottone, 1 = interna, 2 = esterna)
// ############################################################################

void setup() {
  Serial.begin(9600);
  cli();  // Disattiva gli interrupt globali

  // controlla la massima frequenza di campionamento
  if(sampling_freq > MAX_INTERRUPT_FREQ){
    sampling_freq = MAX_INTERRUPT_FREQ;
  }
  // controlla il massimo numero di samples 
  if(num_samples > MAX_SAMPLES){
    num_samples = MAX_SAMPLES;
  }
  // controlla il voltaggio
  if(trigger_voltage > MAX_VOLTAGE){
    trigger_voltage = MAX_VOLTAGE;
  }
  trigger_threshold = (uint16_t)((trigger_voltage / 5.0) * 1023); // Valore da V --> a valore ADC (scala 0-1023)

  cbi(DDRC, SAMPLE_PIN); // Configura il pin dove avverrà il campionamento come ingresso
  cbi(DDRC, EXTERNAL_TRIGGER_PIN); // Configura il pin di trigger come ingresso
  sbi(DDRD, LED_PIN); // Configura il pin D4 come output
  cbi(PORTD, LED_PIN); // Spegne il led
  
  cbi(DDRD, END_BUTTON_PIN); // Configura il pin del bottone per terminare il campionamento come ingresso 
  cbi(DDRD, START_BUTTON_PIN); // Configura il pin del bottone per l'avvio di campionamento come ingresso 
  sbi(PORTD, END_BUTTON_PIN); // Attiva la resistenza pull-up
  sbi(PORTD, START_BUTTON_PIN);
  
  BUTTON_setup(); // Setup per il bottone
  ADC_setup(); // Setup per l'ADC
  TIMER1_sampling_setup(); // Setup per il timer

  // Se il segnale di trigger è esterno
  if(trigger_source == 2){
    // Imposta il campionamento sul pin di trigger esterno
    ADMUX = (ADMUX & 0xF0) | EXTERNAL_TRIGGER_PIN; 
  }else{
    // Imposta il campionamento sul pin di campionamento stesso
    ADMUX = (ADMUX & 0xF0) | SAMPLE_PIN; 
  }
  // Se non c'è nessun delay allora si mette la flag a true
  if(timeout_ms == 0){
    timeout_flag = true;
  }

  sei();  // Riattiva gli interrupt globali
}

void loop() {
  
  // Se il trigger non è stato attivato e la fonte di trigger è il bottone
  if(!trigger_active && trigger_source == 0){
    // Se il bottone viene premuto
    if (!bit_is_set(PIND, START_BUTTON_PIN)) {
      trigger_active = true;  // Attiva il trigger
      Serial.println("Trigger bottone attivato!");
    }
  }
  // Se il trigger non è attivo e la sorgente di trigger è interna
  if(!trigger_active && trigger_source == 1){
    current_sample_value = ADC_sample(); // Campiona
    // Confronta il valore del campione con la soglia di trigger
    if (current_sample_value >= trigger_threshold) {
      trigger_active = true;  // Attiva il trigger
      Serial.println("Trigger interno attivato!");
    }
  }
  // Se il trigger di campionamento non è stato attivato e il trigger è esterno 
  if(!trigger_active && trigger_source == 2){
    current_sample_value = ADC_sample(); // Campiona
    // Quando il valore arriva alla soglia, reimposto il canale da campionare sul pin corretto  
    if(current_sample_value >= trigger_threshold){
      trigger_active = true;  // Attiva il trigger
      ADMUX = (ADMUX & 0xF0) | SAMPLE_PIN; // Imposta il campionamento sul pin di campionamento
      Serial.println("Trigger esterno attivato!");
    }
  }

  // Se non è stato fatto il delay ed il trigger si è attivato
  if(!timeout_flag && trigger_active){
    // Volevo usare la interrupt con il ISR(TIMER1_COMPB_vect), ma su wokwi non funziona
    Serial.println("Inizio delay");
    delay(timeout_ms);
    Serial.println("Fine delay");
    timeout_flag = true;
  }

  // Se il trigger è stato attivato e delay è passato
  if (trigger_active && timeout_flag){
    sbi(TIMSK1, OCIE1A); // Abilita gli interrupt del timer per il confronto con OCR1A
  }
  
  // Se il campionamento è terminato
  if(done_sampling) {
    sbi(PORTD, LED_PIN);
    Serial.println("Campionamento finito");
    print_compressed_values(); // Stampa i dati compressi
    while (1);  // Ferma il programma
  }
}

// Interrupt chiamato dal TIMER1 che alla frequenza desiderata chiama il sampling del segnale analogico
ISR(TIMER1_COMPA_vect) {
  sbi(ADCSRA, ADSC); // Avvia la conversione ADC
}

// Interrupt per la pressione del bottone e quindi la fine del campionamento
ISR(INT0_vect) {
  done_sampling = true; // Modifica la flag quando il bottone viene premuto
  cbi(EIMSK, INT0); // disattiva l'interrupt sul bottone
}

ISR(ADC_vect){
  if (sample_index < num_samples) {
    samples[sample_index++] = ADC;
  }
  else {
    done_sampling = true; // Attiva il flag di fine campionamento
    cbi(ADCSRA, ADEN);  // Spegne l'ADC
    cbi(ADCSRA, ADIE);  // Disabilita l'interrupt ADC
    cbi(ADCSRA, ADATE); // Disabilita l'auto-trigger
    TCCR1B = 0;         // Ferma il Timer1
    cbi(TIMSK1, OCIE1A); // Disabilita l'interrupt del Timer1
  }
}

void TIMER1_sampling_setup(){
  cli(); // Disabilita gli interrupt durante la configurazione

  // Resetta i registi
  TCCR1A = 0;
  TCCR1B = 0;
  TCNT1 = 0;
  
  sbi(TCCR1B, WGM12); // Attiva la modalità CTC (Clear Timer on Compare Match)
  TCCR1B |= _BV(CS11) | _BV(CS10); // Imposta il prescaler a 64 e avvia il contatore 
  OCR1A = (F_CPU / (TIMER1_SAMPLING_PRESCALER * sampling_freq)) - 1; // Calcola il valore di OCR1A in base alla frequenza impostata

  sei(); // Riabilita gli interrupt 
}

void ADC_setup(){
  cli();  // Disabilita gli interrupt durante la configurazione

  // Usa Vcc come riferimento (REFS0 = 1, REFS1 = 0)
  ADMUX = (ADMUX & 0x3F) | _BV(REFS0);
  // Configura il risultato come "right-adjusted" (ADLAR = 0)
  cbi(ADMUX, ADLAR);
  // Imposta il prescaler dell'ADC a 128 (ADPS[2:0] = 111)
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);
  // Abilita il modulo ADC (ADEN = 1)
  sbi(ADCSRA, ADEN);
  sbi(ADCSRA, ADIE); // Abilita l'interrupt ADC (per l'ADC_vect)

  
  sei();// Riabilita gli interrupt 
}

void BUTTON_setup(){
  sbi(EICRA, ISC01); // Configura INT0 per il fronte di discesa (FALLING)
  sbi(EIMSK, INT0); // Abilita l'interrupt per INT0 (pin 2)
}

uint16_t ADC_sample(){
  sbi(ADCSRA, ADSC); // Avvia la conversione ADC
  // Attendi che la conversione sia completata (ADSC diventa 0)
  while (bit_is_set(ADCSRA, ADSC));
  // Ritorna direttamente il valore di ADCL e ADCH combinati
  return ADC;
}

// Funzione che stampa i valori secondo la regola di compressione
void print_compressed_values() {
  if (num_samples == 0) return;

  uint16_t prev_sample = samples[0];
  uint16_t repeat_count = 1;

  for (uint16_t i = 1; i < num_samples; i++) {
    if (samples[i] == prev_sample) {
      repeat_count++;
    } else {
      // Stampa il valore precedente e, se necessario, il numero di ripetizioni
      Serial.print("Valore: 0x");
      Serial.print(prev_sample | 0x000, HEX);
      Serial.print(" ");
      
      if (repeat_count > 1) {
        Serial.print("Ripetizioni: 0x");
        Serial.print(repeat_count | 0x800, HEX); // MSB a 1
        Serial.print(" ");
      }
      
      prev_sample = samples[i];
      repeat_count = 1;
    }
  }

  // Stampa l'ultimo valore e il numero di ripetizioni se necessario
  Serial.print("Valore: 0x");
  Serial.print(prev_sample, HEX);
  Serial.print(" ");
  
  if (repeat_count > 1) {
    Serial.print("Ripetizioni: 0x");
    Serial.print(repeat_count | 0x800, HEX); // MSB a 1
    Serial.print(" ");
  }
  
  Serial.println();
}