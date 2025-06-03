#define _BV(bit) (1 << (bit))
#define _NBV(bit) ~(1 << (bit))
//#define bit_is_set(sfr, bit)   ((sfr) & (1 << (bit)))
#define bit_is_set(sfr, bit)  (((sfr) & (1 << (bit))) != 0)

#define sbi(reg, bit) (reg |= _BV(bit))
#define cbi(reg, bit) (reg &= _NBV(bit))

#define cli() __asm__ __volatile__ ("cli"); // Disattiva gli interrupt Globali 
#define sei() __asm__ __volatile__ ("sei"); // Attiva gli interrupt Globali 

// Gestione flag binarie
#define MFLAG GPIOR0
// gestione di flag tramite GPIOR0 a 8 bit 
#define ADC_DONE 0 // bit 0 di GPIOR0 indica ADC conversion complete
#define TIMEOUT_FLAG 1 // Flag per indicare se è decorso il tempo specificato di inizio ritardato del campionamento
#define TRIGGER_ACTIVE 2 // Flag per indicare se il trigger del voltage è stato raggiunto
#define DONE_SAMPLING 3 // Flag per indicare se il campionamento è terminato

volatile uint16_t aRead;


// COSTANTI
#define BUTTON_PIN 2 // Ingresso del bottone per la fine del campionamento
#define START_BUTTON_PIN 3 // Ingresso del bottone per l'inizio del campionamento
#define EXTERNAL_TRIGGER_PIN 2 // Porta utilizzata per il trigger esterno
#define SAMPLE_PIN 3 // Porta utilizzata per il sampling
#define LED_PIN 4

#define F_CPU 16000000UL // Frequenza dell'oscillatore della CPU (trovata nei datasheet)
#define TIMER1_SAMPLING_PRESCALER 64 // Valore del prescaler per quanto riguarda il sampling del Timer1
#define ADC_PRESCALER 128 // Valore del prescaler dell'ADC
#define MAX_INTERRUPT_FREQ 9500UL // Calcolata in base alla velocità di conversione dell'ADC con il prescaler a 128
#define MAX_SAMPLES 400 // aleatoria per non occupare tutta la memoria 
#define NUM_BYTE (MAX_SAMPLES/4) * 5 // Numero di byte di cui ho bisogno in base ai campioni
#define MAX_VOLTAGE 5.0 // Massimo voltaggio di trigger

// VARIABILI GLOBALI
uint16_t sample_index = 0; // Indica a che punto siamo con i campioni
volatile uint8_t samples[NUM_BYTE]; // Array che contiene i valori campionati
uint16_t current_sample_value = 0; // Variabile di appoggio
uint16_t trigger_threshold = 0; // Variabile in cui verrà trasformato il valore del trigger da Volt in "digitale" (valore compreso tra 0 e 1023)


// ######## Modificare queste variabili per modificare i dati in input ########
uint16_t num_samples = 0; // Numero di campioni voluti
uint16_t sampling_freq = 0; // Frequenza in Hz (>0)
float trigger_voltage = 0; // Trigger in Volt --> da 0V a 5.0V
uint16_t timeout_ms = 0; // Ritardo nell'avvio del campionamento in ms
// ######## FLAGS ########
uint8_t trigger_source = 0; // Sorgente di trigger (0 = esterno, 1 = ingresso analogico riservato, 2 = ingresso da campionare)
// ############################################################################

void setup() {
  Serial.begin(9600);
  
  set_values(); // Chiede i valori all'utente e li imposta

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
  

  BUTTON_setup(); // Setup per il bottone
  ADC_setup(); // Setup per l'ADC
  TIMER1_sampling_setup(); // Setup per il timer

  // se il segnale di trigger è su un ingresso analogico esterno
  if(trigger_source == 1){
    // Imposta il campionamento sul pin di trigger esterno
    ADMUX = (ADMUX & 0xF0) | EXTERNAL_TRIGGER_PIN; 
  }
  // se è sullo stesso segnale da campionare
  else if (trigger_source == 2){
    // Imposta il campionamento sul pin di campionamento stesso
    ADMUX = (ADMUX & 0xF0) | SAMPLE_PIN; 
  }
  // Se non c'è nessun delay allora si mette la flag a true
  if(timeout_ms == 0){
    sbi(MFLAG, TIMEOUT_FLAG);
  }

  sei();  // Riattiva gli interrupt globali
}

void loop() {
  
  // Se il campionamento è terminato
  if(bit_is_set(MFLAG, DONE_SAMPLING)) {
    sbi(PORTD, LED_PIN); // Accendiamo il led di fine campionamento
    Serial.println("Campionamento finito");
    // TODO aggiungere la logica di rappresentazione dei dati compressi
    Serial.println("Vuoi eseguire un altro campionamento con gli stessi dati o vuoi modificarli? (Y, stessi dati -- N, cambia i dati)");
    while (bit_is_set(MFLAG, DONE_SAMPLING)){
      while (Serial.available() == 0);
      char input = Serial.read();
      clearSerialBuffer();  // Svuota il buffer
      if (input == 'Y' || input == 'y') {
        Serial.println("Campionamento con gli stessi dati in avvio");
        cbi(MFLAG, DONE_SAMPLING);
        cbi(PORTD, LED_PIN); // Spegniamo il led di fine campionamento
        //EIMSK |= _BV(INT0) | _BV(INT1); // Abilita gli interrupt esterni su INT0 (D2) e INT1 (D3)
      } else if (input == 'N' || input == 'n') {
        cbi(MFLAG, DONE_SAMPLING);
        cbi(PORTD, LED_PIN); // Spegniamo il led di fine campionamento
        Serial.println("Cambio dei dati");
        //EIMSK |= _BV(INT0) | _BV(INT1); // Abilita gli interrupt esterni su INT0 (D2) e INT1 (D3)
        clearSerialBuffer(); 
        set_values();
      } else {
        clearSerialBuffer();
        Serial.println("Input non valido. Premi Y o N.");
      }
    }
  }

  // Se il trigger non è attivo e la sorgente di trigger è interna
  if(!bit_is_set(MFLAG, TRIGGER_ACTIVE) && trigger_source == 2){
    current_sample_value = ADC_sample(); // Campiona
    // Confronta il valore del campione con la soglia di trigger
    if (current_sample_value >= trigger_threshold) {
      sbi(MFLAG, TRIGGER_ACTIVE);  // Attiva il trigger
      Serial.println("Trigger interno attivato!");
    }
  }
  // Se il trigger di campionamento non è stato attivato e il trigger è esterno 
  if(!bit_is_set(MFLAG, TRIGGER_ACTIVE) && trigger_source == 1){
    current_sample_value = ADC_sample(); // Campiona
    // Quando il valore arriva alla soglia, reimposto il canale da campionare sul pin corretto  
    if(current_sample_value >= trigger_threshold){
      sbi(MFLAG, TRIGGER_ACTIVE);  // Attiva il trigger
      ADMUX = (ADMUX & 0xF0) | SAMPLE_PIN; // Imposta il campionamento sul pin di campionamento
      Serial.println("Trigger esterno attivato!");
    }
  }

  // Se non è stato fatto il delay ed il trigger si è attivato
  if(!bit_is_set(MFLAG, TIMEOUT_FLAG) && bit_is_set(MFLAG, TRIGGER_ACTIVE)){
    // Volevo usare la interrupt con il ISR(TIMER1_COMPB_vect), ma su wokwi non funziona
    Serial.println("Inizio delay");
    delay(timeout_ms);
    Serial.println("Fine delay");
    sbi(MFLAG, TIMEOUT_FLAG);
  }

  // Se il trigger è stato attivato e delay è passato e non abbiamo finito di campionare
  if (bit_is_set(MFLAG, TRIGGER_ACTIVE) && bit_is_set(MFLAG, TIMEOUT_FLAG) && !bit_is_set(MFLAG, DONE_SAMPLING)){
    // se la conversione è stata fatta
    if(bit_is_set(MFLAG, ADC_DONE)){
      pack_ADC(aRead); // inserisce il valore nell'array di valori
      sample_index++; // Incrementa l'indice dei valori attuali
      cbi(MFLAG, ADC_DONE); // pulisce il bit per la prossima conversione
    }
  }
}
// Interrupt chiamato dal TIMER1 che alla frequenza desiderata chiama il sampling del segnale analogico
ISR(TIMER1_COMPB_vect) {
  // se il campionamento è terminato
  if(sample_index > num_samples){
    sbi(MFLAG, DONE_SAMPLING); // Attiva il flag di fine campionamento
    cbi(TIMSK1, OCIE1B); // Disabilita gli interrupt per questa ISR quando il campionamento è finito
  }else{
    sbi(ADCSRA, ADSC); // Avvia la conversione ADC
  }
}

// Quando la conversione ADC è completa
ISR(ADC_vect){
  aRead = ADC; // Aggiorna la variabile del valore campionato
  sbi(MFLAG, ADC_DONE); // Attiva la flag di fine campionamento
}

// Interrupt per la pressione del bottone e quindi la fine del campionamento
ISR(INT0_vect) {
  sbi(MFLAG, DONE_SAMPLING); // Modifica la flag quando il bottone viene premuto
  //cbi(EIMSK, INT0); // disabilita gli interrupt su D2
}

// Interrupt per la pressione del bottone e quindi l'inizio del campionamento
ISR(INT1_vect) {
  // controlla che il trigger source sia effettivamente quello del. bottone prima di attivarlo
  if(trigger_source == 0){
    sbi(MFLAG, TRIGGER_ACTIVE); // Attiva il trigger quando il bottone viene premuto
    //cbi(EIMSK, INT1); // Disabilita gli interrupt su D3
  }
}

void TIMER1_sampling_setup(){
  // Resetta i registi
  TCNT1 = 0;
  
  sbi(TIMSK1, OCIE1B); // Abilita gli interrupt del timer per il confronto con OCR1B

  sbi(TCCR1B, WGM12); // Attiva la modalità CTC (Clear Timer on Compare Match)
  TCCR1B |= _BV(CS11) | _BV(CS10); // Imposta il prescaler a 64 e avvia il contatore 
  OCR1B = (F_CPU / (TIMER1_SAMPLING_PRESCALER * sampling_freq)) - 1; // Calcola il valore di OCR1A in base alla frequenza impostata
}

void ADC_setup(){
  ADMUX = _BV(REFS0); // Usa Vcc come riferimento (REFS0 = 1, REFS1 = 0)
  cbi(ADMUX, ADLAR); // Configura il risultato come "right-adjusted" (ADLAR = 0)
  ADCSRA |= _BV(ADPS2) | _BV(ADPS1) | _BV(ADPS0);   // Imposta il prescaler dell'ADC a 128 (ADPS[2:0] = 111)

  sbi(ADCSRA, ADEN); // Abilita il modulo ADC 
  //sbi(ADCSRA, ADATE); // Abilita il Trigger Automatico, cioè quando viene generato un interrupt nei bit ADTS[0:2] in base al tipo impostato, è come se venisse impostato ADSC = 1
  sbi(ADCSRA, ADIE); // Abilita interrupt su ADC, quando l’ADC termina una conversione, genera un interrupt che esegue una ISR (ADC_vect).
  //ADCSRB = _BV(ADTS2) | _BV(ADTS0); // Trigger su Timer1 Compare Match B (ADTS[2:0] = 101)
}

void BUTTON_setup(){
  // Configura i pin dei bottoni come input con pull-up
    DDRD &= _NBV(PD2) & _NBV(PD3); // PD2 (D2) e PD3 (D3) come input
    PORTD |= _BV(PD2) | _BV(PD3);   // Attiva le pull-up interne

    // Abilita gli interrupt esterni su INT0 (D2) e INT1 (D3)
    EIMSK |= _BV(INT0) | _BV(INT1);  

    // Configura INT0 e INT1 per attivarsi sul fronte di discesa (falling edge)
    EICRA |= _BV(ISC01) | _BV(ISC11);  // ISC01 e ISC11 = 1, ISC00 e ISC10 = 0
}

uint16_t ADC_sample(){
  sbi(ADCSRA, ADSC); // Avvia la conversione ADC
  // Attendi che la conversione sia completata (ADSC diventa 0)
  while (bit_is_set(ADCSRA, ADSC));
  // Ritorna direttamente il valore di ADCL e ADCH combinati
  return ADC;
}

void pack_ADC(uint16_t adcValue) {
    static uint16_t bitBuffer = 0;  // Buffer per accumulare i bit
    static uint8_t bitCount = 0;    // Contatore dei bit accumulati
    static uint16_t index = 0;       // Indice nell'array globale

    bitBuffer |= adcValue << bitCount; // Accumula nel buffer
    bitCount += 10; // Aggiunge 10 bit

    while (bitCount >= 8) { // Estrae i byte completi
        samples[index++] = bitBuffer & 0xFF;
        bitBuffer >>= 8;
        bitCount -= 8;
    }

    // Se l'array è pieno, gestire eventuale sovrascrittura o overflow
    if (index >= NUM_BYTE) {
        index = 0; // Reset o gestione overflow
    }
}

void reset_flags(){
  cbi(MFLAG, TIMEOUT_FLAG);
  cbi(MFLAG, TRIGGER_ACTIVE);
}

void decompress_and_print() {
    uint16_t lastValue = 0xFFF; // Valore iniziale non valido
    uint16_t repeatCount = 0;    // Contatore ripetizioni
    uint16_t sample10 = 0; // Buffer per il valore 
    uint16_t compressedRep = 0; // Contatore delle ripetizioni

    printf("Compressed Data:\n");

    for (uint16_t i = 0; i < MAX_SAMPLES; i++) {
        // Ricostruzione del valore a 10 bit dall'array a 8 bit
        sample10 = ((samples[i * 10 / 8] << 8) | samples[i * 10 / 8 + 1]) >> (6 - (i * 10) % 8);
        sample10 &= 0x3FF;  // Mascheriamo per mantenere solo i 10 bit utili

        if (sample10 == lastValue) {
            repeatCount++;  // Incrementiamo il contatore se il valore si ripete
        } else {
            if (lastValue != 0xFFF) {
                printf("%03X ", lastValue);  // Stampiamo il valore precedente

                if (repeatCount > 1) {
                    compressedRep = 0x800 | (repeatCount & 0x7FF); // Mantiene solo i primi 11 bit
                    printf("%03X ", compressedRep);  // Stampa della compressione
                }
            }
            // Aggiorniamo il nuovo valore e resettiamo il conteggio
            lastValue = sample10;
            repeatCount = 1;
        }
    }
}

void clearSerialBuffer() {
    while (Serial.available() > 0) {
        Serial.read();  // Svuota il buffer
    }
}

void set_values() {
  Serial.println("Inserisci la frequenza di campionamento in Hz (1Hz - 9500Hz):");
  while (Serial.available() == 0);
  sampling_freq = Serial.parseFloat();
  clearSerialBuffer();  // Svuota il buffer
  Serial.println(sampling_freq);

  Serial.println("Inserisci la sorgente del trigger (0 = esterno, 1 = ingresso analogico riservato, 2 = ingresso da campionare):");
  do{
    while (Serial.available() == 0);
    trigger_source = Serial.parseInt();
    clearSerialBuffer();
    if (trigger_source != 0 && trigger_source != 1 && trigger_source != 2){
      Serial.println("Valore non ammesso, riprova");
    }
  }while(trigger_source != 0 && trigger_source != 1 && trigger_source != 2);
  Serial.println(trigger_source);
  
  if(trigger_source != 0){
    Serial.println("Inserisci il livello di trigger (0V - 5V):");
    while (Serial.available() == 0);
    trigger_voltage = Serial.parseFloat();
    clearSerialBuffer();
    Serial.println(trigger_voltage);
  }
  
  Serial.println("Inserisci il ritardo di avvio campionamento in ms (max .....):");
  while (Serial.available() == 0);
  timeout_ms = Serial.parseFloat();
  clearSerialBuffer();
  Serial.println(timeout_ms);

  Serial.println("Inserisci il numero di campioni da registrare (max ....):");
  while (Serial.available() == 0);
  num_samples = Serial.parseInt();
  clearSerialBuffer();
  Serial.println(num_samples);

  Serial.println("Dati inseriti correttamente.");
}