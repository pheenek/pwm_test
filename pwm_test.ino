
#define MAX_OCR_value 0xC8
#define MIN_OCR_value 0x64
#define OCR_VAL_range (MAX_OCR_value - MIN_OCR_value)

#define MAX_angle 180
#define MIN_angle 0
#define HOME_angle 90
#define ANGLE_range (MAX_angle - MIN_angle)

typedef struct Servo_t
{
    uint8_t currentAngle;
}Servo_t;

Servo_t *testServo;

void config_pwm_timer(void);
void servo_set_angle(Servo_t *servo, uint8_t angle);

void setup() {
  Serial.begin(115200);
  config_pwm_timer();
  testServo = (Servo_t*) malloc(sizeof(Servo_t));

  servo_set_angle(testServo, HOME_angle);
}

void loop() {
  servo_set_angle(testServo, MIN_angle);
  _delay_ms(1000);
  servo_set_angle(testServo, HOME_angle);
  _delay_ms(1000);
  servo_set_angle(testServo, MAX_angle);
  _delay_ms(1000);
}

void config_pwm_timer(void)
{
  Serial.println("Configuring PWM timer");
    cli();
    TIMSK4 = 0x00;
    TCCR4A = 0x00;
    TCCR4B = 0x00;

    /**
     * Set waveform generation mode
     * - Fast PWM
     * - TOP ->ICRn
     * - Update OCR at BOTTOM
     * - TOV set at TOP
     * (WGM3:0 = 14 => WGM1:0=2, WGM3:2 = 3
     */
    TCCR4A |= (2 << WGM40);
    TCCR4B |= (3 << WGM42);

    /**
     * Clear OC4C on compare match, set at BOTTOM
     * - Generates a non-inverted PWM signal
     */
    TCCR4A |= (2 << COM4C0);

    OCR4A = 100;

    /**
     * Select clock prescaler
     * - 64 prescaler (Running at clk_I/O = 16MHz, ftimer = 250kHz)
     */
    TCCR4B |= (3 << CS40);

    /**
     * Set the counter TOP
     * ICR4 = 4999
     * - ICR4H = 19 (0x13)
     * - ICR4L = 135 (0x87)
     */
    ICR4H = 0x13;
    ICR4L = 0x87;

    /**
     * Config PWM pin
     */
    DDRH |= (1 << DDH0);

    /**
     * Enable Output compare match interrupt for timer 4 channel C, and timer 4 overflow interrupt
     */
    // TIMSK4 |= (1 << OCIE4C) | (1 << TOIE4);

    Serial.print("TCCR4A: ");
    Serial.print(TCCR4A);
    Serial.print(", TCCR4B: ");
    Serial.print(TCCR4B);
    Serial.print(", ICR4: ");
    Serial.print(ICR4);
    Serial.print(", TIMSK4: ");
    Serial.println(TIMSK4);
    sei();
}

void servo_set_angle(Servo_t *servo, uint8_t angle)
{
    /**
     * Sanitize input values, get rid of invalid values nice and early
     */
    if (angle > MAX_angle || angle < MIN_angle) return;

    /**
     * Calculate new OCR value
     */
    uint16_t ocrValue = ((angle * OCR_VAL_range) / ANGLE_range) + MIN_OCR_value;
    OCR4CH = (uint8_t) ocrValue >> 8;
    OCR4CL = (uint8_t) ocrValue;

    uint16_t actualOCRVal = OCR4C;
    Serial.print("Actual OCR value: ");
    Serial.println(actualOCRVal);

    /**
     * Update the servo object's current angle
     */
    servo->currentAngle = angle;
}


ISR(TIMER4_OVF_vect)
{

}

ISR(TIMER4_COMPC_vect)
{

}
