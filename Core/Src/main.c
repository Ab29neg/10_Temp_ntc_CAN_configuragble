#include "stm32f1xx.h"
#include <math.h>

// ---------- CAN Base ID Configuration ----------
uint8_t high = 0x01;      // Default High byte of CAN base ID
uint8_t low  = 0x23;      // Default Low byte of CAN base ID
uint32_t tim = 9000000;
volatile uint32_t base_id; // Final 11-bit base CAN ID
volatile float B_value = 3614.0f;
volatile float R0_value = 10000.0f;


// ---------- Simple Delay Function ----------
void delay(volatile uint32_t count) {
    while (count--);
}

// ---------- Initialize CAN1 Peripheral and GPIO ----------
void can_filter_init(void) {
    // Use Filter 0 for FIFO1, allow all standard IDs
    CAN1->FMR |= CAN_FMR_FINIT;         // Enter filter init
    CAN1->FM1R |= 1 << 0;               // Identifier mask mode
    CAN1->FS1R |= 1 << 0;               // 32-bit scale
    CAN1->FFA1R |= 1 << 0;              // Assign filter 0 to FIFO1
    CAN1->FA1R |= 1 << 0;               // Activate filter 0

    CAN1->sFilterRegister[0].FR1 = 0x00000000; // Accept all
    CAN1->sFilterRegister[0].FR2 = 0x00000000;

    CAN1->FMR &= ~CAN_FMR_FINIT;        // Exit filter init
}

void can_init(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;
    RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

    // PA12 (TX) as AF Push-Pull
    GPIOA->CRH &= ~(0xF << 16);
    GPIOA->CRH |=  (0xB << 16);

    // PA11 (RX) as Input Floating
    GPIOA->CRH &= ~(0xF << 12);
    GPIOA->CRH |=  (0x4 << 12);

    // CAN init
    CAN1->MCR &= ~CAN_MCR_SLEEP;
    while (CAN1->MSR & CAN_MSR_SLAK);

    CAN1->MCR |= CAN_MCR_INRQ;
    while (!(CAN1->MSR & CAN_MSR_INAK));

    // 500 kbps assuming 36 MHz PCLK1
    CAN1->BTR = 0x001c0000; // TS1=13, TS2=2, SJW=1, Prescaler=4

    CAN1->MCR &= ~CAN_MCR_INRQ;
    while (CAN1->MSR & CAN_MSR_INAK);

    // Initialize CAN filter to accept all standard IDs
      CAN1->FMR |= CAN_FMR_FINIT;  // Enter filter init mode

      CAN1->FA1R |= 1 << 0;        // Activate filter 0
      CAN1->FS1R |= 1 << 0;        // Set filter scale to 32-bit
      CAN1->FM1R &= ~(1 << 0);     // Identifier Mask mode
      CAN1->sFilterRegister[0].FR1 = 0x00000000;  // ID filter
      CAN1->sFilterRegister[0].FR2 = 0x00000000;  // ID mask
      CAN1->FFA1R &= ~(1 << 0);    // Assign filter to FIFO0

      CAN1->FMR &= ~CAN_FMR_FINIT; // Exit filter init mode
}

// ---------- Send a CAN Message ----------
void can_send(uint32_t id, uint8_t* data, uint8_t len) {
    while ((CAN1->TSR & CAN_TSR_TME0) == 0); // Wait for empty mailbox

    // Load standard ID
    CAN1->sTxMailBox[0].TIR = (id << 21);      // 11-bit ID
    CAN1->sTxMailBox[0].TIR &= ~(CAN_TI0R_RTR | CAN_TI0R_IDE); // Data frame, standard ID

    CAN1->sTxMailBox[0].TDTR = len;            // Data Length Code

    // First 4 bytes (low)
    CAN1->sTxMailBox[0].TDLR = (data[3] << 24) | (data[2] << 16) |
                               (data[1] << 8)  | data[0];
    // Next 4 bytes (high)
    CAN1->sTxMailBox[0].TDHR = (data[7] << 24) | (data[6] << 16) |
                               (data[5] << 8)  | data[4];

    CAN1->sTxMailBox[0].TIR |= CAN_TI0R_TXRQ; // Request transmission
}

// ---------- Receive a CAN Message ----------
int can_receive(uint32_t *id, uint8_t *data, uint8_t *len) {
    // Check if there's a message in FIFO0
    if ((CAN1->RF0R & CAN_RF0R_FMP0) == 0)
        return 0; // No message available

    // Get the message ID (Standard ID)
    *id = (CAN1->sFIFOMailBox[0].RIR >> 21) & 0x7FF;

    // Get data length (DLC)
    *len = CAN1->sFIFOMailBox[0].RDTR & 0xF;

    // Read data bytes
    uint32_t rdlr = CAN1->sFIFOMailBox[0].RDLR;
    uint32_t rdhr = CAN1->sFIFOMailBox[0].RDHR;

    for (int i = 0; i < 4; i++) {
        data[i] = (rdlr >> (8 * i)) & 0xFF;
        data[i + 4] = (rdhr >> (8 * i)) & 0xFF;
    }

    // Release the FIFO (make it ready for the next message)
    CAN1->RF0R |= CAN_RF0R_RFOM0;

    return 1; // Message received
}
// ---------- Initialize ADC1 ----------
void adc_init(void) {
    // Enable clocks for ADC1, GPIOA, and GPIOB
    RCC->APB2ENR |= RCC_APB2ENR_ADC1EN | RCC_APB2ENR_IOPAEN | RCC_APB2ENR_IOPBEN;

    // Set PA0–PA7 to analog mode (CRL: 8 pins)
    GPIOA->CRL = 0x00000000;

    // PB0 and PB1 to analog mode (bits 0–7 of GPIOB->CRL)
    GPIOB->CRL &= ~(0xFF); // Clear bits for PB0 and PB1
    // 0x00 = analog input mode for each pin (4 bits per pin)
    // No need to set anything since 0 already = analog input

    // Power on ADC and start calibration
    ADC1->CR2 |= ADC_CR2_ADON;
    for (volatile int i = 0; i < 1000; i++);
    ADC1->CR2 |= ADC_CR2_ADON;
}

// ---------- Read ADC Value for a Given Channel ----------
uint16_t adc_read(uint8_t channel) {
    ADC1->SQR3 = channel;            // Set channel
    ADC1->CR2 |= ADC_CR2_ADON;       // Start conversion
    while (!(ADC1->SR & ADC_SR_EOC));
    return ADC1->DR;
}

// ---------- Convert ADC Value to Integer Temperature (°C) ----------
int16_t get_temp_int(uint16_t adc_val) {
    float Vref = 3.3f;
    float Vadc = (adc_val * Vref) / 4095.0f;

    float R_fixed = 1000.0f;
    float R_ntc = (Vadc * R_fixed) / (Vref - Vadc);

    float T0 = 298.15f;       // 25°C in Kelvin

    float tempK = 1.0f / ((1.0f / T0) + (1.0f / B_value) * log(R_ntc / R0_value));
    return (int16_t)(tempK - 273.15f); // Return integer °C
}

// ---------- Main Program ----------
int main(void) {
    can_init();     // Initialize CAN1 (PA11, PA12)
    adc_init();     // Initialize ADC1 for PA0–PA9 analog input

    base_id = ((high << 8) | low) & 0x7FF; // Compute initial 11-bit base CAN ID

    uint8_t data0[8] = {0}, data1[8] = {0}; // Buffers for 10 temperatures
    uint8_t rx_data[8], len;               // Buffer for received CAN data
    uint32_t rx_id;
    uint8_t test_data[8] = {0xAA, 0xBB, 0xCC, 0xDD, 1, 2, 3, 4};

    while (1) {
    	 can_send(base_id, test_data, 5);
        // ---------- Check for ID Change Command ----------
    	 if (can_receive(&rx_id, rx_data, &len)) {
    	     if (rx_id == 0x8F && len >= 4 &&
    	         rx_data[0] == high && rx_data[1] == low) {
    	         high = rx_data[2];
    	         low  = rx_data[3];
    	         base_id = ((high << 8) | low) & 0x7FF;
    	     }
    	     else if (rx_id == 0x7F && len >= 4) {
    	         // Change tim (delay)
    	         tim = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
    	     }
    	     else if (rx_id == 0x6F && len >= 4) {
    	         // Change B_value
    	         uint32_t B_raw = (rx_data[3] << 24) | (rx_data[2] << 16) | (rx_data[1] << 8) | rx_data[0];
    	         B_value = *((float*)&B_raw); // Interpret raw 32 bits as float
    	     }
    	     else if (rx_id == 0x5F && len >= 4) {
    	         // Change R0_value
    	         uint32_t R0_raw = (rx_data[0] << 24) | (rx_data[1] << 16) | (rx_data[2] << 8) | rx_data[3];
    	         R0_value = *((float*)&R0_raw); // Interpret raw 32 bits as float
    	     }
    	 }


        // ---------- Read and Pack 10 Temperatures ----------
        for (int i = 0; i < 10; i++) {
            uint16_t adc = adc_read(i);        // Read ADC channel i
            int16_t temp = get_temp_int(adc);  // Convert to °C (int)
            if (i < 5)
                data0[i] = (uint8_t)temp;       // Store first 5 temps
            else
                data1[i - 5] = (uint8_t)temp;   // Store next 5 temps
        }

        // Transmit Two CAN Messages
        can_send((base_id<<1 ) | 0, data0, 8); // Temps 0–4
        can_send((base_id<<1 ) | 1, data1, 8); // Temps 5–9

        delay(tim); // Wait ~1 second
    }
}
