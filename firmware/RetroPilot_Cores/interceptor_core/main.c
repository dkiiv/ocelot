// ********************* Includes *********************
#include "../../config.h"
#include "libc.h"

#include "main_declarations.h"
#include "critical.h"
#include "faults.h"

#include "drivers/registers.h"
#include "drivers/interrupts.h"
#include "drivers/llcan.h"
#include "drivers/llgpio.h"
#include "drivers/adc.h"

#include "board.h"

#include "drivers/clock.h"
#include "drivers/dac.h"
#include "drivers/timer.h"

#include "gpio.h"
#include "crc.h"

// kinda janky, but for now uncomment this to use as a pedal interceptor
// #define IC_GAS

#define CAN CAN1

#define IC_USB
#define DEBUG

#ifdef IC_USB
  #include "drivers/uart.h"
  #include "drivers/usb.h"
#else
  // no serial either
  void puts(const char *a) {
    UNUSED(a);
  }
  void puth(unsigned int i) {
    UNUSED(i);
  }
  void puth2(unsigned int i) {
    UNUSED(i);
  }
#endif

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

// ********************* serial debugging *********************

#ifdef IC_USB

void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
  return 0;
}
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete(void) {}
void usb_cb_enumeration_complete(void) {}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xe0: uart read
    case 0xe0:
      ur = get_ring_by_number(setup->b.wValue.w);
      if (!ur) {
        break;
      }
      // read
      while ((resp_len < MIN(setup->b.wLength.w, MAX_RESP_LEN)) &&
                         getc(ur, (char*)&resp[resp_len])) {
        ++resp_len;
      }
      break;
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}

#endif

// ***************************** can port ***************************
// addresses to be used on CAN
#ifdef IC_GAS 
  #define CAN_GAS_INPUT  0x200
  #define CAN_GAS_OUTPUT 0x201U
#else
  #define CAN_GAS_INPUT  0x300
  #define CAN_GAS_OUTPUT 0x301U
#endif
#define CAN_GAS_SIZE 6
#define COUNTER_CYCLE 0xFU

void CAN1_TX_IRQ_Handler(void) {
  // clear interrupt
  CAN->TSR |= CAN_TSR_RQCP0;
}

// two independent values
uint16_t gas_set_0 = 0;
uint16_t gas_set_1 = 0;

#define MAX_TIMEOUT 5U
uint32_t timeout = 0;
uint32_t current_index = 0;

// Faults 
// Hardware
#define NO_FAULT 0U
#define FAULT_STARTUP 1U
#define FAULT_SENSOR 2U
// CAN
#define FAULT_SEND 3U
#define FAULT_SCE 4U
#define FAULT_TIMEOUT 5U
// Bad messages
#define FAULT_BAD_CHECKSUM 6U
#define FAULT_INVALID_CKSUM 7U
#define FAULT_REQ_TOO_HIGH 8U
#define FAULT_REQ_INVALID 9U

uint8_t state = FAULT_STARTUP;

const uint8_t crc_poly = 0x1D;  // standard crc8
uint8_t crc8_lut_1d[256];

void CAN1_RX0_IRQ_Handler(void) {
  while ((CAN->RF0R & CAN_RF0R_FMP0) != 0) {
    #ifdef DEBUG
      puts("CAN RX\n");
    #endif
    int address = CAN->sFIFOMailBox[0].RIR >> 21;
    if (address == CAN_GAS_INPUT) {
      // softloader entry
      if (GET_BYTES_04(&CAN->sFIFOMailBox[0]) == 0xdeadface) {
        if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x0ab00b1e) {
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
        } else if (GET_BYTES_48(&CAN->sFIFOMailBox[0]) == 0x02b00b1e) {
          enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
          NVIC_SystemReset();
        } else {
          puts("Failed entering Softloader or Bootloader\n");
        }
      }

      // normal packet
      uint8_t dat[6];
      for (int i=0; i<6; i++) {
        dat[i] = GET_BYTE(&CAN->sFIFOMailBox[0], i);
      }
      uint16_t value_0 = (dat[2] << 8) | dat[1];
      uint16_t value_1 = (dat[4] << 8) | dat[3];
      bool enable = ((dat[5] >> 7) & 1U) != 0U;
      uint8_t index = dat[5] & COUNTER_CYCLE;
      if (dat[0] == lut_checksum(dat, CAN_GAS_SIZE, crc8_lut_1d)) {
        if (((current_index + 1U) & COUNTER_CYCLE) == index) {
          #ifdef DEBUG
            puts("setting gas ");
            puth(value_0);
            puts("\n");
          #endif
          if (enable) {
            set_gpio_output(GPIOB, 0, 0);
            gas_set_0 = value_0;
            gas_set_1 = value_1;
          } else {
            set_gpio_output(GPIOB, 0, 1);
            // clear the fault state if values are 0
            if ((value_0 == 0U) && (value_1 == 0U)) {
              state = NO_FAULT;
            } else {
              state = FAULT_INVALID_CKSUM;
            }
            gas_set_0 = 0;
            gas_set_1 = 0;
          }
          // clear the timeout
          timeout = 0;
        }
        current_index = index;
      } else {
        // wrong checksum = fault
        state = FAULT_BAD_CHECKSUM;
      }
    }
    // next
    CAN->RF0R |= CAN_RF0R_RFOM0;
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  llcan_clear_send(CAN);
}

uint32_t pdl0 = 0;
uint32_t pdl1 = 0;
bool override = 0;
uint32_t magnitude = 0;

unsigned int pkt_idx = 0;

int led_value = 0;

void TIM3_IRQ_Handler(void) {
  #ifdef DEBUG
    puth(TIM3->CNT);
    puts(" PDL0: ");
    puth(pdl0);
    puts(" PDL1: ");
    puth(pdl1);
    puts(" magnitude: ");
    puth(magnitude);
    puts("\n");
  #endif

  // check timer for sending the user pedal and clearing the CAN
  if ((CAN->TSR & CAN_TSR_TME0) == CAN_TSR_TME0) {
    uint8_t dat[6];
    dat[1] = (pdl0 >> 0) & 0xFFU;
    dat[2] = (pdl0 >> 8) & 0xFFU;
    dat[3] = (pdl1 >> 0) & 0xFFU;
    dat[4] = (pdl1 >> 8) & 0xFFU;
    dat[5] = ((state & 0xFU) << 4) | pkt_idx;
    dat[0] = lut_checksum(dat, CAN_GAS_SIZE, crc8_lut_1d);
    CAN->sTxMailBox[0].TDLR = dat[0] | (dat[1] << 8) | (dat[2] << 16) | (dat[3] << 24);
    CAN->sTxMailBox[0].TDHR = dat[4] | (dat[5] << 8);
    CAN->sTxMailBox[0].TDTR = 6;  // len of packet is 5
    CAN->sTxMailBox[0].TIR = (CAN_GAS_OUTPUT << 21) | 1U;
    ++pkt_idx;
    pkt_idx &= COUNTER_CYCLE;
  } else {
    // old can packet hasn't sent!
    state = FAULT_SEND;
    #ifdef DEBUG
      puts("CAN MISS\n");
    #endif
  }

  // blink the LED
  current_board->set_led(LED_GREEN, led_value);
  led_value = !led_value;

  TIM3->SR = 0;

  // up timeout for gas set
  if (timeout == MAX_TIMEOUT) {
    state = FAULT_TIMEOUT;
    set_gpio_output(GPIOB, 0, 1);
  } else {
    timeout += 1U;
  }
}

// ***************************** main code *****************************

void pedal(void) {
  // read/write
  pdl0 = adc_get(12);
  pdl1 = adc_get(13);

#ifdef IC_GAS
  // write the pedal to the DAC
  if (state == NO_FAULT) {
    dac_set(0, MAX(gas_set_0, pdl0));
    dac_set(1, MAX(gas_set_1, pdl1));
  } else {
    dac_set(0, pdl0);
    dac_set(1, pdl1);
  }
#else
  // TODO: set this some other way. override should be set by the user
  magnitude = ABS((int32_t) pdl0 - (int32_t) pdl1);
  override = magnitude > 0x900;
  // 0x7FF + pdl0
  // write the pedal to the DAC
  if (state == NO_FAULT) {
    dac_set(0, (override ? pdl0 : gas_set_1));
    dac_set(1, (override ? pdl1 : gas_set_0));
  } else {
    dac_set(0, pdl0);
    dac_set(1, pdl1);
  }
#endif

  watchdog_feed();
}


int main(void) {
  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)

  // Should run at around 732Hz (see init below)
  REGISTER_INTERRUPT(TIM3_IRQn, TIM3_IRQ_Handler, 1000U, FAULT_INTERRUPT_RATE_TIM3)

  disable_interrupts();

  // init devices
  clock_init();
  peripherals_init();
  detect_configuration();
  detect_board_type();

  // init board
  current_board->init();

  set_gpio_mode(GPIOC, 3, MODE_ANALOG);

#ifdef IC_USB
  // enable USB
  usb_init();
#endif

  // pedal stuff
  dac_init();
  adc_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 15);
  NVIC_EnableIRQ(TIM3_IRQn);

  // relay
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 0, 1);

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);
  watchdog_init();

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main pedal loop
  while (1) {
    pedal();
  }

  return 0;
}
