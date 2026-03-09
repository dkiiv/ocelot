/*
For generating vehicle CAN data, from DBC, to verify functionality of other custom CAN modules
*/

// ********************* Includes *********************
#include "../config.h"
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

#include "drivers/uart.h"
#include "drivers/usb.h"
#include <string.h>

#define ENTER_BOOTLOADER_MAGIC 0xdeadbeef
uint32_t enter_bootloader_mode;

// cppcheck-suppress unusedFunction ; used in headers not included in cppcheck
void __initialize_hardware_early(void) {
  early();
}

#include "tesla_bench/can.h"

#define usb_debugger
#ifdef usb_debugger
// ********************* usb debugging *********************
void debug_ring_callback(uart_ring *ring) {
  char rcv;
  while (getc(ring, &rcv) != 0) {
    (void)putc(ring, rcv);
  }
}

int usb_cb_ep1_in(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  CAN_FIFOMailBox_TypeDef *reply = (CAN_FIFOMailBox_TypeDef *)usbdata;
  int ilen = 0;
  while (ilen < MIN(len/0x10, 4) && can_pop(&can_rx_q, &reply[ilen])) {
    ilen++;
  }
  return ilen*0x10;
}
// send on serial, first byte to select the ring
void usb_cb_ep2_out(void *usbdata, int len, bool hardwired) {
  UNUSED(hardwired);
  uint8_t *usbdata8 = (uint8_t *)usbdata;
  uart_ring *ur = get_ring_by_number(usbdata8[0]);
  if ((len != 0) && (ur != NULL)) {
    if ((usbdata8[0] < 2U)) {
      for (int i = 1; i < len; i++) {
        while (!putc(ur, usbdata8[i])) {
          // wait
        }
      }
    }
  }
}
// send on CAN
void usb_cb_ep3_out(void *usbdata, int len, bool hardwired) {
  UNUSED(usbdata);
  UNUSED(len);
  UNUSED(hardwired);
}
void usb_cb_ep3_out_complete() {
  if (can_tx_check_min_slots_free(MAX_CAN_MSGS_PER_BULK_TRANSFER)) {
    usb_outep3_resume_if_paused();
  }
}

void usb_cb_enumeration_complete() {
  puts("USB enumeration complete\n");
  is_enumerated = 1;
}

int usb_cb_control_msg(USB_Setup_TypeDef *setup, uint8_t *resp, bool hardwired) {
  UNUSED(hardwired);
  unsigned int resp_len = 0;
  uart_ring *ur = NULL;
  switch (setup->b.bRequest) {
    // **** 0xd1: enter bootloader mode
    case 0xd1:
      // this allows reflashing of the bootstub
      // so it's blocked over wifi
      switch (setup->b.wValue.w) {
        case 0:
          // only allow bootloader entry on debug builds
          #ifdef ALLOW_DEBUG
            if (hardwired) {
              puts("-> entering bootloader\n");
              enter_bootloader_mode = ENTER_BOOTLOADER_MAGIC;
              NVIC_SystemReset();
            }
          #endif
          break;
        case 1:
          puts("-> entering softloader\n");
          enter_bootloader_mode = ENTER_SOFTLOADER_MAGIC;
          NVIC_SystemReset();
          break;
        default:
          puts("Bootloader mode invalid\n");
          break;
      }
      break;
    // **** 0xd8: reset ST
    case 0xd8:
      NVIC_SystemReset();
      break;
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
    // **** 0xf1: Clear CAN ring buffer.
    case 0xf1:
      if (setup->b.wValue.w == 0xFFFFU) {
        puts("Clearing CAN Rx queue\n");
        can_clear(&can_rx_q);
      } else if (setup->b.wValue.w < BUS_MAX) {
        puts("Clearing CAN Tx queue\n");
        can_clear(can_queues[setup->b.wValue.w]);
      } else {
        puts("Clearing CAN CAN ring buffer failed: wrong bus number\n");
      }
      break;
    // **** 0xf2: Clear UART ring buffer.
    case 0xf2:
      {
        uart_ring * rb = get_ring_by_number(setup->b.wValue.w);
        if (rb != NULL) {
          puts("Clearing UART queue.\n");
          clear_uart_buff(rb);
        }
        break;
      }
    default:
      puts("NO HANDLER ");
      puth(setup->b.bRequest);
      puts("\n");
      break;
  }
  return resp_len;
}
#endif

// ***************************** can port *****************************
void CAN1_TX_IRQ_Handler(void) {
  process_can(0);
}

void CAN2_TX_IRQ_Handler(void) {
  process_can(1);
}

void CAN3_TX_IRQ_Handler(void) {
  process_can(2);
}

#define MIN(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a < _b) ? _a : _b; })

#define MAX(a,b) \
 ({ __typeof__ (a) _a = (a); \
     __typeof__ (b) _b = (b); \
   (_a > _b) ? _a : _b; })

#define CLIP(x, minVal, maxVal) \
  MAX(minVal, MIN(x, maxVal))

uint64_t msg = 0;
unsigned char *byte = 0;

void CAN1_RX0_IRQ_Handler(void) {
  // PTCAN connects here
  while ((CAN1->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN1->sFIFOMailBox[0].RIR | 1;  /*!< CAN receive FIFO mailbox identifier register */
    to_fwd.RDTR = CAN1->sFIFOMailBox[0].RDTR;    /*!< CAN receive FIFO mailbox data length control and time stamp register */
    to_fwd.RDLR = CAN1->sFIFOMailBox[0].RDLR;    /*!< CAN receive FIFO mailbox data low register */
    to_fwd.RDHR = CAN1->sFIFOMailBox[0].RDHR;    /*!< CAN receive FIFO mailbox data high register */

    uint32_t address = (CAN1->sFIFOMailBox[0].RIR >> 21);
    uint8_t ide = (CAN1->sFIFOMailBox[0].RIR >> 2) & 0x01;
    if(ide){
      address = (CAN1->sFIFOMailBox[0].RIR >> 3);
    }

    switch (address) {
      default: ;
    }

    can_send(&to_fwd, 2, false);
    can_rx(0);
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;
    switch (address) {
      default: ;
    }
    can_rx(1);
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  can_sce(CAN2);
  llcan_clear_send(CAN2);
}

void CAN3_RX0_IRQ_Handler(void) {
  // EPSCAN connects here
  while ((CAN3->RF0R & CAN_RF0R_FMP0) != 0) {

    CAN_FIFOMailBox_TypeDef to_fwd;
    to_fwd.RIR = CAN3->sFIFOMailBox[0].RIR | 1; // TXQ
    to_fwd.RDTR = CAN3->sFIFOMailBox[0].RDTR;
    to_fwd.RDLR = CAN3->sFIFOMailBox[0].RDLR;
    to_fwd.RDHR = CAN3->sFIFOMailBox[0].RDHR;

    uint32_t address = (CAN3->sFIFOMailBox[0].RIR >> 21);
    uint8_t ide = (CAN3->sFIFOMailBox[0].RIR >> 2) & 0x01;
    if(ide){
      address = (CAN3->sFIFOMailBox[0].RIR >> 3);
    }

    switch (address) {
      default: ;
    }

    can_send(&to_fwd, 0, false);
    can_rx(2);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

#define MAX_FADE 8192U
void set_led(uint8_t color, bool enabled) {
  switch (color){
    case LED_RED:
      set_gpio_output(GPIOC, 9, !enabled);
      break;
     case LED_GREEN:
      set_gpio_output(GPIOC, 7, !enabled);
      break;
    case LED_BLUE:
      set_gpio_output(GPIOC, 6, !enabled);
      break;
    default:
      break;
  }
}

void TIM3_IRQ_Handler(void) {
  // cmain loop, 100hz
  if ((CAN1->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
    send_MSG_0x3C2();  // 0x3C2 | 962  | VCLEFT_switchStatus       | 8 bytes | no CS / no CTR
    send_MSG_0x103();  // 0x103 | 259  | VCRIGHT_doorStatus        | 8 bytes | no CS / no CTR
    send_MSG_0x249();  // 0x249 | 585  | SCCM_leftStalk            | 3 bytes | CS@byte0 / CTR@byte1-low
    send_MSG_0x118();  // 0x118 | 280  | DI_systemStatus           | 8 bytes | CS@byte0 / CTR@byte1-low
    send_MSG_0x343();  // 0x343 | 835  | VCRIGHT_status            | 8 bytes | no CS / no CTR
    send_MSG_0x229();  // 0x229 | 553  | SCCM_rightStalk           | 3 bytes | CS@byte0 / CTR@byte1-low
    send_MSG_0x129();  // 0x129 | 297  | SCCM_steeringAngleSensor  | 8 bytes | CS@byte0 / CTR@byte1-low
    send_MSG_0x102();  // 0x102 | 258  | VCLEFT_doorStatus         | 8 bytes | no CS / no CTR
    send_MSG_0x238();  // 0x238 | 568  | STW_ACTN_RQ               | 8 bytes | CS@byte7 / no CTR        
    send_MSG_0x3E9();  // 0x3E9 | 1001 | DAS_bodyControls          | 8 bytes | CS@byte7 / CTR@byte6-high
    send_MSG_0x3F5();  // 0x3F5 | 1013 | ID3F5VCFRONT_lighting     | 8 bytes | no CS / no CTR
    tick_counter();    // advance msg counters
  }
  TIM3->SR = 0;
}

// ***************************** main code *****************************

void loop(void) {
  // used for testing, remove for production
  for (uint32_t fade = 0U; fade < MAX_FADE; fade += 1U) {
    set_led(LED_BLUE, true);
    delay(fade >> 4);
    set_led(LED_BLUE, false);
    delay((MAX_FADE - fade) >> 4);
  }
  for (uint32_t fade = MAX_FADE; fade > 0U; fade -= 1U) {
    set_led(LED_GREEN, true);
    delay(fade >> 4);
    set_led(LED_GREEN, false);
    delay((MAX_FADE - fade) >> 4);
  }
}

int main(void) {
  // Init LEDs
  set_gpio_mode(GPIOC, 9, MODE_OUTPUT);
  set_gpio_mode(GPIOC, 7, MODE_OUTPUT);
  set_gpio_mode(GPIOC, 6, MODE_OUTPUT);
  set_gpio_output(GPIOC, 9, 1);
  set_gpio_output(GPIOC, 7, 1);
  set_gpio_output(GPIOC, 6, 1);

  // GPS OFF, remove for production
  set_gpio_output(GPIOC, 14, 0);
  set_gpio_output(GPIOC, 5, 0);

  // Init interrupt table
  init_interrupts(true);

  REGISTER_INTERRUPT(CAN1_TX_IRQn, CAN1_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_RX0_IRQn, CAN1_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN1_SCE_IRQn, CAN1_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_1)
  REGISTER_INTERRUPT(CAN2_TX_IRQn, CAN2_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_RX0_IRQn, CAN2_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN2_SCE_IRQn, CAN2_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_2)
  REGISTER_INTERRUPT(CAN3_TX_IRQn, CAN3_TX_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_RX0_IRQn, CAN3_RX0_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)
  REGISTER_INTERRUPT(CAN3_SCE_IRQn, CAN3_SCE_IRQ_Handler, CAN_INTERRUPT_RATE, FAULT_INTERRUPT_RATE_CAN_3)

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
  // enable USB
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOVAL;
  USBx->GOTGCTL |= USB_OTG_GOTGCTL_BVALOEN;
  usb_init();

  // init can
  bool llcan_speed_set = llcan_set_speed(CAN1, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan1 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN2, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan2 speed");
  }
  llcan_speed_set = llcan_set_speed(CAN3, 5000, false, false);
  if (!llcan_speed_set) {
    puts("Failed to set llcan3 speed");
  }

  bool ret = llcan_init(CAN1);
  UNUSED(ret);
  ret = llcan_init(CAN2);
  UNUSED(ret);
  ret = llcan_init(CAN3);
  UNUSED(ret);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  dac_init();
  adc_init();

  // turn on relay
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 0, 1);

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main loop
  while (1) {
    loop();
  }

  return 0;
}