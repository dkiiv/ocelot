/*
For PLA operation at all speeds on select VW models.
The 3 sins of the VW EPS rack which have lead to this:
1: Nonlinear feedforward response
2: Speed dependant feedforward
3: Poor lateral torque (HCA5)

**Use this at your own discretion.**
The EPS exits controls authoritatively if the driver grabs the wheel while in PLA operation.
Considering a handshake (routine) is need to enter PLA, this makes using PLA "safe" in
atleast some sense of the word.

On top of this OEM EPS safety, this module provides some other features.
Such as, rate limits tied to speed, angle limits tied to speed, proper
engagement behavior, timeouts, and signal integrity verification.
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

#include "vw_pla/can.h"

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

// fault states
#define NO_FAULT 0U
#define FAULT_BAD_CHECKSUM 1U
#define FAULT_SEND 2U
#define FAULT_SCE 3U
#define FAULT_STARTUP 4U
#define FAULT_TIMEOUT 5U
#define FAULT_INVALID 6U
#define FAULT_COUNTER 7U

uint8_t state = FAULT_STARTUP;

const uint8_t crc_poly = 0x1D;  // standard crc8 SAE J1850
uint8_t crc8_lut_1d[256];

#define HCA_1           0x0D2  // RX
#define BREMSE_1        0x1A0  // RX
#define BREMSE_3        0x4A0  // RX
#define GK_1            0x390  // RX
#define LENKHILFE_2     0x3D2  // RX
#define PLA_1           0x3D4  // TX (RX too if OEM present)
#define MESSAGE_1       0x2FF  // TX

#define M1_CYCLE        0xFU

#define PLA_ANGLE    (pla_rdlr >> 16U) & 0x7FFF
#define PLA_SIGN     (pla_rdlr >> 31U) & 1U

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

bool send = false;
bool moduleSleep = false;
bool filter = false;
bool hca_counter_fault = false;
bool hca_checksum_fault = false;
bool pla_exit = false;
bool pla_override = false;
bool pla_sign = false;
bool pla_limit = false;
bool oempla_active = false;
int pla_angle = 0;
int pla_angle_last = 0;
uint8_t sleepCounter = 0;
uint8_t hca_stat = 0;
uint8_t pla_stat = 0;
uint8_t hca_rx_counter = 0;
uint8_t pla_wd_counter = 0;
uint8_t M1_counter = 0;
uint16_t pla_angle_limit = 0;
uint16_t pla_rate_limit = 0;
uint16_t vego = 0;
uint32_t pla_rdlr = 0;
uint32_t oempla_rdlr = 0;
uint32_t hca_rdlr = 0;
uint32_t debug_rdlr = 0;
uint64_t msg = 0;
unsigned char *byte = 0;

int angle_pla(void) {
  pla_sign = PLA_SIGN;
  int pla_angle = PLA_ANGLE;
  return pla_sign ? -pla_angle : pla_angle;
}

int interpolate(int x, bool rateLimit) {
  int speeds[] = {0, 1800, 9000};            // kph (scaled by 0.01)
  int values_rate[] = {119, 50, 9};          // angle/frame (scaled by 0.04375)
  int values_limit[] = {11428, 9348, 1028};  // angle limit (scaled by 0.04375)
  int len = 3;
  int* values = rateLimit ? values_rate : values_limit;
  x = CLIP(x, speeds[0], speeds[len - 1]);
  for (int i = 0; i < len - 1; i++) {
    if (x <= speeds[i + 1]) {
      int x0 = speeds[i], x1 = speeds[i + 1];
      int y0 = values[i], y1 = values[i + 1];

      // int linear interpolation: y0 + (x - x0) * (y1 - y0) / (x1 - x0)
      int numerator = (x - x0) * (y1 - y0);
      int denominator = x1 - x0;
      return y0 + numerator / denominator;
    }
  }
  return 0;
}

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

    #ifdef DEBUG_CAN
    puts("CAN1 RX: ");
    puth(address);
    puts("\n");
    #endif

    switch (address) {
      case (HCA_1):
        /*
                  PLA_1 EPS stat + 7
                  10 - driver override flag reset
                  11 - entry request
                  13 - active
                  15 - standby
        */
        // toggle filter on when HCA RX is status 11, or 15
        msg = ((uint64_t)to_fwd.RDHR << 32U) | to_fwd.RDLR;
        byte = (uint8_t *)&msg;
        hca_counter_fault = !((byte[1] & 0xFU) == ((hca_rx_counter + 1) & 0xFU));
        hca_checksum_fault = !((byte[0] & 0xFFU) == (byte[1] ^ byte[2] ^ byte[3] ^ byte[4]));

        pla_exit = hca_counter_fault || hca_checksum_fault || pla_override;
        hca_stat = ((byte[1] >> 4U) & 0b1111);
        filter = ((hca_stat == 11U || hca_stat == 13U) && !pla_exit && !oempla_active);
        if (hca_stat == 10U || hca_stat == 11U || hca_stat == 13U || hca_stat == 15U) {
          if (!pla_exit){
            pla_wd_counter = 0;  // reset exit counter on proper RX of PLA control
          }
          if (pla_override && hca_stat == 10U){
            pla_override = false;
          }

          // cleaning mHCA_1 fwd to EPS
          hca_rdlr = to_fwd.RDLR;
          hca_rdlr = (hca_rdlr & 0xFFFF0F00) | 0x00003000;  // mask off checksum and set HCA status 3
          byte[1] = (byte[1] & 0x0F) | 0x30;  // set HCA status 3 for checksum calc
          hca_rdlr = hca_rdlr | (byte[1] ^ byte[2] ^ byte[3] ^ byte[4]);  // xor-checksum
          to_fwd.RDLR = hca_rdlr;
        }

        hca_rx_counter = (byte[1] & 0xFU);
        break;
      case (PLA_1):
        // TODO: add module pla override if OEM status != 8
        oempla_rdlr = to_fwd.RDLR;
        byte = (uint8_t *)&oempla_rdlr;
        oempla_active = ((byte[1] >> 4U) & 0b1111) != 8U;
        break;
      case (BREMSE_1):
        // set vEgo to 0
        vego = (to_fwd.RDLR >> 17U) & 0x7FFF;
        sleepCounter = 0;  // reset sleep timer on RX of BR1
        break;
      case (BREMSE_3):
        // set WSS-HR to 0
        if (filter) {
          to_fwd.RDHR &= 0x0000FFFF;
        }
        break;
      case (GK_1):
        // set BCM reverse light on
        if (filter) {
          to_fwd.RDLR |= 0x10000000;
        }
        break;
      default:
        // FWD as-is
        break;
    }
    // send to CAN3
    // TODO: remove HCA_1 once confirmed working
    if ((address != PLA_1) || (address != HCA_1)){
      can_send(&to_fwd, 2, false);
    }
    // next
    can_rx(0);
  }
}

void CAN1_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN1);
  llcan_clear_send(CAN1);
}

void CAN2_RX0_IRQ_Handler(void) {
  while ((CAN2->RF0R & CAN_RF0R_FMP0) != 0) {
    uint16_t address = CAN2->sFIFOMailBox[0].RIR >> 21;
    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif
    switch (address) {
      default: ;
    }
    can_rx(1);
  }
}

void CAN2_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
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

    #ifdef DEBUG_CAN
    puts("CAN2 RX: ");
    puth(address);
    puts("\n");
    #endif

    switch (address) {
      case (LENKHILFE_2):
                          // if LH2_PLA_Abbr == 2 latch override on
        pla_override = ((to_fwd.RDHR >> 20U) == 2U) || pla_override;
        if (pla_override) {
          to_fwd.RDHR = (to_fwd.RDHR & 0x0FFFFF) | 0x200000;
          msg = ((uint64_t)to_fwd.RDHR << 32U) | to_fwd.RDLR;
          byte = (uint8_t *)&msg;
          to_fwd.RDLR = (to_fwd.RDLR & 0xFFFFFF00) | (byte[1] ^ byte[2] ^ byte[3] ^ byte[4] ^ byte[5] ^ byte[6]);
        }
        break;
      default:
        // FWD as-is
        break;
    }
    // send to CAN1
    can_send(&to_fwd, 0, false);
    // next
    can_rx(2);
  }
}

void CAN3_SCE_IRQ_Handler(void) {
  state = FAULT_SCE;
  can_sce(CAN3);
  llcan_clear_send(CAN3);
}

int to_signed(int d, int bits) {
  int d_signed = d;
  if (d >= (1 << MAX((bits - 1), 0))) {
    d_signed = d - (1 << MAX(bits, 0));
  }
  return d_signed;
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
  // below is a debug msg for the filter, checking operation
  if (send){
    if ((CAN1->TSR & CAN_TSR_TME1) == CAN_TSR_TME1) {
      CAN_FIFOMailBox_TypeDef to_send;
      to_send.RDLR = debug_rdlr;
      to_send.RDHR = ((uint16_t)M1_counter << 8U) | (pla_limit << 2U) | (pla_exit << 1U) | filter;
      to_send.RDTR = 8;
      to_send.RIR = (MESSAGE_1 << 21) | 1U;
      // sending to bus 0 (powertrain)
      can_send(&to_send, 0, false);
    }
    if ((CAN3->TSR & CAN_TSR_TME2) == CAN_TSR_TME2) {
      CAN_FIFOMailBox_TypeDef to_send;

      if ((hca_stat == 10U || hca_stat == 11U || hca_stat == 13U || hca_stat == 15U) && !oempla_active) {
        pla_stat = (hca_stat == 10U ? 8U : hca_stat - 7U);
        pla_rdlr = (hca_rdlr & 0xFFFF0000) | ((uint16_t)pla_stat << 12U);
        if (hca_stat == 13U && !pla_exit) {
          pla_rate_limit = interpolate(vego, 1);
          pla_angle_limit = interpolate(vego, 0);
          pla_angle = angle_pla();
          pla_angle_last = CLIP(pla_angle, pla_angle_last - pla_rate_limit, pla_angle_last + pla_rate_limit);  // rate limit
          pla_angle_last = CLIP(pla_angle_last, -pla_angle_limit, pla_angle_limit);                            // angle limit
          pla_limit = (pla_angle != pla_angle_last);
          // set angle direction bit, angle < 0 = 1
          if (pla_angle_last < 0){
            pla_rdlr = pla_rdlr | 0x80000000;
            pla_rdlr = ((pla_rdlr & 0x8000FF00) | ((uint32_t)(pla_angle_last * -1) << 16U));
          } else {
            pla_rdlr = pla_rdlr & 0x7FFFFFFF;
            pla_rdlr = ((pla_rdlr & 0x8000FF00) | ((uint32_t)pla_angle_last << 16U));
          }
        } else {
          pla_angle_last = angle_pla();
        }
      } else {
        // TODO: look into replicating OEM module functionality? maybe not needed.. (mimicking angle/sign)
        pla_rdlr = oempla_rdlr;
      }

      pla_rdlr = (pla_rdlr & 0xFFFFF000) | ((uint16_t)M1_counter << 8U);
      byte = (uint8_t *)&pla_rdlr;
      pla_rdlr = pla_rdlr | (byte[1] ^ byte[2] ^ byte[3]);
      debug_rdlr = pla_rdlr;
      to_send.RDLR = pla_rdlr;
      to_send.RDHR = 0x00000000;
      to_send.RDTR = 8;
      to_send.RIR = (PLA_1 << 21) | 1U;
      // sending to bus 2 (EPS)
      can_send(&to_send, 2, false);
      M1_counter += 1;
      M1_counter &= M1_CYCLE;
    }
  }

    // if PLA isnt seen for 0.5s filter force cancels
  if (pla_wd_counter >= 50) {
    filter = 0;
  }

    // sleep when BR1 is dead for >750ms
  if (sleepCounter >= 75) {
    moduleSleep = 1;
    send = 0;
    filter = 0;
    hca_counter_fault = 0;
    hca_checksum_fault = 0;
    pla_exit = 0;
    pla_override = 0;
    pla_sign = 0;
    pla_angle_last = 0;
    hca_stat = 0;
    pla_stat = 0;
    hca_rx_counter = 0;
    pla_wd_counter = 0;
    pla_angle_limit = 0;
    pla_rate_limit = 0;
    pla_rdlr = 0;
    vego = 0;
    msg = 0;
    byte = 0;
  } else {
    moduleSleep = 0;
    send = !send;
    pla_wd_counter += 1;
    pla_wd_counter &= 100;
  }

  sleepCounter +=1;
  sleepCounter &= 100;
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
  /*
  if (state == FAULT_STARTUP || state == FAULT_SCE || state == NO_FAULT) {
    state = NO_FAULT;
    set_gpio_output(GPIOB, 0, 0);  // toggle relay, disconnecting EPS beginning filter
  } else {
    set_gpio_output(GPIOB, 0, 1);  // toggle relay, reconnecting EPS, ending filter
  }
  */
  //watchdog_feed();  // uncomment for production
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

  gen_crc_lookup_table(crc_poly, crc8_lut_1d);

  // 48mhz / 65536 ~= 732
  timer_init(TIM3, 7);
  NVIC_EnableIRQ(TIM3_IRQn);

  dac_init();
  adc_init();

  // turn on relay
  set_gpio_mode(GPIOB, 0, MODE_OUTPUT);
  set_gpio_output_type(GPIOB, 0, OUTPUT_TYPE_PUSH_PULL);
  set_gpio_output(GPIOB, 0, 1);

  //watchdog_init();  // uncomment for production

  puts("**** INTERRUPTS ON ****\n");
  enable_interrupts();

  // main loop
  while (1) {
    loop();
  }

  return 0;
}