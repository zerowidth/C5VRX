#pragma once

/* P4 pins wired to the C5, per docs/p4-receiver-plan.md. */
#define PIN_C5_EN   26 /* open-drain to the C5's EN */
#define PIN_C5_BOOT 8  /* open-drain to the C5's GPIO 28 */
#define PIN_C5_RX   32 /* from the C5's UART0 TX, GPIO 11 */
#define PIN_C5_TX   27 /* to the C5's UART0 RX, GPIO 12 */

typedef struct {
    const char *name;
    int c5;
    int p4;
} wire_t;

/* The sample clock and the 14 I/Q lanes. */
extern const wire_t BUS_WIRES[];
extern const int BUS_WIRE_COUNT;
