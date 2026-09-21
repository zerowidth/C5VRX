#pragma once
#include <stdbool.h>
#include <stdint.h>

/* Presents this board to a RotorHazard server as two timing nodes over USB
 * serial: node 0 is the C5's own receiver, node 1 is the RX5808. Both see the
 * same passes, so the race log compares them directly.
 *
 * The protocol is RotorHazard's node API level 36: the server writes one
 * command byte, then either reads a fixed-size reply plus a checksum, or
 * sends a payload plus a checksum. See src/node/commands.cpp upstream.
 */
#define RHNODE_COUNT 2
#define RHNODE_C5 0
#define RHNODE_RX5808 1

/* One RSSI sample per node per millisecond, already on the 0-255 scale
 * RotorHazard uses. Runs the median filter, crossing detection and lap
 * bookkeeping that its node firmware would. */
void rhnode_feed(uint8_t node, uint8_t rssi, uint32_t now_ms);

/* Feed one received byte. Returns true once a valid command has been seen, at
 * which point the board is talking to a server and must stay quiet otherwise. */
bool rhnode_rx_byte(uint8_t b);

bool rhnode_active(void);

/* Worst-case measurement pass, reported to the server as the node loop time. */
void rhnode_set_loop_us(uint32_t us);

/* True if this byte can only be a RotorHazard command, never our own text. */
bool rhnode_is_command_byte(uint8_t b);
