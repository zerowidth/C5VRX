/* void deint_row(uint8_t *out, const uint8_t *above, const uint8_t *below, const uint8_t *old, int n)
 *
 * n 16-byte blocks of V Y0 U Y1 pixel pairs: each Y is the median of the Ys above, below and in the
 * previous field, and the chroma is below's. All pointers 16-byte aligned. */
    .text
    .align 4
    .global deint_row
    .type deint_row, @function
deint_row:
    addi sp, sp, -16
    li t0, 0xFF00FF00
    sw t0, 0(sp)
    not t0, t0
    sw t0, 4(sp)
    mv a5, sp
    esp.vldbc.32.ip q6, a5, 0
    addi a5, sp, 4
    esp.vldbc.32.ip q7, a5, 0
1:
    esp.vld.128.ip q0, a1, 16
    esp.vld.128.ip q1, a2, 16
    esp.vld.128.ip q2, a3, 16
    esp.vmin.u8 q3, q0, q1
    esp.vmax.u8 q4, q0, q1
    esp.vmin.u8 q4, q4, q2
    esp.vmax.u8 q3, q3, q4
    esp.andq q3, q3, q6
    esp.andq q5, q1, q7
    esp.orq q3, q3, q5
    esp.vst.128.ip q3, a0, 16
    addi a4, a4, -1
    bnez a4, 1b
    addi sp, sp, 16
    ret
    .size deint_row, .-deint_row
