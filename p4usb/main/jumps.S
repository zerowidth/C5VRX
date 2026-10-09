/* void jumps(const int8_t *s, int n, int8_t *out)
 *
 * Over n blocks of 16 samples, the largest rise and the largest fall from one sample to the next, each
 * clamped to an int8: 16 maxima, then 16 minima. s and out 16-byte aligned; s is read one block past the last. */
    .text
    .align 4
    .global jumps
    .type jumps, @function
jumps:
    li a3, 1
    esp.movx.w.sar.bytes a3
    esp.zero.q q3
    esp.zero.q q4
    esp.vld.128.ip q0, a0, 16
1:
    esp.vld.128.ip q1, a0, 16
    esp.src.q q2, q0, q1
    esp.vsub.s8 q2, q2, q0
    esp.vmax.s8 q3, q3, q2
    esp.vmin.s8 q4, q4, q2
    esp.orq q0, q1, q1
    addi a1, a1, -1
    bnez a1, 1b
    esp.vst.128.ip q3, a2, 16
    esp.vst.128.ip q4, a2, 16
    ret
    .size jumps, .-jumps
