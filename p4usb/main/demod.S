/* void bs_demod(const uint8_t *src, int8_t *dst, int16_t *blocks, uint32_t n, uint8_t *prev, int odd)
 *
 * For n groups of 16 samples of two bytes each, takes each sample's phase byte (on the odd addresses if
 * odd is set), writes its difference from the phase before as an int8, and writes the sum of each four
 * differences as an int16. prev holds the last phase from call to call. src and dst 16-byte aligned. */
    .text
    .align 4
    .global bs_demod
    .type bs_demod, @function

/* q5 holds the previous 16 phases; \p is the register the phases unzip into. */
.macro demod16 p
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a0, 16
    esp.vunzip.8 q0, q1
    /* Each phase's predecessor: the last of the previous 16, then the first 15 of these. */
    esp.src.q q3, q5, \p
    esp.orq q5, \p, \p
    /* Differences in 16 bits, since the 8-bit subtract saturates; their low bytes are the wrapped ones. */
    esp.orq q2, \p, \p
    esp.zero.q q6
    esp.vzip.8 q2, q6
    esp.zero.q q7
    esp.vzip.8 q3, q7
    esp.vsub.s16 q2, q2, q3
    esp.vsub.s16 q6, q6, q7
    esp.vunzip.8 q2, q6
    esp.vst.128.ip q2, a1, 16
    /* Sign-extend, then add neighbours twice for the sums of four. */
    esp.zero.q q7
    esp.vcmp.lt.s8 q3, q2, q7
    esp.vzip.8 q2, q3
    esp.vunzip.16 q2, q3
    esp.vadd.s16 q2, q2, q3
    esp.orq q3, q2, q2
    esp.vunzip.16 q2, q3
    esp.vadd.s16 q2, q2, q3
    esp.vst.l.64.ip q2, a2, 8
.endm

bs_demod:
    /* PIE takes its integer operands from a0..a5 only. */
    mv t1, a5
    li a5, 15
    esp.movx.w.sar.bytes a5
    esp.vldbc.8.ip q5, a4, 0
    bnez t1, 2f
1:
    demod16 q0
    addi a3, a3, -1
    bnez a3, 1b
    j 3f
2:
    demod16 q1
    addi a3, a3, -1
    bnez a3, 2b
3:
    esp.movi.8.a q5, a5, 15
    sb a5, 0(a4)
    ret
    .size bs_demod, .-bs_demod
