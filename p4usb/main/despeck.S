/* void despeck(int8_t *s, int16_t *blocks, uint32_t n, const int8_t *k)
 *
 * Over n blocks of 16 demodulated samples, in place: a sample half a turn or more from the mean of the two
 * either side of it has wrapped, and goes to the end of the range it wrapped past. Then writes the sum of
 * each four samples as an int16. k is -1, 4, 63 and 0x80. s 16-byte aligned, read a block before and after. */
    .text
    .align 4
    .global despeck
    .type despeck, @function
despeck:
    li t3, 15
    li t4, 14
    li t5, 1
    li t6, 2
    li a5, 3
    esp.vldbc.8.ip q5, a3, 1
    esp.vldbc.8.ip q6, a3, 1
    mv a4, a0
    addi a0, a0, -16
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a0, 16
1:
    esp.vld.128.ip q2, a0, 16
    /* Four times the sample less its four neighbours, which the shift turns into half its distance from their mean. */
    esp.zero.qacc
    esp.vmulas.s8.qacc q1, q6
    esp.movx.w.sar.bytes t3
    esp.src.q q3, q0, q1
    esp.vmulas.s8.qacc q3, q5
    esp.movx.w.sar.bytes t4
    esp.src.q q3, q0, q1
    esp.vmulas.s8.qacc q3, q5
    esp.movx.w.sar.bytes t5
    esp.src.q q3, q1, q2
    esp.vmulas.s8.qacc q3, q5
    esp.movx.w.sar.bytes t6
    esp.src.q q3, q1, q2
    esp.vmulas.s8.qacc q3, q5
    esp.srcmb.s8.qacc q3, a5, 1
    esp.vldbc.8.ip q7, a3, 1
    /* q4 marks samples too far above, q3 too far below. */
    esp.vcmp.gt.s8 q4, q3, q7
    esp.notq q3, q3
    esp.vcmp.gt.s8 q3, q3, q7
    esp.orq q7, q4, q3
    /* 0x80 for those above and 0x7f for those below. */
    esp.vldbc.8.ip q4, a3, -1
    esp.andq q4, q4, q7
    esp.xorq q4, q4, q3
    esp.notq q7, q7
    esp.andq q7, q7, q1
    esp.orq q4, q4, q7
    esp.vst.128.ip q4, a4, 16
    /* Sign-extend, then add neighbours twice for the sums of four. */
    esp.zero.q q7
    esp.vcmp.lt.s8 q3, q4, q7
    esp.vzip.8 q4, q3
    esp.vunzip.16 q4, q3
    esp.vadd.s16 q4, q4, q3
    esp.orq q3, q4, q4
    esp.vunzip.16 q4, q3
    esp.vadd.s16 q4, q4, q3
    esp.vst.l.64.ip q4, a1, 8
    esp.orq q0, q1, q1
    esp.orq q1, q2, q2
    addi a2, a2, -1
    bnez a2, 1b
    ret
    .size despeck, .-despeck
