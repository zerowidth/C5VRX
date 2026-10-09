/* void luma_row(const int8_t *s, uint8_t *dst, const uint64_t *uv, int n, const int8_t (*k)[16], int shift)
 *
 * n blocks of 16 pixels as V Y0 U Y1 pairs. Each Y is a weighted sum of seven samples from s, less an
 * offset, shifted right and clamped to -128..127, then flipped to 0..255; uv holds each 8 pixels' V U pair
 * four times over. k is the offset to take from each sample, weights 1 to 3, two vectors whose product
 * starts each sum, weights 0, 4, 5 and 6, and 0x80s. s, dst, uv and k 16-byte aligned; s is read one block past the last. */
    .text
    .align 4
    .global luma_row
    .type luma_row, @function
luma_row:
    slli a3, a3, 5
    add t0, a1, a3
    li a3, 5
    li t3, 1
    li t4, 2
    li t5, 3
    li t6, 4
    esp.vld.128.ip q3, a4, 16
    esp.vld.128.ip q5, a4, 16
    esp.vld.128.ip q6, a4, 16
    esp.vld.128.ip q7, a4, 16
    esp.vld.128.ip q0, a0, 16
    esp.vsub.s8 q0, q0, q3
1:
    esp.zero.qacc
    esp.vld.128.ip q1, a4, 16
    esp.vld.128.ip q2, a4, 16
    esp.vmulas.s8.qacc q1, q2
    esp.vld.128.ip q2, a4, 16
    esp.vmulas.s8.qacc q0, q2
    esp.vld.128.ip q1, a0, 16
    esp.vsub.s8 q1, q1, q3
    /* Each later sample is this block and the next shifted down a byte more. */
    esp.movx.w.sar.bytes t3
    esp.src.q q2, q0, q1
    esp.vmulas.s8.qacc q2, q5
    esp.movx.w.sar.bytes t4
    esp.src.q q2, q0, q1
    esp.vmulas.s8.qacc q2, q6
    esp.movx.w.sar.bytes t5
    esp.src.q q2, q0, q1
    esp.vmulas.s8.qacc q2, q7
    esp.vld.128.ip q4, a4, 16
    esp.movx.w.sar.bytes t6
    esp.src.q q2, q0, q1
    esp.vmulas.s8.qacc q2, q4
    esp.vld.128.ip q4, a4, 16
    esp.movx.w.sar.bytes a3
    esp.src.q q2, q0, q1
    esp.vmulas.s8.qacc q2, q4
    esp.vld.128.ip q4, a4, 16
    /* The shift count has to come from one of few registers, and the rest hold arguments. */
    li t3, 6
    esp.movx.w.sar.bytes t3
    li t3, 1
    esp.src.q q2, q0, q1
    esp.vmulas.s8.qacc q2, q4
    esp.orq q0, q1, q1
    /* The last operand is what makes it clamp; with 0 it wraps. */
    esp.srcmb.s8.qacc q2, a5, 1
    esp.vld.128.ip q1, a4, -96
    esp.xorq q2, q2, q1
    esp.vld.128.ip q1, a2, 16
    esp.vzip.8 q1, q2
    esp.vst.128.ip q1, a1, 16
    esp.vst.128.ip q2, a1, 16
    bne a1, t0, 1b
    ret
    .size luma_row, .-luma_row
