/* void chroma_mix(const int8_t *s, const int8_t *lo, int32_t *out, int pairs)
 *
 * For each 16 samples, the sums of samples times lo's next four 16-byte vectors (cos and sin for the
 * first 8, then for the last 8), stored as cos and minus sin for each half. s and lo 16-byte aligned. */
    .text
    .align 4
    .global chroma_mix
    .type chroma_mix, @function
chroma_mix:
    esp.zero.xacc
    li a6, 0
1:
    esp.vld.128.ip q0, a0, 16
    esp.vld.128.ip q1, a1, 16
    esp.vld.128.ip q2, a1, 16
    esp.vld.128.ip q3, a1, 16
    esp.vld.128.ip q4, a1, 16
    /* XACC runs on, so each sum is the difference of two readings. */
    esp.vmulas.s8.xacc q0, q1
    esp.movx.r.xacc.l a4
    sub t0, a4, a6
    sw t0, 0(a2)
    esp.vmulas.s8.xacc q0, q2
    esp.movx.r.xacc.l a5
    sub t0, a4, a5
    sw t0, 4(a2)
    esp.vmulas.s8.xacc q0, q3
    esp.movx.r.xacc.l a4
    sub t0, a4, a5
    sw t0, 8(a2)
    esp.vmulas.s8.xacc q0, q4
    esp.movx.r.xacc.l a5
    sub t0, a4, a5
    sw t0, 12(a2)
    mv a6, a5
    addi a2, a2, 16
    addi a3, a3, -1
    bnez a3, 1b
    ret
    .size chroma_mix, .-chroma_mix
