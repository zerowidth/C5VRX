/* void tnr_row(uint8_t *row, const uint8_t *before, int n, const uint8_t *k)
 *
 * n 16-byte blocks: each byte of row moves towards before's by k[1] sixteenths of their difference d,
 * or of k[0] - d once d passes half of k[0], and not at all past k[0]. row and before 16-byte aligned. */
    .text
    .align 4
    .global tnr_row
    .type tnr_row, @function
tnr_row:
    li a4, 4
    esp.movx.w.sar a4
    esp.vldbc.8.ip q5, a3, 1
    esp.vldbc.8.ip q6, a3, 0
1:
    esp.vld.128.ip q0, a0, 0
    esp.vld.128.ip q1, a1, 16
    esp.vsub.u8 q2, q1, q0
    esp.vsub.u8 q3, q0, q1
    esp.vsub.u8 q4, q5, q2
    esp.vmin.u8 q2, q2, q4
    esp.vsub.u8 q4, q5, q3
    esp.vmin.u8 q3, q3, q4
    esp.vmul.u8 q2, q2, q6
    esp.vmul.u8 q3, q3, q6
    esp.vadd.u8 q0, q0, q2
    esp.vsub.u8 q0, q0, q3
    esp.vst.128.ip q0, a0, 16
    addi a2, a2, -1
    bnez a2, 1b
    ret
    .size tnr_row, .-tnr_row
