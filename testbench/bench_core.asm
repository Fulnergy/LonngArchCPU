# ============================================================
# bench_core — dCache full line + eviction 完整性测试
#   T2: st 16 words 填满 0x100-0x13C (全 cache line)
#   T3: 3-way conflict → evict way0 → write 0xEF to 0x4100
#   T4: ld 回 0x100-0x13C, 验证 eviction 后 16 字完整
# ============================================================

    b         test_miss
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0
    addi.w    $r0, $r0, 0

test_miss:                          # PC=0x40 iCache miss #2
    # ── 地址: set=4 ──
    addi.w    $r1, $r0, 0x100      # tag=0  line base
    lu12i.w   $r2, 0x2
    ori       $r2, $r2, 0x100      # tag=1
    lu12i.w   $r3, 0x4
    ori       $r3, $r3, 0x100      # tag=2

    # == T2: 填满整条 cache line 0x100-0x13C ==
    addi.w    $r5, $r0, 0xA0       # start value
    addi.w    $r6, $r0, 0          # offset = 0
    addi.w    $r7, $r0, 60         # max offset = 15*4

fill_loop:
    add.w     $r15, $r1, $r6       # addr = base + offset
    st.w      $r5, $r15, 0         # [addr] = val
    addi.w    $r5, $r5, 1
    addi.w    $r6, $r6, 4
    bne       $r6, $r7, fill_loop
    add.w     $r15, $r1, $r6
    st.w      $r5, $r15, 0         # last: [0x13C]=0xAF

    # == T2.5: 立即回读验证 (在 eviction 前) ==
    addi.w    $r5, $r0, 0xA0
    addi.w    $r6, $r0, 0

chk_loop:
    add.w     $r15, $r1, $r6
    ld.w      $r12, $r15, 0
    addi.w    $r5, $r5, 1
    addi.w    $r6, $r6, 4
    bne       $r6, $r7, chk_loop
    add.w     $r15, $r1, $r6
    ld.w      $r12, $r15, 0        # last read

    # == T3: dirty eviction + write-allocate ==
    ld.w      $r0, $r2, 0          # fill way1 clean (0x2100)
    addi.w    $r8, $r0, 0xEF
    st.w      $r8, $r3, 0          # miss→evict way0(dirty)+write-allocate!

    # == T4: eviction 后回读 0x100-0x13C ==
    addi.w    $r5, $r0, 0xA0
    addi.w    $r6, $r0, 0

verify_loop:
    add.w     $r15, $r1, $r6
    ld.w      $r13, $r15, 0
    addi.w    $r5, $r5, 1
    addi.w    $r6, $r6, 4
    bne       $r6, $r7, verify_loop
    add.w     $r15, $r1, $r6
    ld.w      $r13, $r15, 0        # last word

    # final: 验证 [0x4100]=0xEF
    ld.w      $r10, $r3, 0

dead_loop:
    b         dead_loop
