add x1, x1, x1
sub x2, x2, x1
xor x3, x1, x3
or x4, x1, x3
and x5, x5, x1
sll x6, x6, x1
srl x7, x6, x3
slt x8, x9, x4
sltu x9, x9, x4
addi x1, x1, 0x220
xori x2, x2, 0x333
ori x3, x3, 0x110
andi x4, x4, 0x321
slli x5, x1, 0x1
srli x6, x15, 0x10
srai x7, x15, 0x1
slti x8, x8, -0x1
sltiu x9, x10, -0x1

for (int i = 0; i < n; i++) {
    mem[addr+i+n] = mem[i] & 0xdf
}



data starts at 0x4c

x15 <- n + addr
x16 <- 0 + addr

jal cond
body:

    lw x18, 0(x16)
    andi x18, x18, 0xdf
    addi x16, x16, 1
    sh x18, 0(x16)
    
cond:
    bne x15, x16, body
    nop
    addr

    