import os, sys
from typing import List
from ast import literal_eval

class registers:
    def __init__(self, registers : List[int], areNumbers: bool = True):
        self.regFile = registers
        self.areNumbers = areNumbers
    
    def registersToHexFile(self, filename = ""):
        if filename != "":
            if os.path.isfile(filename):
                print(f"overwriting file {filename}")
            else:
                print("making new file")
            with open(filename, "w") as f:
                f.write(self.__regInfoIntoOneBigString(printPretty= False))
            print("done")
        else:
            self.printRegInfo()

    def printRegInfo(self, _32BitOut : bool = True, displayBase10Neg : bool = False, printOnlyTheseRegNums : List[int] = []):
        """
        Print all the registers in hex, 32 bit asks if you want it to be sexted
        """
        if _32BitOut == False:
            for count, reg in enumerate(self.regFile):
                if displayBase10Neg:
                    print(f"r{count}: {hex(reg)}")
                    continue
                print(f"r{count}: {hex((reg + (1 << 32)) % (1 << 32))}")
            return
        print(self.__regInfoIntoOneBigString(printPretty= True, printOnlyTheseRegNums=printOnlyTheseRegNums))

    def __regInfoIntoOneBigString(self, printPretty : bool = False, printOnlyTheseRegNums : List[int] = []):
        ##################################################
        #                                                #
        # private function                               #
        # returns a massive string of all the reg info   #
        #                                                #
        ##################################################
        _80s = "00000000"
        _8fs = "ffffffff"
        bigString = ""
        #if we dont specificy that we want any reg in particular, return all regs
        if len(printOnlyTheseRegNums) == 0:
            for regnum, reg in enumerate(self.regFile):
                if(reg < 0):
                    #wizard shit from stackoverflow
                    reg = hex((reg + (1 << 32)) % (1 << 32))[2:]
                    #get number of f's to add onto the front
                    reg = _8fs[0:len(reg)-8] + reg
                    #print the reg info

                    bigString += (printPretty and f"r{regnum}: 0x{reg}\n") or (not printPretty and f"{reg}\n")
                    continue
                if(reg > 0):
                    #read documentation from previous part
                    reg = hex((reg + (1 << 32)) % (1 << 32))[2:]
                    reg = _80s[0:8-len(reg)] + reg
                    bigString += (printPretty and f"r{regnum}: 0x{reg}\n") or (not printPretty and f"{reg}\n")
                else:
                    bigString += (printPretty and f"r{regnum}: 0x{_80s}\n") or (not printPretty and f"{_80s}\n")
        else:
            #this is bad code, but just prints out very specific regs instead of all of them
            for regnum in printOnlyTheseRegNums:
                #ONLY PART THAT HAS CHANGED SINCE THE FIRST PART
                reg = self.regFile[regnum]
                if(reg < 0):
                    #wizard shit from stackoverflow
                    reg = hex((reg + (1 << 32)) % (1 << 32))[2:]
                    #get number of f's to add onto the front
                    reg = _8fs[0:len(reg)-8] + reg
                    #print the reg info

                    bigString += (printPretty and f"r{regnum}: 0x{reg}\n") or (~printPretty and f"{reg}\n")
                    continue
                if(reg > 0):
                    #read documentation from previous part
                    reg = hex((reg + (1 << 32)) % (1 << 32))[2:]
                    reg = _80s[0:8-len(reg)] + reg
                    bigString += (printPretty and f"r{regnum}: 0x{reg}\n") or (not printPretty and f"{reg}\n")
                else:
                    bigString += (printPretty and f"r{regnum}: 0x{_80s}\n") or (not printPretty and f"{_8fs}\n")
        return bigString[:-1]

# opcode is the same
# [funct3, funct7]
nameToFunct3Funct7 = {
    "add": ["000", "0000000"],
    "sub": ["000", "0100000"],
    "xor": ["100", "0000000"],
    "or": ["110", "0000000"],
    "and": ["111", "0000000"],
    "sll": ["001", "0000000"],
    "srl": ["101", "0000000"],
    "sra": ["101", "0100000"],
    "slt": ["010", "0000000"],
    "sltu": ["011", "0000000"],
    "addi": ["000", ""],
    "xori": ["100", ""],
    "ori": ["110", ""],
    "andi": ["111", ""],
    "slti": ["010", ""],
    "sltiu": ["011", ""],
    "lb": ["000", ""],
    "lh": ["001", ""],
    "lw": ["010", ""],
    "lbu": ["100", ""],
    "lhu": ["101", ""],
    "sb": ["000", ""],
    "sh": ["001", ""],
    "sw": ["010", ""],
    "beq": ["000", ""],
    "bne": ["001", ""],
    "blt": ["100", ""],
    "bge": ["101", ""],
    "bltu": ["110", ""],
    "bgeu": ["111", ""],
    # Add more instructions as needed
}



class RTypeInstruction:
    def __init__(self, name : str, rs1: int, rs2: int, rd: int):
        ################################
        # all regs are 0-31            #
        # rs1 : first arg in comp      #
        # rs2 : sec arg in comp        #
        # rd  : desination reg of comp #
        ################################
        self.opcode = "0110011"
        self.funct3 = nameToFunct3Funct7[name][0]
        self.funct7 = nameToFunct3Funct7[name][1]
        self.rs1 = rs1
        self.rs2 = rs2
        self.rd  = rd
        self.name = name


    def computeOnReg(self, regs: registers = None) -> registers:
        match(self.name):
            case "add":
                # print(f"rd before: {regs.regFile[self.rd]}")
                # print(f"rs1 before: {regs.regFile[self.rs1]}")
                # print(f"rs2 after: {regs.regFile[self.rs2]}")
                regs.regFile[self.rd] = regs.regFile[self.rs1] + regs.regFile[self.rs2]
                # print(f"rd after: {regs.regFile[self.rd]}")
                return regs
            case "sub":
                regs.regFile[self.rd] = regs.regFile[self.rs1] - regs.regFile[self.rs2]
                return regs
            case "xor":
                regs.regFile[self.rd] = regs.regFile[self.rs1] ^ regs.regFile[self.rs2]
                return regs
            case "or":
                regs.regFile[self.rd] = regs.regFile[self.rs1] | regs.regFile[self.rs2]
                return regs
            case "and":
                regs.regFile[self.rd] = regs.regFile[self.rs1] & regs.regFile[self.rs2]
                return regs
            case "sll":
                regs.regFile[self.rd] = (regs.regFile[self.rs1] << regs.regFile[self.rs2]) & 0xFFFFFFFF
                return regs
            case "srl":
                regs.regFile[self.rd] = (regs.regFile[self.rs1] >> regs.regFile[self.rs2] ) & 0xFFFFFFFF
                return regs
            case "sra":
                # yoinked from stackoverflow
                # x is an n-bit number to be shifted m times
                x = regs.regFile[self.rs1]
                n = 32
                m = regs.regFile[self.rs2]
                if x & 2**(n-1) != 0:  # MSB is 1, i.e. x is negative
                    filler = int('1'*m + '0'*(n-m),2)
                    regs.regFile[self.rd] = (x >> m) | filler  # fill in 0's with 1's
                else:
                    regs.regFile[self.rd] = x >> m
                return regs
            case "slt":
                regs.regFile[self.rd] = regs.regFile[self.rs1] < regs.regFile[self.rs2]
                return regs
            case "sltu":
                print(f"pure val, sign removed ")
                regs.regFile[self.rd] = 1 if (regs.regFile[self.rs1] if regs.regFile[self.rs1] > 0 else -1*regs.regFile[self.rs1] +1)\
                <  (regs.regFile[self.rs2] if regs.regFile[self.rs2] > 0 else -1*regs.regFile[self.rs2] +1) else 0
                return regs

    def encodeInstToLilEndianHex(self, printBigEndianInst: bool = False):
        localinst = self.funct7 + f'{self.rs2:05b}' + f'{self.rs1:05b}' + self.funct3 + f'{self.rd:05b}' + self.opcode
        # return localinst
        # return reverseHexEndianness(localinst, printBigEndianInst)
        return binaryToHexLilEndian(localinst, printBigEndianInst)

    def printInst(self):
        print(f"r{self.rd} = r{self.rs1} {self.name} r{self.rs2}")



class ITypeInstruction:
    def __init__(self, name : str, rs1: int, imm: int, rd: int):
        ################################
        # all regs are 0-31            #
        # rs1 : first arg in comp      #
        # imm : imm arg in comp        #
        # rd  : desination reg of comp #
        ################################
        self.opcode = "0010011"
        self.funct3 = nameToFunct3Funct7[name][0]
        self.rs1 = rs1
        if name == "srai":
            # srai, imm[5:11] = 20
            imm += 512
        self.imm = imm
        self.rd  = rd
        self.name = name

    def encodeInstToLilEndianHex(self, printBigEndianInst: bool = False):
        localinst = f'{self.imm:012b}' + f'{self.rs1:05b}' + self.funct3 + f'{self.rd:05b}' + self.opcode
        # return localinst
        return binaryToHexLilEndian(localinst, printBigEndianInst)

    def computeOnReg(self, regs: registers = None) -> registers:
        match(self.name):
            case "addi":
                regs.regFile[self.rd] = regs.regFile[self.rs1] + self.imm
                return regs
            case "xori":
                regs.regFile[self.rd] = regs.regFile[self.rs1] ^ self.imm
                return regs
            case "ori":
                regs.regFile[self.rd] = regs.regFile[self.rs1] | self.imm
                return regs
            case "andi":
                regs.regFile[self.rd] = regs.regFile[self.rs1] & self.imm
                return regs
            case "slli":
                regs.regFile[self.rd] = (regs.regFile[self.rs1] << self.imm) & 0xFFFFFFFF
                return regs
            case "srli":
                regs.regFile[self.rd] = (regs.regFile[self.rs1] >> self.imm) & 0xFFFFFFFF
                return regs
            case "srai":
                # yoinked from stackoverflow
                # x is an n-bit number to be shifted m times
                x = regs.regFile[self.rs1]
                n = 32
                m = self.imm
                if x & 2**(n-1) != 0:  # MSB is 1, i.e. x is negative
                    filler = int('1'*m + '0'*(n-m),2)
                    regs.regFile[self.rd] = (x >> m) | filler  # fill in 0's with 1's
                else:
                    regs.regFile[self.rd] = x >> m
                return regs
            case "slti":
                regs.regFile[self.rd] = regs.regFile[self.rs1] < self.imm
                return regs
            case "sltui":
                print(f"pure val, sign removed ")
                regs.regFile[self.rd] = 1 if (regs.regFile[self.rs1] if regs.regFile[self.rs1] > 0 else -1*regs.regFile[self.rs1] +1)\
                <  (self.imm if self.imm > 0 else -1*self.imm +1) else 0
                return regs

    def printInst(self):
        print(f"r{self.rd} = r{self.rs1} {self.name} r{self.imm}")

class LoadStoreInstruction(ITypeInstruction):
    def __init__(self, memory : List[int]):
        self.memory = memory

    def computeOnRegAndMem(self, regs: registers = None) -> registers:
        match(self.name):
            case "lb":
                regs.regFile[self.rd] = self.memory[regs.regFile[self.rs1] + self.imm] & 0xFF
                return regs
            case "lh":
                regs.regFile[self.rd] = self.memory[regs.regFile[self.rs1] + self.imm] & 0xFFFF
                return regs
            case "lw":
                regs.regFile[self.rd] = self.memory[regs.regFile[self.rs1] + self.imm] & 0xFFFFFF
                return regs
            case "lbu":
                regs.regFile[self.rd] = abs(self.memory[regs.regFile[self.rs1] + self.imm] & 0xFF)
                return regs
            case "lhu":
                regs.regFile[self.rd] = abs(self.memory[regs.regFile[self.rs1] + self.imm] & 0xFFFF)
                return regs


class STypeInstruction:
    def __init__(self, name : str, rs1: int, rs2: int, imm: int, memory : List[int]):
        ################################
        # all regs are 0-31            #
        # rs1 : address in memory      #
        # imm : imm arg in comp        #
        # r2  : source address         #
        ################################
        self.opcode = "0000011"
        self.funct3 = nameToFunct3Funct7[name][0]
        self.rs1 = rs1
        self.rs2  = rs2
        self.imm = imm
        self.memory = memory
        self.name = name

    def encodeInstToLilEndianHex(self, printBigEndianInst: bool = False):
        localinst = f'{self.imm:012b}'[5:12] + f'{self.rs2:05b}' + f'{self.rs1:05b}' + self.funct3 + f'{self.imm:12b}'[0:5] + self.opcode
        # return localinst
        # return reverseHexEndianness(localinst, printBigEndianInst)
        return binaryToHexLilEndian(localinst, printBigEndianInst)

    def computeOnReg(self, regs: registers = None) -> registers:
        match(self.name):
            case "sb":
                self.memory[regs.regFile[self.rs1] + self.imm] = hex(int(f"0x{self.memory[regs.regFile[self.rs1] + self.imm]}",16) & 0xFFFFFF00 + regs.regFile[self.rs2] & 0xFF)[2:]
                return regs
            case "sh":
                self.memory[regs.regFile[self.rs1] + self.imm] = hex(int(f"0x{self.memory[regs.regFile[self.rs1] + self.imm]}",16) & 0xFFFF0000 + regs.regFile[self.rs2] & 0xFFFF)[2:]
                return regs
            case "sw":
                print("data to be stored")
                print(hex(int(f"0x{regs.regFile[self.rs2]}", 16))[2:])
                print("destination in mem:")
                print(regs.regFile[self.rs1])
                self.memory[regs.regFile[self.rs1] + self.imm] = hex(int(f"0x{regs.regFile[self.rs2]}", 16))[2:]
                return regs

    def printInst(self):
        print(f"M[r{self.rs1} + r{self.imm}]  = r{self.rs2}")


def binaryToHexLilEndian(hexString: str, printBigEndianInst : bool = False) -> str:
    i = 0
    finalInst = ""
    while True:
        byte =  "0" + hex(int(hexString[i : i+8],2))[2:] + "0"
        finalInst = byte[-3:-1] + "\n" + finalInst
        i += 8
        if i == 32:
            break
    if printBigEndianInst:
        localHexInst = hex(int(hexString,2))
        print(localHexInst)
    return finalInst[:-1]

def HexBigToLilEndian(hexString: str) -> str:
    i = 0
    finalInst = ""
    while True:
        i += 2
        byte =  "0" + hexString[i : i+2] + "0"
        finalInst = byte[-3:-1] + "\n" + finalInst
        if i == 8:
            break
    return finalInst[:-1]

def makeMemoryFile(filename, *instructions):
    """
    Memory is only 256 and 1/4 instructions long, 
    but this only takes 32 instructions to not make every test too complicatied
    Any unspecified instructions will be padded with no ops
    assumes that the instruction is already encoded
    """
    noOpLilEnd = "33\n00\n00\n00\n"
    allInst = ''
    for inst in instructions[0:31]:
        allInst += inst + "\n"
    # russ made the memory file longer than it needed to be by one byte by accident
    allInst += noOpLilEnd * (256-len(instructions)) + "00"
    if filename != "":
            if os.path.isfile(filename):
                print(f"overwriting file {filename}")
            else:
                print("making new file")
            with open(filename, "w") as f:
                f.write(allInst)
            print("done")
    else:
        Exception("filename cant be blank")

def makeMemoryArray(*instructions):
    """
    Memory is only 32 instructions long, so only add a max of 32 instructions
    Any unspecified instructions will be padded with no ops
    assumes that the instruction is already encoded
    """
    noOpLilEnd = "33000000\n"
    allInst = []
    for inst in instructions[0:31]:
        allInst.append(inst + "\n")
    allInst += [noOpLilEnd for i in range(256-len(instructions))]

def RTypeTestSuite():
    inputRegs = [
        0x0,
        0x45,
        0x10,
        0x0,
        0xffffff80,
        0x3f,
        0x0,
        0xffff,
        0xe8,
        0x0,
        0xFFa5F910, #r1 for xor
        0x005aF610, # r2 for xor
        0x0, # output
        0xFFA60, #rs1 for or
        0xFF560,
        0x0, # out
        0xFF0, # and
        0xFFF,
        0x0, #out
        0xFF000001, #sll
        0x2, # sl by this amount
        0x0, #out
        0xFF00000F, #srl
        0x4,
        0x0, #out
        0xFF00000F, #sra
        0x4,
        0x0, #out
        0xAAAAAAAA, #slt
        0xAAAAAAAB,
        0x0, #out
        0x31
    ]
    # print(inputRegs[1])
    regfile = registers(inputRegs)
    regfile.registersToHexFile("regs_in.hex")
    print("all regs:")
    regfile.printRegInfo()

    # compute on regs
    RTypeInstruction("add", 1, 2, 3).computeOnReg(regfile)
    print("reg file after add")
    regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    RTypeInstruction("sub", 1, 2, 3).computeOnReg(regfile)
    print("reg file after sub")
    regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    RTypeInstruction("add", 4, 5, 6).computeOnReg(regfile)
    RTypeInstruction("sub", 7, 8, 9).computeOnReg(regfile)
    RTypeInstruction("xor", 10, 11, 12).computeOnReg(regfile)
    RTypeInstruction("or", 13, 14, 15).computeOnReg(regfile)
    RTypeInstruction("and", 16, 17, 18).computeOnReg(regfile)
    RTypeInstruction("sll", 19, 20, 21).computeOnReg(regfile)
    RTypeInstruction("srl", 22, 23, 24).computeOnReg(regfile)
    RTypeInstruction("sra", 25, 26, 27).computeOnReg(regfile)
    RTypeInstruction("slt", 28, 29, 30).computeOnReg(regfile)

    # put to memory
    makeMemoryFile("mem_in.hex",
    RTypeInstruction("add", 1, 2, 3).encodeInstToLilEndianHex(),
    RTypeInstruction("sub", 1, 2, 3).encodeInstToLilEndianHex(),
    RTypeInstruction("add", 4, 5, 6).encodeInstToLilEndianHex(),
    RTypeInstruction("sub", 7, 8, 9).encodeInstToLilEndianHex(),
    RTypeInstruction("xor", 10, 11, 12).encodeInstToLilEndianHex(),
    RTypeInstruction("or", 13, 14, 15).encodeInstToLilEndianHex(),
    RTypeInstruction("and", 16, 17, 18).encodeInstToLilEndianHex(),
    RTypeInstruction("sll", 19, 20, 21).encodeInstToLilEndianHex(),
    RTypeInstruction("srl", 22, 23, 24).encodeInstToLilEndianHex(),
    RTypeInstruction("sra", 25, 26, 27).encodeInstToLilEndianHex(),
    RTypeInstruction("slt", 28, 29, 30).encodeInstToLilEndianHex(),
    "00\n00\n00\n00"
    )

    outputRegs = [
        0x0,
        0x45,
        0x10,
        0x35,
        0xffffff80,
        0x3f,
        0xffffffbf,
        0xffff,
        0xe8,
        0xff17,
        0xFFA5F910, #r1 for xor
        0x005AF610, # r2 for xor
        0xFFFF0F00, # output
        0xFFA60, #rs1 for or
        0xFF560,
        0xFFF60, # out
        0xFF0, # and
        0xFFF,
        0xFF0, #out
        0xFF000001, #sll
        0x2, # sl by this amount
        0xFC000004, #out
        0xFF00000F, #srl
        0x4,
        0x0FF00000, #out
        0xFF00000F, #sra
        0x4,
        0xFFF00000, #out
        0xAAAAAAAA, #slt
        0xAAAAAAAB,
        0x1, #out
        0x31
    ]
    for index, value in enumerate(regfile.regFile):
        # print(f"{value}, {outputRegs[index]}")
        if value != outputRegs[index]:
            raise Exception(f"input reg {value} and output reg {outputRegs[index]}\ndont match in reg {index}")

    regfile = registers(outputRegs)
    regfile.registersToHexFile("test_regs_out.hex")

def testUInst():
    # I type instructions
    inputRegs = [
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3
    ]
    # print(inputRegs[1])
    regfile = registers(inputRegs)
    regfile.registersToHexFile("regs_in.hex")
    print("all regs:")
    regfile.printRegInfo()



    # put to memory
    makeMemoryFile("mem_in.hex",
    "33\n00\n00\n00",
    "33\n00\n00\n00",
    "33\n00\n00\n00",
    "b7\n20\n00\n00",
    "00\n00\n00\n00"
    )

def testBranchInst():
    # I type instructions
    inputRegs = [
        0x0,
        0x1,
        0x2,
        0x3,
        0xFFFFFFFF,
        0x00FFFFFF,
        0x6,
        0x6,
        0x8,
        0x9,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3
    ]
    # print(inputRegs[1])
    regfile = registers(inputRegs)
    regfile.registersToHexFile("regs_in.hex")
    print("all regs:")
    regfile.printRegInfo()



    # # beq
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00730863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )

    # beq, misaligned
    makeMemoryFile("mem_in.hex",
    "33\n00\n00\n00",
    "33\n00\n00\n00",
    HexBigToLilEndian("0x00730963"),
    HexBigToLilEndian("0x12345678"),
    HexBigToLilEndian("0x12345678"),
    HexBigToLilEndian("0x12345678"),
    HexBigToLilEndian("0x12345678"),
    "00\n00\n00\n00",
    )

    # # branch ne
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00209863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )

    # # branch lt
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00114863"),
    # HexBigToLilEndian("0x0020c863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )
    
    # # branch ge
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x0020d863"),
    # HexBigToLilEndian("0x00115863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )

    # # branch signed lt
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00524863"),
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00115863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )

    # # branch signed ge
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x0042d863"),
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00115863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )

    # # branch ltu
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00526863"),
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00316863"),
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00115863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )

    # # branch bgeu
    # makeMemoryFile("mem_in.hex",
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00317863"),
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00527863"),
    # "33\n00\n00\n00",
    # "33\n00\n00\n00",
    # HexBigToLilEndian("0x00115863"),
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # "00\n00\n00\n00",
    # )

def testStoreInstr():
    # I type instructions
    inputRegs = [
        0x20,
        0x3,
        0x2,
        0x12345678,
        0xFFFFFFFF,
        0x00FFFFFF,
        0x6,
        0x6,
        0x3,
        0x9,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3,
        0x0,
        0x1,
        0x2,
        0x3
    ]
    # print(inputRegs[1])
    regfile = registers(inputRegs)
    regfile.registersToHexFile("regs_in.hex")
    print("all regs:")
    regfile.printRegInfo()



    # store word
    makeMemoryFile("mem_in.hex",
    "33\n00\n00\n00",
    "33\n00\n00\n00",
    "33\n00\n00\n00",
    "33\n00\n00\n00",
    "33\n00\n00\n00",
    HexBigToLilEndian("0x00402423"),
    HexBigToLilEndian("0x00400823"),
    HexBigToLilEndian("0x01002203"),
    "00\n00\n00\n00",
    )


if __name__ == "__main__":
    # if len(sys.argv) < 2:
    #     print("python3 .\makeTestCase.py reg info")
    #     assert Exception("wrong use")
    # regfile = registers(literal_eval(sys.argv[1]))
    
    # PYTHON'S HEX IS NOT 4 BYTE LONG, WILL JUST MAKE
    # THE INT AS BIG AS NEEDED
    # RTypeTestSuite()
    # testBranchInst()
    testStoreInstr()

    ################ test add ####################
    # print()
    # Inst = RTypeInstruction("add", 1, 2, 3)
    # Inst.printInst()
    # Inst.encodeInstToLilEndianHex()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()
    # addInst = Inst.encodeInstToLilEndianHex(True)
    # makeMemoryFile("mem_in.hex", addInst, addInst, addInst)

    # ################ test sub ####################
    # print()
    # Inst = RTypeInstruction("sub", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test xor ####################
    # print()
    # Inst = RTypeInstruction("xor", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test or ####################
    # print()
    # Inst = RTypeInstruction("or", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test and ####################
    # print()
    # Inst = RTypeInstruction("and", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test sll ####################
    # print()
    # Inst = RTypeInstruction("sll", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test srl ####################
    # print()
    # Inst = RTypeInstruction("srl", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test sra ####################
    # print()
    # Inst = RTypeInstruction("sra", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test slt ####################
    # print()
    # Inst = RTypeInstruction("slt", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()

    # ################ test sltu ####################
    # print()
    # Inst = RTypeInstruction("sltu", 1, 2, 3)
    # Inst.printInst()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()


    # # print(inputRegs[1])
    # regfile = registers(inputRegs)
    # regfile.registersToHexFile("firstfile.txt")
    # print("all regs:")
    # regfile.printRegInfo()

    # ################ test add ####################
    # print()
    # Inst = RTypeInstruction("add", 1, 2, 3)
    # Inst.printInst()
    # Inst.encodeInstToLilEndianHex()
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()
    # addInst = Inst.encodeInstToLilEndianHex()
    # makeMemoryFile("testmemfile.txt", addInst, addInst, addInst)

    # ################ test sw ####################
    # print()
    # Inst = STypeInstruction("sw", 1, 2, 8, makeMemoryArray())
    # print("initial reg info")
    # regfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # newregfile = Inst.computeOnReg(regfile)
    # print("end reg info")
    # newregfile.printRegInfo(printOnlyTheseRegNums=[1,2,3])
    # print()
    # addInst = Inst.encodeInstToLilEndianHex(True)
    # makeMemoryFile("mem_in.hex", addInst, addInst, addInst)



    # # addInst.computeOnReg()