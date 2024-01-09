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

def testStoreInst():
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

    haltInst = "00\n00\n00\n00"
    noOp = "33\n00\n00\n00"

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