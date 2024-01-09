import subprocess
import os

def assemble_riscv(input_assembly):
    # Temporary file names
    asm_file = "temp_input.s"
    obj_file = "temp_output.o"
    elf_file = "temp_output.elf"
    bin_file = "temp_output.bin"

    # Write the assembly to a temporary file
    with open(asm_file, 'w') as f:
        f.write(input_assembly)

    # Assemble and link the assembly file using the 32-bit toolchain
    subprocess.run(["riscv32-linux-gnu-as", "-o", obj_file, asm_file])
    subprocess.run(["riscv32-unknown-elf-ld", "-o", elf_file, obj_file])

    # Convert the ELF file to binary
    subprocess.run(["riscv32-unknown-elf-objcopy", "-O", "binary", elf_file, bin_file])

    # Read the binary and convert it to the required hex format
    with open(bin_file, 'rb') as f:
        bytes_data = f.read()

    os.remove(asm_file)
    os.remove(obj_file)
    os.remove(elf_file)
    os.remove(bin_file)

    return bytes_data

def format_hex_dump(bytes_data):
    hex_data = ["@00000000"]
    for i in range(0, len(bytes_data), 4):
        word = bytes_data[i:i+4]
        hex_data.append(" ".join([f"{byte:02X}" for byte in word]))
    return "\n".join(hex_data)

if __name__ == "__main__":
    sample_program = """
    .section .data
    .section .text
    .globl _start
    _start:
        add x5, x6, x7
        sub x2, x0, x1
        lb x11, x0, 0
    """
    bytes_data = assemble_riscv(sample_program)
    hex_dump = format_hex_dump(bytes_data)
    print(hex_dump)
