Trust the SHOUTING Rainbow: Basic Benchmark 1

Simply compare your reg_out with the provided reg_out in this folder to check for correctness.

Also, in the DATA section in mem_in, you can copy and paste a sequence of bytes that will be captialized by the code. the input chars are located at 0x70,
and the captialized characters will be stored right after the end of the sequence starting at 0x70. You must input the character length of your sequence 
into x16 before running the code. 

As an example, the characters "65 78 61 6D 70 6C 65" are in the DATA section, so x16 is equal to 7; the number of bytes in the sequence.
When you run the code, the data right after the above sequence becomes "45 58 41 4d 50 4c 45", instead of all zeros like before. 
Converting to ascii, the input "65 78 61 6D 70 6C 65" is "example" and the output "45 58 41 4d 50 4c 45" is "EXAMPLE". 
Feel free to input your own sequence and see it captialized. 