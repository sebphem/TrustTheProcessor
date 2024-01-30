Trust the Tuition and Housing Profit: Basic Benchmark 2

Iteratively adds a custom number (x2) up to a custom total (x5), and will store and load the value from memory at a custom milestone.
Stores the sum calculated in memory at address 0x20.

You can set these inputs to whatever you wish, but for consistency, the you shouldn't change the inputs for the actual benchmark.

For the benchmark, the total is set as the yearly revenue made by northwestern from student tuition and housing ($166,191,000),
the iterative number is $10,000, and you load and store from memory every $100,000. See regs_in for labels. The runtime should 
be between 1:30 and 2:00 minutes (faster, if you have a good processor).

After running, to check for correctness, check to the value in your reg_out at address 0x20, and make sure it is equal to 0x09e802c0.

Be careful with using really small iterative numbers and really large totals, as you could end up taking hundreds of millions of cycles.

If you want to test your pure adding speed, you can set the store and load value to the total (so it will only store and load once), 
and if you want to store and load every iteration, you can set it to the iterative number.  