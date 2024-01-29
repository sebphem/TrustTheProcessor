Trust the Towers of Hanoi: Advanced Benchmark 2

x1 is your n, or the number of disks in the Towers of Hanoi problem. There are three pillars, the source, target, and auxiliary
pillar, known to you as tower 1, 3, and 2 respectively. These numbers are stored in x2-x4. Do NOT change them.

The only number you should change is x1, and it should be in the range [1,8]. Less than 1 disk does not make sense, and once you have 
over 8 disks, the moves stored in memory start to become really long and overwrite the stack. We only have 1KB of memory, so we're limited 
by this constraint. 

This benchmark takes in n, and stores it in memory half words that can be easily read as "moves" in the Towers of Hanoi problem.
These halfwords start at 0xb0, and look something like 0x0121, or 0x0223. The way you read them is from left to right, for example:

0x0abc  --> disk a is moved from tower b to tower c.
So 0x0121 --> disk 1 is moved from tower 2 to tower 1.

For example, with 2 disks, this is the starting state:
     |         |         |
    ---        |         |
   ------      |         |

The first move is 0x0112:
     |         |         |
     |         |         |
   ------     ---        |

Next 0x0213:
     |         |         |
     |         |         |
     |        ---      ------     

And finally 0x0123:
     |         |         |
     |         |        ---
     |         |       ------   

With 8 disks, there are 255 moves, so the recursion takes a really long time. Be patient, you are not stuck in an infinite loop. 

There is no reg_out to compare with your results, just take a look at your mem_out starting at 0xb0 to see all of the moves with your given n.

See if you notice any pattern with the disk number of each move. 