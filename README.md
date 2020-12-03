# RL-PC-Cleaning
Open source for the RL-assisted PC cleaning scheme. We propose to mitigate the tail latency of PC cleaning of the DM-SMR drive by using reinforcement learning technology. Specifically, we build a real-time lightweight Q-learning model to analyze the idle window of I/O workloads, based on which PC cleaning is judiciously scheduled, thereby maximally utilizing the I/O idle window and effectively hiding the tail latency from regular requests. In this project, we implement the controller of the DM-SMR drive as an *Linux device mapper target*, which enables you to conduct the experiments and test your design strategies based on a real conventional HDD disk with this prototype.


### Run:
1. Compile the whole project and generate the driver module.
2. Insert the driver module.
3. Create a simulated SMR disk.
4. Perform the block I/O workload and set up the *blktrace* tool.
5. Analyze the collected data using *blkparse & btt* tool and get the latency information.


### Notes: 
1. The linux kernel version is *4.10.0-42-generic (ubuntu 16.04)*.
2. We build our scheme based on the initial version published on [*FAST'15 - Skylight*](http://sssl.ccs.neu.edu/skylight)[1]. This is a very creative and enlightening work.
3. We use [*hfplayer*](https://github.com/umn-cris/hfplayer) as our block I/O replayer. This amazing work has been published on *FAST'17*[2] and you can find more details on their paper and Github website.
4. In order to get more accuate results, you have to shut down the write cache and the automatic power-saving mechanism of your disk with the help of *hdparm* tool.

### Support:
Please post your question in the github Issues page: https://github.com/dlzrmr99/RL-PC-Cleaning/issues.


### Citations:
[1] Aghayev A, Shafaei M, Desnoyers P. Skylight—a window on shingled disk operation[J]. ACM Transactions on Storage (TOS), 2015, 11(4): 1-2 <br/>
[2] Haghdoost A, He W, Fredin J, et al. On the accuracy and scalability of intensive I/O workload replay[C]//15th {USENIX} Conference on File and Storage Technologies ({FAST} 17). 2017: 315-328.
