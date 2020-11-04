"data" stores the preliminary data of the simulation test and the analytical results which are used to calculate the Maximum Deviation.

"draw" stores the analytical results which are used for drawing figures.

"Results" stores the results of each simulation test and simulation results of averaging the results obtained by the eight simulation tests.

"Simulate.m" is the function to calculate the simulation results according to the data stored in the folder "average results", calculate the Maximum Deviation according to the simulation results and the data stored in the folder "data", and draw Fig. 6-8 in our manuiscript.
"protocol.m" is the function to realize the realism test of 802.11p protocol and calculate the mean and variance of service time. 
"Add.m" is the function to increase the number of transmitting vehicles. 
"fun.m" is the function to describe the PSFFA equation for AC0.
"fun1.m" is the function to describe the PSFFA equation for AC1.
"Increase.m" is the function to double the contention window.
"Pop.m" is the function to decrease the number of packets for ACq in a vehicle.
"Push.m" is the function to store the length of packets and the number of packets in the queue which are increased by one.
"RandSel.m" is the function to randomly generate a value for backoff counter.
