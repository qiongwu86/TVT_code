
function y = Add(CollitionStations,index)
CollitionStations(1) = CollitionStations(1) + 1; %number of transmitting vehicles is increased by one
i = CollitionStations(1);
CollitionStations(i+1) = index; % store the vehicle which is transmitting
y = CollitionStations;
