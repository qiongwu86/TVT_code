function y = Push(PacketBuff,index,PacketLength)
i = PacketBuff(index,1); % number of packet in the waiting spacing
if i < 5000     %length(PacketBuff(index,:))-l
    PacketBuff(index,i+2) = PacketLength; % length of packet
    PacketBuff(index,1) = PacketBuff(index,1) + 1; % number of packets in the waiting spacing is increase by 1
end
y = PacketBuff;
