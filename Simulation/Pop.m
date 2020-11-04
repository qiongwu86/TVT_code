
function y = Pop(PacketBuff,n)
PacketBuff(n,:) = [PacketBuff(n,1),PacketBuff(n,3:5001),0];
%here£¬accomadate with packet buffer length
PacketBuff(n,1) = PacketBuff(n,1) - 1;% number of packets is decreased by ong.
y = PacketBuff;
