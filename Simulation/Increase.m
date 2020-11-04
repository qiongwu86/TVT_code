function y = Increase(CW,n,W)
i = log2(CW(n)/W);

if  i < 1
    i = i + 1;
end
y = W*2^i; % Double contention window