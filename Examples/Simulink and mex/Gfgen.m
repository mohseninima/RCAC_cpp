function [Nu, Du] = Gfgen(Gf)

lz = size(Gf,1);
lu = size(Gf,2);

[num, den] = tfdata(Gf);
order = length(den{1,1});
Nu = zeros(lz,lu*order);
Du = zeros(lz,lz*(order-1));

for m = 1:lu
    for n = 1:lz
        if num{n,m} == 0        
            num{n,m} = 0*den{1,1};
        end
    end
end

%Note that Gf is assumed to be expressed with a common denominator.
if lz == 1 && lu == 1
    Nu = num{1,1};
else
    for l = 1:order
        for m = 1:lu
            for n = 1:lz
                Nu(n,m+lu*(l-1)) = num{n,m}(l);
            end
        end
    end
end

%Did this the lazy way...., probably could be more direct

denrc = den{1,1}(2:end);

if lz == 1
    Du = denrc;
else
    for l = 1:order-1
        for m = 1:lz
            for n = 1:lz
                if m == n
                    Du(n,m+lz*(l-1)) = denrc(l);
                else
                    Du(n,m+lz*(l-1)) = 0;
                end
            end
        end
    end
end

end
%
