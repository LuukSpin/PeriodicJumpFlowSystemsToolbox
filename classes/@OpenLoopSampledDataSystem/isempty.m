function flag = isempty(obj)
arguments 
    obj (1,1) OpenLoopSampledDataSystem
end

if obj.nu == 0 && obj.ny == 0 && obj.nwc == 0 && obj.nwd == 0 && obj.nzc == 0 && obj.nzd == 0
    flag = true;
else
    flag = false;
end

end