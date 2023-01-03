function SDSystem = minus(objSD1, objSD2)
arguments
    objSD1 (1,1) OpenLoopSampledDataSystem
    objSD2 (1,1) OpenLoopSampledDataSystem
end

SDSystem = objSD1 + (-objSD2);

end