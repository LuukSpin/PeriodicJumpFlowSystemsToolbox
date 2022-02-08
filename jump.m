function xplus = jump(x,Ad,Bwd,wd)
x = x(1:end-1);
xplus = [Ad*x + Bwd*wd;0]; 
end