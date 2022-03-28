function y = lerp(x1,x2,y1,y2,x)

y = y1 + (x - x1)*(y2-y1)/(x2-x1);

end

