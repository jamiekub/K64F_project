function mat = myrotz(ang)
    mat = [ cosd(ang), sind(ang), 0;
           -sind(ang), cosd(ang), 0;
                    0,         0, 1];
end