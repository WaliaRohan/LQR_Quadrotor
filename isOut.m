function check = isOut(x, y, z)

sprintf('%f | %.2f | %12f', x, y, z);

if ((x <= 10 && x >= -10) && (y <= 10 && y >= -10) && (z <= 10 && z >= -10))
    check = false;
else
    check = true;
end