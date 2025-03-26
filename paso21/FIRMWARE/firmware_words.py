with open('paso21_fix.hex', 'a') as f2:
    with open('paso21.hex', 'r') as f:
        for line in f:
            b = [line[9:-2][0:8], line[9:-2][8:16], line[9:-2][16:24], line[9:-2][24:32]]
            hex = []
            for x in b:
                c = x[6:] + x[0:-2]
                d = c[0:2] + c[6:8] + c[2:6]
                e = d[0:4] + d[6:8] + d[4:6]
                hex.append(e)
            f2.write(' '.join(hex) + '\n')
