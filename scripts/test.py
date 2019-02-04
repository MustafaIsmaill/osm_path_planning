with open('gps_path.txt') as f:
    lines = f.read().splitlines()

xarr = []
yarr = []

for line in lines:
	x, y = line.split(',')
	xarr.append(float(x))
	yarr.append(float(y))
