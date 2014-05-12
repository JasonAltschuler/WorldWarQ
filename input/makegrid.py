import re
import sys
import random




def diamondsquare(heights, n, x1, y1, x2, y2):
	if n == 2:
		return

### DIAMOND ATTEMPT
	# counter += 1
	

	# if counter % 2 == 1:
	# 	counter1 += 1
	# 	factor = n / pow(2, counter1)
	# 	for a in range(0, n):
	# 		for b in range(0, n):
	# 			tl = heights[(a-factor)%n][(b-factor)%n];
	# 			tr = heights[(a-factor)%n][(b+factor)%n];
	# 			bl = heights[(a+factor)%n][(b-factor)%n];
	# 			br = heights[(a+factor)%n][(b+factor)%n];

	# 			if tl != 0 or tr != 0 or bl != 0 or br != 0:
	# 				heights[a][b] = (tl+tr+bl+br)/4 + random.uniform(-factor/3, factor/3);


	# else:
	# 	counter2 += 1
	# 	factor = n / pow(2, counter2)
	# 	for a in range(0, n):
	# 		for b in range(0, n):
	# 			up = heights[(a-factor)%n][b];
	# 			right = heights[a][(b+factor)%n];
	# 			left = heights[a][(b-factor)%n];
	# 			down = heights[(a+factor)%n][b];

	# 			if up != 0 or right != 0 or left != 0 or down != 0:
	# 				heights[a][b] = (up+right+left+down)/4 + random.uniform(-factor/3, factor/3);


	# diamondsquare(heights, n/2 + 1, counter, counter1, counter2);


	# grab corners
	tl = heights[x1][y1];
	tr = heights[x1][y2];
	bl = heights[x2][y1];
	br = heights[x2][y2];

	# calculate center and midpoints
	tm = (tl+tr)/2 + random.uniform(-n/3, n/3);
	lm = (tl+bl)/2 + random.uniform(-n/3, n/3);
	rm = (tr+br)/2 + random.uniform(-n/3, n/3);
	bm = (br+bl)/2 + random.uniform(-n/3, n/3);
	# print("tm " + str(tm));
	# print("lm " + str(lm));
	# print("rm " + str(rm));
	# print("bm " + str(bm));

	center = (tl+tr+bl+br)/4 + random.uniform(-n/3, n/3);
	# print("center " + str(center));


	middle_x = (x1+x2)/2;
	middle_y = (y1+y2)/2;
	# print(middle_x);
	# print(str(middle_y) + "\n");

	heights[middle_x][middle_y] = center;
	heights[x1][middle_y] = tm;
	heights[middle_x][y1] = lm;
	heights[middle_x][y2] = rm;
	heights[x2][middle_y] = bm;

	diamondsquare(heights, n/2 + 1, x1, y1, middle_x, middle_y);
	diamondsquare(heights, n/2 + 1, x1, middle_y, middle_x, y2);
	diamondsquare(heights, n/2 + 1, middle_x, y1, x2, middle_y);
	diamondsquare(heights, n/2 + 1, middle_x, middle_y, x2, y2);



def makegrid(n, filename):
	fp = open(filename, 'w');

	fp.write("OFF\n");

	nVertices = n * n;
	nFaces = (n-1) * (n-1);
	nEdges = n * (n-1) + n * (n-1);

	fp.write(str(nVertices) + " " + str(nFaces) + " " + str(nEdges) + "\n");


	# initial corner heights
	rand = round(random.uniform(0, n/3.0), 2)
	heights[0][0] = rand;
	rand = round(random.uniform(0, n/3.0), 2)	
	heights[0][n-1] = rand;
	rand = round(random.uniform(0, n/3.0), 2)
	heights[n-1][0] = rand;
	rand = round(random.uniform(0, n/3.0), 2)
	heights[n-1][n-1] = rand;

	# DIAMOND ATTEMPT
	# diamondsquare(heights, n, 0, 0, 0);

	diamondsquare(heights, n, 0, 0, n-1, n-1);

	# for a in range(0, n):
	# 	for b in range(0, n):
	# 		print(heights[a][b]);
	# 	print();

        #added by kyle: scale factor:
	scale = 100
	heightscale = 50
        
	# vertex coordinates
	for a in range(0, n):
		for b in range(0, n):
			fp.write(str(a*scale) + " " + str(b*scale) + " " + str(heights[a][b]*heightscale) + "\n");

	# now write faces with their vertices (do two triangle at a time)
  
    
	for a in range(0, n-1):
		for b in range(0, n-1):
			vertex1 = a*n + b
			vertex2 = vertex1 + 1
			vertex3 = (a+1)*n + b + 1
			vertex4 = vertex3 - 1
			fp.write("3 " + str(vertex1) + " " + str(vertex2) + " " + str(vertex3) + "\n")
			fp.write("3 " + str(vertex3) + " " + str(vertex4) + " " + str(vertex1) + "\n")

def is_power2(num):
	return num != 0 and ((num & (num - 1)) == 0)



###################### MAIN ######################

n = int(sys.argv[1]);

global heights
# will hold our vertex heights
heights = [[0 for x in range(0,n+1)] for x in range (0,n+1)];

if not(is_power2(n)):
	print("arg1 must be a power of 2");
else:
	makegrid(n + 1, sys.argv[2]);
