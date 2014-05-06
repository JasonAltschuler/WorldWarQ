import re
import sys


def makegrid(n, filename):
	fp = open(filename, 'w');

	fp.write("OFF\n");

	nVertices = n * n;
	nFaces = (n-1) * (n-1);
	nEdges = n * (n-1) + n * (n-1);

	fp.write(str(nVertices) + " " + str(nFaces) + " " + str(nEdges) + "\n");



	# now write vertex coordinates
	for a in range(0, n):
		for b in range(0, n):
			fp.write(str(a) + " " + str(b) + " 0\n");

	# now write faces with their vertices
	# for a in range(0, nFaces):

	# 	if (a%n == n):
	# 		continue;
	# 	vertex1 = a%n + a/n;
	# 	vertex2 = (a+1)%n + (a+1)/n;
	# 	vertex3 = (a+n+1)%n + (a+n+1)/n + n - 1;
	# 	vertex4 = (a+n)%n + (a+n)/n + n -1;
		
	for a in range(0, n-1):
		for b in range(0, n-1):
			vertex1 = a*n + b;
			vertex2 = vertex1 + 1;
			vertex3 = (a+1)*n + b + 1;
			vertex4 = vertex3 - 1;
			fp.write("4 " + str(vertex1) + " " + str(vertex2) + " " + str(vertex3) 
						+ " " + str(vertex4) + "\n");






n = int(sys.argv[1]);
# filename = open(sys.argv[2], 'w');
# n = 10;
# filename = open("test.txt", 'w');
# print("reached");
makegrid(n + 1, sys.argv[2]);
