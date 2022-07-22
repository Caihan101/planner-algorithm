from turtle import color
import matplotlib.pyplot as plt;


def TxtReader(dir1):
	f1 = open(dir1, "r")
	line1 = f1.readline()
	x, y = [], []
	while line1:
		linedate1 = line1.split(",")
		if line1 == "\n":
			break
		x.append(float(linedate1[0]))
		y.append(float(linedate1[1]))
		line1 = f1.readline()
	plt.plot(x, y, "bo")

def PathReader(dir1, color1):
	f1 = open(dir1, "r")
	line1 = f1.readline()
	x, y = [], []
	while line1:
		linedate1 = line1.split(",")
		if line1 == "\n":
			break
		x.append(float(linedate1[0]))
		y.append(float(linedate1[1]))
		line1 = f1.readline()
	plt.plot(x, y, color1)

def BoxReader(dir, color):
	f = open(dir, "r")
	line = f.readline()
	x_min_box, y_min_box, x_max_box, y_max_box = [], [], [], []
	while line:
		linedate = line.split(",")
		if line == "\n":
			break
		x_min_box.append(float(linedate[0]))
		y_min_box.append(float(linedate[1]))
		x_max_box.append(float(linedate[2]))
		y_max_box.append(float(linedate[3]))
		line = f.readline()
	x_box, y_box = [], []
	for i in range(0, len(x_min_box)):
		x_box.clear()
		y_box.clear()
		x_box.append(x_min_box[i])
		y_box.append(y_min_box[i])
		x_box.append(x_max_box[i])
		y_box.append(y_min_box[i])
		x_box.append(x_max_box[i])
		y_box.append(y_max_box[i])
		x_box.append(x_min_box[i])
		y_box.append(y_max_box[i])
		x_box.append(x_min_box[i])
		y_box.append(y_min_box[i])
		plt.plot(x_box, y_box, color)

TxtReader("./data/newEdge.txt")
PathReader("./data/rrtStarPath.txt", "r")
BoxReader("./data/boxData.txt", "g")
#PathReader("./data/astarPath.txt", "r")
PathReader("./data/optPath.txt", "p")
plt.show()

