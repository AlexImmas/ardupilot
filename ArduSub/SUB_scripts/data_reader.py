import matplotlib.pyplot as plt

lat = []
lon = []
alt = []
time = []

file = open("log.txt","r+")
curLine = file.readline().rstrip()
while curLine:
    curLine_values = curLine.split(", ")
    lat.append(curLine_values[1])
    lon.append(curLine_values[2])
    alt.append(curLine_values[3])
    time.append(curLine_values[0])

    for x in range(1000):
        curLine = file.readline().rstrip()

file.close()

print(len(lat))
plt.plot(time, alt)
plt.show()
