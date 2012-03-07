import serial
import matplotlib.pyplot as plt

plt.ion()
ser = serial.Serial('/dev/ttyUSB0', 38400)
counter = 0
data_list = []
for i in range(4):
    data_list.append([])
fig1 = plt.figure()

while 1:
    fun = ser.readline()
    counter += 1
    for i, word in enumerate(fun.split()):
        try:
            print i, word
            value = float(word)
            data_list[i].append(word)
        except ValueError:
            print "null value???", word
    if counter % 5 == 0:
        fig1.clear()
        for datas in data_list:
            plt.plot(datas[-200:])
        plt.draw()


#except KeyboardInterrupt:
    #plt.figure()
    #for datas in data_list:
        #plt.plot(datas)
    #plt.show()


