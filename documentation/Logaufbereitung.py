import matplotlib.pyplot as plt
import numpy as np
import tkinter as tk
from tkinter import filedialog
import csv
import re
clean_vars = []    
special_characters = "!@#$%^&*()_-+=<>,./?;:'\"[]{}\\|`~"
translation_table = str.maketrans("", "", special_characters)

root = tk.Tk()
root.withdraw()
f = open("export.txt", 'w', newline='')
writer = csv.writer(f)
file_path = filedialog.askopenfilename()
#falsche Spaltenanzahl rausfiltern
with open(file_path, 'r') as csvfile:
    reader = csv.reader(csvfile, delimiter=',')

          
    for row in reader:
        # clean up your file here and append it to the list
        #print (len(row))
        if len(row)==4:
            OK=1
            #Zeilen mit korupten Daten rausschmeißen
            for column in row:
                try:
                    myvar= int(column)
                    
                except:
                    OK=0
                    print("Bäh")
                #clean_vars.append([char for char in column if char.isalnum()])
            if OK:
                clean_vars.append([char for char in row if char])
                #print (row)
                
        else:
            print("Ihh")
    
#na=np.genfromtxt(file_path,delimiter=',',invalid_raise=False)

na = np.asarray(clean_vars)

na=na.astype(int)
#print (na)

    




x1=np.int16(na[:,0:1])
writer.writerow(x1)
#x2=na[:,2:3]
#x3=na[:,1:2]
y=np.uint16(na[:,1:2])
#writer.writerow(x2)
z=np.int16(na[:,3:4])
#writer.writerow(y)

z1=np.int16(na[:,2:3])
#writer.writerow(z1)
f.close()
#y=na[:,8:9]
#fig, axs = plt.subplots(3,sharex='all')

fig = plt.figure()

gs = fig.add_gridspec(3, hspace=0.1)
axs = gs.subplots(sharex=True)
plt.subplots_adjust(left=0.1, right=0.8, top=0.9, bottom=0.1)
fig.set_figwidth(10)
fig.suptitle('EBiCS log plotter')

axs[0].plot(x1, label="Battery Current (mA)")
#axs[0].plot(x2, label="Battery Current calulated (mA)")

#axs[0].plot(x3, label="Drehmoment an der Kassette")
#axs[2].plot(z1, label="i_q")
axs[0].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)
axs[1].plot(y,color='g', label="Torque Signal (mV)")
axs[1].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)
axs[2].plot(z,color='r', label="i_q")
axs[2].plot(z1,color='b', label="i_q_setpoint")
axs[2].legend(bbox_to_anchor=(1.01, 0), loc="lower left",
              mode="expand", borderaxespad=0)

plt.show()
