import sys
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import numpy as np
file_name = "data.txt"

# s1=[]
# s2=[]
# s3=[]
# s4=[]
# s5=[]
# s6=[]
# s7=[]
# s8=[]

# with open(file_name, "r") as f:
#     for i in f.readlines():
#         s = i.split(',')
#         s1.append(int(s[0]))
#         s2.append(int(s[1]))
#         s3.append(int(s[2]))
#         s4.append(int(s[3]))
#         s5.append(int(s[4]))
#         s6.append(int(s[5]))
        # s7.append(int(s[6]))
        # s8.append(int(s[7]))


lines=[]
while True:
    try:
        lines.append(input())
    except:
        break
 
# print(lines)

xs = []
ys = []
zs = []

for i in range(len(lines)):
    print(lines[i])
    if lines[i].find('Translation:') != -1:
        x = float(lines[i+1][-8:-3])
        print(x)
        xs.append(x)
        y = float(lines[i+2][-8:-3])
        ys.append(y)
        z = float(lines[i+3][-8:-3])
        zs.append(z)

print(xs)
print(ys)
print(zs)

fig = plt.figure()
ax = plt.axes(projection='3d')
ax.scatter3D(xs,ys,zs)
plt.show()

    
