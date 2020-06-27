import matplotlib.pyplot as plt
import numpy as np
import math

# ファイルをオープンする
test_data = open("3DClassData_20200625_1.txt", "r")

dataArray=test_data.read().split('\n')

# ファイルをクローズする
test_data.close()

X=[]
Y=[]
Plotcolor=[]
classLen=2
classify=0

# 一行ずつ読み込んでは表示する
for line in dataArray:
  vs=line.split()

  if len(vs)>=3:
    X.append(float(vs[1])*math.cos(float(vs[2]))/1000)
    Y.append(float(vs[1])*math.sin(float(vs[2]))/1000)
    Plotcolor.append([int(vs[0])/classLen,int(vs[0])*(int(vs[0])%2)/classLen,1-int(vs[0])/classLen])

    # print(int(vs[0]))
    # print(vs[1])
    # print(vs[2])

  # print("\n")


# print(Plotcolor)

plt.scatter(X,Y,s=1,color=Plotcolor)

plt.show()

