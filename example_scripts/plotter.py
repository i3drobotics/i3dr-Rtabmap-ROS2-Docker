import matplotlib.pyplot as plt

x = [1,2,3,4,5,6,7,8,9,10]
y = [i**2 for i in x]
plt.title('Graph')
plt.xlabel('x-axis')
plt.ylabel('y-axis')
plt.plot(x,y)
plt.show()