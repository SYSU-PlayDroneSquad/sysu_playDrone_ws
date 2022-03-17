import numpy as np

x = np.zeros((2, 48))
y = np.ones(24)

x[1, 0:len(y)] = y

print(x)
