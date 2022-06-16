import numpy as np
import random


### p(x|y) = η*p(y|x)*p(x)


# prior (Suppose Day1 is cloudy)
x = np.array([0, 1, 0])

# probability distribution
p = np.array([
    [0.8, 0.2, 0],
    [0.4, 0.4, 0.2],
    [0.2, 0.6, 0.2]
])
pt = p.transpose()

# To calculate the stationary distribution
for i in range(1000):
	x_next = pt.dot(x)
	n = 1 / sum(x_next) # Normalizer
	x = x_next*n

print(x_next)


