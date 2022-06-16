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

# The probability distribution of next day
x_next = pt.dot(x)
print(x_next)


# To randomly generate sequences of “weathers”
# we can use 'random.choices' to sample from a probability distribution
w = ['sunny','cloudy','rainy']
random_number = random.choices(w, pt.dot(x_next))
print(random_number)

