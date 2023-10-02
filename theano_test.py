import numpy as np
import theano
import theano.tensor as T

print("Using device:", theano.config.device)

# Declare Theano symbolic variables
x = T.matrix("x")
y = T.matrix("y")

# Create a simple expression
z = x + y

# Compile the expression to a callable function
f = theano.function([x, y], z)

# Generate some data
data_x = np.random.rand(3, 3).astype(theano.config.floatX)
data_y = np.random.rand(3, 3).astype(theano.config.floatX)

print("Result:")
print(f(data_x, data_y))
