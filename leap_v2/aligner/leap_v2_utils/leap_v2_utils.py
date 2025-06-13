import numpy as np

#this goes from [0, 1] to [lower, upper]
def scale(x, lower, upper):
    return (x * (upper - lower) + lower)
#this goes from [lower, upper] to [0,1]
def unscale(x, lower, upper):
    return (((2.0 * x - upper - lower)/(upper - lower)) + 1)/2

#this averages the last two numbers and concats to the first two
def compress(four_input):
    return [four_input[0],four_input[1], (four_input[2] + four_input[3])/2]