######################################################################
# Function to perform the statistical analysis of the acquired data

from scipy import stats
from scipy.stats import ttest_rel

algorithmA = [1.59, 2.02, 2, 1.49, 2.06, 1.52, 1.57, 2.02, 2.03, 1.59, 2.1, 2.1, 2.14, 2.09, 2.08, 2.09, 2.1, 2.12, 2.06, 2.09, 2.18, 2.22, 2.2, 2.17, 2.16, 2.18, 2.24, 2.17, 2.19, 2.19]
algorithmB = [2.47, 2.49, 2.59, 2.48, 2.58, 2.57, 2.51, 2.54, 2.54, 2.54, 2.58, 2.57, 2.59, 2.51, 3.01, 3.02, 2.52, 3.03, 2.24, 3, 3.06, 3.04, 3.12, 3.09, 3.06, 2.57, 3.04, 3.11, 3.02, 3.07]

# the function ttest_rel returns the t-value and the p-value of a two-tailed t-test
t_value, p_value = stats.ttest_rel(algorithmA, algorithmB)

# we want a one-tailed t-test, so we divide the p-value by 2
my_p_value=float("{:.6f}".format(p_value/2)) 

# print the results
print("t-value: ", t_value)
print("p-value: ", my_p_value)
