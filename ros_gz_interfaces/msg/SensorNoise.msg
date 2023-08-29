# A message for specifying sensor noise.

# Noise type
uint8 NONE               = 0
uint8 GAUSSIAN           = 2
uint8 GAUSSIAN_QUANTIZED = 3

# Optional header data.
std_msgs/Header header

# The type of noise
uint8 type

# Noise mean
# Used by GAUSSIAN and GAUSSIAN_QUANTIZED
float64 mean

# Noise standard deviation
# Used by GAUSSIAN and GAUSSIAN_QUANTIZED
float64 stddev

# Noise mean bias
# Used by GAUSSIAN and GAUSSIAN_QUANTIZED
float64 bias_mean

# Noise standard deviation bias
float64 bias_stddev

# Noise precision
# Used by GAUSSIAN_QUANTIZED
float64 precision

# For type "gaussian*", the standard deviation of the noise used to
# drive a process to model slow variations in a sensor bias.
float64 dynamic_bias_stddev

# For type "gaussian*", the correlation time in seconds of the
# noise used to drive a process to model slow variations in a sensor bias.
# A typical value, when used, would be on the order of
# 3600 seconds (1 hour).
float64 dynamic_bias_correlation_time


