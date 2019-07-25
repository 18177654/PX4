uint64 timestamp	# time since system start (microseconds)

# Adaptive control
float32 u		# control input
float32 ym		# reference model

# Adaptive parameters
float32 the11 		# adaptive parameter theta_11
float32 the12 		# adaptive parameter theta_12
float32 the21 		# adaptive parameter theta_21
float32 the22 		# adaptive parameter theta_22
float32 the3 		# adaptive parameter theta_3
float32 c0 		# adaptive parameter c_0
float32 rho 		# adaptive parameter rho
