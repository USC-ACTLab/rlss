dimensions
	n = 3 # 3 numbers for an hyperplane
	p = 36 # number of points (32 curve, 4 obstacle)
end

parameters
	A(p,n) # constraint matrix
	H(n,n) diagonal psd # cost matrix
end

variables
	w(n) # weight vector for hyperplane
end

minimize
	quad(w, H)
subject to
	A*w <= -1
end

