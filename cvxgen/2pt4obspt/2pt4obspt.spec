dimensions
	n = 3 # 3 numbers for an hyperplane
	p = 6 # number of points (2 curve, 4 obstacle)
end

parameters
	A(p,n) # constraint matrix
	H(n,n) diagonal # cost matrix
end

variables
	w(n) # weight vector for hyperplane
end

minimize
	quad(w, H)
end

subject to
	A*w <= -1
end
