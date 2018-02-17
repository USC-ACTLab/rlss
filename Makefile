all:
	g++ main.cpp curve.cpp trajectory.cpp hyperplane.cpp obstacle.cpp optimization.cpp vectoreuc.cpp bezier_integrate_2d_8pts.cpp ../../lib/csv/CSVparser.cpp -g -o main -lnlopt_cxx -larmadillo -lstdc++fs -lgsl -lgslcblas -lm -I../../lib -std=c++17
