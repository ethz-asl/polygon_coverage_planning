# Usage: ./runGLKH problem_file tour_file
#!/bin/bash

par=$(basename $1).pid$$.par

echo "PROBLEM_FILE = $1" > $par
echo "ASCENT_CANDIDATES = 500" >> $par
echo "INITIAL_PERIOD = 1000" >> $par
echo "MAX_CANDIDATES = 30" >> $par
echo "MAX_TRIALS = 1000" >> $par
echo "OUTPUT_TOUR_FILE = $2" >> $par
echo "POPULATION_SIZE = 1" >> $par
echo "PRECISION = 10" >> $par
echo "RUNS = 1" >> $par
echo "SEED = 1" >> $par
echo "TRACE_LEVEL = 1" >> $par

mkdir -p TMP
rosrun polygon_coverage_solvers GLKH $par $3
/bin/rm -f $par
