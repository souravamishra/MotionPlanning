#!/bin/sh


################################
# Run test.SimpleAccountTest
################################
#java -classpath $CLASSPATH ${OPTS} test.SimpleAccountTest

################################
# Run test.DatabaseAccountTest
################################
for i in {1..50}
do
	./motionplanning 2>result.out
done
