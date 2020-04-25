CC=g++
CFLAGS=-I.
DEPS = OccGrid.h Robot.h RobotNetwork.h
OBJ = OccGrid.o Robot.o RobotNetwork.o RobotNetwork_test.o

%.o: %.cpp $(DEPS)
	$(CC) -c -o $@ $< $(CFLAGS)

test: $(OBJ)
	$(CC) -o $@ $^ $(CFLAGS)