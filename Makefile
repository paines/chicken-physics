CSC = csc

ALL: physics.so physics.import.so

physics.so: physics.scm
	$(CSC) -s -O3 -d1 physics.scm -j physics

physics.import.so: physics.import.scm
	$(CSC) -s -O3 -d0 physics.import.scm

clean: physics.so
	$(RM) -f physics.so physics.import.so
