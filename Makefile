Khexplorer: Khexplorer.o KheperaInterface.o KheperaUtility.o Controller.o ValueSystem.o Operator.o ThreadableBase.o SmartKhepera.o
	g++ -o $@ $+ -pthread
	
%.o: %.cpp 
	g++ -std=c++11 -c  $+  #compiler option: '-std=c++11' for older gcc version
	
clean:
	rm -f *.o	
	
veryclean: clean
	rm -f test
	
