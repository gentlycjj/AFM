INCLUDE = $(shell pkg-config --cflags opencv)  
LIBS = $(shell pkg-config --libs opencv) -lbcm2835
SOURCES = AFM.cpp
# ....  
OBJECTS = $(SOURCES:.cpp=.o)  
# .....  
TARGET = AFM200x200  
$(TARGET):$(OBJECTS)  
	g++ -o $(TARGET) $(OBJECTS) -I $(INCLUDE) $(LIBS)  
$(OBJECTS):$(SOURCES)  
	g++ -c $(SOURCES)  
clean:  
	rm $(OBJECTS) $(TARGET)  
# .... $@...... $< .........  
%.o:%.cpp
	g++ -I $(INCLUDE) -o $@ -c $< 
