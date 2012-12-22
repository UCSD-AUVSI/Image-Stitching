CC=g++
CFLAGS =-ggdb -O2
INC = -I/usr/local/include/opencv -I/
LIBS = -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_video -lopencv_features2d -lopencv_ml -lopencv_highgui -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching
SOURCES = stitch.cpp gpc.cpp DataTypes.cpp GPSFeaturesFinder.cpp GeoReference.cpp util.cpp test.cpp

HEADERS = MultiFeaturesFinder.h DataTypes.h

OBJECTS=$(SOURCES:.cpp=.o) 

stitch: $(OBJECTS) $(HEADERS)
	$(CC) $(CFLAGS) $(LDFLAGS) $(INC) $(LIBS) $(OBJECTS) -o $@

%.o: %.cpp
	$(CC) -c $(CFLAGS) $(INC) $(LIBS) $<

clean: 
	rm -f *.o stitch

