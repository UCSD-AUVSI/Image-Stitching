CC=g++ 
LDFLAGS =-ggdb
INC = -I/usr/local/Cellar/opencv/2.4.2/include/opencv
LIBS = -lopencv_core -lopencv_imgproc -lopencv_calib3d -lopencv_video -lopencv_features2d -lopencv_ml -lopencv_highgui -lopencv_objdetect -lopencv_contrib -lopencv_legacy -lopencv_stitching

SOURCES = stitch.cpp gpc.c DataTypes.cpp GPSFeaturesFinder.cpp Georeference.cpp util.cpp

stitch: $(SOURCES)
	$(CC) $(LDFLAGS) $(INC) $(LIBS) $(SOURCES) -o $@

clean: 
	rm -f *.o stitch

