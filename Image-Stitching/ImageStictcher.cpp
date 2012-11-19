
//#include "stdafx.h"

#include <cv.h>
#include <highgui.h>
#include "opencv2/stitching/stitcher.hpp"

#include <iostream>

#include <vector>

using namespace std;
using namespace cv;


/* Global Variables */

vector<Mat> inputImages = vector<Mat>(); //vector to store all images in
Stitcher stitcherObj;										 // stitcher to stitch images


/*
 * Constructor  
 */


ImageStitcher( )
{
	inputImages.stackPush( recievedImage );
}



void threadImages( Stitcher stitcher )
{
	/* Take the images in the inputImage Mat vector and add them into 
	 * a new vector, imagesStitch, to be stitched. 
	 */
  int i = inputImages.size();
  vector<Mat> imagesStitch = vector<Mat>(i);
		
  int j;
	for( j = 0; j < i; j++ )		//adds images into new vector
	{
		imageStitch[j] = inputImages.pop();
	}
	
	
	/* Call stitcher on new vector of images */
	stitcher.stitch( imagesStitch, pano );
		
	/* If the images were able to stitch properly, stitcher will return true.
	 * In this case return the new image and push it into the BEGINNING of the
	 * inputImages vector */
	if( stitcher == true )
	{
		imwrite( "newImage.jpg", pano );	// writes pano into image
		
		Mat result = imread( "newImage.jpg" );	//store new image into vector
		inputImages.queuePush( result );
	}
	
	/* If stitcher returned false - not enough images were inputted into the
	 * vector to be stitched
	 *
	 * I assume that we will just wait until other images are recieved.
	 */
	
	else
	{//push back into
	}
		sleep(200ms);
	
}




/* Pushes image to the front of the vector.
 * This method is used once the image has been stitched and is put back into 
 * the vector.
 */

void queuePush( Mat panoImage )
{	
	vector<Mat>::iterator it;
	inputImages.insert( inputImages.begin(), panoImage, 1, inputImages.begin(),
											inputImages.begin()+1 );
}




/* Pushes image to the end of the vector.
 * This is used when a new image is recieved, the image must be put 
 * in order of the time recieved into directory, therefore the newer images
 * are added to the END. 
 */

void stackPush( Mat newImage )
{
	inputImages.push_back( newImage );
	
	stitcherObj = Stitcher::createDefault(true);
	threadImages( stitcherObj, inputImages );
}


/* Removes and returns the Mat image at the beginning of the vector storing 
 * the images.  
 */


Mat pop()
{
	if( inputImages.size() >= 1 )
	{
		Mat imageAtFront = inputImages[0];
		
		vector<Mat>::iterator it;
		inputImages.erase( inputImages.begin() );
		
		return imageAtFront;
	}
	else
		return; 
}

