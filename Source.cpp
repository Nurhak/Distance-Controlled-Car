// 140207084 Mohamad Nour Ahmad
// Electronics and Telecommunication engineering

#include <opencv2/objdetect/objdetect.hpp>   // include neccessary header files
#include <opencv2/highgui/highgui.hpp>       
#include <opencv2/imgproc/imgproc.hpp> 
#include <iostream>
#include <stdio.h>
#include <ctime>    // include time library to enable using clock() function
#include <cmath>    // include math library for use it in tan(x) calculation
#include <unistd.h>			//Used for UART
#include <fcntl.h>			//Used for UART
#include <termios.h>		//Used for UART

using namespace std;
using namespace cv;

// my own written function to calculate distance and calibrate the camera
inline int px_distance(float x, float h, float teta, float alfa, int m);
// this function returns the distance of the detected object in meters 
inline float x_distance(float rx, float h, float teta, float alfa, int m);

int main(int argc, const char** argv)
{
	int uart0_filestream = -1;printf("Hello here we start \n");
	uart0_filestream = open("/dev/ttyS0", O_RDWR | O_NOCTTY | O_NDELAY);		//Open in non blocking read/write mode
	if (uart0_filestream == -1)
	{
		//ERROR - CAN'T OPEN SERIAL PORT
		printf("Error - Unable to open UART.  Ensure it is not in use by another application\n");
	}
		struct termios options;
	tcgetattr(uart0_filestream, &options);
	options.c_cflag = B115200 | CS8 | CLOCAL | CREAD;		//<Set baud rate
	options.c_iflag = IGNPAR;
	options.c_oflag = 0;
	options.c_lflag = 0;
	tcflush(uart0_filestream, TCIFLUSH);
	tcsetattr(uart0_filestream, TCSANOW, &options);
	
	clock_t begin = clock();          // used to begin calculating processing time 
	float interval = 0.0;             // used to calculating processing time 
	CascadeClassifier nesne;          //make nesne object in Cascade Classifier 
	nesne.load("/home/pi/fifthterm/cascade.xml");        //the database which I have used to detect vehicles
	VideoCapture vid;                 //make vid named VideoCapture object which used to capture from camera 
	vid.open(0);                      //capture from webcam

	if (!vid.isOpened())     //check if webcam is opened
	{
		cout << "Unable to open webcam ! :-( " << endl;
		system("Pause");
		return -1;
	}


	int height = vid.get(CV_CAP_PROP_FRAME_HEIGHT); //get camera parameters to print it 
	int width = vid.get(CV_CAP_PROP_FRAME_WIDTH);   //in order to give information to users
	height = 240; width = 320;
	char str[55];  //character array which used top left information
	char str1[65]; //character array which used to print lines with corresponding distance value
	char dist_str[20]; // used to print distance on the observed object
	char time_str[10]; // used to print processing time to evaluate program performance
	int r[11];  // store the pixel equivalent of real distances
	int i = 0;  //counter
	float hold; //holds the pixel_y parameter of observed object
	float dist = 0;  //distance of observed object (car)

					 //camera calibration parameters
	float high = 0.08;
	float teta = .01;
	float alfa = (3.14 / 11);
	//	int horizon = px_distance(100, high, teta, alfa, height);
	//	int edge1 = 100;
	//	int edge2 = (width - edge1);

	for (int j = 1; j <= 11; j++) { // print the lines which indicates to real distances
		r[i] = px_distance(j, high, teta, alfa, height); //insert camera calibration parameters
		i++;
	}
	Mat temp_img;
	Mat res;
	Mat frame;
	Mat grires;
	//	namedWindow("observed_car", 1);
	namedWindow("optimized", 1);
	char motor_stt = 'c';
	unsigned char tx_char[10];
		unsigned char *p_tx_buffer;
	
	p_tx_buffer = &tx_char[0];
	Size size(width, height);

	clock_t end = clock();

	while (true)
	{
		clock_t begin = clock();
		p_tx_buffer = &tx_char[0];

		vid >> frame;
		//vid >> res;
		//resize(res, frame, size);
		//		Rect myrect(edge1, horizon,(edge2-edge1), (height - horizon));
		//		temp_img = frame (myrect);
		cvtColor(frame, grires, CV_BGR2GRAY);  //convert to grayscale image
											   //		grires = grires + Scalar(55,55,55);
											   //		grires.convertTo(grires,-1,2,0);
		vector<Rect> nesvek;  //make a vector which holds rectangle objects

							  //detecting the existence of objects which determined (classified) in nesne object
							  //which uses cascade.xml database 
		nesne.detectMultiScale(grires, nesvek, 1.1, 3, 0, Size(60,60));
		//here Size indicates scanning size of predictable objects if make it big we can not 
		//observe small objects

		for (int i = 0; i < nesvek.size(); i++) //this used to print rectangles surrounding observed objects
		{
			Point pt1((nesvek[i].x + nesvek[i].width), (nesvek[i].y + nesvek[i].height));
			Point pt2((nesvek[i].x), (nesvek[i].y));
			rectangle(frame, pt1, pt2, cvScalar(255, 0, 0, 0), 2, 8, 0);
			hold = (nesvek[i].y + (nesvek[i].height) / 2);
			dist = x_distance(hold, high, teta, alfa, height);//here we calculate the distance of each object
			cout << "Vehicle " << i << " at distance " << dist;
//			_snprintf_s(dist_str, 20, "dist %f", dist);
			if (dist < 0.5) {
				motor_stt = 'c';
				cout << "motor_stt " << motor_stt;
			}
			else if( dist >= 0.5 && dist <= 1.5) {
				motor_stt = 'a';
				cout << "motor_stt " << motor_stt;
			}
			else if(dist >1.5){
				motor_stt = 'c';	
			}
			
			cout << endl;
			putText(frame, dist_str, Point((nesvek[i].x), (nesvek[i].y)), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 255));
		}
		*p_tx_buffer++ = motor_stt;
		cout << "interval " << interval << endl;
		//printing information about program and processing time 
	//	putText(frame, "Mohamad Nour Ahmad press ESC to end program", Point(1, 15), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 255));
	//	_snprintf_s(time_str, 10, "%f", interval);
	//	putText(frame, time_str, Point(1, 35), CV_FONT_HERSHEY_COMPLEX, 0.5, Scalar(255, 255, 255));

		
		//	imshow("observed_car", temp_img);
		imshow("optimized", frame);
		if (waitKey(50) == 27) {
			break;
		}
		clock_t end = clock();
		interval = double(end - begin) / CLOCKS_PER_SEC; //calculate the processing time
		
		if (uart0_filestream != -1)
		{
		int count = write(uart0_filestream, &tx_char[0], (p_tx_buffer - &tx_char[0]));		//Filestream, bytes to write, number of bytes to write
		if (count < 0)
			{
			printf("UART TX error\n");
			}
		}
	}
		close(uart0_filestream);
	return 0;
}


int px_distance(float x, float h, float teta, float alfa, int m) {

	float rx;
	rx = (((((h - x*tan(teta)) / (h*tan(teta) + x)) / tan(alfa)) + 1)*((m - 1) / 2)) + 1;
	return int(rx);
}

float x_distance(float rx, float h, float teta, float alfa, int m) {
	float beta, delta, distance;
	beta = 2 * (rx - 1) / (m - 1) - 1;
	delta = beta * (tan(alfa));
	distance = h*(1 - delta * tan(teta)) / (delta + tan(teta));
	return (distance);
}
