#include "Main.h"
#include "util.h"
#include <math.h>
#include <Windows.h>
#include <math.h>

// OpenCV includes
#include "opencv2/video/tracking.hpp"
#include "opencv2/imgproc/imgproc.hpp"
#include "opencv2/highgui/highgui.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/features2d/features2d.hpp"
#include "opencv2/nonfree/features2d.hpp"
#include "opencv2/calib3d/calib3d.hpp"

#include <iostream>
#include <ctype.h>

using namespace cv;

float gridL = 6;

//inicializadores
GLfloat mouse_x, mouse_y;

//constantes
const double translateCameraConst = .1;
const double rotateCameraConst = 0.03;
const double rotateCameraMouseConst = 0.003;

bool buffer[250];

double frustrumTop, frustrumBottom, frustrumLeft, frustrumRight;
double g_Width, g_Height;
double zfar = 100.0f;
double znear = 0.1f;
GLdouble fovy = 45;

double focalDistance = 1;
double mousex, mousey;
double deltax, deltay;
dMatrix KMdMatrix(4, dVector(4));
dMatrix projM = dMatrix(4, dVector(4));
dMatrix extM = dMatrix(4, dVector(4));
dMatrix intrM = dMatrix(4, dVector(4));

GLfloat  K[9], projMatrix[16], extrinsic[16], intrisic[16];

dMatrix Ext = dMatrix(4, dVector(4));

//double sx = 22.3, sy = 14.9;

//double params_WEBCAM[] = { g_Width*focalDistance / sx,   // fx
//							g_Height*focalDistance / sy,  // fy
//							g_Width / 2,      // cx
//							g_Height / 2 };    // cy

//PnPProblem pnp_detection(params_WEBCAM);

cv::SiftFeatureDetector detector;
std::vector<cv::KeyPoint> keypoints;
Mat kp, kp2;


void FimDoPrograma()
{
	exit(1);
}

cv::VideoCapture cap;
Mat frame;
cv::Mat input;
void initCV()
{
	//cap.open("Resources\\inputData\\luis.mp4");
	cap.open(0);
	if (!cap.isOpened())
	{
		cout << "error, could not open the capture" << endl;
		system("pause");
		exit(1);
	}

	namedWindow("video", WINDOW_AUTOSIZE);
}

void initialize()
{
	initCV();

	glClearColor(0.0f, 0.0f, 0.0f, 0.5f);				// Black Background
	glClearDepth(1.0f);									// Depth Buffer Setup
	glEnable(GL_DEPTH_TEST);
	glDepthFunc(GL_LEQUAL);							// Enables Depth Testing
	glShadeModel(GL_SMOOTH);							// Enable Smooth Shading

	glHint(GL_PERSPECTIVE_CORRECTION_HINT, GL_NICEST);	// Really Nice Perspective Calculations

	//inicializa a matriz extrinsica;
	Ext[0][0] = 1; Ext[1][1] = 1; Ext[2][2] = -1; Ext[3][3] = 1;
}

void cameraTranslate(double ctx, double cty, double ctz)
{
	Ext[0][3] += ctx;
	Ext[1][3] += cty;
	Ext[2][3] += ctz;
}

/*
angle em rad
*/
void cameraRotateY(double angle)
{
	dMatrix roty(4, dVector(4));
	dMatrix iNeg(4, dVector(4));

	roty[0][0] = cos(angle);
	roty[0][2] = sin(angle);
	roty[1][1] = 1;
	roty[2][0] = -sin(angle);
	roty[2][2] = cos(angle);
	roty[3][3] = 1;

	iNeg[0][0] = -1;
	iNeg[0][1] = 0;
	iNeg[0][2] = 0;
	iNeg[0][3] = 0;

	iNeg[1][0] = 0;
	iNeg[1][1] = -1;
	iNeg[1][2] = 0;
	iNeg[1][3] = 0;
	iNeg[2][0] = 0;

	iNeg[2][0] = 0;
	iNeg[2][1] = 0;
	iNeg[2][2] = -1;
	iNeg[2][3] = 0;

	iNeg[3][0] = 0;
	iNeg[3][1] = 0;
	iNeg[3][2] = 0;
	iNeg[3][3] = -1;

	dVector T(4);
	T[0] = Ext[0][3];
	T[1] = Ext[1][3];
	T[2] = Ext[2][3];
	T[3] = 1;

	dMatrix R = getRotationNN(Ext);
	dMatrix Rt = transpose(R);
	dVector C = multiplicacaoN1(Rt, T);
	C = multiplicacaoN1(iNeg, C);

	R = multiplicacaoNN(R, roty);
	T = multiplicacaoN1(R, C);
	T = multiplicacaoN1(iNeg, T);
	Ext = R;
	Ext[0][3] = T[0];
	Ext[1][3] = T[1];
	Ext[2][3] = T[2];
}



void normalizeCamera()
{
	double tx = Ext[0][3], ty = Ext[1][3], tz = Ext[2][3];

	Ext[0][3] = 0; Ext[1][3] = 0; Ext[2][3] = 0;

	for (int i = 0; i < 4; i++)
	{
		Ext[i] = normalize(Ext[i]);
	}

	Ext[0][3] = tx; Ext[1][3] = ty; Ext[2][3] = tz;
}

void myreshape(GLsizei w, GLsizei h)
{
	g_Width = w;
	g_Height = h;

	glViewport(0, 0, g_Width, g_Height);

	frustrumTop = tan(fovy * 3.14159 / 360) * 0.1;
	frustrumBottom = -frustrumTop;
	frustrumLeft = g_Width / g_Width * frustrumBottom;
	frustrumRight = g_Width / g_Height * frustrumTop;

	znear = 0.1f;
}

void drawGrid()
{
	glPushMatrix();

	glTranslatef(-(gridL / 2), 0, -(gridL / 2));

	glColor3f(.3, .3, .3);

	glBegin(GL_LINES);

	for (int i = 0; i <= gridL; i++)
	{
		glVertex3f(i, 0, 0);
		glVertex3f(i, 0, gridL);
		glVertex3f(0, 0, i);
		glVertex3f(gridL, 0, i);
	};

	glEnd();

	glPopMatrix();

	glPushMatrix();

	glTranslatef(-(gridL / 2), 0, -(gridL / 2));

	glColor3f(1, 0, 0);

	glBegin(GL_LINES);

	for (int i = 0; i <= gridL; i++)
	{
		glVertex3f(i, 0, 0);
		glVertex3f(i, gridL, 0);
		glVertex3f(0, i, 0);
		glVertex3f(gridL, i, 0);
	};

	glEnd();

	glPopMatrix();

	glPushMatrix();

	glTranslatef(-(gridL / 2), 0, -(gridL / 2));

	glColor3f(.3, .3, .3);

	glBegin(GL_LINES);

	for (int i = 0; i <= gridL; i++)
	{
		glVertex3f(0, i, 0);
		glVertex3f(0, i, gridL);
		glVertex3f(0, 0, i);
		glVertex3f(0, gridL, i);
	};

	glEnd();

	glPopMatrix();

	glPushMatrix();

	glTranslatef((gridL / 2), 0, -(gridL / 2));

	glColor3f(.3, .3, .3);

	glBegin(GL_LINES);

	for (int i = 0; i <= gridL; i++)
	{
		glVertex3f(0, i, 0);
		glVertex3f(0, i, gridL);
		glVertex3f(0, 0, i);
		glVertex3f(0, gridL, i);
	};

	glEnd();

	glPopMatrix();

	glPushMatrix();

	glTranslatef(-(gridL / 2), 0, (gridL / 2));

	glColor3f(.3, .3, .3);

	glBegin(GL_LINES);

	for (int i = 0; i <= gridL; i++)
	{
		glVertex3f(i, 0, 0);
		glVertex3f(i, gridL, 0);
		glVertex3f(0, i, 0);
		glVertex3f(gridL, i, 0);
	};

	glEnd();

	glPopMatrix();

	glPushMatrix();

	glTranslatef(-(gridL / 2), gridL, -(gridL / 2));

	glColor3f(.3, .3, .3);

	glBegin(GL_LINES);

	for (int i = 0; i <= gridL; i++)
	{
		glVertex3f(i, 0, 0);
		glVertex3f(i, 0, gridL);
		glVertex3f(0, 0, i);
		glVertex3f(gridL, 0, i);
	};

	glEnd();

	glPopMatrix();
}


void cameraPoseFromHomography(const Mat& H, Mat& pose)
{
	pose = Mat::eye(3, 4, CV_32FC1); //3x4 matrix
	float norm1 = (float)norm(H.col(0));
	float norm2 = (float)norm(H.col(1));
	float tnorm = (norm1 + norm2) / 2.0f;

	Mat v1 = H.col(0);
	Mat v2 = pose.col(0);

	cv::normalize(v1, v2); // Normalize the rotation

	v1 = H.col(1);
	v2 = pose.col(1);

	cv::normalize(v1, v2);

	v1 = pose.col(0);
	v2 = pose.col(1);

	Mat v3 = v1.cross(v2);  //Computes the cross-product of v1 and v2
	Mat c2 = pose.col(2);
	v3.copyTo(c2);

	pose.col(3) = H.col(2) / tnorm; //vector t [R|t]
}

void updateCV()
{
	cap >> frame;

	Mat gFrame, bFrame,homografia, pose;
	
	if (cap.get(CV_CAP_PROP_POS_FRAMES) == 500)
	{
		cap.set(CV_CAP_PROP_POS_FRAMES, 10);
	}

	cv::SiftFeatureDetector detector2;
	std::vector<cv::KeyPoint> keypoints2;

	detector2.detect(frame, keypoints2);
	detector2.compute(frame, keypoints2, kp2);

	FlannBasedMatcher matcher;
	std::vector<DMatch> matches;
	matcher.match(kp, kp2, matches);

	double max_dist = 0; double min_dist = 100;

	for (int i = 0; i < kp.rows; i++)
	{
		double dist = matches[i].distance;
		if (dist < min_dist) min_dist = dist;
		if (dist > max_dist) max_dist = dist;
	}

	//printf("-- Max dist : %f \n", max_dist);
	//printf("-- Min dist : %f \n", min_dist);

	//encontrando os good matches
	std::vector< DMatch > good_matches;

	for (int i = 0; i < kp.rows; i++)
	{
		if (matches[i].distance <= max(1.8 * min_dist, 0.002))
		{
			good_matches.push_back(matches[i]);
		}
	}

	//desenhando só os good matches
	Mat img_matches;
	
	std::vector<Point2f> obj;
	std::vector<Point2f> scene;

	for (int i = 0; i < good_matches.size(); i++)
	{
		//keypoints dos good matches
		obj.push_back(keypoints[good_matches[i].queryIdx].pt);
		scene.push_back(keypoints2[good_matches[i].trainIdx].pt);
	}
	
	if (good_matches.size() < 12){
		good_matches.clear();
	}

	drawMatches(input, keypoints, frame, keypoints2,
		good_matches, img_matches, Scalar::all(-1), Scalar::all(-1),
		vector<char>(), DrawMatchesFlags::NOT_DRAW_SINGLE_POINTS);

	if (good_matches.size()>12){

		homografia = findHomography(obj, scene, CV_RANSAC);

		cameraPoseFromHomography(homografia, pose);


		std::cout << "rows: "<< homografia.rows << std::endl;
		std::cout << "cols: "<< homografia.cols << std::endl;
		//obtendo cantos do objeto
		std::vector<Point2f> imgCorners(4);
		imgCorners[0] = cvPoint(0, 0);
		imgCorners[1] = cvPoint(input.cols, 0);
		imgCorners[2] = cvPoint(input.cols, input.rows);
		imgCorners[3] = cvPoint(0, input.rows);

		std::vector<Point2f> frameCorners(4);
		perspectiveTransform(imgCorners, frameCorners, homografia);

		line(img_matches, frameCorners[0] + Point2f(input.cols, 0), frameCorners[1] + Point2f(input.cols, 0), Scalar(0, 255, 0), 4);
		line(img_matches, frameCorners[1] + Point2f(input.cols, 0), frameCorners[2] + Point2f(input.cols, 0), Scalar(0, 255, 0), 4);
		line(img_matches, frameCorners[2] + Point2f(input.cols, 0), frameCorners[3] + Point2f(input.cols, 0), Scalar(0, 255, 0), 4);
		line(img_matches, frameCorners[3] + Point2f(input.cols, 0), frameCorners[0] + Point2f(input.cols, 0), Scalar(0, 255, 0), 4);

	}

	if (frame.data == NULL){
		initCV();
	}
	else {
		imshow("video", img_matches);
	}
	
	
}

// aqui o sistema de coordenadas da tela está variando de -1 a 1 no eixo x e y
void mydisplay()
{
	// OpenCV Processing

	updateCV();

	// End of OpenCV Processing

	glMatrixMode(GL_PROJECTION);

	glLoadIdentity();

	glViewport(0, 0, g_Width, g_Height);

	glFrustum(frustrumLeft, frustrumRight, frustrumBottom, frustrumTop, znear, zfar);

	glMatrixMode(GL_MODELVIEW);

	glClearColor(0, 0, 0, 0);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	GLfloat extrinsic[16] =
	{
		Ext[0][0], Ext[1][0], Ext[2][0], Ext[3][0],
		Ext[0][1], Ext[1][1], Ext[2][1], Ext[3][1],
		Ext[0][2], Ext[1][2], Ext[2][2], Ext[3][2],
		Ext[0][3], Ext[1][3], Ext[2][3], Ext[3][3],
	};

	glMatrixMode(GL_MODELVIEW);

	glLoadMatrixf(extrinsic);

	glPushMatrix();

	glTranslatef(0, -(gridL / 2), 9);

	drawGrid();

	glPopMatrix();

	glColor3f(0.2, 0.0, 0.2);

	glPushMatrix();

	glTranslatef(0, 0, 9);
	glutWireSphere(0.3, 100, 100);

	glPopMatrix();

	glColor3f(0.2, 0.2, 0.0);

	glPushMatrix();

	glTranslatef(0.8, 0, 9);
	glutWireTeapot(0.3);

	glPopMatrix();

	glColor3f(0.0, 0.2, 0.2);

	glPushMatrix();

	glTranslatef(-0.6, 0, 9);
	glutWireCube(0.3);

	glPopMatrix();

	glFlush();
	glutPostRedisplay();
	glutSwapBuffers();
}

void handleKeyboardPressed(unsigned char key, int x, int y){
	buffer[(int) key] = true;
}

void handleKeyboardUp(unsigned char key, int x, int y){
	buffer[(int) key] = false;
}

void idleFunction()
{
	if (buffer['w'] == true & !buffer['b'])//camera pra frente 
		cameraTranslate(0, 0, translateCameraConst);
	if (buffer['W'] == true & !buffer['b'])//camera pra frente
		cameraTranslate(0, 0, translateCameraConst);
	if (buffer['s'] == true & !buffer['b'])
		cameraTranslate(0, 0, -translateCameraConst);
	if (buffer['S'] == true & !buffer['b'])
		cameraTranslate(0, 0, -translateCameraConst);
	if (buffer['a'] == true & !buffer['b'])
		cameraTranslate(translateCameraConst, 0, 0);
	if (buffer['A'] == true & !buffer['b'])
		cameraTranslate(translateCameraConst, 0, 0);
	if (buffer['d'] == true & !buffer['b'])
		cameraTranslate(-translateCameraConst, 0, 0);
	if (buffer['D'] == true & !buffer['b'])
		cameraTranslate(-translateCameraConst, 0, 0);
	if (buffer['j'] == true & !buffer['b'])
		cameraRotateY(rotateCameraConst);
	if (buffer['l'] == true & !buffer['b'])
		cameraRotateY(-rotateCameraConst);
	if (buffer['J'] == true & !buffer['b'])
		cameraRotateY(rotateCameraConst);
	if (buffer['L'] == true & !buffer['b'])
		cameraRotateY(-rotateCameraConst);
	if (buffer[27] == true)//ESC
		FimDoPrograma();
}

int main(int argc, char **argv)
{
	input = cv::imread("Resources/inputData/cristal.jpg", 0); //Load as grayscale

	detector.detect(input, keypoints);
	detector.compute(input, keypoints,kp);
	// Add results to image and save.
	cv::Mat output;
	cv::drawKeypoints(input, keypoints, output);
	cv::imwrite("Resources/OutputData/sift_result.jpg", output);

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGBA | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(0, 0);
	glutCreateWindow("OpenGL");
	glutDisplayFunc(mydisplay);
	glutReshapeFunc(myreshape);
	glutKeyboardUpFunc(handleKeyboardUp);
	glutKeyboardFunc(handleKeyboardPressed);
	glutIdleFunc(idleFunction);
	initialize();
	glutMainLoop();
	return 0;
}




