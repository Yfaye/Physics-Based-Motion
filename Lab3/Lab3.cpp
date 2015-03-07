/* CS6555 Computer Animation, Fall 2014
* Lab 3£ºPhysics-Based Motion Control System
* Edited by Fei Yan (Email:hcifaye@gwu.edu)
* Reference: Lab 0's code ; Lab 1's code ; Lab 2's code; 
*			 NeHe's OpenGL tutorial Lesson 30;
* Basic Steps:
*		Step 1: initialize balls with given position and volecity
*		Step 2: detect each ball for collision with floor
*				detect each ball for collision with all the other balls
*		Step 3: if collision happens, use volecity after collision replacing current volecity
*		Step 4: renew all the current position and volecity by time increasement
*		Step 5: repeate step 2 to 4
*/

// window
#include "stdafx.h"

// standard
#include <assert.h>
#include <math.h>

// glut
#include <GL/glut.h>

//
#define pi = 3.14159265358979

//================================
// global variables
//================================
// screen size
int g_screenWidth = 0;
int g_screenHeight = 0;

// Total number of balls
static int number = 20;

// Original Ball Position
GLfloat location[20][3] = { { -3.0f, 7.0f, 0.6f }, 
							{ 9.0f, 8.5f, 1.0f }, 
							{ 4.0f, 7.2f, 0.7f }, 
							{ -4.5f, 6.8f, 0.8f }, 
							{ 3.0f, 8.6f, 0.0f }, 
							{ 5.0f, 9.8f, 0.1f }, 
							{ -4.0f, 9.0f, 0.1f }, 
							{ 4.0f, 12.0f, 0.5f }, 
							{ 0.0f, 8.2f, 0.0f }, 
							{ 1.0f, 7.6f, 0.5f } ,
							{ -2.0f, 8.0f, 0.7f }, 
							{ 10.0f, 9.5f, 1.1f }, 
							{ 5.0f, 8.2f, 0.8f }, 
							{ -3.5f, 7.8f, 0.9f }, 
							{ 4.0f, 9.6f, 0.1f }, 
							{ 6.0f, 10.8f, 0.2f }, 
							{ -3.0f, 10.0f, 0.2f }, 
							{ 5.0f, 13.0f, 0.6f }, 
							{ 1.0f, 9.2f, 0.1f }, 
							{ 2.0f, 8.6f, 0.6f }};

// Original Ball Velocity in X, Y, Z axies
GLfloat velocity[20][3] = { { 1.5, 0, 0 }, 
							{ -1, 0, 0 },
							{ -1, 0, 0 },
							{ 1.2, 0, 0 },
							{ -1, 0, 0 }, 
							{ 0.8, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.5, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.7, 0, 0 },
							{ 0.6, 0, 0 }, 
							{ -1, 0, 0 },
							{ -1, 0, 0 },
							{ 0.3, 0, 0 },
							{ -1, 0, 0 }, 
							{ 0.4, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.6, 0, 0 }, 
							{ -1, 0, 0 }, 
							{ 0.7, 0, 0 }};

// balls' volecity after time increased
GLfloat velocity_Next[20][3] = {0};

// balls' original position matrix
GLfloat position[20][3] = { 0 };

// balls' position after time increased by 0.03
GLfloat position_Next[20][3];

// time incrase by 0.03
GLfloat timeIncrease = 0.03f;

// gravity acceleration simulation: velocity along -y axis would in crease 2.0 units in every 0.03 time increasement
GLfloat acceleration[3] = { 0, -2.0, 0 }; 

//the matrix for 20 balls, each row is one M for one ball
GLfloat ballM[20][16]; 

//the matrix for single ball implementing glMatrixMult
static GLfloat M[16] = { 0 }; 

//coefficient of collision: velocity after collision would reduce to 0.8 of original volecity 
GLfloat e = 0.8f; 

//=====================================================================
// Vector Normalization : normalize the vector 
//=====================================================================
void Normalization(GLfloat N_tempV[3])
{
	GLfloat squa_quaterion = N_tempV[0] * N_tempV[0] + N_tempV[1] * N_tempV[1] + N_tempV[2] * N_tempV[2];
	if (squa_quaterion != 0) // avoid being divided by 0
	{
		GLfloat base_quaternion = sqrt(squa_quaterion);
		N_tempV[0] = N_tempV[0] / base_quaternion;
		N_tempV[1] = N_tempV[1] / base_quaternion;
		N_tempV[2] = N_tempV[2] / base_quaternion;
	}
}

//=====================================================================
// Vector Dot Multiply : two vectors' dot product
//=====================================================================
GLfloat VectorDotMult(GLfloat TempV1[3], GLfloat TempV2[3])
{
	GLfloat Vresult = TempV1[0] * TempV2[0] + TempV1[1] * TempV2[1] + TempV1[2] * TempV2[2];
	return Vresult;
}
	


//=====================================================================
// Distance: Calculate the distances between two balls
//=====================================================================
GLfloat Distance(GLfloat TempV1[3], GLfloat TempV2[3]){

	GLfloat Distance = sqrt((TempV1[0] - TempV2[0])*(TempV1[0] - TempV2[0]) + (TempV1[1] - TempV2[1])*(TempV1[1] - TempV2[1]) + (TempV1[2] - TempV2[2])*(TempV1[2] - TempV2[2]));
	return Distance;

}

//=================================================================================================================
// Matrix Initialization: set up ballM matrix and position matrix by read in the original position of all the balls
//=================================================================================================================
void init()
{
	for (int j = 0; j<number; j++){
		ballM[j][0] = 1.0f;
		ballM[j][5] = 1.0f;
		ballM[j][10] = 1.0f;
		for (int i = 0; i<3; i++){
			ballM[j][12 + i] = location[j][i];
			position[j][i] = ballM[j][12 + i];
		}
		ballM[j][15] = 1.0f;
	}
}

//===========================================================================================================================
//Ball Collision Detection: Detect collision between current ball and other balls, current ball is determined by given index
//Ball Collision Happen: when the distance between two balls < 1.0
//Velocity Calculation after Collision: x_axis is the line connecting two balls, u1x and u2x is the velocity along the x_axis
//                                      u1y and u2y is the velocity on the direction perpendicular with the x_axis. 
//										after collision, u1x and u2x change the direction, u1y and u2y remain the same.
//===========================================================================================================================

void BallCollision(int index) 
{
	// detect collision between current ball and all the other balls
	for (int i = index + 1; i < number; i++)
	{
		if (Distance(position[index], position[i])<1.0) // r of ball is 0.5, when distance < 2r, ball collision happens.
		{
			// calculate the x_axis for current ball
			GLfloat x_axis[3];
			for (int j = 0; j < 3; j++)
			{
				x_axis[j] = position[i][j] - position[index][j];
			}
			Normalization(x_axis);

			// calculate u1x u1y
			GLfloat u1x[3], u1y[3];
			GLfloat tempU1 = VectorDotMult(x_axis, velocity[index]);
			for (int j = 0; j < 3; j++)
			{
				u1x[j] = tempU1*x_axis[j];
				u1y[j] = velocity[index][j] - u1x[j];
			}

			// calculate the x_axis for the other ball
			for (int j = 0; j < 3; j++)
			{
				x_axis[j] = position[index][j] - position[i][j];
			}
			Normalization(x_axis);

			// calculate u2x u2y
			GLfloat u2x[3], u2y[3];
			GLfloat tempU2 = VectorDotMult(x_axis, velocity[i]);
			for (int j = 0; j < 3; j++)
			{
				u2x[j] = tempU2*x_axis[j];
				u2y[j] = velocity[i][j] - u2x[j];
			}

			// calculate the velocity
			GLfloat v1x[3], v2x[3];
			for (int j = 0; j < 3; j++)
			{
				v1x[j] = (u1x[j] + u2x[j] - (u1x[j] - u2x[j]))*0.5;
				v2x[j] = (u1x[j] + u2x[j] - (u2x[j] - u1x[j]))*0.5;
				velocity[index][j] = v1x[j] + u1y[j];
				velocity[i][j] = v2x[j] + u2y[j];
			}
			continue;
		}
	}
}

//===========================================================================================================================
//Floor Collision Detection: Detect collision between current ball and floor, current ball is determined by given index
//Floor Collision Happen: when the distance between the ball and floor < 0.5
//===========================================================================================================================
void FloorCollision(int index) 
{
if (position[index][1]<0.5) // Floor Collision Happens
	{ 
	// velocity in Y axis changes direction and decrease by e
	velocity[index][1] = -e*velocity[index][1]; 

	// velocity in X,Z axises remains the same
	velocity[index][0] = velocity[index][0];
	velocity[index][2] = velocity[index][2];
	}
}

//===========================================================================================================================
//Ball Movement: current ball movement, with floor collision detection
//===========================================================================================================================
void BallMove(int index){

	FloorCollision(index);
	BallCollision(index);
	for (int i = 0; i<3; i++){
		velocity_Next[index][i] = velocity[index][i] + acceleration[i] * timeIncrease;
		velocity[index][i] = velocity_Next[index][i];
		position_Next[index][i] = position[index][i] + velocity_Next[index][i] * timeIncrease;
		position[index][i] = position_Next[index][i];
		ballM[index][12 + i] = position_Next[index][i];
	}

}

//===========================================================================================================================
//Ball Animation: Show all the balls
//===========================================================================================================================
void BallAnimation()
{
	for (int i = 0; i < number; i++)
	{
		glPushMatrix();
		BallMove(i);
		for (int j = 0; j < 16; j++)
		{
			M[j] = ballM[i][j];
		}
		glMultMatrixf(M);
		glutSolidSphere(0.5, 20, 20);
		glPopMatrix();
	}
}

//===========================================================================================================================
//Ground Render: Show the Groud
//===========================================================================================================================

void GroundRender(){
	glBegin(GL_LINES);
	for (GLfloat x = -100; x < 100; x += 5.0f)
	{
		glVertex3f(x, 0, -100); glVertex3f(x, 0, 100);
	}
	for (GLfloat z = -150; z < 100; z += 5.0f)
	{
		glVertex3f(-150, 0, z); glVertex3f(100, 0, z);
	}
	glEnd();
}

//================================
// timer : triggered every 16ms ( about 60 frames per second )
//================================
void timer(int value) {
	// render
	glutPostRedisplay();

	// reset timer
	glutTimerFunc(16, timer, 0);
}


//================================
// render
//================================
void render(void) {
	// clear buffer
	glClearColor(0.0, 0.0, 0.0, 1.0);
	glClearDepth(1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	// render state
	glEnable(GL_DEPTH_TEST);
	glShadeModel(GL_SMOOTH);

	// enable lighting
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// light source attributes
	GLfloat LightAmbient[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightDiffuse[] = { 0.4f, 0.4f, 0.4f, 1.0f };
	GLfloat LightSpecular[] = { 0.5f, 0.5f, 0.5f, 1.0f };
	GLfloat LightPosition[] = { 5.0f, 5.0f, 5.0f, 1.0f };

	glLightfv(GL_LIGHT0, GL_AMBIENT, LightAmbient);
	glLightfv(GL_LIGHT0, GL_DIFFUSE, LightDiffuse);
	glLightfv(GL_LIGHT0, GL_SPECULAR, LightSpecular);
	glLightfv(GL_LIGHT0, GL_POSITION, LightPosition);

	// surface material attributes
	GLfloat material_Ka[] = { 1.0f, 1.0f, 0.0f, 1.0f };
	GLfloat material_Kd[] = { 0.43f, 0.47f, 0.54f, 1.0f };
	GLfloat material_Ks[] = { 0.33f, 0.33f, 0.52f, 1.0f };
	GLfloat material_Ke[] = { 0.1f, 0.0f, 0.1f, 1.0f };
	GLfloat material_Se = 10;

	glMaterialfv(GL_FRONT, GL_AMBIENT, material_Ka);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, material_Kd);
	glMaterialfv(GL_FRONT, GL_SPECULAR, material_Ks);
	glMaterialfv(GL_FRONT, GL_EMISSION, material_Ke);
	glMaterialf(GL_FRONT, GL_SHININESS, material_Se);


	// modelview matrix
	glMatrixMode(GL_MODELVIEW);
	glLoadIdentity();
	gluLookAt(0, 15.0, 15.0,   0.0, 0.0, 0.0,      0.0, 1.0, 0.0);

	// animation	
	GroundRender();
	BallAnimation();

	// disable lighting
	glDisable(GL_LIGHT0);
	glDisable(GL_LIGHTING);

	// swap back and front buffers
	glutSwapBuffers();
}

//================================
// keyboard input
//================================
void keyboard(unsigned char key, int x, int y) {}

//================================
// reshape : update viewport and projection matrix when the window is resized
//================================
void reshape(int w, int h) {
	// screen size
	g_screenWidth = w;
	g_screenHeight = h;

	// viewport
	glViewport(0, 0, (GLsizei)w, (GLsizei)h);

	// projection matrix
	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(65.0, (GLfloat)w / (GLfloat)h, 1.0, 30.0);
}

//================================
// main
//================================
int main(int argc, char** argv) {
	// create opengL window
	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_DOUBLE | GLUT_RGB | GLUT_DEPTH);
	glutInitWindowSize(800, 600);
	glutInitWindowPosition(100, 100);
	glutCreateWindow("Lab 3 - Computer Animation - Fei Yan");

	// user initialization
	init();

	// set callback functions
	glutDisplayFunc(render);
	glutReshapeFunc(reshape);
	glutKeyboardFunc(keyboard);
	glutTimerFunc(16, timer, 0);

	// main loop
	glutMainLoop();

	return 0;
}