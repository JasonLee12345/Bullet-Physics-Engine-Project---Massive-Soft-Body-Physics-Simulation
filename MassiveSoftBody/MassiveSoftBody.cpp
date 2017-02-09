#include "stdafx.h"
#pragma warning (disable : 4996)

#include <iostream>
#include <vector>
#include <string>
#include <fstream>
#include <sstream>
#include <algorithm>
#include <functional> 
#include <cctype>

#include <GL/freeglut.h>
#include "Vector3.h"
#include <tiffio.h>

#include <btBulletCollisionCommon.h>
#include <BulletDynamics/ConstraintSolver/btSequentialImpulseConstraintSolver.h>
#include <BulletDynamics/Dynamics/btSimpleDynamicsWorld.h>

#include <BulletSoftBody/btSoftBody.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <BulletSoftBody/btSoftBodySolvers.h>
#include <BulletSoftBody/btSoftBodyHelpers.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>


/* Bullet Physics Simulation */
btBroadphaseInterface* broadphase = nullptr;
btDefaultCollisionConfiguration* collisionConfiguration = nullptr;
btCollisionDispatcher* dispatcher = nullptr;
btSequentialImpulseConstraintSolver* solver = nullptr;
btSoftBodySolver* softBodySolver = nullptr;
btSoftRigidDynamicsWorld* dynamicsWorld = nullptr;
const btVector3 GRAVITY = btVector3(0.0f, -9.81f, 0.0f);

/* Frame Rate and Timing */
int curTimeMilli = 0;
int lastTimeMilli = 0;
int dtMilli = 0;
float curTime = 0.0f;
const unsigned int FPS = 60;
const float INV_FPS = 1.0f / static_cast<float>(FPS);
bool paused = true;

/* Bullet Physics Object: Ground */
btVector3 groundScale = btVector3(50.0f, 0.2f, 50.0f);
btRigidBody* ground = nullptr;
btCollisionShape* groundCollisionShape = nullptr;
btDefaultMotionState* groundMotionState = nullptr;
GLfloat groundAmbient[] = {1.0, 1.0, 1.0};

/* Bullet Physics Object: Cloth */
float tetMeshAmbient[] = { 0.3f, 0.3f, 0.3f, 0.0f };
float tetMeshDiffuse[] = { 0.9f, 0.5f, 0.4f, 0.0f };
float tetMeshSpecular[] = { 0.3f, 0.3f, 0.3f, 0.0f };
float tetMeshShininess = 20.0;
btSoftBody* tetMesh;

/* Bullet Physics Object: Compound_Jacks */
btVector3 c1Scale = btVector3(0.5f, 0.1f, 0.1f);
//btQuaternion jackRotation = btQuaternion(0.0f, 0.0f, 0.0f, 1.0f);
float initialPosition[] = {-5.0f, -9.5f, 4.0f};
float jackPositionArray[3];
float jackTransformation[16];
#define JACK_NUM 100
GLfloat jackAmbient[] = {0.0, 0.0, 1.0};
btRigidBody* jack[JACK_NUM];
btRigidBody* jackStack1[JACK_NUM];
#define JACKS1_NUM 80
GLfloat jackAmbient1[] = {0.0, 0.5, 0.0};
btRigidBody* jackStack2[JACKS1_NUM];
btRigidBody* jackStack3[JACKS1_NUM];
#define JACKS2_NUM 50
GLfloat jackAmbient2[] = {1.0, 0.0, 0.0};
btRigidBody* jackStack4[JACKS2_NUM];
btRigidBody* jackStack5[JACKS2_NUM];


const std::string frameName = "screenshots/frame";			// Name common to each frame: "frame0.tiff", "frame1.tiff", etc.
const unsigned int RECORDING_LENGTH = 10u;					// 4[s] long animation
const unsigned int FRAME_COUNT = FPS * RECORDING_LENGTH;	// Total number of frames for the animation
unsigned int frameCount = 0u;
unsigned int frameID = 0u;
#define WIN_WIDTH 1024
#define WIN_HEIGHT 768


bool SaveTiffImage(const std::string& filename, const std::string& description, int x, int y, int width, int height, int compression) {
	TIFF *file;
	GLubyte *image, *p;

	file = TIFFOpen(filename.c_str(), "w");
	if ( file == NULL ) {
		std::cerr << "[SaveTiffImage] Error: Cannot open file: " << filename << std::endl;
		return false;
	}

	image = new GLubyte[width * height * sizeof(GLubyte) * 3];

	glPixelStorei(GL_PACK_ALIGNMENT, 1);
	glReadPixels(x, y, width, height, GL_RGB, GL_UNSIGNED_BYTE, image);

	TIFFSetField(file, TIFFTAG_IMAGEWIDTH, (uint32) width);
	TIFFSetField(file, TIFFTAG_IMAGELENGTH, (uint32) height);
	TIFFSetField(file, TIFFTAG_BITSPERSAMPLE, 8);
	TIFFSetField(file, TIFFTAG_COMPRESSION, compression);
	TIFFSetField(file, TIFFTAG_PHOTOMETRIC, PHOTOMETRIC_RGB);
	TIFFSetField(file, TIFFTAG_SAMPLESPERPIXEL, 3);
	TIFFSetField(file, TIFFTAG_PLANARCONFIG, PLANARCONFIG_CONTIG);
	TIFFSetField(file, TIFFTAG_ROWSPERSTRIP, 1);
	TIFFSetField(file, TIFFTAG_IMAGEDESCRIPTION, description.c_str());

	p = image;
	//--------------------------------------------------------------------------
	// Write each scanline to the open tiff file.
	//--------------------------------------------------------------------------
	for ( int i = height - 1; i >= 0; i-- ) {
    	if ( TIFFWriteScanline(file, p, i, 0) < 0 ) {
			free(image);
			TIFFClose(file);
			return false;
    	}
    	p += width * sizeof(GLubyte) * 3;
	}

	TIFFClose(file);
	return true;
}


void SaveFrame() {
	if ( paused == true ) return;

	if ( frameCount < FRAME_COUNT ) {
		frameID++;
		std::stringstream stream;
		stream << frameName;
		stream << frameID;
		stream << ".tiff";
		SaveTiffImage(stream.str(), "screen-frame", 0, 0, WIN_WIDTH, WIN_HEIGHT, 1);
		frameCount++;
	}
	else {
		std::cout << "[Recording Complete] Exiting Program." << std::endl;
		std::exit(0);
	}
}


btQuaternion EulerRotation(float psi, float phi, float theta) {
	float halfTheta = theta * 0.5f;
	float halfPhi = phi * 0.5f;
	float halfPsi = psi * 0.5f;

	float cosHalfPhi = std::cos(halfPhi);
	float sinHalfPhi = std::sin(halfPhi);
	
	float cosHalfTheta = std::cos(halfTheta);
	float sinHalfTheta = std::sin(halfTheta);

	float cosHalfPsi = std::cos(halfPsi);
	float sinHalfPsi = std::sin(halfPsi);

	btQuaternion quat;
	quat.setW(cosHalfPhi * cosHalfTheta * cosHalfPsi + sinHalfPhi * sinHalfTheta * sinHalfPsi);
	quat.setY(sinHalfPhi * cosHalfTheta * cosHalfPsi - cosHalfPhi * sinHalfTheta * sinHalfPsi);
	quat.setZ(cosHalfPhi * sinHalfTheta * cosHalfPsi + sinHalfPhi * cosHalfTheta * sinHalfPsi);
	quat.setX(cosHalfPhi * cosHalfTheta * sinHalfPsi - sinHalfPhi * sinHalfTheta * cosHalfPsi);
	return quat;
}


void initializeBulletWorld() {
	broadphase = new btDbvtBroadphase();
	collisionConfiguration = new btSoftBodyRigidBodyCollisionConfiguration();
	dispatcher = new btCollisionDispatcher(collisionConfiguration);
	solver = new btSequentialImpulseConstraintSolver();
	dynamicsWorld = new btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration, softBodySolver);
	dynamicsWorld->setGravity(GRAVITY);
}


void initializeGround() {
	btQuaternion groundRotation = EulerRotation(0.0, 0.0, 0.0);
	btVector3 groundPosition = btVector3(0.0f, -10.2f, 0.0f);
	btTransform defaultTransform(groundRotation, groundPosition);

	groundCollisionShape = new btBoxShape(groundScale);
	groundMotionState = new btDefaultMotionState(defaultTransform);

	btVector3 inertia(0.0f, 0.0f, 0.0f);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(0, groundMotionState, groundCollisionShape, inertia);
	ground = new btRigidBody(rigidBodyCI);
	dynamicsWorld->addRigidBody(ground);
}


void initializeJack(btRigidBody* jack[], int i, float r1, float r2, float r3, btScalar mass, float p[3], float fric) {
	btQuaternion jackRotation = EulerRotation(r1, r2, r3);
	btVector3 jackPosition = btVector3(0.0f, 0.0f, 0.0f);
	jackPosition.setX(p[0]);
	jackPosition.setY(p[1]);
	jackPosition.setZ(p[2]);
	btTransform defaultTransform(jackRotation, jackPosition);
	///////////////////////////////////////////////////////////////////////////////
	btCollisionShape* localBoxShape = new btBoxShape(c1Scale);

	btTransform localTrans;
	localTrans.setIdentity();
	localTrans.setOrigin(btVector3(0,0,0));
	localTrans.setRotation(EulerRotation(0.f, 0.f, 1.57f));

	btCompoundShape* jackCompoundShape = new btCompoundShape;
	jackCompoundShape->addChildShape(localTrans,localBoxShape);
	///////////////////////////////////////////////////////////////////////////////
	btCollisionShape* localBoxShape1 = new btBoxShape(c1Scale);

	btTransform localTrans1;
	localTrans1.setIdentity();
	localTrans1.setOrigin(btVector3(0,0,0));
	localTrans1.setRotation(EulerRotation(0.f, 1.57f, 0.f));

	jackCompoundShape->addChildShape(localTrans1,localBoxShape1);
	///////////////////////////////////////////////////////////////////////////////
	btCollisionShape* localBoxShape2 = new btBoxShape(c1Scale);

	btTransform localTrans2;
	localTrans2.setIdentity();
	localTrans2.setOrigin(btVector3(0,0,0));
	localTrans2.setRotation(EulerRotation(0.f, 0.f, 0.0f));

	jackCompoundShape->addChildShape(localTrans2, localBoxShape2);
	///////////////////////////////////////////////////////////////////////////////
	btVector3 localInertia(0,0,0);
	jackCompoundShape->calculateLocalInertia(mass, localInertia);
	///////////////////////////////////////////////////////////////////////////////
	btDefaultMotionState* jackMotionState = new btDefaultMotionState(defaultTransform);
	btRigidBody::btRigidBodyConstructionInfo rigidBodyCI(mass, jackMotionState, jackCompoundShape, localInertia);
	rigidBodyCI.m_friction = fric;
	jack[i] = new btRigidBody(rigidBodyCI);
	dynamicsWorld->addRigidBody(jack[i]);
}


void trim(std::string& s) {
	s.erase(s.begin(), std::find_if(s.begin(), s.end(), std::not1(std::ptr_fun<int, int>(std::isspace))));
	s.erase(std::find_if(s.rbegin(), s.rend(), std::not1(std::ptr_fun<int, int>(std::isspace))).base(), s.end());
}


bool initializeTetgenMesh(const std::string& eleFilename, const std::string& faceFilename, const std::string& nodeFilename) {
	std::ifstream ele;
	std::ifstream face;
	std::ifstream node;

	ele.open(eleFilename.c_str());
	if ( ele.is_open() == false ) {
		std::cerr << "[initializeTetgenMesh] Error: Cannot open file: " << eleFilename << std::endl;
		return false;
	}

	face.open(faceFilename.c_str());
	if ( face.is_open() == false ) {
		std::cerr << "[initializeTetgenMesh] Error: Cannot open file: " << faceFilename << std::endl;
		return false;
	}

	node.open(nodeFilename.c_str());
	if ( node.is_open() == false ) {
		std::cerr << "[initializeTetgenMesh] Error: Cannot open file: " << nodeFilename << std::endl;
		return false;
	}

	std::stringstream stream;
	stream << ele.rdbuf();
	std::string eleContent = stream.str();
	stream.str("");

	stream << node.rdbuf();
	std::string nodeContent = stream.str();
	stream.str("");

	stream << face.rdbuf();
	std::string faceContent = stream.str();

	tetMesh = btSoftBodyHelpers::CreateFromTetGenData(dynamicsWorld->getWorldInfo(), eleContent.c_str(), faceContent.c_str(), nodeContent.c_str(), true, true, true);
	tetMesh->m_cfg.viterations = 60;
	tetMesh->m_cfg.piterations = 60;
	tetMesh->m_cfg.kDG = 0.008f;
	tetMesh->m_cfg.kDF = 0.1f;
	tetMesh->m_cfg.kMT = 0.1f;
	tetMesh->m_materials[0]->m_kLST = 0.04f;
	tetMesh->m_materials[0]->m_kAST = 0.04f;
	tetMesh->setTotalMass(50.0f);

	//--------------------------------------------------------------------------
	// Manually parse the faces from the file.
	//--------------------------------------------------------------------------
	unsigned int faceCount, index0, index1, index2, throwAway;
	std::sscanf(faceContent.c_str(), "%d %d", &faceCount, &throwAway);
	std::string line;
	std::getline(stream, line);
	for ( unsigned int i = 0; i < faceCount; i++ ) {
		std::getline(stream, line);
		trim(line);
		std::sscanf(line.c_str(), "%d %d %d %d", &throwAway, &index0, &index1, &index2);
		tetMesh->appendFace(index0, index1, index2);
	}

	for ( unsigned int i = 0; i < tetMesh->m_nodes.size(); i++ ) 
		tetMesh->m_nodes[i].m_x.setY(tetMesh->m_nodes[i].m_x.y() + 2.0);

	dynamicsWorld->addSoftBody(tetMesh);

	ele.close();
	face.close();
	node.close();
}


void initPositionXZ() {
	jackPositionArray[0] = initialPosition[0];
	//jackPositionArray[1] = initialPosition[1];
	jackPositionArray[2] = initialPosition[2];
}


void onInit() {
	glClearColor(0.0f, 0.0f, 0.0f, 1.0f);
	glEnable (GL_DEPTH_TEST);

	initializeBulletWorld();
	initializeTetgenMesh("BendTorus.ele", "BendTorus.face", "BendTorus.node");
	//initializeTetgenMesh("cube.1.ele", "cube.1.face", "cube.1.node");
	initializeGround();

	initPositionXZ();
	jackPositionArray[1] = initialPosition[1];
	for(int i=0; i<JACK_NUM; i++) {
		//initializeJack(i, 0.6234f, 0.23423f, 0.923f, 10.0f, jackPositionArray, 5);
		if(i%10 == 0 && i != 0) {
			jackPositionArray[2] -= 1;
			jackPositionArray[0] = -5;
		}
		initializeJack(jack, i, 0.0f, 0.0f, 0.0f, 4.0f, jackPositionArray, 5);
		jackPositionArray[0] += 1;
	}

	initPositionXZ();
	jackPositionArray[1] ++;
	for(int i=0; i<JACK_NUM; i++) {
		if(i%10 == 0 && i != 0) {
			jackPositionArray[2] -= 1;
			jackPositionArray[0] = -5;
		}
		initializeJack(jackStack1, i, 0.0f, 0.0f, 0.0f, 5.0f, jackPositionArray, 5);
		jackPositionArray[0] += 1;
	}

	initPositionXZ();
	jackPositionArray[1] ++;
	for(int i=0; i<JACKS1_NUM; i++) {
		if(i%10 == 0 && i != 0) {
			jackPositionArray[2] -= 1;
			jackPositionArray[0] = -5;
		}
		initializeJack(jackStack2, i, 0.0f, 0.0f, 0.0f, 5.0f, jackPositionArray, 5);
		jackPositionArray[0] += 1;
	}

	initPositionXZ();
	jackPositionArray[1] ++;
	for(int i=0; i<JACKS1_NUM; i++) {
		if(i%10 == 0 && i != 0) {
			jackPositionArray[2] -= 1;
			jackPositionArray[0] = -5;
		}
		initializeJack(jackStack3, i, 0.0f, 0.0f, 0.0f, 5.0f, jackPositionArray, 5);
		jackPositionArray[0] += 1;
	}

	initPositionXZ();
	jackPositionArray[1] ++;
	for(int i=0; i<JACKS2_NUM; i++) {
		if(i%10 == 0 && i != 0) {
			jackPositionArray[2] -= 1;
			jackPositionArray[0] = -5;
		}
		initializeJack(jackStack4, i, 0.0f, 0.0f, 0.0f, 5.0f, jackPositionArray, 5);
		jackPositionArray[0] += 1;
	}

	initPositionXZ();
	jackPositionArray[1] ++;
	for(int i=0; i<JACKS2_NUM; i++) {
		if(i%10 == 0 && i != 0) {
			jackPositionArray[2] -= 1;
			jackPositionArray[0] = -5;
		}
		initializeJack(jackStack5, i, 0.0f, 0.0f, 0.0f, 5.0f, jackPositionArray, 5);
		jackPositionArray[0] += 1;
	}
}


void renderGround() {
	glMaterialfv(GL_FRONT, GL_AMBIENT, groundAmbient);

	glPushMatrix();
		glTranslatef(0.0, -10.2, 0.0);
		glScalef(groundScale.getX() * 2.0f, groundScale.getY() * 2.0f, groundScale.getZ() * 2.0f);
		glutSolidCube(1.0f);
	glPopMatrix();
}


void renderJack(btRigidBody* jack[], int i) {
	btTransform transform;
	jack[i]->getMotionState()->getWorldTransform(transform);
	transform.getOpenGLMatrix(jackTransformation);

	glPushMatrix();
		glMultMatrixf(jackTransformation);

		glPushMatrix();
			glScalef(c1Scale.getX() * 2.0f, c1Scale.getY() * 2.0f, c1Scale.getZ() * 2.0f);
			glutSolidCube(1.0f);
		glPopMatrix();

		glPushMatrix();
			glRotatef(90, 0, 1, 0);
			glScalef(c1Scale.getX() * 2.0f, c1Scale.getY() * 2.0f, c1Scale.getZ() * 2.0f);
			glutSolidCube(1.0f);
		glPopMatrix();

		glPushMatrix();
			glRotatef(90, 0, 0, 1);
			glScalef(c1Scale.getX() * 2.0f, c1Scale.getY() * 2.0f, c1Scale.getZ() * 2.0f);
			glutSolidCube(1.0f);
		glPopMatrix();

	glPopMatrix();
}


void renderCloth() {
	glMaterialfv(GL_FRONT, GL_AMBIENT, tetMeshAmbient);
	glMaterialfv(GL_FRONT, GL_DIFFUSE, tetMeshDiffuse);
	glMaterialfv(GL_FRONT, GL_SPECULAR, tetMeshSpecular);
	glMaterialfv(GL_FRONT, GL_SHININESS, &tetMeshShininess);

	//--------------------------------------------------------------------------
	// Calculate cloth face normals.
	//--------------------------------------------------------------------------
	float x, y, z;
	Vector3f a, b, c;
	Vector3f faceNormal;
	glBegin(GL_TRIANGLES);
	for ( int i = 0; i < tetMesh->m_faces.size(); i++ ) {
		for ( unsigned int j = 0; j < 3; j++ ) {
			x = tetMesh->m_faces[i].m_n[j]->m_x.x();
			y = tetMesh->m_faces[i].m_n[j]->m_x.y();
			z = tetMesh->m_faces[i].m_n[j]->m_x.z();

			if ( j == 0 ) a = Vector3f(x, y, z);
			if ( j == 1 ) b = Vector3f(x, y, z);
			if ( j == 2 ) c = Vector3f(x, y, z);
		}

		faceNormal = -Vector3f::Cross(b - a, c - a);
		Vector3f::Normalize(faceNormal);
		glNormal3fv(faceNormal);
		glVertex3fv(a);
		glNormal3fv(faceNormal);
		glVertex3fv(b);
		glNormal3fv(faceNormal);
		glVertex3fv(c);
	}
	glEnd();
}


void my_display(void)
{
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	/////////////////////////////////////////////////////////////////
	glLoadIdentity();
	gluLookAt(6.5 , -4.0 , -10.0 , 0.0 , -8.0 , 0.0 , 0.0 , 1.0 , 0.0);

	renderGround();

	glMaterialfv(GL_FRONT, GL_AMBIENT, jackAmbient);
	for(int i=0; i<JACK_NUM; i++)
		renderJack(jack, i);

	for(int i=0; i<JACK_NUM; i++)
		renderJack(jackStack1, i);

	glMaterialfv(GL_FRONT, GL_AMBIENT, jackAmbient1);
	for(int i=0; i<JACKS1_NUM; i++)
		renderJack(jackStack2, i);

	for(int i=0; i<JACKS1_NUM; i++)
		renderJack(jackStack3, i);

	glMaterialfv(GL_FRONT, GL_AMBIENT, jackAmbient2);
	for(int i=0; i<JACKS2_NUM; i++)
		renderJack(jackStack4, i);

	for(int i=0; i<JACKS2_NUM; i++)
		renderJack(jackStack5, i);

	renderCloth();

   glutSwapBuffers();

   SaveFrame();
}


void onIdle() {
	curTimeMilli = glutGet(GLUT_ELAPSED_TIME);
	dtMilli = curTimeMilli - lastTimeMilli;
	float elapsedTime = dtMilli / static_cast<float>(1000.0f);
	lastTimeMilli = curTimeMilli;
	curTime += elapsedTime;

	if ( curTime > INV_FPS ) {
		if ( !paused ) {
			dynamicsWorld->stepSimulation(INV_FPS, 10);
			curTime = 0.0f;
			glutPostRedisplay();
		}
	}
}
// ////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////
// ////////////////////////////////////////////////////////////////////////////////////////////////////
void my_reshape(int w, int h)
{
   glViewport (0, 0, (GLsizei) w, (GLsizei) h); 
   
   glMatrixMode (GL_PROJECTION);
   glLoadIdentity ();
   gluPerspective(65.0, (GLfloat) w/(GLfloat) h, 1.0, 100.0);
   
   glMatrixMode(GL_MODELVIEW);
   glLoadIdentity();
}


void mymouse(int button, int state, int x, int y) 
{
	switch (button) {	   
		case GLUT_LEFT_BUTTON:
			if(state == GLUT_DOWN)
			{
				paused = !paused;
				glutPostRedisplay();
			}
			if (state == GLUT_UP)
			{

			}
			break;

		case GLUT_MIDDLE_BUTTON:

		case GLUT_RIGHT_BUTTON:
			if (state == GLUT_DOWN)
			{
			 
			}
			break;

		default:
			break;
	}
}


void window_size ( int keys, int x, int y )  
{
  switch ( keys ) {
    case GLUT_KEY_UP:     
      glutFullScreen (); 
      break;
    case GLUT_KEY_DOWN:               
      glutReshapeWindow (1000 , 600); 
      break;
    default:
      break;
  }
}


void onExit() {
	dynamicsWorld->removeRigidBody(ground);
	delete ground;
	delete groundMotionState;
	delete groundCollisionShape;

	delete dynamicsWorld;
	delete solver;
	delete dispatcher;
	delete collisionConfiguration;
	delete broadphase;
}


void glInitAll (void) 
{
	glClearColor (0.0, 0.0, 0.0, 0.5);
	glColor3f(1.0, 1.0, 1.0);
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glShadeModel (GL_FLAT);
	//////////////////////////////////////////////////////////////////////////////////
    GLfloat more_ambient[]	=	{0.25, 0.25, 0.25, 1.0};
    glMaterialfv(GL_FRONT, GL_AMBIENT, more_ambient);
	//////////////////////////////////////////////////////////////////////////////////
	GLfloat light_ambient[]		= {0.0,		0.0,	0.0,	0.5};
    GLfloat light_diffuse[]		= {1.0,		1.0,	1.0,	1.0};
    GLfloat light_specular[]	= {1.0,		1.0,	1.0,	1.0};
	/* light_position is NOT default value */
    GLfloat light_position[]	= {1.0,		1.0,	5.0,	0.0};
    GLfloat global_ambient[]	= {0.75,	0.75,	0.75,	1.0};

    glLightfv(GL_LIGHT0, GL_AMBIENT, light_ambient);
    glLightfv(GL_LIGHT0, GL_DIFFUSE, light_diffuse);
    glLightfv(GL_LIGHT0, GL_SPECULAR, light_specular);
    glLightfv(GL_LIGHT0, GL_POSITION, light_position);
	/////////////////////////////////////////////////////////////////////////////////

    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);

	glEnable(GL_LIGHTING);
    glEnable(GL_LIGHT0);

    glFrontFace(GL_CW); 
    glEnable(GL_AUTO_NORMAL);
    glEnable(GL_NORMALIZE);
    glDepthFunc(GL_LESS);
    glEnable(GL_DEPTH_TEST);
}


void glutInitAll(int argc,char **argv,int a,int b,int c,int d,char* f)
{
   glutInit(&argc, argv);
   glutInitDisplayMode (GLUT_DOUBLE | GLUT_RGB);
   glutInitWindowSize (a, b); 
   glutInitWindowPosition (c, d);
   glutCreateWindow (f);
}


int main(int argc, char** argv)
{
   glutInitAll(argc, argv, WIN_WIDTH, WIN_HEIGHT, 150, 150, "Let Bullet Fly");
   glInitAll();
   onInit();

   glutDisplayFunc(my_display);
   glutReshapeFunc(my_reshape);
   glutIdleFunc(onIdle);
   
   glutMouseFunc(mymouse);
   glutSpecialFunc(window_size);

   std::atexit(onExit);

   glutMainLoop();
   return 0;   /* ANSI C requires main to return int. */
}
