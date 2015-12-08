#include <iostream>
#include <string>
#include <ctime>
using namespace std;

#include <gl/glut.h>
#include <opencv2/opencv.hpp>

#include "Denoising.h"

Mesh mesh;

double rotX = 0.0;
double rotY = 0.0;
bool isPress = false;
int  prevX, prevY;

const int    WIN_WIDTH  = 700;
const int    WIN_HEIGHT = 700;

void description() {
	cout << "*** Mesh Denoising via L0 Minimization ***" << endl;
	cout << "[Keyboard]" << endl;
	cout << "  d: denoising via L0 minimization" << endl;
	cout << "  n: add noise to the mesh" << endl;
	cout << "  s: save current image" << endl;
	cout << endl;
}

void loadMesh(const std::string& filename) {
	if(!OpenMesh::IO::read_mesh(mesh, filename)) {
		cerr << "Failed to load file \"" << filename << "\"" << endl;
	}
}

void renderMesh() {
	Mesh::FaceIter f_it;
	Mesh::FaceVertexIter fv_it;

	glBegin(GL_TRIANGLES);
	for(f_it = mesh.faces_begin(); f_it != mesh.faces_end(); ++f_it) {
		fv_it = mesh.fv_iter(f_it.handle());

		Mesh::Point normal = mesh.calc_face_normal(f_it.handle());
		glNormal3d(normal[0], normal[1], normal[2]);
		for(; fv_it; ++fv_it) {
			Mesh::Point pt = mesh.point(fv_it.handle());
			glVertex3d(pt[0], pt[1], pt[2]);
		}
	}
	glEnd();
}

void glSetMaterial() {
	static float ambient[]  = { 0.24725, 0.1995, 0.0745, 1.0 };
	static float diffuse[]  = { 0.75164, 0.60648 , 0.22648, 1.0 };
	static float specular[] = { 0.628281, 0.555802,0.366065,1.0 };
	static float shineness  =  51.2;

	glMaterialfv(GL_FRONT_AND_BACK, GL_AMBIENT, ambient);
	glMaterialfv(GL_FRONT_AND_BACK, GL_DIFFUSE, diffuse);
	glMaterialfv(GL_FRONT_AND_BACK, GL_SPECULAR, specular);
	glMaterialf(GL_FRONT_AND_BACK, GL_SHININESS, shineness);
}

void saveImage() {
	unsigned char* pixel_data = new unsigned char[WIN_WIDTH * WIN_HEIGHT * 3];
	glReadBuffer(GL_FRONT);
	glReadPixels(0, 0, WIN_WIDTH, WIN_HEIGHT, GL_RGB, GL_UNSIGNED_BYTE, (void*)pixel_data);

	cv::Mat img(WIN_HEIGHT, WIN_WIDTH, CV_8UC3);
	for(int y=0; y<WIN_HEIGHT; y++) {
		for(int x=0; x<WIN_WIDTH; x++) {
			for(int d=0; d<3; d++) {
				img.data[(y*WIN_WIDTH+x)*3+d] = pixel_data[((WIN_HEIGHT-y-1)*WIN_WIDTH+x)*3+(2-d)];
			}
		}
	}

	string outfile;
	cout << "save as: ";
	cin >> outfile;

	cv::imwrite(outfile, img);
	cout << "Successfully saved to \"" << outfile << "\"" << endl;
	delete [] pixel_data;
}

void keyboard(unsigned char key, int x, int y) {
	switch(key) {
	case 'd':
		denoise(mesh);
		break;

	case 'n':
		addNoise(mesh);
		break;

	case 's':
		saveImage();
		break;

	case 0x1b:
		exit(0);
		break;
	}
	glutPostRedisplay();
}

void mouse(int button, int state, int x, int y) {
	if(button == GLUT_LEFT_BUTTON) {
		if(state == GLUT_DOWN) {
			isPress = true;
			prevX = x;
			prevY = y;
		}

		if(state == GLUT_UP) {
			isPress = false;
		}
	}
}

void motion(int x, int y) {
	if(isPress) {
		int dx = x - prevX;
		int dy = y - prevY;

		rotX += dy / 2.0;
		rotY += dx / 2.0;

		prevX = x;
		prevY = y;

		glutPostRedisplay();
	}
}

void display() {
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
	glLoadIdentity();
	gluLookAt(0.7, 0.7, 0.7, 0, 0.1, 0, 0, 1, 0);

	static float light_pos[] = { 1, 1, 1, 0 };
	glLightfv(GL_LIGHT0, GL_POSITION, light_pos);
	
	glSetMaterial();
	
	glPushMatrix();
	glRotated(rotY, 0, 1, 0);
	glRotated(rotX, 1, 0, 0);
	renderMesh();
	glPopMatrix();

	glutSwapBuffers();
}

void reshape(int width, int height) {
	glViewport(0, 0, width, height);

	glMatrixMode(GL_PROJECTION);
	glLoadIdentity();
	gluPerspective(20.0, (double)width / height, 1, 100);

	glMatrixMode(GL_MODELVIEW);
}

void glInitFunc() {
	glClearColor(0, 0, 0, 1);

	// ŒõŒ¹‚Ì‰Šú‰»
	glEnable(GL_LIGHTING);
	glEnable(GL_LIGHT0);

	// ‚»‚Ì‚Ù‚©
	glEnable(GL_NORMALIZE);
	glEnable(GL_DEPTH_TEST);
}

void glSetCallbacks() {
	glutDisplayFunc(display);
	glutReshapeFunc(reshape);
	glutMouseFunc(mouse);
	glutMotionFunc(motion);
	glutKeyboardFunc(keyboard);
}

int main(int argc, char** argv) {
    if (argc <= 1) {
        std::cerr << "usage: denoise [input OBJ file]" << std::endl;
        exit(1);
    }
    const std::string filename = argv[1];

	glutInit(&argc, argv);
	glutInitDisplayMode(GLUT_RGBA | GLUT_DOUBLE | GLUT_DEPTH);
	glutInitWindowPosition(100, 100);
	glutInitWindowSize(WIN_WIDTH, WIN_HEIGHT);
	glutCreateWindow("L0 Mesh Denoising");

	loadMesh(filename);
	description();

	glSetCallbacks();
	glInitFunc();

	glutMainLoop();
}
