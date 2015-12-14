/** Part of this code was taken from the SDK distribution of Sixense to control the Joystick RAZER HYDRA,
from LINUX Operating System (Ubuntu/Debian) and implement a python application using YARP communication,
what allows to configure the handle-control to use the magnetic-position function.

This project is managed by Dannier Castro L <danniercl@gmail.com>, for the ARCOSlab's investigation
projects, University of Costa Rica. Lab Coordinator: Federico Ruiz
**/

#include <yarp/os/Network.h>
#include <yarp/os/Port.h>
//#include <yarp/os/PortWriter.h>
//#include <yarp/os/BufferedPort.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/Time.h>
#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>

// Add variables that allow port communication by YARP
yarp::os::Network yarp2;
//yarp::os::Network::setLocalMode(true);
yarp::os::Port output;
//yarp::os::BufferedPort<Bottle> output;
//output.open("/razer");
yarp::os::Bottle bot;

#include <GL/freeglut.h>

#include <sixense.h>
#include <sixense_math.hpp>
//#ifdef WIN32
//#ifdef ENV64BIT
// Define the OS, WE USE LINUX
#ifdef __linux__
#include <sixense_utils/mouse_pointer.hpp>
#endif
#include <sixense_utils/derivatives.hpp>
#include <sixense_utils/button_states.hpp>
#include <sixense_utils/event_triggers.hpp>
#include <sixense_utils/controller_manager/controller_manager.hpp>

#include <deque>

static bool open_port = true;

// whether or not we are currently logging position data to a file, and the file pointer to which to log
static int is_logging = 0;
static FILE *log_file = 0;

// whether or not to write the current controller positions on the screen.
static bool display_pos_enabled = true;

// Zoom factor for the camera, press [ and ] to zoom in or out
static float camera_dist = 1.0f;

// The current mode of the real-time graph display
static int graph_mode = 0; // 0 == off, 1 == pos, 2 == vel, 3 == accel
static bool graph_paused = false;
static bool auto_graph_bounds = false;
static float graph_bounds[2] = {-750, 750};

// flags that the controller manager system can set to tell the graphics system to draw the instructions
// for the player
static bool controller_manager_screen_visible = true;
std::string controller_manager_text_string;

// these are used by the graphics to highlight one of the controller 3d objects for a number of frames
static int flash_left_controller_frames=0, flash_right_controller_frames=0;

// pressing 'm' turns on drawing of 2d mouse cursors controlled by each controller
static bool draw_mouse_pointers_enabled = false;
static float left_mouse_pos[2]={0,0}, right_mouse_pos[2]={0,0};
static float left_mouse_roll=0.0f, right_mouse_roll=0.0f;

// Log a number of samples for graphing
const int log_history_size = 1000;
std::deque<sixenseMath::Vector3> pos_hist, vel_hist, accel_hist;


// Draw a text string, at the given row, column
static void shapesPrintf (int row, int col, const char *fmt, ...)
{
	static char buf[256];
	int viewport[4];
	void *font = GLUT_BITMAP_HELVETICA_12;
	va_list args;

	va_start(args, fmt);
#if defined(WIN32) && !defined(__CYGWIN__)
	(void) _vsnprintf (buf, sizeof(buf), fmt, args);
#else
	(void) vsnprintf (buf, sizeof(buf), fmt, args);
#endif
	va_end(args);

	glGetIntegerv(GL_VIEWPORT,viewport);

	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(0,viewport[2],0,viewport[3],-1,1);

	glRasterPos2i
		(
		glutBitmapWidth(font, ' ') * col,
		- glutBitmapHeight(font) * (row+0) + viewport[3]
	);
	glutBitmapString (font, (unsigned char*)buf);

	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

// Draw a text string, centered in the middle of the screen 
static void shapesPrintfCentered (const char *fmt, ...)
{
	static char buf[256];
	int viewport[4];
	void *font = GLUT_BITMAP_HELVETICA_12;
	va_list args;

	va_start(args, fmt);
#if defined(WIN32) && !defined(__CYGWIN__)
	(void) _vsnprintf (buf, sizeof(buf), fmt, args);
#else
	(void) vsnprintf (buf, sizeof(buf), fmt, args);
#endif
	va_end(args);

	glGetIntegerv(GL_VIEWPORT,viewport);

	glPushMatrix();
	glLoadIdentity();

	glMatrixMode(GL_PROJECTION);
	glPushMatrix();
	glLoadIdentity();

	glOrtho(0,viewport[2],0,viewport[3],-1,1);

	glRasterPos2i
		(
		viewport[2]/2 - glutBitmapLength(font, (unsigned char*)buf)/2,
		viewport[3]/2 - glutBitmapHeight(font)/2
		);
	glutBitmapString (font, (unsigned char*)buf);

	glPopMatrix();
	glMatrixMode(GL_MODELVIEW);
	glPopMatrix();
}

// Write the current controller position data point to a log file
static void updateLog() {
	int base, cont;
	sixenseAllControllerData acd;
	bot.clear();

	if( true ) {

		for( base=0; base<sixenseGetMaxBases(); base++ ) {
			sixenseSetActiveBase(base);

			sixenseGetAllNewestData( &acd );

			for( cont=0; cont<sixenseGetMaxControllers(); cont++ ) {

				if( sixenseIsControllerEnabled( cont ) ) {


        				bot.addDouble(acd.controllers[cont].pos[0]);
					bot.addDouble(acd.controllers[cont].pos[1]);
					bot.addDouble(acd.controllers[cont].pos[2]);

					bot.addDouble(acd.controllers[cont].rot_mat[0][0]);
					bot.addDouble(acd.controllers[cont].rot_mat[0][1]);
					bot.addDouble(acd.controllers[cont].rot_mat[0][2]);

					bot.addDouble(acd.controllers[cont].rot_mat[1][0]);
					bot.addDouble(acd.controllers[cont].rot_mat[1][1]);
					bot.addDouble(acd.controllers[cont].rot_mat[1][2]);

					bot.addDouble(acd.controllers[cont].rot_mat[2][0]);
					bot.addDouble(acd.controllers[cont].rot_mat[2][1]);
					bot.addDouble(acd.controllers[cont].rot_mat[2][2]);


				}

			}
		}

	// send the bottle
	output.write(bot);
	// wait a while
	//yarp::os::Time::delay(0.1);
	}
}


// Draw the two 3d objects representing the controllers
static void drawObjects() {
	int base, cont, i, j;
	sixenseAllControllerData acd;
	float rot_mat[4][4];
	float colors[4][3] = {
		1.0f, 0.0f, 0.0f,
		0.8f, 0.8f, 0.0f,
		0.0f, 1.0f, 0.0f,
		0.0f, 0.0f, 1.0f };

		int left_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::ControllerManager::P1L );
		int right_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::ControllerManager::P1R );

		// Go through each of the connected systems
		for( base=0; base<sixenseGetMaxBases(); base++ ) {
			sixenseSetActiveBase(base);

			// Get the latest controller data
			sixenseGetAllNewestData( &acd );

			// For each possible controller
			for( cont=0; cont<sixenseGetMaxControllers(); cont++ ) {

				// See if it's enabled
				if( sixenseIsControllerEnabled( cont ) ) {

					// Set up the color of the object. If we're flashing this controller, set a color additive
					float flash_multiplier = 0.0f;

					if( cont == left_index ) { // if this is the left controller
						if( flash_left_controller_frames ) { // and we're supposed to flash the left controller
							flash_left_controller_frames--;

							flash_multiplier = 0.2f;
						}
					}

					if( cont == right_index ) { // if this is the left controller
						if( flash_right_controller_frames ) { // and we're supposed to flash the left controller
							flash_right_controller_frames--;

							flash_multiplier = 0.2f;
						}
					}

					// draw one hand darker than the other one
					if( cont == 0 ) {
						glColor3d(colors[base][0]+flash_multiplier, colors[base][1]+flash_multiplier, colors[base][2]+flash_multiplier );
					} else {
						glColor3d(0.6f*colors[base][0]+flash_multiplier, 0.6f*colors[base][1]+flash_multiplier, 0.6f*colors[base][2]+flash_multiplier );
					}

					glPushMatrix();
					for( i=0; i<3; i++ ) 
						for( j=0; j<3; j++ ) 
							rot_mat[i][j] = acd.controllers[cont].rot_mat[i][j];

					rot_mat[0][3] = 0.0f;
					rot_mat[1][3] = 0.0f;
					rot_mat[2][3] = 0.0f;
					rot_mat[3][0] = acd.controllers[cont].pos[0]/500.0f;
					rot_mat[3][1] = acd.controllers[cont].pos[1]/500.0f;
					rot_mat[3][2] = acd.controllers[cont].pos[2]/500.0f;
					rot_mat[3][3] = 1.0f;

					glMultMatrixf( (GLfloat*)rot_mat );
					glScaled( 0.15f, 0.15f, 0.15f );


					glutSolidSphere( 1, 5, 5 );
					glTranslated( 0, 0, -12 );
					glutSolidCylinder( 0.5, 12, 5, 5 );


					glPopMatrix();

				}
			}
		}
}


// Compute the velocity and acceleration and keep them in a list
void collectDataForGraph()
{
	if( graph_paused ) return;

	// Get the latest data for the left controller
	int left_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::ControllerManager::P1L );
	sixenseControllerData cd;
	sixenseGetNewestData( left_index, &cd );

	// Use a sixenseUtils::Derivatives object to compute velocity and acceleration from the position
	static sixenseUtils::Derivatives derivs;

	// update the derivative object
	derivs.update( &cd );

	// Push the newest derivative computations onto the history queue
	pos_hist.push_back( sixenseMath::Vector3( cd.pos ) );
	vel_hist.push_back( derivs.getVelocity() );
	accel_hist.push_back( derivs.getAcceleration() );

	// Constrain the queues to a maximum size
	if( vel_hist.size() > log_history_size ) {
		vel_hist.pop_front();
	}

	if( accel_hist.size() > log_history_size ) {
		accel_hist.pop_front();
	}

	if( pos_hist.size() > log_history_size ) {
		pos_hist.pop_front();
	}

}

// Draw a plot line of the sequence of values. Dynmically fit the y axis to keep the lines on the screen
// regardless of their range.
void drawGraph( std::deque<sixenseMath::Vector3> &hist_list ) {

	// Keep track of the y bounds of the graph. These will change with time to dynamically
	// fit the full y range on the screen
	static float graph_min_y = graph_bounds[0], graph_max_y = graph_bounds[1];

	if( auto_graph_bounds ) {

		// Go through all the elements in the list and get the max and min of the y coordinate

		float pad_scale = 0.1f;
		float new_min_y = 99999.0f, new_max_y = -99999.0f;

		for( int i=0; i<(int)hist_list.size(); i++ ) {
			for( int axis=0; axis<3; axis++ ) {
				if( hist_list[i][axis] < new_min_y ) new_min_y = hist_list[i][axis];
				if( hist_list[i][axis] > new_max_y ) new_max_y = hist_list[i][axis];
			}
		}

		// Add some padding to the range
		float range = new_max_y - new_min_y;
		new_min_y -= range * pad_scale;
		new_max_y += range * pad_scale;

		// Filter the y extents so they move smoothly
		float filter_val = 0.999f;

		graph_min_y = graph_min_y * filter_val + new_min_y * (1.0f-filter_val);
		graph_max_y = graph_max_y * filter_val + new_max_y * (1.0f-filter_val);

	} else {
		graph_min_y = graph_bounds[0];
		graph_max_y = graph_bounds[1];
	}

	const float axis_colors[3][3] = {
		{ 1, 0, 0 },
		{ 0, 1, 0 },
		{ 0, 0, 1 }
	};

	if( hist_list.size() < 2 ) return;

	glLineWidth( 2.0f );

	// Draw
	glBegin( GL_LINES );

	int axis = 2;
	for( int axis=0; axis<3; axis++ ) {

		glColor3f( axis_colors[axis][0], axis_colors[axis][1], axis_colors[axis][2] );

		for( int i=0; i<(int)hist_list.size()-1; i++ ) {

			float x_pos_0 = (float)i/(float)log_history_size;
			float x_pos_1 = (float)(i+1)/(float)log_history_size;

			float y_pos_0 = (hist_list[i][axis]-graph_min_y)/(graph_max_y-graph_min_y);
			float y_pos_1 = (hist_list[i+1][axis]-graph_min_y)/(graph_max_y-graph_min_y);

			glVertex3f( x_pos_0, y_pos_0, 0 );
			glVertex3f( x_pos_1, y_pos_1, 0 );

		}
	}

	glEnd();


}

// Draw the position, velocity or acceleration graphs, depending on the current graph_mode



// Write a bunch of instruction text, as well as the current position and rotation information
void draw_controller_info() {
	const double t = glutGet(GLUT_ELAPSED_TIME) / 1000.0;
	const double a = t*90.0;
	sixenseAllControllerData acd;
	int i, base, cont;
	int hpb_on;
	float camera_offset[3] = { 0, -1.0f, -6.0f };

	glClearColor(0.6f,0.6f,0.7f,1.0f);

	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glEnable(GL_LIGHTING);

	glColor3d(0.2,0.2,0.2);

	glPushMatrix();

	glTranslatef( camera_offset[0]*camera_dist, camera_offset[1]*camera_dist, camera_offset[2]*camera_dist );

	int next_line = 1;

	// Update the text
	sixenseSetActiveBase(0);
	collectDataForGraph();
	glColor3d(0.1,0.1,0.4);
	shapesPrintf (next_line, 3, "There is/are %d controllers", sixenseGetNumActiveControllers() );

	next_line++;

	shapesPrintf (next_line, 3, "Razer to Python" );

	next_line++;

	shapesPrintf (next_line, 3, "Run Razer.py right now, then press L in this window to send data." );

	next_line++;
	next_line++;

	if( is_logging ) {
		glColor3d(0.8,0.1,0.4);
		shapesPrintf (next_line, 3, "Press 'L' to STOP to send data by YARP port." );
	} else {
		glColor3d(0.1,0.1,0.4);
		shapesPrintf (next_line, 3, "Press 'L' to START to send data by YARP port." );
	}  


	next_line++;
	next_line++;

	shapesPrintf (next_line, 3, "Press Q to EXIT." );
	next_line++;
/*

*/

}

// This is the callback that gets registered with the sixenseUtils::controller_manager. It will get called each time the user completes
// one of the setup steps so that the game can update the instructions to the user. If the engine supports texture mapping, the 
// controller_manager can prove a pathname to a image file that contains the instructions in graphic form.
// The controller_manager serves the following functions:
//  1) Makes sure the appropriate number of controllers are connected to the system. The number of required controllers is designaged by the
//     game type (ie two player two controller game requires 4 controllers, one player one controller game requires one)
//  2) Makes the player designate which controllers are held in which hand.
//  3) Enables hemisphere tracking by calling the Sixense API call sixenseAutoEnableHemisphereTracking. After this is completed full 360 degree
//     tracking is possible.
void controller_manager_setup_callback( sixenseUtils::ControllerManager::setup_step step ) {

	if( sixenseUtils::getTheControllerManager()->isMenuVisible() ) {

		// Turn on the flag that tells the graphics system to draw the instruction screen instead of the controller information. The game
		// should be paused at this time.
		controller_manager_screen_visible = true;

		// Ask the controller manager what the next instruction string should be.
		controller_manager_text_string = sixenseUtils::getTheControllerManager()->getStepString();

		// We could also load the supplied controllermanager textures using the filename: sixenseUtils::getTheControllerManager()->getTextureFileName();

	} else {

		// We're done with the setup, so hide the instruction screen.
		controller_manager_screen_visible = false;

	}

}

// Draw the grey screen with a single yellow line of text to prompt the user through the setup steps.
void draw_controller_manager_screen() {

	glClearColor( 0.2f, 0.2f, 0.2f, 1.0f );
	glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);

	glColor3d(0.8,0.8,0.0);
	shapesPrintfCentered( controller_manager_text_string.c_str() );

}

// This function causes the 3D objects to flash when the buttons are pressed. It does so using two different techniques
// available using sixenseUtils
void check_for_button_presses( sixenseAllControllerData *acd ) {

	// Ask the controller manager which controller is in the left hand and which is in the right
	int left_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::ControllerManager::P1L );
	int right_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::ControllerManager::P1R );



	// First use the 'ButtonStates' class to flash the object when the 1 button is pressed, or the trigger is pulled.
	// ButtonStates is a simple class that reports when a button's state just transitioned from released to pressed
	// or vice versa. It also detects when the trigger crosses a programmable threshold.
	static sixenseUtils::ButtonStates left_states, right_states;

	left_states.update( &acd->controllers[left_index] );
	right_states.update( &acd->controllers[right_index] );

	// Do something if the button was pressed
	if( left_states.buttonJustPressed( SIXENSE_BUTTON_1 ) ) {
		flash_left_controller_frames = 20;
	}

	if( right_states.buttonJustPressed( SIXENSE_BUTTON_1 ) ) {
		flash_right_controller_frames = 20;
	}

	// Or if the trigger was pulled
	if( left_states.triggerJustPressed() ) {
		flash_left_controller_frames = 20;
	}

	if( right_states.triggerJustPressed() ) {
		flash_right_controller_frames = 20;
	}


	// Now do the same thing but use event triggers to flash the object when a button is pressed, or when the 
	// controller moves to a certain height.
	// EventTriggers are very flexible objects that can be used to check for transitions of controller state including buttons being pressed, controllers moving a certain distance,
	// or exceeding a certain velocity.
	class FlashObjectTrigger : public sixenseUtils::EventTriggerBase {
		int &enable_for_frames;
	public:
		FlashObjectTrigger( int &i ) : enable_for_frames( i ) {}
		virtual void trigger() const {
			enable_for_frames = 20;
		}
	};

	// First make a couple of BinaryEventSwitch that flash the object when the test parameter changes from false to true. Use a null trigger for when it transitions
	// from true to false.
	static sixenseUtils::EventSwitchBase *left_button_switch = new sixenseUtils::BinaryEventSwitch( new FlashObjectTrigger( flash_left_controller_frames ), new sixenseUtils::NullEventTrigger );
	static sixenseUtils::EventSwitchBase *right_button_switch = new sixenseUtils::BinaryEventSwitch( new FlashObjectTrigger( flash_right_controller_frames ), new sixenseUtils::NullEventTrigger );
	left_button_switch->test( ((acd->controllers)[left_index].buttons & SIXENSE_BUTTON_4) ? 1.0f : 0.0f ); // test against the current state of the 4 button
	right_button_switch->test( ((acd->controllers)[right_index].buttons & SIXENSE_BUTTON_4) ? 1.0f : 0.0f );

	// First make a couple of BinaryEventSwitch that flash the object when the controller moves above a 200mm. Do nothing when it transitions back down.
	// ValuatorEventSwitches can be used to test against any floating point value, including position, velocity, trigger positions, joystick positions, rotation angles, etc.
	static sixenseUtils::EventSwitchBase *left_height_switch = new sixenseUtils::ValuatorEventSwitch( 200.0f, new FlashObjectTrigger( flash_left_controller_frames ), new sixenseUtils::NullEventTrigger );
	static sixenseUtils::EventSwitchBase *right_height_switch = new sixenseUtils::ValuatorEventSwitch( 200.0f, new FlashObjectTrigger( flash_right_controller_frames ), new sixenseUtils::NullEventTrigger );
	left_button_switch->test( (acd->controllers)[left_index].pos[1] ); // test the y position (height)
	right_button_switch->test( (acd->controllers)[right_index].pos[1] );

}

#ifdef WIN32
// use the sixenseUtils::MousePointer class to compute the 2d mouse position pointed to by the two controllers.
void update_mouse_pointers( sixenseAllControllerData *acd ) {

	// Ask the controller manager which controller is in the left hand and which is in the right
	int left_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::ControllerManager::P1L );
	int right_index = sixenseUtils::getTheControllerManager()->getIndex( sixenseUtils::ControllerManager::P1R );


	static sixenseUtils::MousePointer left_mouse_pointer, right_mouse_pointer;

	left_mouse_pointer.setSensitivity( 1.5f );
	right_mouse_pointer.setSensitivity( 1.5f );

	Vector2 pos = left_mouse_pointer.update( &acd->controllers[left_index] );
	pos.fill( left_mouse_pos ); // fill just copies the elements into a float array
	left_mouse_roll = left_mouse_pointer.getRollAngle(); // store off the roll of the controller as well

	pos = right_mouse_pointer.update( &acd->controllers[right_index] );
	pos.fill( right_mouse_pos ); // fill just copies the elements into a float array
	right_mouse_roll = right_mouse_pointer.getRollAngle(); // store off the roll of the controller as well

}
#endif

// glut calls this function each frame
static void display(void)
{


	// update the controller manager with the latest controller data here
	sixenseSetActiveBase(0);
	sixenseAllControllerData acd;
	sixenseGetAllNewestData( &acd );
	sixenseUtils::getTheControllerManager()->update( &acd );

	check_for_button_presses( &acd );

#ifdef WIN32
	update_mouse_pointers( &acd );
#endif

	// Either draw the controller manager instruction screen, or display the controller information
	if( controller_manager_screen_visible ) {
		draw_controller_manager_screen();
	} else {
		draw_controller_info();
	}

	glutSwapBuffers();

	if( is_logging ) {
		updateLog();
	}
}


static void 
	toggleLogging() {
		if( is_logging ) {
			is_logging = 0;

		} else {
			is_logging = 1;
		}
}

static void
	key(unsigned char key, int x, int y)
{
	switch (key)
	{
	case 27 :
	case 'Q':
	case 'q': glutLeaveMainLoop () ;      break;

	case 'L': 
	case 'l': toggleLogging();            break;

	default:
		break;
	}

	glutPostRedisplay();
}

static void
	idle(void)
{
	glutPostRedisplay();
}


int main(int argc, char *argv[])
{

	printf("I am at MAIN \n");
	output.open("/razer");

	int i;
	float hemi_vec[3] = { 0, 1, 0 };
	glutInitWindowSize(400,120);
	glutInitWindowPosition(40,40);
	glutInit(&argc, argv);

	glutCreateWindow("Razer to Python");
	glutDisplayFunc(display);
	glutKeyboardFunc(key);
	glutIdleFunc(idle);

	// Init sixense
	sixenseInit();

	// Init the controller manager. This makes sure the controllers are present, assigned to left and right hands, and that
	// the hemisphere calibration is complete.
	sixenseUtils::getTheControllerManager()->setGameType( sixenseUtils::ControllerManager::ONE_PLAYER_TWO_CONTROLLER );
	sixenseUtils::getTheControllerManager()->registerSetupCallback( controller_manager_setup_callback );
	


	glutMainLoop();


	sixenseExit();

	return EXIT_SUCCESS;
}
