/* 
 * Test using RVO2 and opengl to show how the agents navigate.  This is 
 * the ExampleBlocks example.  Blocks act as obstacles with no roadmap
 * provided.
 *
 * Jordan Stadler
 * Dec 2012
 *
 */

#include <GL/freeglut.h>

#include <iostream>
#include <vector>
#include <map>

#include "RVO.h"

#ifndef M_PI
static const float M_PI = 3.14159265358979323846f;
#endif

#define GL_WIN_SIZE_X 600
#define GL_WIN_SIZE_Y 600
#define BOUND 50

#define SWARM 150
#define MAX_SPEED 0.5
#define BOUND 50

#define SHOW_GOALS 1

static const double DEG2RAD = 3.14159/180;

float scaleX = 600./200.;
float scaleY = 600./200.;

int pause = 0;

struct RoadmapVertex {
  RVO::Vector2 position;
  std::vector<int> neighbors;
  std::vector<float> distToGoal;
}v;

struct color {
	float r;
	float g;
	float b;
};
std::vector<color> agent_colors;


/* Store the roadmap. */
std::vector<RoadmapVertex> roadmap;


/* Store the goals of the agents. */
std::vector<int> goals;

/* Create a new simulator instance. */
RVO::RVOSimulator* sim = new RVO::RVOSimulator();

std::vector<RVO::Vector2> ob;

// used for drawing and building intersection logic
std::vector<RVO::Vector2> intersections;

RVO::Vector2 agent_pos;
size_t agents;

using namespace std;

// struct for triple of double values
struct double3{
	double x, y, z;
};
typedef double3 vector3, position3;

// Boid class controls a single unit of a swarm/flock
class Boid
{
private:
	position3 pos;
	vector3 velocity;
public:
	Boid();
	~Boid();
	void updateVelocity( vector3 *v1, vector3 *v2, vector3 *v3 );
	void updateVelocity( vector3 *v1, vector3 *v2, vector3 *v3, vector3 *v4 );
	void updateVelocity( vector3 *v1, vector3 *v2, vector3 *v3, vector3 *v4, vector3 *v5 );
	void updateVelocity( vector3 *v1, vector3 *v2, vector3 *v3, vector3 *v4, vector3 *v5, vector3 *v6);
	void updatePosition();
	vector3 getPosition();
	vector3 getVelocity();
	vector3 stepTowardCenter( vector3 * com );
	vector3 offsetFromOtherObjects( vector<Boid> * boids, int id );
	vector3 matchNearbyVelocities( vector<Boid> * boids, int id );
	void limitSpeed();
	vector3 bind();
	vector3 towardGoal();
	vector3 thingsToAvoid(vector<position3> avoid);
};

int counter = 0;

position3 goal = {-50.0, 25.0, 50.0};

vector<position3> trees;

// addition of double3's
double3 add_double3( double3 val1, double3 val2){
	double3 temp = {val1.x+val2.x, val1.y+val2.y, val1.z+val2.z};
	return temp;
}

// distance between 2 double3's
double distance_double3( double3 pos1, double3 pos2 ){
	return sqrt( ((pos2.x - pos1.x)*(pos2.x- pos1.x)) + ((pos2.y - pos1.y)*(pos2.y- pos1.y)) + ((pos2.z - pos1.z)*(pos2.z- pos1.z)) );
}

// normalizes a given double3
void normalize_double3( double3 *val ) {
	double mag = sqrt((val->x * val->x) + (val->y * val->y) + (val->z * val->z));
	val->x = val->x/mag;
	val->y = val->y/mag;
	val->z = val->z/mag;
}

// returns a randomized double3 within the bounds
double3 random_triple(){
	double3 random;
	random.x = (((double)rand() / RAND_MAX) * BOUND*2)-BOUND;
	random.y = (((double)rand() / RAND_MAX) * BOUND*2)-BOUND;
	random.z = (((double)rand() / RAND_MAX) * BOUND*2)-BOUND;
	return random;
}

// constructor
Boid::Boid(){
	//pos = random_triple();
	pos.x = -25 - 50*((float)rand()/RAND_MAX);
	pos.y = 200 + 100*((float)rand()/RAND_MAX);
	//pos.y = -250;
	pos.z = 25 + 50*((float)rand()/RAND_MAX);
	velocity.x = 0.0; velocity.y = 0.0; velocity.z = 0.0;
}

// destructor
Boid::~Boid(){}

// returns the positions of the boid
vector3 Boid::getPosition() {
	return pos;
}

// returns the velocity of the boid
vector3 Boid::getVelocity() {
	return velocity;
}

vector<Boid> birds;

// updates a voids velocity given vectors rule vectors
void Boid::updateVelocity( vector3 * v1, vector3 * v2, vector3 * v3, vector3 * v4, vector3 * v5, vector3 * v6){
	velocity.x += v1->x + v2->x + v3->x + v4->x + v5->x + v6->x;
	velocity.y += v1->y + v2->y + v3->y + v4->y + v5->y + v6->y;
	velocity.z += v1->z + v2->z + v3->z + v4->z + v5->z + v6->z;
}

// updates the position of a boid using the volocity vector
void Boid::updatePosition() {
	pos.x += velocity.x;
	pos.y += velocity.y;
	pos.z += velocity.z;
}

// steer toward the center of the swarm
vector3 Boid::stepTowardCenter( vector3 * com) {
	vector3 step;
	step.x = com->x - pos.x;
	step.y = com->y - pos.y;
	step.z = com->z - pos.z;
	//normalize_double3(&step);
	step.x /= 100.; step.y /= 100.; step.z /= 100.;
	return step;
}

// avoid colliding with other boids
vector3 Boid::offsetFromOtherObjects( vector<Boid> * boids, int id ) {
	int count = 0;
	vector3 c = {0.0, 0.0, 0.0};
	for( vector<Boid>::iterator it = boids->begin(); it != boids->end(); it++){
		if( count != id ){
			vector3 temp = it->getPosition();
			if( distance_double3(temp, pos) < 1.0 ){
				c.x = c.x - (temp.x - pos.x);
				c.y = c.y - (temp.y - pos.y);
				c.z = c.z - (temp.z - pos.z);
			}
		}
		count++;
	}
	c.x /= 8;
	c.y /= 8;
	c.z /= 8;
	return c;
}

// match velocity with nearby boids
vector3 Boid::matchNearbyVelocities( vector<Boid> * boids, int id ){
	int count = 0;
	vector3 vel = { 0.0, 0.0, 0.0};
	for( vector<Boid>::iterator it = boids->begin(); it != boids->end(); it++){
		if( count != id){
			vel = add_double3(vel, it->getVelocity());
		}
		count++;
	}

	vel.x /= boids->size()-1;
	vel.y /= boids->size()-1;
	vel.z /= boids->size()-1;

	vel.x /=8;
	vel.y /=8;
	vel.z /=8;

	return vel;
}

// limit the speed of a boid so the swarm doesnt get too much speed and isnt sporatic
void Boid::limitSpeed() {
	double vlim = MAX_SPEED;
	double mag = sqrt( (velocity.x*velocity.x) + (velocity.y+velocity.y) + (velocity.z+velocity.z) );
	if (  mag > vlim ) {
		velocity.x = (velocity.x / mag) * vlim;
		velocity.y = (velocity.y / mag) * vlim;
		velocity.z = (velocity.z / mag) * vlim;
	}
}

// boids are bound within a volume, if they escape then steer them back
vector3 Boid::bind() {
	// x  -7.5 -> -92.5
	// y  0 -> 100
	// z  7.5 -> 92.5
	vector3 v = {0.0, 0.0, 0.0};
	if(pos.x < -92.5){
		v.x = MAX_SPEED*1000; 
	}else if (pos.x > -7.5){
		v.x = (-1)*MAX_SPEED*1000;
	}

	if(pos.y < 0){
		v.y = MAX_SPEED*1000; 
	}else if (pos.y > 100){
		v.y = (-1)*MAX_SPEED*1000;
	}

	if(pos.z < 7.5){
		v.z = MAX_SPEED*1000; 
	}else if (pos.z > 92.5){
		v.z = (-1)*MAX_SPEED*1000;
	}
	return v;
}

// steer toward the swarms goal
vector3 Boid::towardGoal() {
	vector3 temp;
	temp.x = (goal.x - pos.x)/100;
	temp.y = (goal.y - pos.y)/100;
	temp.z = (goal.z - pos.z)/100;
	return temp;
}

// steer away from obstacles or predators
vector3 Boid::thingsToAvoid( vector<position3> avoid ){
	vector3 away = {0.0, 0.0, 0.0};
	double x, y, z, dist;

	for( size_t i = 0; i < avoid.size(); i++) {
		x = avoid[i].x - pos.x;
		y = avoid[i].y - pos.y;
		z = avoid[i].z - pos.z;
		
		dist = sqrt( x*x + y*y + z*z);

		if(dist < 10.0){
			away.x += (avoid[i].x - pos.x) /10.;
			away.y += (avoid[i].y - pos.y) /10.;
			away.z += (avoid[i].z - pos.z) /10.;
		}
	}
	away.x *= -1; away.y *= -1; away.z *= -1;
	return away;
}

// determine the center of the swarm.  Needed to steer boids toward the center.
position3 centerOfBoids( vector<Boid> * boids ) {
	position3 com = { 0.0, 0.0, 0.0 };
	for( vector<Boid>::iterator it = boids->begin(); it != boids->end(); it++){
		com = add_double3(com, it->getPosition());
	}
	com.x = com.x/boids->size();
	com.y = com.y/boids->size();
	com.z = com.z/boids->size();
	return com;
}

void update_boids( vector<Boid> * boids) {
	int id = 0;
	// zero out each vector
	vector3 v1  = {0.0, 0.0, 0.0};
	vector3 v2  = {0.0, 0.0, 0.0};
	vector3 v3  = {0.0, 0.0, 0.0};
	vector3 v4  = {0.0, 0.0, 0.0};
	vector3 v5  = {0.0, 0.0, 0.0};
	vector3 v6 =  {0.0, 0.0, 0.0};
	vector3 com  = {0.0, 0.0, 0.0};

	com = centerOfBoids(boids);
	
	// loop through each boid, makes forces based on rules, update velocity and position
	for( vector<Boid>::iterator it = boids->begin(); it != boids->end(); it++){
		// each force influences an individual boids next step, need to determine
		// where it should go
		v1 = it->stepTowardCenter(&com);
		v2 = it->offsetFromOtherObjects( boids, id);
		v3 = it->matchNearbyVelocities( boids, id);
		v4 = it->bind();
		v5 = it->towardGoal();
		v6 = it->thingsToAvoid(trees);

		it->updateVelocity( &v1, &v2, &v3, &v4, &v5, &v6 );
		it->limitSpeed();		// ensure boids dont get too much speed
		it->updatePosition();	// update boid position
		id++;
	}
}

//camera values
double xpos = 0, ypos = 0, zpos = 0, xrot = 0, yrot = 0, angle=0.0;
int lastx, lasty;
double xrotrad, yrotrad;

// swiftless tutorial for camera - direct copy (see top comment )
void camera (void) {
    glRotatef(xrot,1.0,0.0,0.0);  //rotate our camera on teh x-axis (left and right)
    glRotatef(yrot,0.0,1.0,0.0);  //rotate our camera on the y-axis (up and down)
    glTranslated(-xpos,-ypos,-zpos); //translate the screen to the position of our camera
}

// swiftless tutorial for camera - direct copy (see top comment )
void mouseMovement(int x, int y) {
    int diffx=x-(int)lastx; //check the difference between the current x and the last x position
    int diffy=y-(int)lasty; //check the difference between the current y and the last y position
    lastx=x; //set lastx to the current x position
    lasty=y; //set lasty to the current y position
    xrot += (float) diffy; //set the xrot to xrot with the addition of the difference in the y position
    yrot += (float) diffx;    //set the xrot to yrot with the addition of the difference in the x position
}

// makes a 100x100 intersection centered at x,y
void makeIntersection( double x, double y ) {
	RVO::Vector2 center;
	center = RVO::Vector2(x,y);
	intersections.push_back(center);
	ob.clear();
	// center
	ob.push_back(RVO::Vector2(x+5.0f, y-5.0f));
	ob.push_back(RVO::Vector2(x+5.0f, y+5.0f));
	ob.push_back(RVO::Vector2(x-5.0f, y+5.0f));
	ob.push_back(RVO::Vector2(x-5.0f, y-5.0f));
	
	sim->addObstacle(ob);
	ob.clear();

	// bottom
	ob.push_back(RVO::Vector2(x-5.0f, y+50.0f));
	ob.push_back(RVO::Vector2(x-5.0f, y+10.0f));
	ob.push_back(RVO::Vector2(x+5.0f, y+10.0f));
	ob.push_back(RVO::Vector2(x+5.0f, y+50.0f));

	sim->addObstacle(ob);
	ob.clear();

	//top
	ob.push_back(RVO::Vector2(x-5.0f, y-10.0f));
	ob.push_back(RVO::Vector2(x-5.0f, y-50.0f));
	ob.push_back(RVO::Vector2(x+5.0f, y-50.0f));
	ob.push_back(RVO::Vector2(x+5.0f, y-10.0f));

	sim->addObstacle(ob);
	ob.clear();

	//left
	ob.push_back(RVO::Vector2(x-50.0f, y+5.0f));
	ob.push_back(RVO::Vector2(x-50.0f, y-5.0f));
	ob.push_back(RVO::Vector2(x-10.0f, y-5.0f));
	ob.push_back(RVO::Vector2(x-10.0f, y+5.0f));

	sim->addObstacle(ob);
	ob.clear();

	//right
	ob.push_back(RVO::Vector2(x+10.0f, y+5.0f));
	ob.push_back(RVO::Vector2(x+10.0f, y-5.0f));
	ob.push_back(RVO::Vector2(x+50.0f, y-5.0f));
	ob.push_back(RVO::Vector2(x+50.0f, y+5.0f));
	sim->addObstacle(ob);
	ob.clear();
}

// setup all my buiuldings as obstacles
void makeStructures(){
	
	// this can be a park
	/*ob.clear();
	ob.push_back(RVO::Vector2(-10.0f, 10.0f));
	ob.push_back(RVO::Vector2(-90.0f, 10.0f));
	ob.push_back(RVO::Vector2(-90.0f, 90.0f));
	ob.push_back(RVO::Vector2(-10.0f, 90.0f));
	sim->addObstacle(ob);*/

	// lots of small buildings
	ob.clear();
	ob.push_back(RVO::Vector2(10.0f, 10.0f));
	ob.push_back(RVO::Vector2(30.0f, 10.0f));
	ob.push_back(RVO::Vector2(30.0f, 30.0f));
	ob.push_back(RVO::Vector2(10.0f, 30.0f));
	sim->addObstacle(ob);
	ob.clear();
	ob.push_back(RVO::Vector2(60.0f, 10.0f));
	ob.push_back(RVO::Vector2(90.0f, 10.0f));
	ob.push_back(RVO::Vector2(90.0f, 30.0f));
	ob.push_back(RVO::Vector2(60.0f, 30.0f));
	sim->addObstacle(ob);
	ob.clear();
	ob.push_back(RVO::Vector2(80.0f, 35.0f));
	ob.push_back(RVO::Vector2(90.0f, 35.0f));
	ob.push_back(RVO::Vector2(90.0f, 90.0f));
	ob.push_back(RVO::Vector2(80.0f, 90.0f));
	sim->addObstacle(ob);
	ob.clear();
	ob.push_back(RVO::Vector2(70.0f, 35.0f));
	ob.push_back(RVO::Vector2(75.0f, 35.0f));
	ob.push_back(RVO::Vector2(75.0f, 90.0f));
	ob.push_back(RVO::Vector2(70.0f, 90.0f));
	sim->addObstacle(ob);
	ob.clear();
	ob.push_back(RVO::Vector2(30.0f, 60.0f));
	ob.push_back(RVO::Vector2(65.0f, 60.0f));
	ob.push_back(RVO::Vector2(65.0f, 90.0f));
	ob.push_back(RVO::Vector2(30.0f, 90.0f));
	sim->addObstacle(ob);
	ob.clear();
	ob.push_back(RVO::Vector2(10.0f, 60.0f));
	ob.push_back(RVO::Vector2(20.0f, 60.0f));
	ob.push_back(RVO::Vector2(20.0f, 90.0f));
	ob.push_back(RVO::Vector2(10.0f, 90.0f));
	sim->addObstacle(ob);
	ob.clear();
	ob.push_back(RVO::Vector2(10.0f, 35.0f));
	ob.push_back(RVO::Vector2(30.0f, 35.0f));
	ob.push_back(RVO::Vector2(30.0f, 55.0f));
	ob.push_back(RVO::Vector2(10.0f, 55.0f));
	sim->addObstacle(ob);


	ob.clear();
	ob.push_back(RVO::Vector2(-10.0f, -10.0f));
	ob.push_back(RVO::Vector2(-90.0f, -10.0f));
	ob.push_back(RVO::Vector2(-90.0f, -90.0f));
	ob.push_back(RVO::Vector2(-10.0f, -90.0f));
	sim->addObstacle(ob);

	ob.clear();
	ob.push_back(RVO::Vector2(10.0f, -10.0f));
	ob.push_back(RVO::Vector2(90.0f, -10.0f));
	ob.push_back(RVO::Vector2(90.0f, -90.0f));
	ob.push_back(RVO::Vector2(10.0f, -90.0f));
	sim->addObstacle(ob);
	ob.clear();
}

void generateRoadMapVertices(){
	for( int i = 0; i < intersections.size(); i++){
		v.position = RVO::Vector2(intersections[i].x()-7.5, intersections[i].y()-7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()+7.5, intersections[i].y()-7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()-7.5, intersections[i].y()+7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()+7.5, intersections[i].y()+7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()+25.0, intersections[i].y()+7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()-25.0, intersections[i].y()+7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()+7.5, intersections[i].y()+25.0);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()+7.5, intersections[i].y()-25.0);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()+25.0, intersections[i].y()-7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()-25.0, intersections[i].y()-7.5);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()-7.5, intersections[i].y()+25.0);
		roadmap.push_back(v);
		v.position = RVO::Vector2(intersections[i].x()-7.5, intersections[i].y()-25.0);
		roadmap.push_back(v);
	}

	// odd ones

	// chopped block
	v.position = RVO::Vector2(7.5, 32.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(7.5, 57.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(25.0, 57.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(32.5, 32.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(32.5, 7.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(57.5, 7.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(57.5, 32.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(77.5, 32.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(77.5, 92.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(67.5, 92.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(67.5, 57.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(92.5, 32.5);
	roadmap.push_back(v);

	// park
	v.position = RVO::Vector2(-7.5, 50.0);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-25, 25.0);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-25, 50.0);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-25, 75.0);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-50, 7.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-50, 25);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-50, 50);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-50, 75);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-50, 92.5);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-75, 25.0);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-75, 50.0);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-75, 75.0);
	roadmap.push_back(v);
	v.position = RVO::Vector2(-92.5, 50);
	roadmap.push_back(v);

}

void generateGoalsFromSpawns(){
  v.position = RVO::Vector2(125, 125);
  roadmap.push_back(v);
  v.position = RVO::Vector2(125, 50);
  roadmap.push_back(v);
    v.position = RVO::Vector2(125, -50);
  roadmap.push_back(v);
    v.position = RVO::Vector2(125, -125);
  roadmap.push_back(v);
    v.position = RVO::Vector2( 50, 125);
  roadmap.push_back(v);
    v.position = RVO::Vector2(50, -125);
  roadmap.push_back(v);
    v.position = RVO::Vector2(-50, 125);
  roadmap.push_back(v);
    v.position = RVO::Vector2(-50, -125);
  roadmap.push_back(v);
    v.position = RVO::Vector2(-125, 125);
  roadmap.push_back(v);
      v.position = RVO::Vector2(-125, 50);
  roadmap.push_back(v);
      v.position = RVO::Vector2(-125, -50);
  roadmap.push_back(v);
      v.position = RVO::Vector2(-125, -125);
  roadmap.push_back(v);
}

RVO::Vector2 getRandSpawn() {
	int temp, temp2, x ,y;
	temp = rand() % 4;
	if(temp == 0){ 
			x = 125; 
			temp2 = rand()%4;
			if(temp2 == 0){
				y = 125;
			}else if(temp2 == 1){
				y = 50;
			}else if(temp2 == 2){
				y = -50;
			}else if(temp2 == 3){
				y = -125;
			}
	} else if( temp == 1 ){ x = -125;
			temp2 = rand()%4;
			if(temp2 == 0){
				y = 125;
			}else if(temp2 == 1){
				y = 50;
			}else if(temp2 == 2){
				y = -50;
			}else if(temp2 == 3){
				y = -125;
			}
	} else if( temp == 2 ){ x = 50;
		temp2 = rand()%2;
		y = (temp2) ? 125 : -125;
	} else if( temp == 3 ){ x = -50; 
		temp2 = rand()%2;
		y = (temp2) ? 125 : -125;
	}
	return RVO::Vector2(x,y);
}

RVO::Vector2 temp_pos;
void setupScenario(RVO::RVOSimulator* sim)
{
  // Specify the global time step of the simulation.
  sim->setTimeStep(0.25f);

  // Add (polygonal) obstacles, specifying their vertices in counterclockwise
  // order.
  makeIntersection(0.0, 0.0);
  makeIntersection(0.0, 100.0);
  makeIntersection(100.0, 0.0);
  makeIntersection(100.0, 100.0);
  makeIntersection(0.0, -100.0);
  makeIntersection(-100.0, 0.0);
  makeIntersection(-100.0, -100.0);
  makeIntersection(-100.0, 100.0);
  makeIntersection(100.0, -100.0);
  makeStructures();
  
  sim->processObstacles();

  // goals for the agents
  generateGoalsFromSpawns();
  /*v.position = RVO::Vector2(-BOUND*.9, -BOUND*.9);
  roadmap.push_back(v);
  v.position = RVO::Vector2(BOUND*.9, -BOUND*.9);
  roadmap.push_back(v);
  v.position = RVO::Vector2(-BOUND*.9, BOUND*.9);
  roadmap.push_back(v);
  v.position = RVO::Vector2(BOUND*.9, BOUND*.9);
  roadmap.push_back(v);*/

  generateRoadMapVertices();

  sim->setAgentDefaults(15.0f, 10, 5.0f, 5.0f, 0.5f, 2.0f); // defaults per agent

  // Add agents and their goals.  Want them to move across the intersection
  for (int i = 0; i < 10; ++i) {
    for (int j = 0; j < 5; ++j) {
		temp_pos = getRandSpawn();
		sim->addAgent(temp_pos);
		goals.push_back(rand() % 12);

		temp_pos = getRandSpawn();
		sim->addAgent(temp_pos);
		goals.push_back(rand() % 12);

		temp_pos = getRandSpawn();
		sim->addAgent(temp_pos);
		goals.push_back(rand() % 12);

		temp_pos = getRandSpawn();
		sim->addAgent(temp_pos);
		goals.push_back(rand() % 12);
    }
  }
}

// Sets up the roadmap for underlying agent logic.
// From RVO2's roadmap example.
void buildRoadmap(RVO::RVOSimulator* sim)
{
  // Connect the roadmap vertices by edges if mutually visible.
#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(roadmap.size()); ++i) {
    for (int j = 0; j < static_cast<int>(roadmap.size()); ++j) {
      if (sim->queryVisibility(roadmap[i].position, roadmap[j].position, sim->getAgentRadius(0))) {
        roadmap[i].neighbors.push_back(j);
      }
    }

     // Initialize the distance to each of the four goal vertices at infinity
     // (9e9f).
    roadmap[i].distToGoal.resize(12, 9e9f);
  }

   // Compute the distance to each of the four goals (the first four vertices)
   // for all vertices using Dijkstra's algorithm.

#pragma omp parallel for
  for (int i = 0; i < 12; ++i) {
    std::multimap<float, int> Q;
    std::vector<std::multimap<float, int>::iterator> posInQ(roadmap.size(), Q.end());

    roadmap[i].distToGoal[i] = 0.0f;
    posInQ[i] = Q.insert(std::make_pair(0.0f, i));

    while (!Q.empty()) {
      const int u = Q.begin()->second;
      Q.erase(Q.begin());
      posInQ[u] = Q.end();

      for (int j = 0; j < static_cast<int>(roadmap[u].neighbors.size()); ++j) {
        const int v = roadmap[u].neighbors[j];
        const float dist_uv = RVO::abs(roadmap[v].position - roadmap[u].position);

        if (roadmap[v].distToGoal[i] > roadmap[u].distToGoal[i] + dist_uv) {
          roadmap[v].distToGoal[i] = roadmap[u].distToGoal[i] + dist_uv;

          if (posInQ[v] == Q.end()) {
            posInQ[v] = Q.insert(std::make_pair(roadmap[v].distToGoal[i], v));
          } else {
            Q.erase(posInQ[v]);
            posInQ[v] = Q.insert(std::make_pair(roadmap[v].distToGoal[i], v));
          }
        }
      }
    }
  }

}

// Preferred velocity uses the underlying roadmap to determine the shortest path to the goal.
// From RVO2's roadmap example.
void setPreferredVelocities(RVO::RVOSimulator* sim)
{
#pragma omp parallel for
  for (int i = 0; i < static_cast<int>(sim->getNumAgents()); ++i) {
    float minDist = 9e9f;
    int minVertex = -1;

    for (int j = 0; j < static_cast<int>(roadmap.size()); ++j) {
      if (RVO::abs(roadmap[j].position - sim->getAgentPosition(i)) + roadmap[j].distToGoal[goals[i]] < minDist &&
          sim->queryVisibility(sim->getAgentPosition(i), roadmap[j].position, sim->getAgentRadius(i))) {
          
        minDist = RVO::abs(roadmap[j].position - sim->getAgentPosition(i)) + roadmap[j].distToGoal[goals[i]];
        minVertex = j;
      }
    }

    if (minVertex == -1) {
      // No roadmap vertex is visible; should not happen.
      sim->setAgentPrefVelocity(i, RVO::Vector2(0, 0));
    } else {
      if (RVO::absSq(roadmap[minVertex].position -
        sim->getAgentPosition(i)) == 0.0f) {
          if (minVertex == goals[i]) {
            sim->setAgentPrefVelocity(i, RVO::Vector2());
          } else {
            sim->setAgentPrefVelocity(i, RVO::normalize(roadmap[goals[i]].position - sim->getAgentPosition(i)));
          }
      } else {
        sim->setAgentPrefVelocity(i, RVO::normalize(roadmap[minVertex].position - sim->getAgentPosition(i)));
      }
    }

    // Perturb a little to avoid deadlocks due to perfect symmetry.
    float angle = std::rand() * 2.0f * M_PI / RAND_MAX;
    float dist = std::rand() * 0.0001f / RAND_MAX;

    sim->setAgentPrefVelocity(i, sim->getAgentPrefVelocity(i) + 
        dist * RVO::Vector2(std::cos(angle), std::sin(angle)));
  }
}

// draw the obstacles (roads)
void draw_obstacles(){
	glColor3f( 0.15f, 0.15f, 0.15f);
	size_t obstacles = sim->getNumObstacleVertices();

	glBegin(GL_QUADS);
	for( size_t i =0; i < obstacles-4; i++){
		RVO::Vector2 vec = sim->getObstacleVertex(i);
		glVertex3f(vec.x(), 0.0, vec.y());
	}
	glEnd();
}

// draw the areas where agents can walk as crosswalk
void draw_walkways(){

	for(size_t i = 0; i < intersections.size(); i++){
	glColor3f(0.15f, 0.15f, 0.15f);
	glBegin(GL_QUADS);
	  // top
	  glVertex3f(intersections[i].x()-5.0f, 0.0f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()-5.0f, 0.0f, intersections[i].y()-10.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.0f, intersections[i].y()-10.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.0f, intersections[i].y()-5.0f);

	  // bottom
	  glVertex3f(intersections[i].x()-5.0f, 0.0f, intersections[i].y()+5.0f);
	  glVertex3f(intersections[i].x()-5.0f, 0.0f, intersections[i].y()+10.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.0f, intersections[i].y()+10.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.0f, intersections[i].y()+5.0f);

	  // left
	  glVertex3f(intersections[i].x()-5.0f, 0.0f, intersections[i].y()+5.0f);
	  glVertex3f(intersections[i].x()-5.0f, 0.0f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()-10.0f, 0.0f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()-10.0f, 0.0f, intersections[i].y()+5.0f);

	  // right
	   glVertex3f(intersections[i].x()+5.0f, 0.0f, intersections[i].y()+5.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.0f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()+10.0f, 0.0f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()+10.0f, 0.0f, intersections[i].y()+5.0f);

	glEnd();
	
	glLineWidth(3.0f);
	glColor3f(1.0f, 1.0f, 1.0f);
	glBegin(GL_LINES);
	  glVertex3f(intersections[i].x()-5.0f, 0.05f, intersections[i].y()-10.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.05f, intersections[i].y()-10.0f);
	  glVertex3f(intersections[i].x()-5.0f, 0.05f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.05f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()-5.0f, 0.05f, intersections[i].y()+10.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.05f, intersections[i].y()+10.0f);
	  glVertex3f(intersections[i].x()-5.0f, 0.05f, intersections[i].y()+5.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.05f, intersections[i].y()+5.0f);

	  glVertex3f(intersections[i].x()-5.0f, 0.05f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()-5.0f, 0.05f, intersections[i].y()+5.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.05f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()+5.0f, 0.05f, intersections[i].y()+5.0f);
	  glVertex3f(intersections[i].x()-10.0f, 0.5f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()-10.0f, 0.05f, intersections[i].y()+5.0f);
	  glVertex3f(intersections[i].x()+10.0f, 0.05f, intersections[i].y()-5.0f);
	  glVertex3f(intersections[i].x()+10.0f, 0.05f, intersections[i].y()+5.0f);
	glEnd();
	}
}

void draw_buildings(){
	//glEnable(GL_LIGHTING);
	glDisable(GL_LIGHTING);
	glColor3f( 0.6f, 0.4f, 0.4f);
	glBegin(GL_QUADS);
	
	// big building 1 - tall
	glVertex3f(10, 0, -10);
	glVertex3f(90, 0, -10);
	glVertex3f(90, 100, -10);
	glVertex3f(10, 100, -10);
	glVertex3f(10, 0, -90);
	glVertex3f(90, 0, -90);
	glVertex3f(90, 100, -90);
	glVertex3f(10, 100, -90);
	glVertex3f(10, 0, -10);
	glVertex3f(10, 0, -90);
	glVertex3f(10, 100, -90);
	glVertex3f(10, 100, -10);
	glVertex3f(90, 0, -10);
	glVertex3f(90, 0, -90);
	glVertex3f(90, 100, -90);
	glVertex3f(90, 100, -10);

	//chopped block
	glColor3f(0.8f, 0.0f, 0.0f); // deep red
	glVertex3f(10, 0, 10);
	glVertex3f(30, 0, 10);
	glVertex3f(30, 10, 10);
	glVertex3f(10, 10, 10);
	glVertex3f(10, 0, 30);
	glVertex3f(30, 0, 30);
	glVertex3f(30, 10, 30);
	glVertex3f(10, 10, 30);
	glVertex3f(10, 0, 10);
	glVertex3f(10, 0, 30);
	glVertex3f(10, 10, 30);
	glVertex3f(10, 10, 10);
	glVertex3f(30, 0, 10);
	glVertex3f(30, 0, 30);
	glVertex3f(30, 10, 30);
	glVertex3f(30, 10, 10);
	glVertex3f(10, 10, 10);
	glVertex3f(30, 10, 10);
	glVertex3f(30, 10, 30);
	glVertex3f(10, 10, 30);

	glColor3f(0.0f, 0.6f, 0.6f); // teal
	glVertex3f(60, 0, 10);
	glVertex3f(90, 0, 10);
	glVertex3f(90, 30, 10);
	glVertex3f(60, 30, 10);
	glVertex3f(60, 0, 30);
	glVertex3f(90, 0, 30);
	glVertex3f(90, 30, 30);
	glVertex3f(60, 30, 30);
	glVertex3f(60, 0, 10);
	glVertex3f(60, 0, 30);
	glVertex3f(60, 30, 30);
	glVertex3f(60, 30, 10);
	glVertex3f(90, 0, 10);
	glVertex3f(90, 0, 30);
	glVertex3f(90, 30, 30);
	glVertex3f(90, 30, 10);
	glVertex3f(60, 30, 10);
	glVertex3f(90, 30, 10);
	glVertex3f(90, 30, 30);
	glVertex3f(60, 30, 30);

	glColor3f(0.8f, 0.75f, 0.0f); 
	glVertex3f(80, 0, 35);
	glVertex3f(90, 0, 35);
	glVertex3f(90, 5, 35);
	glVertex3f(80, 5, 35);
	glVertex3f(80, 0, 90);
	glVertex3f(90, 0, 90);
	glVertex3f(90, 5, 90);
	glVertex3f(80, 5, 90);
	glVertex3f(80, 0, 35);
	glVertex3f(80, 0, 90);
	glVertex3f(80, 5, 90);
	glVertex3f(80, 5, 35);
	glVertex3f(90, 0, 35);
	glVertex3f(90, 0, 90);
	glVertex3f(90, 5, 90);
	glVertex3f(90, 5, 35);
	glVertex3f(80, 5, 35);
	glVertex3f(90, 5, 35);
	glVertex3f(90, 5, 90);
	glVertex3f(80, 5, 90);

	glColor3f(0.0f, 0.6f, 0.0f);
	glVertex3f(70, 0, 35);
	glVertex3f(75, 0, 35);
	glVertex3f(75, 2, 35);
	glVertex3f(70, 2, 35);
	glVertex3f(70, 0, 90);
	glVertex3f(75, 0, 90);
	glVertex3f(75, 2, 90);
	glVertex3f(70, 2, 90);
	glVertex3f(70, 0, 90);
	glVertex3f(70, 0, 35);
	glVertex3f(70, 2, 35);
	glVertex3f(70, 2, 90);
	glVertex3f(75, 0, 90);
	glVertex3f(75, 0, 35);
	glVertex3f(75, 2, 35);
	glVertex3f(75, 2, 90);
	glVertex3f(70, 2, 35);
	glVertex3f(75, 2, 35);
	glVertex3f(75, 2, 90);
	glVertex3f(70, 2, 90);

	glColor3f(0.85f, 0.85f, 0.85f);
	glVertex3f(30, 0, 60);
	glVertex3f(65, 0, 60);
	glVertex3f(65, 15, 60);
	glVertex3f(30, 15, 60);
	glVertex3f(30, 0, 90);
	glVertex3f(65, 0, 90);
	glVertex3f(65, 15, 90);
	glVertex3f(30, 15, 90);
	glVertex3f(30, 0, 60);
	glVertex3f(30, 0, 90);
	glVertex3f(30, 15, 90);
	glVertex3f(30, 15, 60);
	glVertex3f(65, 0, 60);
	glVertex3f(65, 0, 90);
	glVertex3f(65, 15, 90);
	glVertex3f(65, 15, 60);
	glVertex3f(30, 15, 60);
	glVertex3f(65, 15, 60);
	glVertex3f(65, 15, 90);
	glVertex3f(30, 15, 90);

	glColor3f(0.0f, 0.0f, 0.85f);
	glVertex3f(10, 0, 60);
	glVertex3f(20, 0, 60);
	glVertex3f(20, 40, 60);
	glVertex3f(10, 40, 60);
	glVertex3f(10, 0, 90);
	glVertex3f(20, 0, 90);
	glVertex3f(20, 40, 90);
	glVertex3f(10, 40, 90);
	glVertex3f(10, 0, 90);
	glVertex3f(10, 0, 60);
	glVertex3f(10, 40, 60);
	glVertex3f(10, 40, 90);
	glVertex3f(20, 0, 90);
	glVertex3f(20, 0, 60);
	glVertex3f(20, 40, 60);
	glVertex3f(20, 40, 90);
	glVertex3f(10, 40, 90);
	glVertex3f(10, 40, 60);
	glVertex3f(20, 40, 60);
	glVertex3f(20, 40, 90);

	glColor3f(0.65f, 0.0f, 0.65f);
	glVertex3f(10, 0, 35);
	glVertex3f(30, 0, 35);
	glVertex3f(30, 4, 35);
	glVertex3f(10, 4, 35);
	glVertex3f(10, 0, 55);
	glVertex3f(30, 0, 55);
	glVertex3f(30, 4, 55);
	glVertex3f(10, 4, 55);
	glVertex3f(10, 0, 55);
	glVertex3f(10, 0, 35);
	glVertex3f(10, 4, 35);
	glVertex3f(10, 4, 55);
	glVertex3f(30, 0, 55);
	glVertex3f(30, 0, 35);
	glVertex3f(30, 4, 35);
	glVertex3f(30, 4, 55);
	glVertex3f(10, 4, 35);
	glVertex3f(30, 4, 35);
	glVertex3f(30, 4, 55);
	glVertex3f(10, 4, 55);

	//ob.push_back(RVO::Vector2(10.0f, 35.0f));
	//ob.push_back(RVO::Vector2(30.0f, 35.0f));
	//ob.push_back(RVO::Vector2(30.0f, 55.0f));
	//ob.push_back(RVO::Vector2(10.0f, 55.0f));
	
	glEnd();

	glColor3f(0.75, 0.75f, 1.0f);
	glPushMatrix();
		glTranslatef(-50, 20, -50);
		glutSolidSphere(40, 50, 50);
	glPopMatrix();
	glEnable(GL_LIGHTING);
}

// draws the entire intersection
void draw_intersection(){
	glDisable(GL_LIGHTING);
	  draw_obstacles();
	  draw_walkways();
	glEnable(GL_LIGHTING);
}

void draw_tree(double x, double y){
	//tree trunks
	glColor3f(0.5f, 0.3f, 0.3f);
	glBegin(GL_QUADS);
	  glVertex3f(x-0.5f, 0.1f, y-0.5f);
	  glVertex3f(x-0.5f, 0.1f, y+0.5f);
	  glVertex3f(x-0.5f, 10.0f, y+0.5f);
	  glVertex3f(x-0.5f, 10.0f, y-0.5f);

	  glVertex3f(x+0.5f, 0.1f, y+0.5f);
	  glVertex3f(x+0.5f, 0.1f, y-0.5f);
	  glVertex3f(x+0.5f, 10.0f, y-0.5f);
	  glVertex3f(x+0.5f, 10.0f, y+0.5f);

	  glVertex3f(x-0.5f, 0.1f, y+0.5f);
	  glVertex3f(x+0.5f, 0.1f, y+0.5f);
	  glVertex3f(x+0.5f, 10.0f, y+0.5f);
	  glVertex3f(x-0.5f, 10.0f, y+0.5f);

	  glVertex3f(x-0.5f, 0.1f, y-0.5f);
	  glVertex3f(x+0.5f, 0.1f, y-0.5f);
	  glVertex3f(x+0.5f, 10.0f, y-0.5f);
	  glVertex3f(x-0.5f, 10.0f, y-0.5f);
	glEnd();

	//tree top
	glColor3f(0.0f, 0.3f, 0.0f);
	glPushMatrix();
		glTranslatef(x, 10.0, y);
		glutSolidSphere(5, 5, 5);
	glPopMatrix();
}

void draw_park(){
	glDisable(GL_LIGHTING);

	// grass
	glColor3f(0.0f, 0.6f, 0.0f);
	glBegin(GL_QUADS);
	  glVertex3f(-10.0f, 0.0f, 10.0f);
	  glVertex3f(-10.0f, 0.0f, 90.0f);
	  glVertex3f(-90.0f, 0.0f, 90.0f);
	  glVertex3f(-90.0f, 0.0f, 10.0f);
	glEnd();

	draw_tree(-35, 35);
	draw_tree(-80, 15);
	draw_tree(-72, 26);
	draw_tree(-11, 77);
	draw_tree(-50, 64);

	glEnable(GL_LIGHTING);
}

// draws a agent at specified location
void draw_agent(float x, float y){
	glPushMatrix();
			glTranslatef(x, 0.0f, y);
			glutSolidSphere(0.5, 5, 5);
			glColor4f(1, .5, .5, 1); // pink head
			glTranslatef(0.0f, 0.5f, 0.0f);
			glutSolidSphere(.25, 5, 5);
	glPopMatrix();
}

// shows roadmap points and optionally goals
void drawRoadMap ( int goals ){
	size_t i = ( goals == 1 ) ? 0 : 4;
	glColor4f(0.0f, 0.0f, 0.0f, 1);
	for( ; i < roadmap.size(); i++ ){
		glPushMatrix();
			glTranslatef(roadmap[i].position.x(), 0.5f, roadmap[i].position.y());
			glutSolidSphere(0.5, 5, 5);
		glPopMatrix();
	}
}

// draw all members of a swarm.  Each boid is dsiplayed as a white sphere
void draw_boids( vector<Boid> * boids ) {
	glColor3f(1.0f, 1.0f, 1.0f);
	for( vector<Boid>::iterator it = boids->begin(); it != boids->end(); it++){
		glPushMatrix();
			vector3 temp = it->getPosition();
			glTranslated(temp.x, temp.y, temp.z);
			glutSolidSphere(.5, 3, 3);
		glPopMatrix();
	}
}

void draw_floor(){
	glDisable(GL_LIGHTING);
	glColor3f(0.6, 0.6f, 0.6f);
	glBegin(GL_QUADS);
	  glVertex3f(150.0f, -0.01f, 150.0f);
	  glVertex3f(150.0f, -0.01f, -150.0f);
	  glVertex3f(-150.0f, -0.01f, -150.0f);
	  glVertex3f(-150.0f, -0.01f, 150.0f);
	glEnd();
	glDisable(GL_LIGHTING);
}

void glutDisplay (void)
{
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT | GL_STENCIL_BUFFER_BIT);
    glLoadIdentity();
	glClearColor(0.5f, 0.5f, 0.5f, 1.0f);
    gluLookAt(10.0, 50.0, 80.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0);

	camera(); // update camera position
	
	// draw each agent
	for(size_t i = 0; i < agents; i+=4){
		agent_pos = sim->getAgentPosition(i);
		glColor4f(1, 0, 0, 1);
		draw_agent(agent_pos.x(), agent_pos.y());

		agent_pos = sim->getAgentPosition(i+1);
		glColor4f(0, 1, 0, 1);
		draw_agent(agent_pos.x(), agent_pos.y());

		agent_pos = sim->getAgentPosition(i+2);
		glColor4f(0, 0, 1, 1);
		draw_agent(agent_pos.x(), agent_pos.y());

		agent_pos = sim->getAgentPosition(i+3);
		glColor4f(1, 1, 0, 1);
		draw_agent(agent_pos.x(), agent_pos.y());
	}

	//draw_obstacles();
	draw_floor();
	draw_buildings();
	draw_intersection();

	draw_park();

	drawRoadMap(SHOW_GOALS);

	draw_boids( &birds ); // draw swarm (white spheres)

    glutSwapBuffers();

}

// check if a given agent has reached its goal. If so warp it.
void checkGoals(){
	RVO::Vector2 tempAgent, tempGoal;
	float dist;
	
	for( size_t i = 0; i < sim->getNumAgents(); i++) {
		tempAgent = sim->getAgentPosition(i);
		tempGoal = roadmap[goals[i]].position;

		dist = sqrt( (tempGoal.x()-tempAgent.x())*(tempGoal.x()-tempAgent.x()) + (tempGoal.y()-tempAgent.y())*(tempGoal.y()-tempAgent.y()) );

		// if goal is reached, warp agent to oposite side (maybe consider randomizing this
		if(dist < 1.5f) {
			tempAgent = RVO::Vector2(tempAgent.x()*-1, tempAgent.y()*-1);
			sim->setAgentPosition(i, tempAgent);
		}
	}
}

// adjucts agents velocities and moves them. Checks if they reached their goals.
void glutIdle (void)
{
	setPreferredVelocities(sim);
	if(!pause) sim->doStep();
	checkGoals();

	update_boids(&birds);

    glutPostRedisplay();
}

void glutKeyboard (unsigned char key, int /*x*/, int /*y*/)
{
	switch (key) {
		case 27:
			exit (1);
			break;
		case 'q':
			exit (1);
			break;
		case 'p':
			pause = (pause) ? 0 : 1;
			break;
		case 'w':
			yrotrad = (yrot / 180 * 3.141592654f);
			xrotrad = (xrot / 180 * 3.141592654f); 
			xpos += float(sin(yrotrad)) ;
			zpos -= float(cos(yrotrad)) ;
			ypos -= float(sin(xrotrad)) ;
			break;
		case 'a':
			yrotrad = (yrot / 180 * 3.141592654f);
			xpos -= float(cos(yrotrad)) * 0.2f;
			zpos -= float(sin(yrotrad)) * 0.2f;
			break;
		case 's':
			yrotrad = (yrot / 180 * 3.141592654f);
			xrotrad = (xrot / 180 * 3.141592654f); 
			xpos -= float(sin(yrotrad));
			zpos += float(cos(yrotrad));
			ypos += float(sin(xrotrad));
			break;
		case 'd':
			yrotrad = (yrot / 180 * 3.141592654f);
			xpos += float(cos(yrotrad)) * 0.2f;
			zpos += float(sin(yrotrad)) * 0.2f;
			break;
		default:
			break;
	}
}

// create all boids
void generate_boids( int swarm_size) {
	// create the swarm
	for(int i = 0 ; i < swarm_size; i++){
		Boid newBoid;
		birds.push_back(newBoid);

	}
}

void glInit() {

	// LIGHTING 	
    GLfloat lightpos[4] = { 1.0, 0.0, 1.0, 1.0 };     // light position
    GLfloat lightamb[4] = { 0.0, 0.0, 0.0, 1.0 };     // ambient colour
    GLfloat lightdif[4] = { 1.0, 1.0, 1.0, 1.0 };     // diffuse colour
    GLfloat global_ambient[4] = {0.2f, 0.2f, 0.2f, 1.0f};

    glLightfv(GL_LIGHT0, GL_POSITION, lightpos);
    // set the ambient light colour
    glLightfv(GL_LIGHT0, GL_AMBIENT, lightamb);
    // set the diffuse light colour
    glLightfv(GL_LIGHT0, GL_DIFFUSE, lightdif);
	// global ambient
    glLightModelfv(GL_LIGHT_MODEL_AMBIENT, global_ambient);
	// turn on lighting
    glEnable(GL_LIGHTING);
    // enable light 0, all the other lights are off
    glEnable(GL_LIGHT0);
	glEnable(GL_DEPTH_TEST);
   
    glMatrixMode(GL_PROJECTION);
    gluPerspective(60.0, 800./600., 1.0, 2500.0);
	glMatrixMode(GL_MODELVIEW);

	glEnable ( GL_COLOR_MATERIAL );
	glColorMaterial( GL_FRONT_AND_BACK,GL_AMBIENT_AND_DIFFUSE );
	glShadeModel(GL_SMOOTH);
}

int main(int argc, char* argv[])
{
	color col = {1.0f, 0.0f, 0.0f}; agent_colors.push_back(col);	// 0 - red
	col.r=0.0f;col.g= 1.0f;col.b=0.0f;agent_colors.push_back(col);	// 1 - green
	col.r=0.0f;col.g= 0.0f;col.b=1.0f;agent_colors.push_back(col);	// 2 - blue
	col.r=1.0f;col.g= 1.0f;col.b=0.0f;agent_colors.push_back(col);	// 3 - yellow
	col.r=1.0f;col.g= 0.0f;col.b=1.0f;agent_colors.push_back(col);	// 4 - magenta
	col.r=0.0f;col.g= 1.0f;col.b=1.0f;agent_colors.push_back(col);	// 5 - teal

	/*draw_tree(-35, 35);
	draw_tree(-80, 15);
	draw_tree(-72, 26);
	draw_tree(-11, 77);
	draw_tree(-50, 64);*/
	position3 tree = {-35, 10, 35}; trees.push_back(tree);
	tree.x = -80; tree.y = 10; tree.z = 15; trees.push_back(tree);
	tree.x = -72; tree.y = 10; tree.z = 26; trees.push_back(tree);
	tree.x = -11; tree.y = 10; tree.z = 77; trees.push_back(tree);
	tree.x = -50; tree.y = 10; tree.z = 64; trees.push_back(tree);
  
  /* Set up the scenario. */
  setupScenario(sim);

  agents = sim->getNumAgents();

  /* Build the roadmap. */
  buildRoadmap(sim);


  generate_boids(SWARM);

  // OpenGL init
	glutInit(&argc, argv);
    glutInitDisplayMode(GLUT_RGB | GLUT_DOUBLE | GLUT_DEPTH);
    glutInitWindowSize(800, 600);
    glutCreateWindow ("RVO2 - Intersection!");
	glInit();
    //glutFullScreen();
    glutSetCursor(GLUT_CURSOR_NONE);
	
	glutPassiveMotionFunc(mouseMovement);
	glutKeyboardFunc(glutKeyboard);
    glutDisplayFunc(glutDisplay);
    glutIdleFunc(glutIdle);

	// Per frame code is in glutDisplay
	glutMainLoop();

  delete sim;

  return 0;
}