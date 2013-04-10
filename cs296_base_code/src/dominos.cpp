/*
* Copyright (c) 2006-2009 Erin Catto http://www.box2d.org
*
* This software is provided 'as-is', without any express or implied
* warranty.  In no event will the authors be held liable for any damages
* arising from the use of this software.
* Permission is granted to anyone to use this software for any purpose,
* including commercial applications, and to alter it and redistribute it
* freely, subject to the following restrictions:
* 1. The origin of this software must not be misrepresented; you must not
* claim that you wrote the original software. If you use this software
* in a product, an acknowledgment in the product documentation would be
* appreciated but is not required.
* 2. Altered source versions must be plainly marked as such, and must not be
* misrepresented as being the original software.
* 3. This notice may not be removed or altered from any source distribution.
*/

/* 
 * Base code for CS 296 Software Systems Lab 
 * Department of Computer Science and Engineering, IIT Bombay
 * Instructor: Parag Chaudhuri
 */


#include "cs296_base.hpp"
#include "render.hpp"

#ifdef __APPLE__
  #include <GLUT/glut.h>
#else
  #include "GL/freeglut.h"
#endif

#include <cstring>
using namespace std;

#include "dominos.hpp"

namespace cs296
{
  dominos_t::dominos_t()
  {
    //Globals
    b2Body* f0;
		float ground_y= -17.0f;
    float bottom_x = 30.0f;

    /*! \b Floor-0
    * - \c b2Body* floor
    *     - \c floor: its a rigid body pointer which will represent the top floor.
    * - \c b2PolygonShape shape
    *     - \c shape : to set shape to the ground. Its an edge starting at (-50,30) to (26,30).  
    * - \c b2BodyDef bd
    *     - \c bd: defines a new body property set with default variables
    * - Setting density to 0 and not making it a dynamic body makes it immovable.
    */
    //Floor-0
    {
      b2Body* floor;
      b2EdgeShape shape;
      shape.Set(b2Vec2(-50.0f, 30.0f), b2Vec2(26.0f, 30.0f));

      b2BodyDef bd;
      floor = m_world->CreateBody(&bd);
      floor->CreateFixture(&shape, 0.0f);
      f0 = floor;
    }
    
    /*! \b Floor-1
    * - \c b2Body* floor
    *     - \c floor: its a rigid body pointer which will represent the second floor.
    * - \c b2PolygonShape shape
    *     - \c shape : to set shape to the ground. Its an edge starting at (0,18.5) to (50,18.5).  
    * - \c b2BodyDef bd
    *     - \c bd: defines a new body property set with default variables
    * - Setting density to 0 and not making it a dynamic body makes it immovable. 
    */
    //Floor-1
    {
      b2Body* floor;
      b2EdgeShape shape;
      shape.Set(b2Vec2(0.0f, 18.5f), b2Vec2(50.0f, 18.5f));

      b2BodyDef bd;
      floor = m_world->CreateBody(&bd);
      floor->CreateFixture(&shape, 0.0f);
    }

    /*! \b Falling \b Box
    * - \c b2PolygonShape shape
    *   - \c shape : Its a square with dimension 2*2
    * - \c b2BodyDef bd
    *   - \c bd defines a body in the world.The world position of the body is set to (-35,40)
    *   - \c bd.type is set to \c b2_dynamicBody, so that it acts as a rigid body
    * - \c b2FixtureDef fd
    *   - \c fd.density : its desity is set to 100000 units, high enough to make the spring compress
    * - At the end, the fixture is attached to the body
    * - \c ResetMassData() for resetting the mass from defaults to 100000
    */
    //Falling Box
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.0f, 2.0f);
      
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 100000.0f;

      b2BodyDef bd;
      bd.position.Set(-35.0f, 40.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      body->CreateFixture(&fd);
      body->ResetMassData();
    }

    /*! \b Spring \b System
    * - \c b2Body* fl_rod
    *   - \c fl_rod : the body representing the flat rod on the criss-cross system
    * - \c b2Body* inc_rod[4][2]
    *   - \c inc_rod[4][2] : matrix of 8 rods forming the criss-cross system
    *   - for a given first component, the obtained two rods form a cross
    * - \c b2Body* box
    *   - \c box : the body representing the punching box attached to the criss-crosses
    * - \c b2PolygonShape shape
    *   - \c shape : Its a rectangle with dimension 0.1x8
    * - \c b2BodyDef bd
    *   - \c bd defines a body in the world.The world position of the body is set to (-35,40)
    *   - \c bd.type is set to \c b2_dynamicBody, so that it acts as a rigid body
    * - \c b2FixtureDef fd
    *   - \c fd.density : its desity is set to 20 units
    *   - \c fd.shape : set to the shape described above
    *   - \c the same fixtureDef is used for all the bodies in the system
    */
    //Spring System
    {
      b2Body* fl_rod, *inc_rod[4][2];
      b2Body* box;

      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 8.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;

    /*!   \b Flat \b Rod
    *   - \c b2BodyDef bd
    *     - \c bd defines a body in the world.The world position of the first rod is set to (-35,35)
    *     - \c bd.angle : set to pi/2 i.e parallel to ground
    *     - \c bd.type is set to \c b2_dynamicBody, so that it acts as a rigid body
    *   - \c b2Body* body
    *     - \c body : body representing the rod on the criss-cross
    *   - At the end, the fixture is attached to the body
    */
      //flat rod
      {
        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        bd.angle = b2_pi/2;
        bd.position.Set(-35.0f, 35.0f);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        fl_rod = body;
      }

      shape.SetAsBox(0.1f, 2.83f);

      fd.shape = &shape;
      fd.density = 20.0f;
      
    /*!   \b Inclined \b Rods
    *   - \c b2BodyDef bd
    *     - \c bd defines a body in the world.The world position of the first rod is set to (-38.5,32.6)
    *     - There after the x-coordinate is incremented by 3 units each time
    *     - \c bd.angle : set to pi/6 for half and -pi/6 for the other half
    *     - \c bd.type is set to \c b2_dynamicBody, so that it acts as a rigid body
    *   - \c b2Body* body
    *     - \c body : body representing the each rod forming the criss-cross
    *   - At the end, the fixture is attached to the body
    *   - Inter-joints are made using \c b2RevoluteJoint.
    *   - The criss-cross system is fixed to ground at the left-most point using \c b2DistanceJoint
    */
      //inclined rods
      for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 2; ++j){
          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.angle = (2*j-1)*b2_pi/6;
          bd.position.Set(-38.5f + 3*i, 32.6f);
          b2Body* body = m_world->CreateBody(&bd);
          body->CreateFixture(&fd);
          inc_rod[i][j] = body;
        }
      }
      
    /*!   \b Punching \b Box
    *   - \c b2PolygonShape shape
    *     - \c shape : Its a rectangle with dimension 2*2.5
    *   - \c b2BodyDef bd
    *     - \c bd defines a body in the world.The world position of the body is set to (-22.7,32.6)
    *     - \c bd.type is set to \c b2_dynamicBody, so that it acts as a rigid body
    *   - \c b2FixtureDef fd
    *     - \c fd.density : its desity is set to 20 units
    *     - \c fd.shape : set to the shape described above
    *   - \c b2Body* body
    *     - \c body : body representing the puncher
    *   - The box is attached to the criss-cross system using \c b2DistanceJoint
    */
      //Punching Box
      {
        b2PolygonShape shape;
        shape.SetAsBox(2, 2.5f);
        
        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;

        b2BodyDef bd;
        bd.position.Set(-22.7f, 32.6f);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        box = body;
      }
      
      b2Vec2 right_end1(0,-2.83);
      b2Vec2 right_end2(0,2.83);
      //joints
      b2RevoluteJointDef jointDef;
      b2DistanceJointDef distDef;
      jointDef.Initialize(inc_rod[1][0], fl_rod, inc_rod[1][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      distDef.Initialize(inc_rod[0][0], f0, inc_rod[0][0]->GetWorldPoint(right_end1), inc_rod[0][0]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&distDef);
      jointDef.Initialize(inc_rod[0][0], inc_rod[0][1], inc_rod[0][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[1][0], inc_rod[1][1], inc_rod[1][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[2][0], inc_rod[2][1], inc_rod[2][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[3][0], inc_rod[3][1], inc_rod[3][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[0][0], inc_rod[1][1], inc_rod[0][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[0][1], inc_rod[1][0], inc_rod[0][1]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[1][0], inc_rod[2][1], inc_rod[1][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[1][1], inc_rod[2][0], inc_rod[1][1]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[2][0], inc_rod[3][1], inc_rod[2][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[2][1], inc_rod[3][0], inc_rod[2][1]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      distDef.Initialize(inc_rod[3][1], box, inc_rod[3][1]->GetWorldCenter(), box->GetWorldCenter());
      (b2DistanceJoint*)m_world->CreateJoint(&distDef);
    }

    /*! \b Ball-1
    * - \c b2Body* sbody
    *   - \c sbody : body representing the first ball being hit by the puncher
    * - \c b2CircleShape circle
    *   - \c circle : Its a circle of radius 1
    * - \c b2BodyDef ballbd
    *   - \c ballbd defines a body in the world.The world position of the body is set to (-15,31)
    *   - \c ballbd.type is set to \c b2_dynamicBody, so that it acts as a rigid body
    * - \c b2FixtureDef ballfd
    *   - \c ballfd.density : its desity is set to 4 units
    *   - \c ballfd.restitution : set to 0, so does not bounce.
    *   - \c ballfd.friction : set to 0, so is smooth.
    * - At the end, the fixture is attached to the body
    */
    //Ball-1
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 4.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(-15.0f, 31.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }

    /*! \b Circular \b Paths
    * - \c b2EdgeShape shape
    *   - \c shape : Its an edge of varies initial and ending points
    *                so that we get a circular arc
    * - \c int i 
    *   - \c i : The circe starts from the right-most end at i=0 going anti-clockwise
    * - \c float x,y
    *   - \c (x,y) : center of the circle of which the arc is a part
    * - \c float r
    *   - \c r : radius of the arc and is equal to 5 units for all the paths
    * - \c b2BodyDef bd
    *   - \c ballbd is a bodydef with default properties here.
    * - \c b2Body b1
    *   - \c b1 is a body representing each and every edge used to form the arcs
    * - A complete circle is formed by 40 such edges
    * - circular path is formed using parametric equation of circle i.e  
    *   	\b X \b = \b x \b + \b r*cos(theta)
    * 		\b Y \b = \b y \b + \b r*sin(theta)
    */
    //Circular-Paths
	{
    {
      b2EdgeShape shape;
      b2BodyDef bd;
      b2Body* b1;

    /*!   \b Roll-1
    *   - Centered at (0,40)
    *   - Starts at the SE point and goes till the SW point    
    */
      float x = 0.0f,y=40.0f,r=5.0f;
      //Roll-1
      for (int i = -5; i < 25; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

    /*!   \b Roll-2
    *   - Centered at (15,40)
    *   - Starts at the right-most point and goes till the bottom-most point    
    */
      x = 15.0f;
      //Roll-2
      for (int i = 0; i < 30; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

    /*!   \b Ramp-1
    *   - Centered at (-4.3,35)
    *   - Starts at the bottom-most point and goes till i=36
    */
      x = -4.3f;y = 35.0f;
      //Ramp-1
      for (int i = 30; i < 36; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

    /*!   \b Ramp-2
	*   - Centered at (4.1,35)
    *   - Starts at i=24 and goes till the bottom-most point
    */
      x = 4.1f;
      //Ramp-2
      for (int i = 24; i < 30; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

    /*!   \b Ramp-3
    *   - Centered at (13.6,35)
    *   - Starts at the bottom-most point and goes till i=39, i.e. almost the right-most point
    */
      x = 13.6f;
      //Ramp-3
      for (int i = 30; i < 39; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }
    }


    //Pulley
    /*! \b The \b pulley \b system  
  	*-  \c x and \c y = Its the centre of the square box relative to which whole pulley system is set.
  	*/
  	/*! - \a The \a left \a box
  	*- \c b2BodyDef *bd
  	*     - \c bd defines pointer of body in a world.The world position of the body is set to (x,y)
  	*     - \c bd->fixedRotation is set \c TRUE so that the body can not rotate.
	*     - \c bd->type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
	*- \c b2FixtureDef *fd1 
	*     - details of the left box required to set density to set to 10units. and friction is set to .5 and coefficient of restitution is set to 0 so that it won't move coz of collisons.
	*     - At the end, fixture is attached to box1 and box1 is then attached to the main world 
	* - \c b2PolygonShape shape2
	*   - \c shape : Its a square with dimension 2*2
	*/

	/*! - \a The \a right \a horizonatal \a bar
	*- \c b2BodyDef *bd
	*     - \c bd defines pointer of body in a world.The world position of the body is set to (x+6,y-9)
	*     - \c bd->fixedRotation is set \c TRUE so that the body can not rotate.
	*     - \c bd->type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
	*- \c b2FixtureDef *fd2
	*     - details of the left box required to set density to set to 4units. and friction is set to .5 and coefficient of restitution is set to 0 so that it won't move coz of collisons.
	*     - At the end, fixture is attached to box1 and box1 is then attached to the main world 
	* - \c b2PolygonShape shape2
	*   - \c shape : Its a square with dimension 3*.2
	*/

	/*!
	* - \c b2RevoluteJointDef jointDef 
	*     - \c jointDef It is to attach both the bodies the horizontal rotating bar and the vertical hidden fixed bar to a common anchor point so that horizontal bar can rotate about this point.
	*     - \c jointDef.bodyA = box
	*     - \c jointDef.bodyB = horizonatal bar
	*     - \c jointDef.localAnchorA = The local anchor point relative to bodyA's origin. 
	*     - \c jointDef.localAnchorB = The local anchor point relative to bodyB's origin. 
	*     - \c collideConnected: Set this flag to true if the attached bodies should collide. 
	*     - Finally, it is also attached to the world.
    */
    float x=28.0f,y=37.0f;
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(x,y);
      bd->fixedRotation = true;
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 12.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,2, b2Vec2(0.f,0.f), 0);
      fd1->shape = &bs1;
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      bd->position.Set(x+6,y-9);  
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 4.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(3,0.2, b2Vec2(0.0f,0.0f), 0);
      fd2->shape = &bs2;    
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd2);
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(x, y); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(x+6, y-10); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(x, y+10); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(x+6, y+10); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }
    //sphere which act like pulley
		
	/*!
	* \b The \b sphere \b to \b act \b like \b pulley 
	*    - \c b2Body* spherebody
	*          - \c spherebody sets the shape as spherically appearing 2d circular body
	*    - \c b2CircleShape circle
	*          - \c b2CircleShape is set to a circle with radius 3.
	*    - \c b2FixtureDef ballfd;
	*          - \c ballfd : A fixture definition is used to create a fixture. This class defines an abstract fixture definition. You can reuse fixture definitions safely like friction is set to 0 ,density is set to 1 units ,setting its shape to cirlce and coefficient of restituation to 0 (Restitution is used to make objects bounce. The restitution value is usually set to be between 0 and 1). A value of zero means the ball won't bounce.
	*    - \c b2BodyDef ballbd
	*          - \c ballbd.position of the spherical ball is set to (x+3,10 + y) i.e above the horizontal rotating bar
	*          - \c ballbd->type is not set to b2_dynamicBody so that it doesnt move and respond to forces.
	*    - At the end sbody is attached to the world and body definition is attached to it
	*/
    {
      b2Body* spherebody;
      b2CircleShape circle;
      circle.m_radius = 3;
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      //ballbd.type = b2_dynamicBody;
      ballbd.position.Set(x+3,y+10);
      spherebody = m_world->CreateBody(&ballbd);
      spherebody->CreateFixture(&ballfd);
    }
    
	/*! \b The \b rotating \b vertical \b bar 
	* - \c b2Body* body      
	*     - \c body : its a rigid body pointer which will represent each of the revolving vertical platform
	* - \c b2PolygonShape shape;
	*     - \c shape : to set shape to the revolving vertical platform. Its a vertical rectangle with dimension .2*2.2   
	* - \c b2FixtureDef fd;
	*     - \c fd : A fixture definition is used to create a fixture. density is set to 1 and setting its shape
	* - \c b2BodyDef bd;
	*     - \c bd : defines a new body "bd" with default variables set .You must set the body type to b2_dynamicBody if you want the body to move in response to forces.It is placed at position at (x,y-4)
	* - At the end, body is attached to the world.
	*/

	/*!- \a Hinge \a for \a vertical \a platform \a ( \a Hidden  \a bar)
	*     - \c b2Body* body
	*           - \c body : its a rigid body pointer 
	*     - \c b2PolygonShape shape2;
	*           - \c shape2 : to set shape to the revolving horizontal platform. Its a vertical rectangle with dimension .2*2   
	*     - \c b2FixtureDef fd;
	*           - \c fd : A fixture definition is used to create a fixture. This class defines an abstract fixture definition. You can reuse fixture definitions safely like setting its shape
	*     - \c b2BodyDef bd;
	*           - \c bd : defines a new body "bd" with default variables set .Its not made dynamic so that it can act as a hinge for horizonatl rotating bar.It is placed at position at (14,16) 
	*     - At the end, body is attached to the world.
	*/

	/*!
	* - \c b2RevoluteJointDef jointDef 
	*     - \c jointDef It is to attach both the bodies the vertical rotating bar and the vertical hidden fixed bar to a common anchor point so that vertical bar can rotate about this point.

	*     - \c jointDef.bodyA = vertical bar
	*     - \c jointDef.bodyB = hidden bar;
	*     - \c jointDef.localAnchorA = The local anchor point relative to bodyA's origin. 
	*     - \c jointDef.localAnchorB = The local anchor point relative to bodyB's origin. 
	*     - \c collideConnected: Set this flag to true if the attached bodies should collide. 
	*     - Finally, it is also attached to the world.
	*/
  
  //vertical bar to support the rigth box of pulley system
    {
      b2PolygonShape shape;
      shape.SetAsBox(.2f, 2.2f);

      b2BodyDef bd;
      bd.position.Set(x, y-4.0f);
      // bd.angle = b2_pi/4;
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      // shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(x, y-6.2f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,-2.0f);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }


	/*! \b Stairs
	* - \c b2BodyDef bd
	*   - \c bd defines a body in the world.
	* - \c b2Body* f
	*   - \c f : represents the actual edge body in the world
	* - The stairs are formed by a total of 7 edge shaped bodies
	* - \c float x
	*   - \c x : the initial x-coordinate of the edges
	* - \c float y
	*   - \c y : the initial y-coordinate of the edges
	* - \c float x_offset
	*   - \c x_offset : the horizontal offset between two consecutive edges
	* - \c float y_offset
	*   - \c y_offset : the vertical offset between two consecutive edges
	* - \c float x1[7], x2[7], y1[7], y2[7] 
	*   - Each edge starts at (x1[i], y1[i]) and ends at (x2[i], y2[i])
	*/

    //Stairs
    {
      b2BodyDef bd;
      b2Body* f;
      float x = 4.0f;
      float y = 18.5f;
      float x_offset = 3.0f;
      float y_offset = 1.0f;
      float x1[7] = {x,x,x+x_offset,x+x_offset,x+2*x_offset,x+2*x_offset,x+3*x_offset};
      float y1[7] = {y,y+y_offset,y+y_offset,y+2*y_offset,y+2*y_offset,y+y_offset,y+y_offset};
      float x2[7] = {x,x+x_offset,x+x_offset,x+2*x_offset,x+2*x_offset,x+3*x_offset,x+3*x_offset};
      float y2[7] = {y+y_offset,y+y_offset,y+2*y_offset,y+2*y_offset,y+y_offset,y+y_offset,y};
      for(int i = 0; i < 7; i++){
        b2EdgeShape shape;
        shape.Set(b2Vec2(x1[i], y1[i]), b2Vec2(x2[i], y2[i]));
        f = m_world->CreateBody(&bd);
        f->CreateFixture(&shape, 0.0f);
      }
    }

    /*! \b Dominos
    * - \c b2BodyDef bd
    *   - \c bd defines a body in the world.
    *   - \c bd.type : set to b2_dynamicbody to make the domino a rigid body
    * - \c b2Body* body
    *   - \c body : represents the actual domino in the world
    * - \c float x1
    *   - \c x1 : the initial x-coordinate of the dominos
    * - \c float y1
    *   - \c y1 : the initial y-coordinate of the dominos
    * - \c float x_offset
    *   - \c x_offset : the horizontal offset between two consecutive dominos
    * - \c float y_offset
    *   - \c y_offset : the vertical offset between two consecutive dominos
    * - \c float x[5], y[5] 
    *   - Each domino' position is set to (x[i], y[i]) using \c bd.position.Set()
    * - \c b2PolygonShape shape
    *   - \c shape : It's a rectangle with dimension 0.1x2
    * - \c b2FixtureDef fd
    *   - \c fd.density : its desity is set to 35 units and friction is set to 0.1
    */
    //dominos
    {
      b2BodyDef bd;
      b2Body* body;
      float len = 2.0f;
      float x1 = 2.5f;
      float y1 = 20.5f;
      float x_offset = 3.0f;
      float y_offset = 1.0f;
      float x[5] = {x1,x1+x_offset,x1+2*x_offset,x1+3*x_offset,x1+4*x_offset};
      float y[5] = {y1,y1+y_offset,y1+2*y_offset,y1+y_offset,y1};

      b2PolygonShape shape;
      shape.SetAsBox(0.1f, len);
    
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 35.0f;
      fd.friction = 0.1f;

      bd.type = b2_dynamicBody;

      for(int i = 0; i < 5; i++){
        bd.position.Set(x[i], y[i]);
        body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
      }
    }

    //Hammer
    {
      float hammer_x = -0.1,
            hammer_y= 18.5f  ,
            hammer_length = 2.5;


      b2BodyDef bd;
      bd.position.Set(hammer_x, hammer_length +hammer_y);
      bd.type = b2_dynamicBody;
      
      b2Body* body = m_world->CreateBody(&bd);
      
      //hammer part 1
      b2PolygonShape shape;
      shape.SetAsBox(0.2f,hammer_length);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 5.0f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      //hammer part 2
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f,0.5f,b2Vec2(-0.1f,hammer_length + 0.7f),0);//TODO
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 20.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape2;
      body->CreateFixture(fd2);

      b2BodyDef ibd;
      ibd.position.Set(hammer_x, hammer_y);
      b2Body* invi_body = m_world->CreateBody(&ibd);
      
      //Rotation of the hammer
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = invi_body;
      jointDef.localAnchorA.Set(0,-3.f);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    /*! \b Inclined \b Plank
    * - \c b2Body* body
    *     - \c body: its a rigid body pointer which will represent the inclined plank.
    * - \c b2PolygonShape shape
    *     - \c shape : Its a rectangle with dimensions 0.1*8, 
    *                                     centered at (-44,7), 
    *                                     inclined at an angle of pi/3  
    * - \c b2BodyDef bd
    *     - \c bd: defines a new body property set with default variables
    * - \c b2FixtureDef fd
    *   - \c fd.shape : set to the shape described above
    */
    //Inclined plank
    {
      b2BodyDef bd;
      b2Body* body;

      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 8.0f, b2Vec2(-44,7), b2_pi/3);
    
      b2FixtureDef fd;
      fd.shape = &shape;
      body = m_world->CreateBody(&bd);
      body->CreateFixture(&fd);
    }
    
    x=-20.0f,y=10.0f;
    //Swing
    //Swing
	/*! Rotating Platfroms with Swinging plank resting on stationary platform
	* - x and y are the coordinates of mid point of stationary platform and then everything is set with respect to it
	*/
    {
      b2Body *box;
      b2BodyDef *bd1 = new b2BodyDef;
      bd1->position.Set(x,y);
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0f;
      fd1->friction = .5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs11;
      bs11.SetAsBox(10,0.2,b2Vec2(0.0f,0.0f),0);
      fd1->shape = &bs11;
      //stationary planks
		/*!Stationary Platform 
		* - \c b2FixtureDef *fd1 -- Horizontal
		* - \c b2FixtureDef *fd2 -- Left Vertical
		* - \c b2FixtureDef *fd3 -- Right Vertical
		* - About three b2FixtureDef
		*   - the vertical sides of this platform are the sides on top of which two roatating platforms are attatched which are holding two balls.
		*   - there desity is set to 10units. and friction is set to .5 and coefficient of restitution is set to 0 so that it won't move coz of collisons.
		*   - All three fixtures have same fixture properties but different lengths and positions such that they make a stand type structu    
		*     - \c bd1->type is not set to b2_dynamicBody so that it doesnt move and respond to forces.
		re.
		*   - Two rectangles are vertical with dimensions .2*4 ( left one ) and .2*2 ( the left one) while third one is horizontal with dimensions 10*.2.
		*   - At the end, all the fixtures are attached to box and box is then attached to the main world 
		* - \c b2BodyDef *bd1
		*     - \c bd1 defines pointer of body in a world.The world position of the body is set to (-20,10)
		*     - \c bd1->type is not set to b2_dynamicBody so that it doesnt move and respond to forces.
		*/
      { 
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 10.0;
        fd2->friction = 1.0;
        fd2->restitution = 0.f;
        fd2->shape = new b2PolygonShape;
        b2PolygonShape bs12;
        bs12.SetAsBox(0.2,4,b2Vec2(-10.0f,3.0f),0);
        fd2->shape = &bs12; 

        b2FixtureDef *fd3 = new b2FixtureDef;
        fd3->density = 10.0;
        fd3->friction = 0.5;
        fd3->restitution = 0.f;
        fd3->shape = new b2PolygonShape;       
        b2PolygonShape bs13;
        bs13.SetAsBox(0.2,2,b2Vec2(10.0f,1.0f),0);
        fd3->shape = &bs13;

        box = m_world->CreateBody(bd1);
        box->CreateFixture(fd1);
        box->CreateFixture(fd2);
        box->CreateFixture(fd3);
      }

      //oscillating plank
		/*Oscillating plank
		* - \c b2BodyDef *bd2
		*     - \c bd2 defines pointer of body in a world.The world position of the body is set to (x-7.5,y-8)
		*     - \c bd2->type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
		* - It has a horizonatl plank which is of 10*.2 in dimension (similar to horizonatl fixture of stationary plank)
		* - It is deflected a bit from its equilibrium position in order to give it some initial velocity when it is released.
		* - TO keep motion in sync it is supported by a thin vertical rod below it in order to keep it stationary.
		*/
		/*!Two Thread between stationary plank and swinging plank
		* - \c b2DistanceJointDef jointDef 
		*     - \c jointDef It is to attach both the bodies ,stationary plank and swinging plank to constrain the motion of the swinging plank only in horizonatl direction. 
		*/
      {
        b2Vec2 right_end(8.0f,0.0f);
        b2Vec2 left_end(-8.0f,0.0f);

        b2BodyDef *bd2 = new b2BodyDef;
        
        bd2->type = b2_dynamicBody;
        bd2->position.Set(x-7.5f,y-8.0f);
        bs11.SetAsBox(10,0.2);
        fd1->shape = &bs11;
        b2Body* plank = m_world->CreateBody(bd2);
        plank->CreateFixture(fd1);

        b2DistanceJointDef jointDef;
        jointDef.Initialize(plank, box, plank->GetWorldPoint(right_end), box->GetWorldPoint(right_end));
        (b2DistanceJoint*)m_world->CreateJoint(&jointDef);
        jointDef.Initialize(plank, box, plank->GetWorldPoint(left_end), box->GetWorldPoint(left_end));
        (b2DistanceJoint*)m_world->CreateJoint(&jointDef);
        
        m_world->CreateJoint(&jointDef);
        
		//Rod below it
		/*!Supporting Rod Bleow Rotating Plank
		* - \c b2BodyDef *bd3
		*     - \c bd3 defines pointer of body in a world.The world position of the body is set to (x+2,y-17)
		*     - \c bd3->type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
		* - \c b2FixtureDef *fd3
		*     - \c fd3->desity is set to 10 units ,friction is set to .8 and coefficient of restitution is set to 0 (default) so that it won't move coz of collisons.
		*     - A rectangle with dimension .1 * (y-1.4) ( its relative to the height of rotating palnk)
		*/ 
        b2BodyDef *bd3 = new b2BodyDef;
        
        bd3->type = b2_dynamicBody;
        bd3->position.Set(x+2,y-17.0f);
        b2PolygonShape bs12;
        bs12.SetAsBox(0.1,y-1.4);
        b2FixtureDef *fd4 = new b2FixtureDef;
        fd4->friction = 0.8;
        fd4->density = 10.0;
        fd4->shape = &bs12;
        b2Body* rod = m_world->CreateBody(bd3);
        rod->CreateFixture(fd4);
        
      }
    }
 //rotating plank 1
	/*!Rotating plank 1 (Left)
	* - \c b2FixtureDef *fd1 -- Horizontal
	* - \c b2FixtureDef *fd2 -- Left Vertical
	* - About two b2FixtureDef
	*   - the vertical side of this platform is the side on by which A big Ball is resting 
	*   - desity = 10units (vertical side) = 1.5( for base/horizontal bar). and friction is set to .5 and coefficient of restitution is set to 0 so that it won't move coz of collisons.
	re.
	*   - One rectangle is vertical with dimensions .2*1 ( on left ) while other one is horizontal with dimensions 12*.2.
	*   - At the end, all the fixtures are attached to box and box is then attached to the main world 
	* - \c b2BodyDef *bd
	*     - \c bd defines pointer of body in a world.The world position of the body is set to (x-10,y+7)
	*     - \c bd->type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
	* - If the plank rotates to the left then ball resting on it will fall on the slant edge made on left side of plank below it.
	*/

	/*!- \a Hinge \a for \a left \a rotating \a platform \a ( \a Hidden  \a bar)
	*     - \c b2Body* body
	*           - \c body : its a rigid body pointer 
	*     - \c b2PolygonShape shape2;
	*           - \c shape2 : to set shape to the rotating horizontal platform. dimension .2*2   
	*     - \c b2FixtureDef fd;
	*           - \c fd : A fixture definition is used to create a fixture.
	*     - \c b2BodyDef bd;
	*           - \c bd : defines a new body "bd" with default variables set .Its not made dynamic.It is placed at position at (x-10.0f, y+7.5f) 
	*/

	/*!
	* - \c b2RevoluteJointDef jointDef 
	*     - \c jointDef It is to attach both the bodies the Horizontal rotating bar and the vertical hidden fixed bar to a common anchor point so that horizontal bar can rotate about this point.
	   - Finally, it is also attached to the world.
	*/
  {
    b2BodyDef *bd = new b2BodyDef; 
    bd->type = b2_dynamicBody;
    bd->position.Set(x-10.0f,y+7.0f);
    b2FixtureDef *fd1 = new b2FixtureDef;
    fd1->density = 1.5;
    fd1->friction = 0.5;
    fd1->restitution = 0.f;
    fd1->shape = new b2PolygonShape;
    b2PolygonShape bs1;
    bs1.SetAsBox(12,0.2, b2Vec2(0.0f,0.0f), 0);
    fd1->shape = &bs1;
    b2FixtureDef *fd2 = new b2FixtureDef;
    fd2->density = 10.0;
    fd2->friction = 0.5;
    fd2->restitution = 0.f;
    fd2->shape = new b2PolygonShape;
    b2PolygonShape bs2;
    bs2.SetAsBox(0.2,1, b2Vec2(11.0f,1.0f), 0);
    fd2->shape = &bs2;
    

    b2Body* box1 = m_world->CreateBody(bd);
    box1->CreateFixture(fd1);
    box1->CreateFixture(fd2);

    b2PolygonShape shape2;
    shape2.SetAsBox(0.2f, 2.0f);
    b2BodyDef bd2;
    bd2.position.Set(x-10.0f, y+7.5f);
    b2Body* body2 = m_world->CreateBody(&bd2);
                           
    //Joint for rotating two bodies
    b2RevoluteJointDef jointDef;
    jointDef.bodyA = box1;
    jointDef.bodyB = body2;
    jointDef.localAnchorA.Set(0,0);
    jointDef.localAnchorB.Set(0.0f,0.0f);
    // jointDef.collideConnected = false;
    m_world->CreateJoint(&jointDef);
  }

  //rotating plank 2
	/*!Rotating plank 2 (Right)
	* - \c b2FixtureDef *fd1 -- Horizontal
	* - \c b2FixtureDef *fd2 -- Left Vertical
	* - About two b2FixtureDef
	*   - the vertical side of this platform is the side on by which a small Ball is resting 
	*   - desity = 1units (vertical side) = 1( for base/horizontal bar). and friction is set to .5 and coefficient of restitution is set to 0 so that it won't move coz of collisons.
	re.
	*   - One rectangle is vertical with dimensions .2*1 ( on right ) while other one is horizontal with dimensions 12*.2.
	*   - At the end, all the fixtures are attached to box and box is then attached to the main world 
	* - \c b2BodyDef *bd
	*     - \c bd defines pointer of body in a world.The world position of the body is set to (x+10.0f,y+3.0f)
	*     - \c bd->type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
	* - If the plank rotates to the right then ball resting on it will fall on the funnel shaped structure on its right and below it.
	*/

	/*!- \a Hinge \a for \a right \a rotating \a platform \a ( \a Hidden  \a bar)
	*     - \c b2Body* body
	*           - \c body : its a rigid body pointer 
	*     - \c b2PolygonShape shape2;
	*           - \c shape2 : to set shape to the rotating horizontal platform. dimension .2*2   
	*     - \c b2FixtureDef fd;
	*           - \c fd : A fixture definition is used to create a fixture.
	*     - \c b2BodyDef bd;
	*           - \c bd : defines a new body "bd" with default variables set .Its not made dynamic.It is placed at position at (x-10.0f, y+7.5f) 
	*/

	/*!
	* - \c b2RevoluteJointDef jointDef 
	*     - \c jointDef It is to attach both the bodies the Horizontal rotating bar and the vertical hidden fixed bar to a common anchor point so that horizontal bar can rotate about this point.
	   - Finally, it is also attached to the world.
	*/
  {
    b2BodyDef *bd = new b2BodyDef; 
    bd->type = b2_dynamicBody;
    bd->position.Set(x+10.0f,y+3.0f);
    b2FixtureDef *fd1 = new b2FixtureDef;
    fd1->density = 1.0;
    fd1->friction = 0.5;
    fd1->restitution = 0.f;
    fd1->shape = new b2PolygonShape;
    b2PolygonShape bs1;
    bs1.SetAsBox(12,0.2);
    fd1->shape = &bs1;
    b2FixtureDef *fd2 = new b2FixtureDef;
    fd2->density = 1.0;
    fd2->friction = 0.5;
    fd2->restitution = 0.f;
    fd2->shape = new b2PolygonShape;
    b2PolygonShape bs2;
    bs2.SetAsBox(0.2,1, b2Vec2(-11.0f,1.0f), 0);
    fd2->shape = &bs2;
    

    b2Body* box1 = m_world->CreateBody(bd);
    box1->CreateFixture(fd1);
    box1->CreateFixture(fd2);

    b2PolygonShape shape2;
    shape2.SetAsBox(0.2f, 2.0f);
    b2BodyDef bd2;
    bd2.position.Set(x+10.0f, y+3.5f);
    b2Body* body2 = m_world->CreateBody(&bd2);
                           
    //Joint for rotating two bodies
    b2RevoluteJointDef jointDef;
    jointDef.bodyA = box1;
    jointDef.bodyB = body2;
    jointDef.localAnchorA.Set(0,0);
    jointDef.localAnchorB.Set(0.0f,0.0f);
    m_world->CreateJoint(&jointDef);
  }

  //Balls on the rotating planks
  /*!BAlls on rotating planks
  *    - \c b2Body* spherebody
    *          - \c spherebody sets the shape as spherically appearing 2d circular body
    *    - \c b2CircleShape circle
    *          - \c b2CircleShape is set to a circle with radius 2 for bigger one and 1 for smaller one.
    *    - \c b2FixtureDef ballfd;
    *          - \c ballfd : A fixture definition is used to create a fixture.
    *          - \c density of bigger ball is twice that of smaller so that at the end it can lift smaller ball giving it good initial impulse.
    *           - \c coeff of restitution is set to .9 for smalller so that it can reflect after collison while its 0 for bigger ball
    *           - \c coeff of friction is set to 0 for smalller so that it does not rotate while reflection during collison while its 0.6 for bigger ball
    *          - \c ballbd.position of the 2 spherical balls is set with respect to the platforms on which they resting so that they are placed near vertical sides of the rotating platforms and when they move these balls will fall from other side.
    *- Balls are placed at position where they are hit by vertical roating bars at position where vertical bar is at its lowest position.
    *    - \c b2BodyDef ballbd
    *          - \c ballbd.type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
  */
  {
    b2Body* spherebody;

    b2CircleShape circle[2];
    circle[0].m_radius = 1;
    circle[1].m_radius = 2;

    b2FixtureDef ballfd;
    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;

    for (int i = 0; i < 2; i++){
      ballfd.shape = &circle[i];
      ballfd.density = 10.0f + i*5.0f;
      ballbd.position.Set(x-i,y+4+5*i);
      spherebody = m_world->CreateBody(&ballbd);
      spherebody->CreateFixture(&ballfd);
    }
  }
  
   //Replacing knife
	/*!
	* \b The \b 3 Balls \b And \b 3 Vertical Rotating Bars System.
	*- first the lowest plank is place with ball on it and vertical roating bar at its end.
	*- x =45.0 and y=22.5 are the coordinates of pivot of the lowest rotating bar. Everything else including the above platforms and roating bars and balls are placed with respect to this point.
	*/

	/*! 3 Platforms 
	*- \c b2EdgeShape shape
	*  - \c shape : to set shape to the fixed stationary solid object 
	*- \c b2BodyDef bd
	*  - \c Body with the given body definition will be formed and setting density to 0 makes it immovable. 
	* - \c b2Body* b1
	  *  - b1: its a rigid body pointer which will represent the ground
	*/    

	/*! 3 Vertical Roating Bars
	* - \c b2Body* body      
	*     - \c body : its a rigid body pointer which will represent each of the revolving vertical bar
	* - \c b2PolygonShape shape;
	*     - \c shape : to set shape to the revolving vertical bars. they are vertical rectangles with dimension .2*1.5   
	* - \c b2FixtureDef fd;
	*     - \c fd : A fixture definition is used to create a fixture. This class defines an abstract fixture definition.Density is set to 10 and setting its shape
	* - \c b2BodyDef bd;
	*     - \c bd : defines a new body "bd" with default variables set .You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
	*- these bars are palced at position relative to right end of horizontal platform. 1st and 3rd are at right and of the platform (x+.2) while the middle one is the left end of the platform (x-5.2)
	*- they were placed in order to make the balls placed on lower platform move when they are hit by vertical bars.
	* - At the end, body is attached to the world.
	*/

	/*!- \a Hinge \a for \a vertical \a bars \a ( \a Hidden  \a bar)
	*     - \c b2Body* body
	*           - \c body : its a rigid body pointer 
	*     - \c b2PolygonShape shape2;
	*           - \c shape2 : to set shape to the revolving horizontal platform. Its a vertical rectangle with dimension .2*2   
	*     - \c b2FixtureDef fd;
	*           - \c fd : A fixture definition is used to create a fixture. This class defines an abstract fixture definition. You can reuse fixture definitions safely like setting its shape
	*     - \c b2BodyDef bd;
	*           - \c bd : defines a new body "bd" with default variables set .Its not made dynamic so that it can act as a hinge for horizonatl rotating bar.It is placed at position at (14,16) 
	*     - At the end, body is attached to the world.
	*/

	/*!
	 * - \c b2RevoluteJointDef jointDef 
	*     - \c jointDef It is to attach both the bodies the vertical rotating bar and the vertical hidden fixed bar to a common anchor point so that vertical bar can rotate about this point.
	*     - \c jointDef.bodyA = vertical bar
	*     - \c jointDef.bodyB = hidden bar;
	*     - \c jointDef.localAnchorA = The local anchor point relative to bodyA's origin. 
	*     - \c jointDef.localAnchorB = The local anchor point relative to bodyB's origin. 
	*     - \c collideConnected: Set this flag to true if the attached bodies should collide. 
	*     - Vertical Bars rotate about these points and then finally hit a ball at their respective lowest position in order to move forward the simulation.
	*     - Finally, it is also attached to the world.
	*/   

	/*!
	* \b The \b 3 \b spheres \b on \b the \b platforms ( middle platform, bottom platform, floor-1)
	*    - \c b2Body* spherebody
	*          - \c spherebody sets the shape as spherically appearing 2d circular body
	*    - \c b2CircleShape circle
	*          - \c b2CircleShape is set to a circle with radius 1.
	*    - \c b2FixtureDef ballfd;
	*          - \c ballfd : A fixture definition is used to create a fixture. This class defines an abstract fixture definition. You can reuse fixture definitions safely like friction is set to 0 ,density is set to 1 units ,setting its shape to cirlce and coefficient of restituation to 0 (Restitution is used to make objects bounce. The restitution value is usually set to be between 0 and 1). A value of zero means the ball won't bounce.
	*          - \c ballbd.position of the 3 spherical balls is set with respect to the platforms on which they resting like the one on middle one is at its right end and on bottom platform its at its left end and 3rd one is just below the 3rd rotating bar on floor-1. 
	*- Balls are placed at position where they are hit by vertical roating bars at position where vertical bar is at its lowest position.
	*    - \c b2BodyDef ballbd
	*          - \c ballbd.type is set to b2_dynamicBody.You must set the body type to b2_dynamicBody if you want the body to move in response to forces.
	*    - At the end spherebodies are attached to the world and body definition is attached to it
	*/

  //3rd and lowest plank with rod for it and ball for it
  x=45.0f,y=22.5f;

        
        {
          b2EdgeShape shape;
          b2BodyDef bd;
          b2Body* b1;
        shape.Set(b2Vec2(x-5.0f, y), b2Vec2(x, y));
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }



       {
        b2PolygonShape shape;
        shape.SetAsBox(.2f, 1.5f);

        b2BodyDef bd;
        bd.position.Set(x+.2f, y+1.7f);
        // bd.angle = -.1;
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 10.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        // shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(x+.2f, y+.2f);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,-1.5f);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
      }

     {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 1.0f;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;

        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(x-5.0f, y+2.0f);
        spherebody = m_world->CreateBody(&ballbd);
        spherebody->CreateFixture(&ballfd);
     } 

     ///2nd and middle plank with rod for it and balll for it
      y=y + 4.0f;
      {
        b2EdgeShape shape;
        b2BodyDef bd;
        b2Body* b1;
        shape.Set(b2Vec2(x-5.0f, y), b2Vec2(x, y));
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }



       {
        b2PolygonShape shape;
        shape.SetAsBox(.2f, 1.5f);

        b2BodyDef bd;
        bd.position.Set(x-5.2f, y+1.7f);
        // bd.angle = .1;
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 10.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        // shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(x-5.2f, y+.2f);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,-1.5f);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
      }

      {
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 1.0f;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 0.0f;

        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        ballbd.position.Set(x, y+2.0f);
        spherebody = m_world->CreateBody(&ballbd);
        spherebody->CreateFixture(&ballfd);
     } 


     ///1st and top plank with rod for it and ball for it
      y=y+4.0f;
      {
        b2EdgeShape shape;
        b2BodyDef bd;
        b2Body* b1;
        shape.Set(b2Vec2(x-5.0f, y), b2Vec2(x, y));
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);
        }

    //Ground
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-80.0f, ground_y), b2Vec2(80.0f, ground_y));

      b2BodyDef bd; 
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
    
    //funnel
    {
      float shift_x = -0,shift_y=-0.5;
      b2BodyDef bd;
      b2Body* f;
      for(int i = 0; i < 2; i++){
        b2EdgeShape shape;
        shape.Set(b2Vec2(shift_x+5.1*i-0.1 , shift_y-5), b2Vec2(shift_x+5.1*i-0.1,shift_y+ -10.0f));
        f = m_world->CreateBody(&bd);
        f->CreateFixture(&shape, 0.0f);
      }
      //left inclined incline
      {
        b2EdgeShape shape;
        shape.Set(b2Vec2(shift_x-0.1,shift_y+ -5), b2Vec2(shift_x - 5.9 , shift_y+0.0f));
        f = m_world->CreateBody(&bd);
        f->CreateFixture(&shape, 0.0f);
      }
      //right inclined incline
      {
        b2EdgeShape shape;
        shape.Set(b2Vec2(shift_x+5.0,shift_y+ -5), b2Vec2(shift_x+18.1 +1.31, shift_y+6.0+1.1));// 13.1  , 11*0.1
        f = m_world->CreateBody(&bd);
        f->CreateFixture(&shape, 0.0f);
      }
    }
   
    //rotating plank  below funnel
    {
      float x=7;
      b2BodyDef *bd = new b2BodyDef; 
      bd->type = b2_dynamicBody;
      //bd->angle=-b2_pi/6;
      bd->position.Set(x,-15.0f);
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 1.5;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(9,0.2, b2Vec2(0.0f,0.0f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 30.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,1, b2Vec2(9.0f,1.0f), -.5);
      fd2->shape = &bs2;


      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(x,-15.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);
                             
      //Joint for rotating two bodies
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = box1;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(-3,0);
      jointDef.localAnchorB.Set(0.0f,0.0f);
      // jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);

    }

    //Swastik
    {
      //Horizontal Rod
      float rod_length=5.3f,
             rod_density=50,
             swas_x = 30-4.5f,
             swas_y = ground_y + 15.0f;
      b2PolygonShape shape;
      shape.SetAsBox(0.2f,rod_length);
      
      b2BodyDef bd;
      bd.position.Set(swas_x, swas_y);
      bd.type = b2_dynamicBody;
      bd.angle = -.1;
      b2Body* body = m_world->CreateBody(&bd);

      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = rod_density;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);


      b2PolygonShape shape2;
      shape2.SetAsBox(rod_length,0.2f);

      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = rod_density;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape2;
      body->CreateFixture(fd2);

      //balls
      b2Body* spherebody;
      b2CircleShape circle;
      circle.m_radius = 0.5;

      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 500.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;

      for (int i = -2; i < 3; i++){
        float temp=0;
        if (i==0) continue;
        else if (i<0) temp = -0.5f;
        else if (i>0) temp = 0.5f;
        b2BodyDef ballbd;
        ballbd.type = b2_dynamicBody;
        float pos_x = swas_x + i*2*circle.m_radius + temp;
        ballbd.position.Set(pos_x , swas_y+0.8);
        spherebody = m_world->CreateBody(&ballbd);
        spherebody->CreateFixture(&ballfd);
      }

      b2BodyDef ibd;
      ibd.position.Set(swas_x, swas_y);
      b2Body* invi_body = m_world->CreateBody(&ibd);

      //Rotation of the cross
      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = invi_body;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }
    
    //Spring2
    {
      b2Body* fl_rod, *inc_rod[4][2];
      b2Body* box;
      float spring_x=bottom_x,spring_y=ground_y+5;

      b2PolygonShape shape;
      shape.SetAsBox(8.f, .1f,b2Vec2(0.0f,0.0f),0);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;

      //flat rod
      {
        b2PolygonShape shape2;
        shape2.SetAsBox(.2,2.f,b2Vec2(-8.0f,2.0f),0);
        b2FixtureDef fd1;
        fd1.shape = &shape2;
        fd1.density = 20.0f;
        
        b2PolygonShape shape3;
        shape3.SetAsBox(.2,2.f,b2Vec2(1.0f,2.0f),0);
        b2FixtureDef fd2;
        fd2.shape = &shape3;
        fd2.density = 20.0f;        


        b2BodyDef bd;
        bd.type = b2_dynamicBody;
        // bd.angle = b2_pi/2;
        bd.position.Set(spring_x,spring_y);
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        body->CreateFixture(&fd1);
        body->CreateFixture(&fd2);
        fl_rod = body;
      }

      shape.SetAsBox(0.1f, 2.83f);


      {
        b2PolygonShape shape;
        shape.SetAsBox(.2f, 1.5f);

        b2BodyDef bd;
        bd.position.Set(x+.2f, y+1.7f);
        // bd.angle = -.1;
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 10.f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        b2PolygonShape shape2;
        // shape2.SetAsBox(0.2f, 2.0f);
        b2BodyDef bd2;
        bd2.position.Set(x+.2f, y+.2f);
        b2Body* body2 = m_world->CreateBody(&bd2);

        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = body2;
        jointDef.localAnchorA.Set(0,-1.5f);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
      }

      fd.shape = &shape;
      fd.density = 20.0f;

      //inclined rods
      for (int i = 0; i < 4; ++i){
        for (int j = 0; j < 2; ++j){
          b2BodyDef bd;
          bd.type = b2_dynamicBody;
          bd.angle = (2*j-1)*b2_pi/6;
          bd.position.Set(spring_x -5.5f + 3*i,spring_y - 35 + 32.6f);
          b2Body* body = m_world->CreateBody(&bd);
          body->CreateFixture(&fd);
          inc_rod[i][j] = body;
        }
      }

      //Punching Box
      {
        b2PolygonShape shape;
        shape.SetAsBox(2 , 2.5f);

        b2FixtureDef fd;
        fd.shape = &shape;
        fd.density = 20.0f;

        b2BodyDef bd;
        bd.position.Set(-22.7f + 35 + spring_x, spring_y - 2.5);
        bd.type = b2_dynamicBody;
        b2Body* body = m_world->CreateBody(&bd);
        body->CreateFixture(&fd);
        //body->ResetMassData
        box = body;
      }
    
      b2Vec2 right_end1(0,-2.83);
      b2Vec2 right_end2(0,2.83);
      //joints
      b2RevoluteJointDef jointDef;
      b2DistanceJointDef distDef;
      jointDef.Initialize(inc_rod[1][0], fl_rod, inc_rod[1][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      distDef.Initialize(inc_rod[0][0], f0, inc_rod[0][0]->GetWorldPoint(right_end1), inc_rod[0][0]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&distDef);
      jointDef.Initialize(inc_rod[0][0], inc_rod[0][1], inc_rod[0][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[1][0], inc_rod[1][1], inc_rod[1][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[2][0], inc_rod[2][1], inc_rod[2][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[3][0], inc_rod[3][1], inc_rod[3][0]->GetWorldCenter());
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[0][0], inc_rod[1][1], inc_rod[0][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[0][1], inc_rod[1][0], inc_rod[0][1]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[1][0], inc_rod[2][1], inc_rod[1][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[1][1], inc_rod[2][0], inc_rod[1][1]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[2][0], inc_rod[3][1], inc_rod[2][0]->GetWorldPoint(right_end2));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      jointDef.Initialize(inc_rod[2][1], inc_rod[3][0], inc_rod[2][1]->GetWorldPoint(right_end1));
      (b2RevoluteJoint*)m_world->CreateJoint(&jointDef);
      distDef.Initialize(inc_rod[3][1], box, inc_rod[3][1]->GetWorldCenter(), box->GetWorldCenter());
      (b2DistanceJoint*)m_world->CreateJoint(&distDef);
    }
    
    //Hammer and Plank System
    {
      float hammer_x = bottom_x + 23.8,hammer_y=0.7 + ground_y;
      //Hammer
      {
        b2BodyDef bd;
        bd.position.Set(hammer_x, 4.0f+hammer_y);
        bd.type = b2_dynamicBody;

        b2Body* body = m_world->CreateBody(&bd);

        //hammer part 1
        b2PolygonShape shape;
        shape.SetAsBox(0.2f,4.0f);
        b2FixtureDef *fd = new b2FixtureDef;
        fd->density = 0.5f;
        fd->shape = new b2PolygonShape;
        fd->shape = &shape;
        body->CreateFixture(fd);

        //hammer part 2
        b2PolygonShape shape2;
        shape2.SetAsBox(2.0f,0.5f,b2Vec2(0.0f,4.7f),0);
        b2FixtureDef *fd2 = new b2FixtureDef;
        fd2->density = 200.f;
        fd2->shape = new b2PolygonShape;
        fd2->shape = &shape2;
        body->CreateFixture(fd2);

        b2BodyDef ibd;
        ibd.position.Set(hammer_x, hammer_y);
        b2Body* invi_body = m_world->CreateBody(&ibd);

        //Rotation of the hammer
        b2RevoluteJointDef jointDef;
        jointDef.bodyA = body;
        jointDef.bodyB = invi_body;
        jointDef.localAnchorA.Set(0,-4.f);
        jointDef.localAnchorB.Set(0,0);
        jointDef.collideConnected = false;
        m_world->CreateJoint(&jointDef);
      }

      //Bottom Right Base and Plank
      {
        float base_x =  hammer_x + 10.0f;
        {
          base_x = base_x -.22f;
          b2EdgeShape shape;
          shape.Set(b2Vec2(base_x,ground_y +  0.0f), b2Vec2(base_x,ground_y + 3.0f));
          b2Body* b1;
          b2BodyDef bd;
          b1 = m_world->CreateBody(&bd);
          b1->CreateFixture(&shape, 0.0f);

          b2EdgeShape shape2;
          shape2.Set(b2Vec2(base_x,ground_y +  3.0f), b2Vec2(base_x+10,ground_y + 3.0f));
          b2Body* b2;
          b2BodyDef bd2;
          b2 = m_world->CreateBody(&bd2);
          b2->CreateFixture(&shape2, 0.0f);
          base_x = base_x +.22f;
        }
    
        {
          //Plank
          b2PolygonShape shape;
          shape.SetAsBox(2.3f,0.2f);

          b2BodyDef bd;
          //bd.position.Set(base_x+0.8f, 3.1f);
          bd.position.Set(base_x+0.82f,ground_y + 3.1f);
          bd.type = b2_dynamicBody;
          b2Body* body = m_world->CreateBody(&bd);
          b2FixtureDef *fd = new b2FixtureDef;
          fd->density = 0.2f;
          //       fd->friction = 0.2f;
          //       fd->restitution = 0.5f;
          fd->shape = new b2PolygonShape;
          fd->shape = &shape;
          body->CreateFixture(fd);
        }
      }

      //Oscillating balls
      {
        float pos_x=6.5f+bottom_x,pos_y=ground_y + 26.7,thrd_len=10.3f;
        b2Body* spherebody;

        b2CircleShape circle;
        circle.m_radius = 0.5;

        b2FixtureDef ballfd;
        ballfd.shape = &circle;
        ballfd.density = 1.0f;
        ballfd.friction = 0.0f;
        ballfd.restitution = 1.0f;
        for (int i = 0; i < 4; ++i){
          b2BodyDef ballbd;
          ballbd.type = b2_dynamicBody;
          float pos_x1 = pos_x + i*(2*circle.m_radius + 0.02f);
          ballbd.position.Set(pos_x1 , pos_y);
          spherebody = m_world->CreateBody(&ballbd);
          spherebody->CreateFixture(&ballfd);

          b2Body* b4;
          {
            b2BodyDef bd;
            bd.position.Set(pos_x1,pos_y + thrd_len);
            b4 = m_world->CreateBody(&bd);
          }

          b2RevoluteJointDef jd;
          b2Vec2 anchor;
          anchor.Set(pos_x1, pos_y+thrd_len);
          jd.Initialize(spherebody, b4, anchor);
          m_world->CreateJoint(&jd);     
        }
      }
    }
  }
  
  


  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
