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
    
    //Floor-1
    {
      b2Body* floor;
      b2EdgeShape shape;
      shape.Set(b2Vec2(0.0f, 18.5f), b2Vec2(50.0f, 18.5f));

      b2BodyDef bd;
      floor = m_world->CreateBody(&bd);
      floor->CreateFixture(&shape, 0.0f);
    }

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

    //Spring System
    {
      b2Body* fl_rod, *inc_rod[4][2];
      b2Body* box;

      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 8.0f);

      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;

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

    //Circular-Paths
    {
      b2EdgeShape shape;
      b2BodyDef bd;
      b2Body* b1;

      float x = 0.0f,y=40.0f,r=5.0f;
      //Roll-1
      for (int i = -5; i < 25; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

      x = 15.0f;
      //Roll-2
      for (int i = 0; i < 30; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

      x = -4.3f;y = 35.0f;
      //Ramp-1
      for (int i = 30; i < 36; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

      x = 4.1f;
      //Ramp-2
      for (int i = 24; i < 30; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }

      x = 13.6f;y = 35.0f;
      //Ramp-3
      for (int i = 30; i < 39; ++i)
      {
        shape.Set(b2Vec2(x + r*cos(i*b2_pi/20), y + r*sin(i*b2_pi/20)), b2Vec2(x + r*cos((i+1)*b2_pi/20), y + r*sin((i+1)*b2_pi/20)));
        b1 = m_world->CreateBody(&bd);
        b1->CreateFixture(&shape, 0.0f);
      }
    }


    //Pulley
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
      //support for right box
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

    //dominoes
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

	 	x=-20.0f,y=10.0f;
		//Swing
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
			{
				b2Vec2 right_end(8.0f,0.0f);
				b2Vec2 left_end(-8.0f,0.0f);

				b2BodyDef *bd2 = new b2BodyDef;
				
				bd2->type = b2_dynamicBody;
				bd2->position.Set(x+10.0f,y-9.0f);
				//bd2->fixedRotation = true;
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
			}
		}

    /*
    //Ground
    b2Body* b1;
    {
      b2EdgeShape shape;
      shape.Set(b2Vec2(-90.0f, 0.0f), b2Vec2(90.0f, 0.0f));
  
      b2BodyDef bd;
      b1 = m_world->CreateBody(&bd);
      b1->CreateFixture(&shape, 0.0f);
    }
      
    //Top horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(6.0f, 0.25f);
  
      b2BodyDef bd;
      bd.position.Set(-31.0f, 30.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }

    //Dominos
    {
      b2PolygonShape shape;
      shape.SetAsBox(0.1f, 1.0f);
  
      b2FixtureDef fd;
      fd.shape = &shape;
      fd.density = 20.0f;
      fd.friction = 0.1f;
    
      for (int i = 0; i < 10; ++i)
  {
    b2BodyDef bd;
    bd.type = b2_dynamicBody;
    bd.position.Set(-35.5f + 1.0f * i, 31.25f);
    b2Body* body = m_world->CreateBody(&bd);
    body->CreateFixture(&fd);
  }
    }
      
    //Another horizontal shelf
    {
      b2PolygonShape shape;
      shape.SetAsBox(7.0f, 0.25f, b2Vec2(-20.f,20.f), 0.0f);
  
      b2BodyDef bd;
      bd.position.Set(1.0f, 6.0f);
      b2Body* ground = m_world->CreateBody(&bd);
      ground->CreateFixture(&shape, 0.0f);
    }


    //The pendulum that knocks the dominos off
    {
      b2Body* b2;
      {
  b2PolygonShape shape;
  shape.SetAsBox(0.25f, 1.5f);
    
  b2BodyDef bd;
  bd.position.Set(-36.5f, 28.0f);
  b2 = m_world->CreateBody(&bd);
  b2->CreateFixture(&shape, 10.0f);
      }
  
      b2Body* b4;
      {
  b2PolygonShape shape;
  shape.SetAsBox(0.25f, 0.25f);
    
  b2BodyDef bd;
  bd.type = b2_dynamicBody;
  bd.position.Set(-40.0f, 33.0f);
  b4 = m_world->CreateBody(&bd);
  b4->CreateFixture(&shape, 2.0f);
      }
  
      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(-37.0f, 40.0f);
      jd.Initialize(b2, b4, anchor);
      m_world->CreateJoint(&jd);
    }
      
    //The train of small spheres
    {
      b2Body* spherebody;
  
      b2CircleShape circle;
      circle.m_radius = 0.5;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 1.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
  
      for (int i = 0; i < 10; ++i)
  {
    b2BodyDef ballbd;
    ballbd.type = b2_dynamicBody;
    ballbd.position.Set(-22.2f + i*1.0, 26.6f);
    spherebody = m_world->CreateBody(&ballbd);
    spherebody->CreateFixture(&ballfd);
  }
    }

    //The pulley system
    {
      b2BodyDef *bd = new b2BodyDef;
      bd->type = b2_dynamicBody;
      bd->position.Set(-10,15);
      bd->fixedRotation = true;
      
      //The open box
      b2FixtureDef *fd1 = new b2FixtureDef;
      fd1->density = 10.0;
      fd1->friction = 0.5;
      fd1->restitution = 0.f;
      fd1->shape = new b2PolygonShape;
      b2PolygonShape bs1;
      bs1.SetAsBox(2,0.2, b2Vec2(0.f,-1.9f), 0);
      fd1->shape = &bs1;
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 10.0;
      fd2->friction = 0.5;
      fd2->restitution = 0.f;
      fd2->shape = new b2PolygonShape;
      b2PolygonShape bs2;
      bs2.SetAsBox(0.2,2, b2Vec2(2.0f,0.f), 0);
      fd2->shape = &bs2;
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 10.0;
      fd3->friction = 0.5;
      fd3->restitution = 0.f;
      fd3->shape = new b2PolygonShape;
      b2PolygonShape bs3;
      bs3.SetAsBox(0.2,2, b2Vec2(-2.0f,0.f), 0);
      fd3->shape = &bs3;
       
      b2Body* box1 = m_world->CreateBody(bd);
      box1->CreateFixture(fd1);
      box1->CreateFixture(fd2);
      box1->CreateFixture(fd3);

      //The bar
      bd->position.Set(10,15);  
      fd1->density = 34.0;    
      b2Body* box2 = m_world->CreateBody(bd);
      box2->CreateFixture(fd1);

      // The pulley joint
      b2PulleyJointDef* myjoint = new b2PulleyJointDef();
      b2Vec2 worldAnchorOnBody1(-10, 15); // Anchor point on body 1 in world axis
      b2Vec2 worldAnchorOnBody2(10, 15); // Anchor point on body 2 in world axis
      b2Vec2 worldAnchorGround1(-10, 20); // Anchor point for ground 1 in world axis
      b2Vec2 worldAnchorGround2(10, 20); // Anchor point for ground 2 in world axis
      float32 ratio = 1.0f; // Define ratio
      myjoint->Initialize(box1, box2, worldAnchorGround1, worldAnchorGround2, box1->GetWorldCenter(), box2->GetWorldCenter(), ratio);
      m_world->CreateJoint(myjoint);
    }

    //The revolving horizontal platform
    {
      b2PolygonShape shape;
      shape.SetAsBox(2.2f, 0.2f);
  
      b2BodyDef bd;
      bd.position.Set(14.0f, 14.0f);
      bd.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd);
      b2FixtureDef *fd = new b2FixtureDef;
      fd->density = 1.f;
      fd->shape = new b2PolygonShape;
      fd->shape = &shape;
      body->CreateFixture(fd);

      b2PolygonShape shape2;
      shape2.SetAsBox(0.2f, 2.0f);
      b2BodyDef bd2;
      bd2.position.Set(14.0f, 16.0f);
      b2Body* body2 = m_world->CreateBody(&bd2);

      b2RevoluteJointDef jointDef;
      jointDef.bodyA = body;
      jointDef.bodyB = body2;
      jointDef.localAnchorA.Set(0,0);
      jointDef.localAnchorB.Set(0,0);
      jointDef.collideConnected = false;
      m_world->CreateJoint(&jointDef);
    }

    //The heavy sphere on the platform
    {
      b2Body* sbody;
      b2CircleShape circle;
      circle.m_radius = 1.0;
  
      b2FixtureDef ballfd;
      ballfd.shape = &circle;
      ballfd.density = 50.0f;
      ballfd.friction = 0.0f;
      ballfd.restitution = 0.0f;
      b2BodyDef ballbd;
      ballbd.type = b2_dynamicBody;
      ballbd.position.Set(14.0f, 18.0f);
      sbody = m_world->CreateBody(&ballbd);
      sbody->CreateFixture(&ballfd);
    }


    //The see-saw system at the bottom
    {
      //The triangle wedge
      b2Body* sbody;
      b2PolygonShape poly;
      b2Vec2 vertices[3];
      vertices[0].Set(-1,0);
      vertices[1].Set(1,0);
      vertices[2].Set(0,1.5);
      poly.Set(vertices, 3);
      b2FixtureDef wedgefd;
      wedgefd.shape = &poly;
      wedgefd.density = 10.0f;
      wedgefd.friction = 0.0f;
      wedgefd.restitution = 0.0f;
      b2BodyDef wedgebd;
      wedgebd.position.Set(30.0f, 0.0f);
      sbody = m_world->CreateBody(&wedgebd);
      sbody->CreateFixture(&wedgefd);

      //The plank on top of the wedge
      b2PolygonShape shape;
      shape.SetAsBox(15.0f, 0.2f);
      b2BodyDef bd2;
      bd2.position.Set(30.0f, 1.5f);
      bd2.type = b2_dynamicBody;
      b2Body* body = m_world->CreateBody(&bd2);
      b2FixtureDef *fd2 = new b2FixtureDef;
      fd2->density = 1.f;
      fd2->shape = new b2PolygonShape;
      fd2->shape = &shape;
      body->CreateFixture(fd2);

      b2RevoluteJointDef jd;
      b2Vec2 anchor;
      anchor.Set(30.0f, 1.5f);
      jd.Initialize(sbody, body, anchor);
      m_world->CreateJoint(&jd);

      //The light box on the right side of the see-saw
      b2PolygonShape shape2;
      shape2.SetAsBox(2.0f, 2.0f);
      b2BodyDef bd3;
      bd3.position.Set(40.0f, 2.0f);
      bd3.type = b2_dynamicBody;
      b2Body* body3 = m_world->CreateBody(&bd3);
      b2FixtureDef *fd3 = new b2FixtureDef;
      fd3->density = 0.01f;
      fd3->shape = new b2PolygonShape;
      fd3->shape = &shape2;
      body3->CreateFixture(fd3);
    }*/
  }

  sim_t *sim = new sim_t("Dominos", dominos_t::create);
}
