/* 
* Copyright (c) 2021 Karol Janic 
*/

#ifndef RIGIDBODY_H
#define RIGIDBODY_H

// declaration of Shape class
class Shape;


// color in RGB model withput alpha channel
// default - white
struct Color
{
    float red = 1.0;
    float green = 1.0;
    float blue = 1.0;
};


// Rigid Body class
class RigidBody
{
public:
    Vector2D position;                  // in [ meter ]
    Vector2D velocity;                  // in [ meter / second ]

    float angularVelocity;          // in [ radian / second ]
    float torque;                   // in [ Newton * meter ]
    float orientation;              // in [ radian ]

    Vector2D force;                 // in [ Newton ]

    float mass;                     // in [ kilogram ]
    float inverseMass;              // in [ 1 / kilogram]
    float inertialMoment;           // in [ kilogram * meter^2 ]
    float inverseInertialMoment;    // in [ 1 / kilogram / meter^2 ] 

    float staticFriction;           // dimensionless 
    float kinetcFriction;           // dimensionless 
    float restitution;              // dimensionless 

    Shape* shape;
    Color bodyColor;

    // constructor
    // _shape - pointer to target body shape
    // ( x, y ) - position of body center
    // _orientation - value of orientation angle to set
    // _mass = body mass
    // _inertialMoment - body moment of intertial
    // _density - body density
    // if we don't want define body mass and body moment of inertial we set this parameters to 0
    // if we don't want define density we set this parameter to 0
    // we have to choose a type of body initialization in RigidBody.cpp in line 7-10
    RigidBody(Shape* _shape, float x, float y, float _orientation, float _mass, float _inertialMoment, float _density);

    // applies additonal force to the body
    // _force - pointer to Vector with additional force to apply
    void ApplyForce(const Vector2D& _force);

    // applies force impulse ( Impulse = average force * delta time ) 
    // impulse - pointer to Vector with force impulse
    // contactVector - pointer to Vector with direction of impulse action
    void ApplyImpulse(const Vector2D& impulse, const Vector2D& contactVector);

    // sets body velocities
    // linearVelocity - pointer to vector with linear velocity
    // _angularVelocity - value of angular velocity to set
    void SetVelocity(const Vector2D& linearVelocity, float _angularVelocity);

    // sets value of torque
    // _torque - value of torque to set
    void SetTorque(float _torque);

    // sets body state to static - no forces act on it 
    void SetStatic();

    // sets values of friction and restitution coefficients
    // _staticFriction - value of static friction to set
    // _kineticFriction - value of kinetic friction to set
    // _restitution - value of coefficien of restitution to set
    void SetFrictions(float _staticFriction, float _kineticFriction, float _restitution);

    // sets body orientation
    // _orientation - value of orientation angle to set 
    void SetOrientation(float _orientation);

    // sets body color
    // ( r, g, b ) values of color proportions in RGB model
    void SetColor(float r, float g, float b);
};

#endif // RIGIDBODY_H