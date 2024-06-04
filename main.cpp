// Programming Particle life - by Programming Chaos
//
// Youtube: https://www.youtube.com/watch?v=xiUpAeos168
//
// Implemented by Joseph21


#define OLC_PGE_APPLICATION
#include "olcPixelGameEngine.h"

// keep the screen dimensions constant and vary the resolution by adapting the pixel size
#define SCREEN_X   1000
#define SCREEN_Y    800
#define PIXEL_X       1
#define PIXEL_Y       1

#define PART_TYPES      6
#define PART_SIZE       3
#define PART_VAR_SIZE   true

#define PART_NUMBER  1500
#define K               0.0005f    // scale down forces so that particles don't move too rapidly
#define FRICTION        0.85f


float fMap( float x, float x_min, float x_max, float res_min, float res_max ) {
    // get x as a percentage of the range between x_min and x_max
    float fPerc = (x - x_min) / (x_max - x_min);
    // return this percentage in the range res_min...res_max
    return res_min + fPerc * (res_max - res_min);
}

float random_range( float a, float b ) {
    int nThreshold = 10000;
    int nRand = rand() % nThreshold;
    return fMap( nRand, 0, nThreshold, a, b );
}


float forces[      PART_TYPES][PART_TYPES];
float minDistances[PART_TYPES][PART_TYPES];
float radii[       PART_TYPES][PART_TYPES];

// particle class -
class particle {
public:
    olc::vf2d position;
    olc::vf2d velocity;
    int type = -1;
    int nID = -1;

    // constructor provide random position on screen, random type and 0 velocity
    particle( olc::PixelGameEngine *pEngine, int id ) {
        position = olc::vf2d( rand() % SCREEN_X, rand() % SCREEN_Y );
        velocity = olc::vf2d( 0, 0 );
        type = rand() % PART_TYPES;

        nID = id;
        pGfx = pEngine;
    }

    // derive the colour from the type using the colorSteps array
    olc::Pixel GetColour() {
        return colorSteps[ type ];
    }

    int GetRadius() {
        return (PART_VAR_SIZE ? (type / 2 + 1) : PART_SIZE);
    }

    void pulse( olc::vi2d &vPos ) {
        olc::vf2d auxVec = position - vPos;
        float fDist = auxVec.mag();
        auxVec.norm();
        if (fDist < 200.0f) {
            velocity += (auxVec * ((200.0f - fDist) * 10.0f * K));
        }
    }

    void update( std::vector<particle> &vSwarm ) {
        olc::vf2d direction, totalForce, acceleration;
        float distance;
        for (auto &p : vSwarm) {
            if (p.nID != this->nID) {
                // start with zero dir. vector
                direction = direction * 0.0f;   // i think this is superfluous
                // get direction from this particle to p
                direction = p.position - this->position;
                // correct for toroidal distances
                if (direction.x >  0.5f * SCREEN_X) { direction.x -= SCREEN_X; }
                if (direction.x < -0.5f * SCREEN_X) { direction.x += SCREEN_X; }
                if (direction.y >  0.5f * SCREEN_Y) { direction.y -= SCREEN_Y; }
                if (direction.y < -0.5f * SCREEN_Y) { direction.y += SCREEN_Y; }

                distance = direction.mag();
                direction.norm();

                // calculate forces - repulsive force first
                if (distance < minDistances[this->type][p.type]) {
                    olc::vf2d force = direction * (abs( forces[this->type][p.type] ) * -3);
                    force *= fMap( distance, 0, minDistances[this->type][p.type], 1.0f, 0.0f );
                    force *= K;
                    totalForce += force;
                }
                // ... then attractive (or primary) force
                if (distance < radii[this->type][p.type]) {
                    olc::vf2d force = direction * forces[this->type][p.type];
                    force *= fMap( distance, 0, radii[this->type][p.type], 1.0f, 0.0f );
                    force *= K;
                    totalForce += force;
                }
            }
        }
        // apply the force to the acceleration, the acceleration to the velocity and the velocity to the position
        acceleration += totalForce;
        velocity     += acceleration;
        position     += velocity;

        // wrap around screen boundaries
        if (position.x <      0.0f) position.x += SCREEN_X;
        if (position.y <      0.0f) position.y += SCREEN_Y;
        if (position.x >= SCREEN_X) position.x -= SCREEN_X;
        if (position.y >= SCREEN_Y) position.y -= SCREEN_Y;

        // add pseudo friction to slow things down
        velocity *= FRICTION;
    }

    // let particle display itself
    void display() {
        pGfx->FillCircle( position.x, position.y, GetRadius(), GetColour());
    }

private:
    olc::Pixel colorSteps[PART_TYPES] = {
        olc::YELLOW,
        olc::GREEN,
        olc::CYAN,
        olc::GREY,
        olc::RED,
        olc::BLUE
    };

    olc::PixelGameEngine *pGfx = nullptr;
};



class ParticleLife : public olc::PixelGameEngine {

public:
    ParticleLife() {
        sAppName = "ParticleLife";
    }

private:

    int nNrParticles = PART_NUMBER;

    std::vector<particle> swarm;

public:
    bool OnUserCreate() override {

        srand( time( 0 ));

        // initialize the particle swarm
        initSwarm();

        setParameters();

        return true;
    }

    void initSwarm() {
        swarm.clear();
        for (int i = 0; i < nNrParticles; i++) {
            particle p( this, i );
            swarm.push_back( p );
        }
    }

    void setParameters( bool bOutput = true ) {
        for (int i = 0; i < PART_TYPES; i++) {
            for (int j = 0; j < PART_TYPES; j++) {

                if (rand() % 2 == 0) {
                    forces[i][j] = - random_range( 0.1f, 0.5f );
                } else {
                    forces[i][j] =   random_range( 0.3f, 1.0f );
                }

                minDistances[i][j] = random_range( 30, 50 );
                radii[i][j] = random_range( 70, 200 );
            }
        }
        if (bOutput) {
            std::cout << "FORCES" << std::endl;
            for (int i = 0; i < PART_TYPES; i++) {
                for (int j = 0; j < PART_TYPES; j++) {
                    if (j == 0) {
                        std::cout << "force " << i << ": ";
                    }
                    std::cout << j << " = ";
                    if (forces[i][j] >= 0.0f) {
                        std::cout << " ";
                    }
                    std::cout << forces[i][j] << ", ";
                }
                std::cout << std::endl;
            }
            std::cout << std::endl;
        }
    }

    bool OnUserUpdate( float fElapsedTime ) override {

        // give impulse with mouse clicks
        if (GetMouse( 0 ).bPressed) {
            olc::vi2d vPos = GetMousePos();
            for (auto &p : swarm) {
                p.pulse( vPos );
            }
        }

        // reset rule set
        if (GetKey( olc::R ).bPressed) {
            setParameters();
        }
        // re-init swarm
        if (GetKey( olc::I ).bPressed) {
            nNrParticles = (int)swarm.size();
            initSwarm();
        }
        // determine accellerator
        int nAccelleration = (GetKey( olc::CTRL ).bHeld ? 10 : 1);
        // add one particle
        if (GetKey( olc::NP_ADD).bPressed || GetKey( olc::NP_ADD).bHeld) {
            for (int i = 0; i < nAccelleration; i++) {
                particle p( this, (int)swarm.size());
                swarm.push_back( p );
            }
        }
        // remove one particle
        if (GetKey( olc::NP_SUB).bPressed || GetKey( olc::NP_SUB).bHeld) {
            for (int i = 0; i < nAccelleration; i++) {
                swarm.pop_back();
            }
        }

        // update and render all particles
        for (auto &p : swarm) {
            p.update( swarm );
        }

        Clear( olc::BLACK );

        for (auto &p : swarm) {
            p.display();
        }
        DrawString( 10, 10, "Particles: " + std::to_string( (int)swarm.size()), olc::MAGENTA );

        return true;
    }

    bool OnUserDestroy() override {

        swarm.clear();

        return true;
    }
};

int main()
{
	ParticleLife demo;
	if (demo.Construct( SCREEN_X / PIXEL_X, SCREEN_Y / PIXEL_Y, PIXEL_X, PIXEL_Y ))
		demo.Start();

	return 0;
}

