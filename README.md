# Swarmz

<a href="http://unlicense.org/"><img src="https://img.shields.io/badge/license-Public%20Domain-blue.svg"></a>

A free, header-only C++ swarming (flocking) library for real-time applications.

An example usage can be seen in [this particle editor plugin](https://cultrarius.github.io/UE4ParticleEditorPlugin#swarm) for the Unreal Engine 4.

# Usage

Just include the header file to start:
```
#include "swarmz.h"

using namespace sw;
```

To create a new swarm, a pointer to the list of `Boids` is required.
A boid is a single member of the swarm.
Each boid has a position and a velocity: 

```
// define our swarm members
vector<Boid> boids;
boids.push_back(Boid(Vec3(1, 0, 0), Vec3(1, 0, 0)));
boids.push_back(Boid(Vec3(1.5, 0, 0), Vec3(1, 1, 0)));
boids.push_back(Boid(Vec3(1, 0.5, 0.5), Vec3(0, 1, 0)));
boids.push_back(Boid(Vec3(4, 4, -2), Vec3(1, 0, 0)));

//create the swarm!
Swarm swarm(&boids);
```

The swarm has a lot of different settings, but the most important ones are the following:
* `PerceptionRadius` - determines the vision radius of each boid. Only boids within this distance influence each other.
* `SeparationWeight` - how much boids repel each other
* `AlignmentWeight` - how much boids want go in the same direction
* `CohesionWeight` - how much boids want to be in the center of the swarm
* `MaxAcceleration` - how fast each boid can change its direction
* `MaxVelocity` - how fast each boid can move

Each setting has a sensible default parameter, but playing around with them is half the fun:

```
// configure swarm
swarm.PerceptionRadius = 100;
swarm.CohesionWeight = 0.2;
```


To calculate the swarm movement, you usually update the swarm a few times a second in a loop.
The swarm does not automatically move the boids, it just updates the velocity and acceleration for each boid.
The position has to be updated by you.

```
// your tick/update loop
while (true) {
  swarm.Update(deltaSeconds);
  
  // each boid now has an updated velocity
  for (auto& boid : boids) {
    // update position
    boid.Position += boid.Velocity * deltaSeconds;
    
    // draw boid
  }
}
```

If you do not want the swarm to even update the boids velocity automatically (because you want to debug the result or you want to scale the acceleration per boid), then you can just call `UpdateAcceleration()` instead of the `Update(deltaSeconds)` on the swarm.

You can of course adjust the swarm parameters as well as the boid list during the loop to change the swarm behaviour over time.
The library is, however, *not* thread-safe (so please no changes during the swarm update)!

That's basically it, have fun! :)
