#include <iostream>
#include "swarmz.h"

using namespace std;
using namespace sw;

int main() {
    cout << "Hello, Swarmz World!" << endl;

    vector<Boid> boids;
    boids.push_back(Boid(Vec3(1, 1, 2)));
    boids.push_back(Boid(Vec3(2, 1, 2)));
    Swarm swarm(&boids);
    swarm.Update(1);
    
    return 0;
}