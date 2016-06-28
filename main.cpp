#include <iostream>
#include <random>
#include "swarmz.h"
#include "timing.h"

using namespace std;
using namespace sw;

int main() {
    cout << "Hello, Swarmz World!" << endl;
    std::random_device rd;
    std::mt19937 eng(rd());


    vector<Boid> boids;
    boids.push_back(Boid(Vec3(1, 0.2, 0.2), Vec3(1, 0, 0)));
    boids.push_back(Boid(Vec3(1.5, 0, 0), Vec3(1, 0, 0)));
    //boids.push_back(Boid(Vec3(2, 2, 0), Vec3(1, 0, 0)));
    Swarm swarm(&boids);
    swarm.Update(1);

    exit(0);
    std::uniform_real_distribution<float> dist(0, 20);
    for (int k = 1; k < 100; k++) {
        int size = k * 100;
        boids.clear();
        for (int i = 0; i < size; i++) {
            boids.push_back(Boid(Vec3(dist(eng), dist(eng), dist(eng)), Vec3::GetRandomUniform(eng)));
        }
        auto start = GetTimeMs64();
        Swarm swarm100(&boids);
        swarm100.Update(1);
        cout << size << ";" << (GetTimeMs64() - start) << "\n";
    }
    cout << endl;

    return 0;
}