#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include <iostream>

namespace sw {

    struct Vec3 {
        float X;
        float Y;
        float Z;

        explicit Vec3(float x = 0, float y = 0, float z = 0) : X(x), Y(y), Z(z) {
        }

        float DistanceTo(const Vec3 &other) const {
            return std::sqrt(std::pow(other.X - X, 2) + std::pow(other.Y - Y, 2) + std::pow(other.Z - Z, 2));
        }

        Vec3 &operator+=(const Vec3 &other) {
            X += other.X;
            Y += other.Y;
            Z += other.Z;
            return *this;
        }
    };

}

namespace std {
    template<>
    struct hash<sw::Vec3> {
        typedef std::size_t result_type;

        result_type operator()(sw::Vec3 const &v) const {
            result_type const h1(std::hash<float>()(v.X));
            result_type const h2(std::hash<float>()(v.Y));
            return h1 ^ (h2 << 1);
        }
    };

    bool operator==(const sw::Vec3 &lhs, const sw::Vec3 &rhs) {
        return lhs.X == rhs.X && lhs.Y == rhs.Y && lhs.Z == rhs.Z;
    }
}

namespace sw {
    struct Boid {
        Vec3 Position;
        Vec3 Velocity;

        explicit Boid(Vec3 pos) : Position(pos) {

        }

        float DistanceTo(const Boid &other) const {
            return Position.DistanceTo(other.Position);
        }

    private:
        friend class Swarm;

        Vec3 Acceleration;
    };

    class Swarm {
    private:
        std::vector<Boid> *boids;
        std::unordered_map<Vec3, std::vector<Boid *>> voxelCache;
    public:
        float SeparationWeight = 1;
        float AlignmentWeight = 1;
        float CehesionWeight = 1;
        float PercepionRadius = 1;
        float BlindspotAngleDeg = 45;

        explicit Swarm(std::vector<Boid> *entities) : boids(entities) {
            // build kd-tree or similar
        }

        void Update(float delta) {
            if (PercepionRadius == 0) {
                PercepionRadius = 1;
            }
            buildVoxelCache();
            for (auto &b : *boids) {
                auto nearby = getNearbyBoids(b, 1);
                std::cout << "Boit " << b.Position.X << ", " << b.Position.Y << ", " << b.Position.Z << "\n";
                for (auto near : nearby) {
                    std::cout << "   --> " << near->Position.X << ", " << near->Position.Y << ", " << near->Position.Z << "\n";
                }
                // Separation: steer to avoid crowding local flockmates

                // Alignment: steer towards the average heading of local flockmates

                // Cohesion: steer to move toward the average position of local flockmates
            }

            for (auto &b : *boids) {
                b.Position += b.Acceleration;
                b.Acceleration = Vec3();
            }
        }

        std::vector<Boid*> getNearbyBoids(const Boid& b, float distance) {
            std::vector<Boid*> result;

            Vec3 voxelPos = getVoxelForBoid(b);
            voxelPos.X -= 1;
            voxelPos.Y -= 1;
            voxelPos.Z -= 1;
            for (int x = 0; x < 3; x++) {
                for (int y = 0; y < 3; y++) {
                    for (int z = 0; z < 3; z++) {
                        auto iter = voxelCache.find(voxelPos);
                        if (iter != voxelCache.end()) {
                            for (Boid* test : iter->second) {
                                if (b.DistanceTo(*test) <= distance) {
                                    result.push_back(test);
                                }
                            }
                        }
                        voxelPos.Z++;
                    }
                    voxelPos.Z -= 4;
                    voxelPos.Y++;
                }
                voxelPos.Y -= 4;
                voxelPos.X++;
            }
            return result;
        }

        void buildVoxelCache() {
            voxelCache.clear();
            float r = std::abs(PercepionRadius) / 2;
            for (auto& b : *boids) {
                voxelCache[getVoxelForBoid(b)].push_back(&b);
            }
        }

        Vec3 getVoxelForBoid(const Boid& b) {
            float r = std::abs(PercepionRadius);
            const Vec3& p = b.Position;
            Vec3 voxelPos;
            voxelPos.X = static_cast<int>(p.X / r);
            voxelPos.Y = static_cast<int>(p.Y / r);
            voxelPos.Z = static_cast<int>(p.Z / r);
            return voxelPos;
        }
    };
}
