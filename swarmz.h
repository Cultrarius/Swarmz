#pragma once

#include <vector>
#include <unordered_map>
#include <cmath>
#include <iostream>
#include <random>

# define PI2 6.28318530717958647692

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

        float Length() const {
            return std::sqrt(std::pow(X, 2) + std::pow(Y, 2) + std::pow(Z, 2));
        }

        float DotProduct(const Vec3& v) const {
            return X * v.X + Y * v.Y + Z * v.Z;
        }

        float AngleTo(const Vec3& v) const {
            float l1 = Length();
            float l2 = v.Length();
            if (l1 == 0 || l2 == 0) {
                return 0;
            }
            return std::acos(DotProduct(v) / (l1 * l2)) * 360 / PI2;
        }

        Vec3 Normalized() const {
            float length = Length();
            if (length == 0) {
                return Vec3();
            }
            return Vec3(X / length, Y / length, Z / length);
        }

        Vec3 Negative() const {
            return Vec3(-X, -Y, -Z);
        }

        Vec3 &operator+=(const Vec3 &other) {
            X += other.X;
            Y += other.Y;
            Z += other.Z;
            return *this;
        }

        Vec3 operator/(float scalar) const {
            return Vec3(X / scalar, Y / scalar, Z / scalar);
        }

        Vec3 operator*(float scalar) const {
            return Vec3(X * scalar, Y * scalar, Z * scalar);
        }

        Vec3 operator+(const Vec3 &other) const {
            return Vec3(X + other.X, Y + other.Y, Z + other.Z);
        }

        Vec3 operator-(const Vec3 &other) const {
            return Vec3(X - other.X, Y - other.Y, Z - other.Z);
        }

        Vec3 ClampLength(float length) const {
            float l = Length();
            if (l > length) {
                return Normalized() * length;
            }
            return *this;
        }

        static Vec3 GetRandomUniform(std::mt19937& engine) {
            std::uniform_real_distribution<float> thetaRange(0.0f, PI2);
            std::uniform_real_distribution<float> oneRange(0, 1);
            float theta = thetaRange(engine);
            float r = sqrt(oneRange(engine));
            float z = sqrt(1.0f - r * r) * (oneRange(engine) > 0.5f ? -1.0f : 1.0f);
            return Vec3(r * cos(theta), r * sin(theta), z);
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
        Vec3 Acceleration;

        explicit Boid(Vec3 pos, Vec3 vel) : Position(pos), Velocity(vel) {
        }
    };

    struct NearbyBoid {
        Boid *boid;
        Vec3 direction;
        float distance;
    };

    class Swarm {
    private:
        std::vector<Boid> *boids;
        std::unordered_map<Vec3, std::vector<Boid *>> voxelCache;
        std::mt19937 eng;

        void updateBoid(Boid& b)
        {
            Vec3 separationSum;
            Vec3 headingSum;
            Vec3 positionSum;
            auto nearby = getNearbyBoids(b);
            for (NearbyBoid& closeBoid : nearby) {
                if (closeBoid.distance == 0) {
                    separationSum += Vec3::GetRandomUniform(eng) * 1000;
                }
                else {
                    separationSum += closeBoid.direction.Negative() / closeBoid.distance;
                }

                headingSum += closeBoid.boid->Velocity;
                positionSum += closeBoid.boid->Position;
            }

            // Separation: steer to avoid crowding local flockmates
            Vec3 separation = nearby.size() > 0 ? separationSum / nearby.size() : separationSum;

            // Alignment: steer towards the average heading of local flockmates
            Vec3 alignment = nearby.size() > 0 ? headingSum / nearby.size() : headingSum;

            // Cohesion: steer to move toward the average position of local flockmates
            Vec3 avgPosition = nearby.size() > 0 ? positionSum / nearby.size() : b.Position;
            Vec3 cohesion = b.Position - avgPosition;

            // calculate boid acceleration
            Vec3 acceleration;
            acceleration += separation * SeparationWeight;
            acceleration += alignment * AlignmentWeight;
            acceleration += cohesion * CohesionWeight;
            b.Acceleration = acceleration.ClampLength(MaxAcceleration);
        }


    public:
        float SeparationWeight = 1;
        float AlignmentWeight = 1;
        float CohesionWeight = 1;
        float PerceptionRadius = 1;
        float BlindspotAngleDeg = 45;
        float MaxAcceleration = 0.1;
        float MaxVelocity = 2;
        Vec3 SteeringTarget;

        explicit Swarm(std::vector<Boid> *entities) : boids(entities) {
            std::random_device rd;
            eng = std::mt19937(rd());
        }

        void Update(float delta) {
            UpdateAcceleration();

            for (auto &b : *boids) {
                b.Velocity = (b.Velocity + b.Acceleration * delta).ClampLength(MaxVelocity);
                b.Position += b.Velocity * delta;
            }
        }

        void UpdateAcceleration() {
            if (PerceptionRadius == 0) {
                PerceptionRadius = 1;
            }
            buildVoxelCache();
            for (auto &b : *boids) {
                updateBoid(b);
            }
        }

        std::vector<NearbyBoid> getNearbyBoids(const Boid &b) const {
            std::vector<NearbyBoid> result;

            Vec3 voxelPos = getVoxelForBoid(b);
            voxelPos.X -= 1;
            voxelPos.Y -= 1;
            voxelPos.Z -= 1;
            for (int x = 0; x < 4; x++) {
                for (int y = 0; y < 4; y++) {
                    for (int z = 0; z < 4; z++) {
                        auto iter = voxelCache.find(voxelPos);
                        if (iter != voxelCache.end()) {
                            int added = 0;
                            for (Boid *test : iter->second) {
                                const Vec3& p1 = b.Position;
                                const Vec3& p2 = test->Position;
                                Vec3 vec(p2.X - p1.X, p2.Y - p1.Y, p2.Z - p1.Z);
                                float distance = vec.Length();
                                float blindAngle = b.Velocity.Negative().AngleTo(vec);
                                if ((&b) != test && distance <= PerceptionRadius && BlindspotAngleDeg <= blindAngle) {
                                    NearbyBoid nb;
                                    nb.boid = test;
                                    nb.distance = distance;
                                    nb.direction = vec;
                                    result.push_back(nb);
                                    added++;
                                    if (added >= 10) {
                                        break;
                                    }
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
            voxelCache.reserve(boids->size());
            for (auto &b : *boids) {
                voxelCache[getVoxelForBoid(b)].push_back(&b);
            }
        }

        Vec3 getVoxelForBoid(const Boid &b) const noexcept {
            float r = std::abs(PerceptionRadius);
            const Vec3 &p = b.Position;
            Vec3 voxelPos;
            voxelPos.X = static_cast<int>(p.X / r);
            voxelPos.Y = static_cast<int>(p.Y / r);
            voxelPos.Z = static_cast<int>(p.Z / r);
            return voxelPos;
        }
    };
}
