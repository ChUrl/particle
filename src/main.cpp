#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

struct Vec2 {
    float x, y;

    Vec2 operator+(const Vec2 &vec) {
        return {x + vec.x, y + vec.y};
    }

    Vec2 operator-(const Vec2 &vec) {
        return {x - vec.x, y - vec.y};
    }

    Vec2 operator*(float scalar) {
        return {x * scalar, y * scalar};
    }

    bool operator==(const Vec2 &vec) {
        return x == vec.x && y == vec.y;
    }

    static float dot(const Vec2 &A, const Vec2 &B) {
        return A.x * B.x + A.y * B.y;
    }

    static float norm(const Vec2 &vec) {
        return std::sqrt(vec.x * vec.x + vec.y * vec.y);
    }
};

struct Particle {
    Vec2 loc;
    Vec2 vel;
    float mass;
    float drag;
    float elasticity;

    bool operator==(const Particle &p) {
        return loc == p.loc && vel == p.vel && mass == p.mass;
    }
};

struct Collision {
    Particle A;
    Particle B;

    float distance() {
        return Vec2::norm(B.loc - A.loc);
    }

    Vec2 normal() {
        return (B.loc - A.loc) * (1 / this->distance());
    }
};

void step(std::vector<Particle> &particle, float dt);
void print(const Particle &particle, float dt, float t);

const unsigned short WIDTH = 50;
const unsigned short HEIGHT = 50;

const float GRAVITY = 0.8;

int main(int argc, char *argv[]) {
    float t = 0.0;      // Verstrichene Zeit in ms
    float dt = 1.0;     // Schrittweite in ms
    float max_t = 60.0; // Berechnete Zeitspanne

    Particle particle = {25.0, 0.0, 0.0, 0.0, 0.95, 0.8, 1.0};
    Particle collider = {25.0, 25.0, 0.0, 0.0, 0.0, 0.0, std::numeric_limits<float>::max()};

    std::vector<Particle> particles;
    particles.push_back(particle);
    particles.push_back(collider);

    print(particles[0], dt, t);
    while (t < max_t) {
        step(particles, dt);
        t += dt;
        print(particles[0], dt, t);
    }

    std::cout << "Simulation Beendet nach " << t << " Sekunden!" << std::endl;
}

void step(std::vector<Particle> &particles, float dt) {
    std::vector<Collision> collisions;
    for (int i = 0; i < particles.size(); ++i) {
        // Apply Forces
        Vec2 force = {0.0, GRAVITY};
        particles[i].vel = particles[i].vel + force;

        // Move Particles
        particles[i].loc = particles[i].loc + particles[i].vel * dt;

        // Apply Drag
        particles[i].vel = particles[i].vel * particles[i].drag;

        // Bounds Collisions
        if (particles[i].loc.x >= WIDTH || particles[i].loc.x <= 0.0) { // Horizontal
            const float dist = particles[i].loc.x - WIDTH;
            particles[i].loc.x = particles[i].loc.x - dist;
            particles[i].vel.x = -particles[i].vel.x * particles[i].elasticity;
        } else if (particles[i].loc.y <= 0.0 || particles[i].loc.y >= HEIGHT) { // Vertical
            const float dist = particles[i].loc.y - HEIGHT;
            particles[i].loc.y = particles[i].loc.y - dist;
            particles[i].vel.y = -particles[i].vel.y * particles[i].elasticity;
        }

        // Detect Collisions
        for (int b = i; b < particles.size(); ++b) { // Duplikate vermeiden: Nur größere Indizes betrachten
            if (particles[i] == particles[b]) {
                continue;
            }

            if (Vec2::norm(particles[b].loc - particles[i].loc) <= 0.1) { // Threshold

                // if (collisions.size() == 0) {
                collisions.push_back({particles[i], particles[b]});
                // } else {
                // Duplicate Check
                // for (int c = 0; c < collisions.size(); ++c) {
                // if (collisions[c].A == particles[i] && collisions[c].B == particles[b] ||
                // collisions[c].B == particles[i] && collisions[c].A == particles[b]) {
                // } else {
                // collisions.push_back({particles[i], particles[b]});
                // }
                // }
                // }
            }
        }
    }

    // Solve Collisions for same Mass
    for (int i = 0; i < collisions.size(); ++i) {
        Vec2 normal = (collisions[i].B.loc - collisions[i].A.loc) * (1 / Vec2::norm(collisions[i].B.loc - collisions[i].A.loc));
        Vec2 tangent = {normal.y, normal.x};

        Vec2 Aproj_t = tangent * Vec2::dot(collisions[i].A.vel, tangent);
        Vec2 Amirror = collisions[i].A.vel + Aproj_t * 2;

        Vec2 Bproj_t = tangent * Vec2::dot(collisions[i].B.vel, tangent);
        Vec2 Bmirror = collisions[i].B.vel + Bproj_t * 2;

        // A.vel muss die Richtung von -Amirror und den Betrag von B.vel bekommen
        // B.vel muss die Richtung von -Bmirror und den Betrag von A.vel bekommen
    }
}

void print(const Particle &particle, float dt, float t) {
    std::cout << std::round(t * 100.0) / 100.0 << "s: ";
    std::cout << "Location = (" << particle.loc.x << ", " << particle.loc.y << "), ";
    std::cout << "Velocity = (" << particle.vel.x << ", " << particle.vel.y << ")" << std::endl;
}
