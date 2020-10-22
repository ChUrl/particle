#include <cmath>
#include <iostream>
#include <limits>
#include <vector>

#include <SFML/Graphics.hpp>

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

    void scale_to(float magnitude) {
        x = x * (magnitude / Vec2::norm(*this));
        y = y * (magnitude / Vec2::norm(*this));
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
    float radius;
    float mass;
    float drag;
    float elasticity;

    bool operator==(const Particle &p) {
        return loc == p.loc && vel == p.vel && mass == p.mass && radius == p.radius && drag == p.drag &&
               elasticity == p.elasticity;
    }
};

struct Collision {
    Particle *A;
    Particle *B;

    float distance() {
        return Vec2::norm(B->loc - A->loc);
    }

    Vec2 normal() {
        return (B->loc - A->loc) * (1 / this->distance());
    }
};

void step(std::vector<Particle> &particle, float dt);
void print(const Particle &particle, float dt, float t);

const unsigned short WIDTH = 500;
const unsigned short HEIGHT = 500;

const float GRAVITY = 1.0;

int main(int argc, char *argv[]) {
    const unsigned short FPS = 60;
    sf::ContextSettings settings;
    settings.antialiasingLevel = 8;

    sf::RenderWindow window(sf::VideoMode(WIDTH, HEIGHT), "Ball", sf::Style::Close, settings);
    window.setFramerateLimit(FPS); // Limit FPS

    float t = 0.0;        // Verstrichene Zeit in ms
    float dt = 1.0 / FPS; // Schrittweite in ms

    // lx, ly, vx, vy, radius, mass, drag, elasticity
    Particle particle = {{220.0, 50.0}, {0.0, 0.0}, 25.0, 1.0, 0.999, 1.0};
    Particle collider = {{200.0, 250.0}, {0.0, 0.0}, 25.0, std::numeric_limits<float>::max(), 1.0, 1.0};

    sf::CircleShape partcircle = sf::CircleShape(particle.radius);
    sf::CircleShape collcircle = sf::CircleShape(collider.radius);

    partcircle.setFillColor(sf::Color::Green);
    collcircle.setFillColor(sf::Color::Red);

    std::vector<Particle> particles;
    particles.push_back(particle);
    particles.push_back(collider);

    print(particles[0], dt, t);

    while (window.isOpen()) {
        sf::Event event;

        while (window.pollEvent(event)) {
            if (event.type == sf::Event::Closed) {
                window.close();
            }
        }

        // Update
        step(particles, dt);
        t += dt;
        print(particles[0], dt, t);
        partcircle.setPosition(particles[0].loc.x + particles[0].radius, particles[0].loc.y - particles[0].radius);
        collcircle.setPosition(particles[1].loc.x + particles[1].radius, particles[1].loc.y - particles[1].radius);

        // Render
        window.clear();
        window.draw(partcircle);
        window.draw(collcircle);
        window.display();
    }

    std::cout << "Simulation Beendet nach " << t << " Sekunden!" << std::endl;
}

void step(std::vector<Particle> &particles, float dt) {
    std::vector<Collision> collisions;
    for (int i = 0; i < particles.size(); ++i) {
        // Apply Drag
        particles[i].vel = particles[i].vel * particles[i].drag;

        // Apply Forces
        Vec2 force = {0.0, GRAVITY};
        particles[i].vel = particles[i].vel + force;

        // Move Particles
        particles[i].loc = particles[i].loc + particles[i].vel * dt;

        // Bounds Collisions
        if (particles[i].loc.x + particles[i].radius >= WIDTH) { // Horizontal Right
            const float dist = std::abs(particles[i].loc.x + particles[i].radius - WIDTH);
            particles[i].loc.x = particles[i].loc.x - dist;
            particles[i].vel.x = -particles[i].vel.x * particles[i].elasticity;
        } else if (particles[i].loc.x - particles[i].radius <= 0.0) { // Left
            const float dist = std::abs(particles[i].loc.x - particles[i].radius);
            particles[i].loc.x = particles[i].loc.x + dist;
            particles[i].vel.x = -particles[i].vel.x * particles[i].elasticity;
        }
        if (particles[i].loc.y - particles[i].radius <= 0.0) { // Vertical Top
            const float dist = std::abs(particles[i].loc.y - particles[i].radius);
            particles[i].loc.y = particles[i].loc.y + dist;
            particles[i].vel.y = -particles[i].vel.y * particles[i].elasticity;
        } else if (particles[i].loc.y + particles[i].radius >= HEIGHT) { // Bottom
            const float dist = std::abs(particles[i].loc.y + particles[i].radius - HEIGHT);
            particles[i].loc.y = particles[i].loc.y - dist;
            particles[i].vel.y = -particles[i].vel.y * particles[i].elasticity;
        }

        // Detect Collisions
        for (int b = i; b < particles.size(); ++b) { // Duplikate vermeiden: Nur größere Indizes betrachten
            if (particles[i] == particles[b]) {
                continue;
            }

            if (Vec2::norm(particles[b].loc - particles[i].loc) < particles[b].radius + particles[i].radius - 1.0) {
                collisions.push_back({&particles[i], &particles[b]});
            }
        }
    }

    // Solve Collisions for same Mass
    for (int i = 0; i < collisions.size(); ++i) {
        Vec2 normal = (collisions[i].B->loc - collisions[i].A->loc);
        normal.scale_to(1);
        Vec2 tangent = {normal.y, normal.x};

        Vec2 Aproj_n = normal * Vec2::dot(collisions[i].A->vel, normal);
        Vec2 Amirror = collisions[i].A->vel + Aproj_n * 2;

        Vec2 Bproj_n = normal * Vec2::dot(collisions[i].B->vel, normal);
        Vec2 Bmirror = collisions[i].B->vel + Bproj_n * 2;

        Amirror.scale_to(Vec2::norm(collisions[i].B->vel));
        collisions[i].A->vel = Amirror * -1;

        Bmirror.scale_to(Vec2::norm(collisions[i].A->vel));
        collisions[i].B->vel = Bmirror * -1;

        collisions[i].A->loc = collisions[i].A->loc - normal * (1.0 / 2.0);
        collisions[i].B->loc = collisions[i].B->loc + normal * (1.0 / 2.0);

        std::cout << "Kollision!" << std::endl;
    }
}

void print(const Particle &particle, float dt, float t) {
    std::cout << std::round(t * 100.0) / 100.0 << "s: ";
    std::cout << "Location = (" << particle.loc.x << ", " << particle.loc.y << "), ";
    std::cout << "Velocity = (" << particle.vel.x << ", " << particle.vel.y << ")" << std::endl;
}
