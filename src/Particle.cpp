#include "../include/Particle.h"
#include <cmath>
#include <thread>
#include <chrono>

Particle::Particle(double x, double y, double energy, double radius, double max_energy)
    : x(x), y(y), vx(0.0), vy(0.0), energy(energy), MAX_ENERGY(max_energy), PARTICLE_RADIUS(radius) {
}

Particle::~Particle() {
}

double Particle::getX() const {
    return x;
}

double Particle::getY() const {
    return y;
}

void Particle::setPosition(double newX, double newY) {
    x = newX;  
    y = newY;
}

double Particle::getVX() const {
    return vx;
}

double Particle::getVY() const {
    return vy;
}

void Particle::setVelocity(double newVX, double newVY) {
    std::lock_guard<std::mutex> lock(particleMutex);
    vx = newVX;
    vy = newVY;
}

double Particle::getEnergy() const {
    return energy;
}

double Particle::getMaxEnergy() const {
    return MAX_ENERGY;
}

void Particle::setEnergy(double newEnergy) {
    energy = std::min(newEnergy, MAX_ENERGY);
}

void Particle::addEnergy(double delta) {
    std::lock_guard<std::mutex> lock(particleMutex);
    energy = std::min(energy + delta, MAX_ENERGY);
}

void Particle::collide(Particle& other) {
    std::lock_guard<std::mutex> lock(particleMutex);
    
    double tempVX = vx;
    double tempVY = vy;
    
    vx = other.vx * 0.8;
    vy = other.vy * 0.8;
    
    other.vx = tempVX * 0.8;
    other.vy = tempVY * 0.8;
    
    energy *= 0.95;
    other.energy *= 0.95;
}

bool Particle::isColliding(const Particle& other) const {
    double dx = x - other.x;
    double dy = y - other.y;
    double distance = std::sqrt(dx*dx + dy*dy);
    
    return distance < (PARTICLE_RADIUS + other.PARTICLE_RADIUS);
}
