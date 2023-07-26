#include <iostream>
#include <vector>

#include "CGL/vector2D.h"

#include "mass.h"
#include "rope.h"
#include "spring.h"

namespace CGL {

    Rope::Rope(Vector2D start, Vector2D end, int num_nodes, float node_mass, float k, vector<int> pinned_nodes)
    {
        // TODO (Part 1): Create a rope starting at `start`, ending at `end`, and containing `num_nodes` nodes.

        masses = std::vector<Mass *>();
        springs = std::vector<Spring *>();
        masses.push_back(new Mass(start, node_mass, false));
        for (int i = 1; i <= num_nodes - 1; ++i) {
            double t = (double) i / (num_nodes - 1);
            masses.push_back(new Mass(start * (1.0 - t) + end * t, node_mass, false));
            springs.push_back(new Spring(masses.at(i-1), masses.at(i), k));
        }
        for (auto &i : pinned_nodes) {
            masses[i]->pinned = true;
        }
}

    void Rope::simulateEuler(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 2): Use Hooke's law to calculate the force on a node
            Vector2D vector_m1_to_m2 = s->m2->position - s->m1->position;
            double norm_m1_to_m2 = vector_m1_to_m2.norm();
            Vector2D unit_m1_to_m2 = vector_m1_to_m2.unit();
            Vector2D m1_force_from_m2 = s->k * unit_m1_to_m2 * (norm_m1_to_m2 - s->rest_length);
            s->m1->forces += m1_force_from_m2;
            s->m2->forces -= m1_force_from_m2;
        }

        for (auto &m : masses)
        {
            float g = 9.8f;
            if (!m->pinned)
            {
                // TODO (Part 2): Add the force due to gravity, then compute the new velocity and position
                m->forces += m->mass * g * gravity;
                Vector2D acceleration = m->forces / m->mass;
                m->velocity += acceleration * delta_t;
                m->position += m->velocity * delta_t;
                // TODO (Part 2): Add global damping
            }

            // Reset all forces on each mass
            m->forces = Vector2D(0, 0);
        }
    }

    void Rope::simulateVerlet(float delta_t, Vector2D gravity)
    {
        for (auto &s : springs)
        {
            // TODO (Part 3): Simulate one timestep of the rope using explicit Verlet （solving constraints)
        }

        for (auto &m : masses)
        {
            if (!m->pinned)
            {
                Vector2D temp_position = m->position;
                // TODO (Part 3.1): Set the new position of the rope mass
                
                // TODO (Part 4): Add global Verlet damping
            }
        }
    }
}
