//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"
#include "Vector.hpp"


void Scene::buildBVH() {
    printf(" - Generating BVH...\n\n");
    this->bvh = new BVHAccel(objects, 1, BVHAccel::SplitMethod::NAIVE);
}

Intersection Scene::intersect(const Ray &ray) const
{
    return this->bvh->Intersect(ray);
}

void Scene::sampleLight(Intersection &pos, float &pdf) const
{
    float emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
        }
    }
    float p = get_random_float() * emit_area_sum;
    emit_area_sum = 0;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        if (objects[k]->hasEmit()){
            emit_area_sum += objects[k]->getArea();
            if (p <= emit_area_sum){
                objects[k]->Sample(pos, pdf);
                break;
            }
        }
    }
}

bool Scene::trace(
        const Ray &ray,
        const std::vector<Object*> &objects,
        float &tNear, uint32_t &index, Object **hitObject)
{
    *hitObject = nullptr;
    for (uint32_t k = 0; k < objects.size(); ++k) {
        float tNearK = kInfinity;
        uint32_t indexK;
        Vector2f uvK;
        if (objects[k]->intersect(ray, tNearK, indexK) && tNearK < tNear) {
            *hitObject = objects[k];
            tNear = tNearK;
            index = indexK;
        }
    }


    return (*hitObject != nullptr);
}

Vector3f shade(const Intersection &p, const Vector3f &wo, const Scene *const scene){
    // Vector3f wo points to Interection p.

    if (p.m->hasEmission()) {
        return p.m->getEmission();
    }

    Vector3f N = p.normal;

    Vector3f L_dir;
    Intersection x_prime;
    float pdf_light;
    scene->sampleLight(x_prime, pdf_light);
    Vector3f p_to_x_prime_vector = x_prime.coords - p.coords;
    Vector3f ws = p_to_x_prime_vector.normalized();
    Ray p_to_x_prime_ray(p.coords, ws, p_to_x_prime_vector.norm());
    Intersection block_detection = scene->intersect(p_to_x_prime_ray);
    if (block_detection.distance > p_to_x_prime_ray.t - EPSILON){
        Vector3f emit = x_prime.emit;
        Vector3f NN = x_prime.normal;
        L_dir = emit * p.m->eval(wo, ws, N) * dotProduct(ws, N) * dotProduct(-ws, NN) / (p_to_x_prime_ray.t * p_to_x_prime_ray.t) / pdf_light;
    }

    Vector3f L_indir;
    if (get_random_float() < scene->RussianRoulette) {
        // The starting point of Vector3f wi is Interection p.
        // Vector3f wo only works here in this function.
        Vector3f wi = normalize(p.m->sample(wo, N));
        Ray p_to_q_ray(p.coords, wi);
        Intersection q = scene->intersect(p_to_q_ray);
        if (q.happened && !q.m->hasEmission()) {
            L_indir = shade(q, wi, scene) * p.m->eval(wo, wi, N) * dotProduct(wi, N)/ p.m->pdf(wo, wi, N) / scene->RussianRoulette;
        }
    }

    return L_dir + L_indir;
}

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection p = this->intersect(ray);
    if (p.happened) {
        return shade(p, ray.direction, this);
    }
    else {
        return Vector3f();
    }
}
