//
// Created by Göksu Güvendiren on 2019-05-14.
//

#include "Scene.hpp"


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

// Implementation of Path Tracing
Vector3f Scene::castRay(const Ray &ray, int depth) const
{
    // TO DO Implement Path Tracing Algorithm here
    Intersection intersec = Scene::intersect(ray);
    if(!intersec.happened) return Vector3f();

    if(intersec.m->hasEmission()) return intersec.m->getEmission();  //打到光源返回光源颜色

    Vector3f L_dir(0.0f), L_indir(0.0f);

    switch(intersec.m->getType())
    {
        case MICROFACET: {}
        case DIFFUSE:
        {
            Intersection lightinter;
            float pdf = 0.0f;
            sampleLight(lightinter, pdf);
            Vector3f obj2light = lightinter.coords - intersec.coords;
            Vector3f obj2lightDir = obj2light.normalized();
            float dist = dotProduct(obj2light, obj2light);

            Ray obj2lightRay(intersec.coords, obj2lightDir);
            Intersection t = Scene::intersect(obj2lightRay);
            //if(t.distance - obj2light.norm() > -EPSILON)
            if((t.coords - lightinter.coords).norm() < EPSILON)
            {
                L_dir = lightinter.emit * intersec.m->eval(ray.direction, obj2lightDir, intersec.normal)\
                        * dotProduct(obj2lightDir, intersec.normal) * dotProduct(-obj2lightDir, lightinter.normal) / dist / pdf;
            }

            if(get_random_float() > RussianRoulette) return L_dir;

            
            Vector3f obj2nextobjdir = intersec.m->sample(ray.direction, intersec.normal).normalized();
            Ray obj2nextobjRay(intersec.coords, obj2nextobjdir);
            Intersection nextobjInter = Scene::intersect(obj2nextobjRay);
            if(nextobjInter.happened && !nextobjInter.m->hasEmission())
            {
                pdf = intersec.m->pdf(ray.direction, obj2nextobjdir, intersec.normal);
                if(pdf >= EPSILON / 100.0)
                    L_indir = castRay(obj2nextobjRay, depth + 1) * intersec.m->eval(ray.direction, obj2nextobjdir, intersec.normal)\
                            * dotProduct(obj2nextobjdir, intersec.normal) / pdf / RussianRoulette;
            }
            break;
        }
        case MIRROR:
        {
            if (get_random_float() > RussianRoulette) {
                return L_dir;
            }
            Vector3f obj2nextobjdir = intersec.m->sample(ray.direction, intersec.normal).normalized();
            Ray obj2nextobjray(intersec.coords, obj2nextobjdir);
            Intersection nextObjInter = intersect(obj2nextobjray);
            if (nextObjInter.happened)
            {
                float pdf = intersec.m->pdf(ray.direction, obj2nextobjdir, intersec.normal);
                if (pdf > EPSILON / 100.0)
                {
                    L_indir = castRay(obj2nextobjray, depth + 1) 
                        * intersec.m->eval(ray.direction, obj2nextobjdir, intersec.normal) 
                        * dotProduct(obj2nextobjdir, intersec.normal)
                        / pdf / RussianRoulette;
                }
            }
            break;
        }
    } 
    return L_dir + L_indir;

}