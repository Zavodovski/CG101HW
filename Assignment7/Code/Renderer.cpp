//
// Created by goksu on 2/25/20.
//

#include <fstream>
#include "Scene.hpp"
#include "Renderer.hpp"
#include <thread>
#include <mutex>
#include <omp.h>

std::mutex mtx;
omp_lock_t lock1;

inline float deg2rad(const float& deg) { return deg * M_PI / 180.0; }

//const float EPSILON = 0.00001;
const float EPSILON = 0.001;

// The main render function. This where we iterate over all pixels in the image,
// generate primary rays and cast these rays into the scene. The content of the
// framebuffer is saved to a file.
void Renderer::Render(const Scene& scene)
{
    std::vector<Vector3f> framebuffer(scene.width * scene.height);

    float scale = tan(deg2rad(scene.fov * 0.5));
    float imageAspectRatio = scene.width / (float)scene.height;
    Vector3f eye_pos(278, 273, -800);
    int m = 0;

    //multi thread
    const int thread_num = 16;
    int thread_step = scene.height / thread_num;
    int process = 0;
    // change the spp value to change sample ammount
    int spp = 256;
    std::cout << "SPP: " << spp << "\n";
    // for (uint32_t j = 0; j < scene.height; ++j) {
    //     for (uint32_t i = 0; i < scene.width; ++i) {
    //         // generate primary ray direction
    //         float x = (2 * (i + 0.5) / (float)scene.width - 1) *
    //                   imageAspectRatio * scale;
    //         float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;

    //         Vector3f dir = normalize(Vector3f(-x, y, 1));
    //         for (int k = 0; k < spp; k++){
    //             framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;  
    //         }
    //         m++;
    //     }
    //     UpdateProgress(j / (float)scene.height);
    // }
    // UpdateProgress(1.f);

    //multithread
    std::thread th[thread_num];
 
    auto castRayMultiThread = [&](uint32_t y_min, uint32_t y_max){
        int width, height;
        width = height = sqrt(spp);
        float step = 1.0f / width;
        for (uint32_t j = y_min; j < y_max; j++) {
            int m = j * scene.width;
            for (uint32_t i = 0; i < scene.width; i++) {
                //float x = (2 * (i + 0.5) / (float)scene.width - 1) * \
                    imageAspectRatio * scale;
                //float y = (1 - 2 * (j + 0.5) / (float)scene.height) * scale;
                //Vector3f dir = normalize(Vector3f(-x, y, 1));
                for (int k = 0; k < spp; k++) {
                    //在对一个像素进行spp次采样的同时将这个像素分为spp个小像素, 做MSAA抗锯齿
                    float x = (2 * (i + step / 2 + step * (k % width)) / (float)scene.width - 1) * \
                        imageAspectRatio * scale;
                    float y = (1 - 2 * (j + step / 2 + step * (k / height)) / (float)scene.height) * scale;
                    Vector3f dir = normalize(Vector3f(-x, y, 1));
                    framebuffer[m] += scene.castRay(Ray(eye_pos, dir), 0) / spp;
                }
                m++;
            }
            //mtx.lock();
            omp_set_lock(&lock1);
            process++;
            UpdateProgress(1.0 * process / scene.height);
            //mtx.unlock();
            omp_unset_lock(&lock1);
        }
    };
 
    //分行进行路径追踪
    #pragma omp parallel for
        for (int i = 0; i < thread_num; i++) {//从第0行出发，一共有0~by-1行
            th[i] = std::thread(castRayMultiThread, i * thread_step, (i + 1) * thread_step);
        }
    //每个线程执行join
    for (int i = 0; i < thread_num; i++) {
        th[i].join();
    }
    UpdateProgress(1.f);


    // save framebuffer to file
    FILE* fp = fopen("binary.ppm", "wb");
    (void)fprintf(fp, "P6\n%d %d\n255\n", scene.width, scene.height);
    for (auto i = 0; i < scene.height * scene.width; ++i) {
        static unsigned char color[3];
        color[0] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].x), 0.6f));
        color[1] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].y), 0.6f));
        color[2] = (unsigned char)(255 * std::pow(clamp(0, 1, framebuffer[i].z), 0.6f));
        fwrite(color, 1, 3, fp);
    }
    fclose(fp);    
}
