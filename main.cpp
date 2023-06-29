#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include <stdio.h>
#include<iostream>
#define GL_SILENCE_DEPRECATION
#include <opencv2/opencv.hpp>
#include <GLFW/glfw3.h> 
#include "HittableList.h"
#include "Sphere.h"
#include "utils.h"
#include "Camera.h"
#include "Material.h"
#include "Ray.h"
#include "Vec3.h"
static void glfw_error_callback(int error, const char* description)
{
    fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

/// <summary>
/// 计算相交的颜色 需要进行伽马校正
/// </summary>
/// <param name="r">ray</param>
/// <param name="sceneObjects">hittableList</param>
/// <param name="depth">限制递归深度</param>
/// <returns>颜色 vec3 </returns>
vec3 ray_color(const ray& r,const hittableList& sceneObjects,int depth) {
    hit_record rec;
    //noraml是球体表面的法向量，球心为p-n的那个球在表面的内部，球心为p+n的球
    //在表面的外部，这个球，并从中随机选择一点，得到target target-p就是反射光线的方向
    if (depth <= 0) return vec3(0, 0, 0);
    //有些物体反射的光线会在t=0时再次击中自己。然而由于精度问题, 这个值可能是t=-0.000001或者是t=0.0000000001或者任意接近0的浮点数。所以我们要忽略掉0附近的一部分范围, 防止物体发出的光线再次与自己相交
    if (sceneObjects.hit(r, 0.001, infinity, rec)) {
        ray scattered;
        vec3 attenuation;
        if (rec.mat_ptr->scatter(r, rec, attenuation, scattered))
            return attenuation * ray_color(scattered, sceneObjects, depth - 1);
        return vec3(0, 0, 0);
    }
    //没交点就显示渐变色
    vec3 unit_direction = unit_vector(r.direction());
    auto t = 0.5 * (unit_direction.y() + 1.0);
    return (1.0 - t) * vec3(1.0, 1.0, 1.0) + t * vec3(0.5, 0.7, 1.0);
}

hittableList random_scene() {
    hittableList world;

    world.add(make_shared<sphere>(
        vec3(0, -1000, 0), 1000, make_shared<lambertian>(vec3(0.5, 0.5, 0.5))));

    int i = 1;
    for (int a = -10; a < 10; a++) {
        for (int b = -10; b < 10; b++) {
            auto choose_mat = random_double();
            vec3 center(a + 0.9 * random_double(), 0.2, b + 0.9 * random_double());
            if ((center - vec3(4, 0.2, 0)).length() > 0.9) {
                if (choose_mat < 0.8) {
                    // diffuse
                    auto albedo = vec3::random() * vec3::random();
                    //world.add(
                    //    make_shared<sphere>(center, 0.2, make_shared<lambertian>(albedo)));
                    world.add(make_shared<moving_sphere>(center, center + vec3(0, random_double(0, .5), 0), 0.0, 1.0, 0.2, make_shared<lambertian>(albedo)));
                }
                else if (choose_mat < 0.95) {
                    // metal
                    auto albedo = vec3::random(.5, 1);
                    auto fuzz = random_double(0, .5);
                    world.add(
                        make_shared<sphere>(center, 0.2, make_shared<metal>(albedo, fuzz)));
                }
                else {
                    // glass
                    world.add(make_shared<sphere>(center, 0.2, make_shared<dielectric>(1.5)));
                }
            }
        }
    }

    world.add(make_shared<sphere>(vec3(0, 1, 0), 1.0, make_shared<dielectric>(1.5)));

    world.add(
        make_shared<sphere>(vec3(-4, 1, 0), 1.0, make_shared<lambertian>(vec3(0.4, 0.2, 0.1))));

    world.add(
        make_shared<sphere>(vec3(4, 1, 0), 1.0, make_shared<metal>(vec3(0.7, 0.6, 0.5), 0.0)));

    return world;
}


// Main code
int main(int, char**)
{
    const int image_width =200;
    const int image_height =100;
    const int samples_per_pixel = 100;
    const int max_depth = 50;
    const auto aspect_ratio = double(image_width) / image_height;
    // 创建一个空白的图像
    cv::Mat image(image_height, image_width, CV_8UC3, cv::Scalar(0, 0, 0));

    //物体 如果将球的半径设为负值, 形状看上去并没什么变化, 但是法相全都翻转到内部去了。所以就可以用这个特性来做出一个通透的玻璃球:【把一个小球套在大球里, 光线发生两次折射, 于是负负得正, 上下不会颠倒】
    hittableList sceneObjects;
    sceneObjects.add(make_shared<sphere>(vec3(0, 0, -1), 0.5, make_shared<lambertian>(vec3(0.1, 0.2, 0.5))));
    sceneObjects.add(make_shared<sphere>(
        vec3(0, -100.5, -1), 100, make_shared<lambertian>(vec3(0.8, 0.8, 0.0))));
    sceneObjects.add(make_shared<sphere>(vec3(1, 0, -1), 0.5, make_shared<metal>(vec3(0.8, 0.6, 0.2), 0.3)));
    sceneObjects.add(make_shared<sphere>(vec3(-1, 0, -1), 0.5, make_shared<dielectric>(1.5)));
    sceneObjects.add(make_shared<sphere>(vec3(-1, 0, -1), -0.45, make_shared<dielectric>(1.5)));
    //相机
    vec3 lookfrom(13, 2, 3);
    vec3 lookat(0, 0, 0);
    vec3 vup(0, 1, 0);
    auto dist_to_focus = 10.0;
    auto aperture = 0.0;
    camera camera(lookfrom, lookat, vup, 20, aspect_ratio, aperture, dist_to_focus,0.0,1.0);
    // 生成图像像素值
    auto world = random_scene();
    for (int j = image_height - 1; j >= 0; --j) {
        std::cerr << "\rScanlines remaining: " << j << ' ' << std::flush;
        for (int i = 0; i < image_width; ++i) {
            vec3 color(0, 0, 0);
            for (int s = 0; s < samples_per_pixel; ++s) {
                auto u = double(i + random_double()) / image_width;
                auto v = double(j + random_double()) / image_height;
                ray r = camera.get_ray(u, v);
                color += ray_color(r, world,max_depth);
            }
            color.write_color(image,j,i,samples_per_pixel); // 将像素值写入到图像中
        }
    }

    // 显示图像
    cv::flip(image, image, 0);
    cv::imshow("Image", image);
    cv::imwrite("image.jpg", image);
    cv::waitKey(0); // 等待按键事件

    return 0;
}
