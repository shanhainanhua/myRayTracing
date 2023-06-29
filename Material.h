#pragma once
#include "Ray.h"
#include "Vec3.h"
#include "Hittable.h"
#include "Vec3.h"
class material {
public:
    virtual bool scatter(
        const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered
    ) const = 0;
};

class lambertian:public material{
public:
    lambertian(const vec3&a):albedo(a){}

    /// <summary>
    /// 判断是否发生散射
    /// </summary>
    /// <param name="r_in">ray入射光</param>
    /// <param name="rec">hit_record记录交点信息包括法向量，p，材质，法向量</param>
    /// <param name="attenuation"></param>
    /// <param name="scattered"></param>
    /// <returns></returns>
    virtual bool scatter(
        const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered
    ) const {
        //vec3 target = rec.p + rec.normal + vec3::random_unit_vector();
        //return 0.5 * ray_color(ray(rec.p, target - rec.p), sceneObjects, depth - 1); 
        vec3 scatter_direction = rec.normal + random_unit_vector();
        scattered = ray(rec.p, scatter_direction,r_in.time());
        attenuation = albedo; //光线的衰减率
        return true;
    }
private:
    vec3 albedo;
};

class metal :public material {
public:
    /// <summary>
    /// 初始化,fuzzy:给反射方向加入一点点随机性, 只要在算出反射向量后, 在其终点为球心的球内随机选取一个点作为最终的终点
    /// </summary>
    /// <param name="a">光线衰减率</param>
    /// <param name="f">模糊程度，fuzz为0不会模糊</param>
    metal(const vec3& a,double f) :albedo(a),fuzz(f<1?f:1) {}
    virtual bool scatter(
        const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered
    ) const {
        vec3 reflected = reflect(unit_vector(r_in.direction()), rec.normal);
        scattered = ray(rec.p, reflected+fuzz*random_in_unit_sphere());
        attenuation = albedo;
        return (dot(scattered.direction(), rec.normal) > 0);
    }
private:
    vec3 albedo;
    double fuzz;
};


/// <summary>
/// 现实世界中的玻璃, 发生折射的概率会随着入射角而改变——从一个很狭窄的角度去看玻璃窗, 它会变成一面镜子。数学上近似的等式, 
/// </summary>
/// <param name="cosine"></param>
/// <param name="ref_idx"></param>
/// <returns></returns>
double schlick(double cosine, double ref_idx) {
    auto r0 = (1 - ref_idx) / (1 + ref_idx);
    r0 = r0 * r0;
    return r0 + (1 - r0) * pow((1 - cosine), 5);
}

class dielectric : public material {
public:
    dielectric(double ri) : ref_idx(ri) {}

    virtual bool scatter(
        const ray& r_in, const hit_record& rec, vec3& attenuation, ray& scattered
    ) const {
        attenuation = vec3(1.0, 1.0, 1.0);
        double etai_over_etat = (rec.front_face) ? (1.0 / ref_idx) : (ref_idx);
        //当光线从高折射律介质射入低折射率介质时, 对于Snell方程可能没有实解，就不会发生折射, 所以就会出现许多小黑点，所以我们认为光线无法发生折射的时候, 他发生了反射
        vec3 unit_direction = unit_vector(r_in.direction());
        double cos_theta = ffmin(dot(-unit_direction, rec.normal), 1.0);
        double sin_theta = sqrt(1.0 - cos_theta * cos_theta);
        if (etai_over_etat * sin_theta > 1.0) {
            vec3 reflected = reflect(unit_direction, rec.normal);
            scattered = ray(rec.p, reflected);
            return true;
        }
        double reflect_prob = schlick(cos_theta, etai_over_etat);
        if (random_double() < reflect_prob)
        {
            vec3 reflected = reflect(unit_direction, rec.normal);
            scattered = ray(rec.p, reflected);
            return true;
        }
        vec3 refracted = refract(unit_direction, rec.normal, etai_over_etat);
        scattered = ray(rec.p, refracted);
        return true;
    }

public:
    double ref_idx;
};