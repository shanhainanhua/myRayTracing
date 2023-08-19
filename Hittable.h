#pragma once
#include "Ray.h"
class material;
struct hit_record {
    vec3 p;
    vec3 normal;
    //光线会如何与表面交互是由具体的材质所决定的。hit_record在设计上就是为了把一堆要传的参数给打包在了一起。当光线射入一个表面(比如一个球体), hit_record中的材质指针会被球体的材质指针所赋值, 而球体的材质指针是在main()函数中构造时传入的。当color()函数获取到hit_record时, 他可以找到这个材质的指针, 然后由材质的函数来决定光线是否发生散射, 怎么散射。
    shared_ptr<material> mat_ptr;
    double t;
    //为了添加纹理需要存储击中的uv信息
    double u;
    double v;

    bool front_face; //是否正面，外部射入
    /// <summary>
    /// 判断光线是从外部射入还是内部射入，永远让法相与入射方向相反, 我们就不用去用点乘来判断射入面是内侧还是外侧了, 但相对的, 我们需要用一个变量储存射入面的信息
    /// </summary>
    /// <param name="r">ray</param>
    /// <param name="outward_normal">使得法向永远与入射方向相反</param>
    inline void set_face_normal(const ray& r, const vec3& outward_normal) {
        front_face = dot(r.direction(), outward_normal) < 0;
        normal = front_face ? outward_normal : -outward_normal;
    }


};
/// <summary>
/// 任何可能与光线相交的物体都继承这个基类
/// 加入一个区间tmin,tmax来判断相交是否有效
/// 计算的结果：点，法线，t都存在一个结构体内
/// </summary>
class hittable {
public:
    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const = 0;
    virtual bool bounding_box(double t0, double t1, aabb& output_box) const = 0;
};

class flip_face : public hittable {
public:
    flip_face(shared_ptr<hittable> p) : ptr(p) {}

    virtual bool hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
        if (!ptr->hit(r, t_min, t_max, rec))
            return false;

        rec.front_face = !rec.front_face;
        return true;
    }

    virtual bool bounding_box(double t0, double t1, aabb& output_box) const {
        return ptr->bounding_box(t0, t1, output_box);
    }

public:
    shared_ptr<hittable> ptr;
};