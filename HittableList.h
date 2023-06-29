#pragma once
#include "Hittable.h"
#include <memory>
#include<vector>
using std::shared_ptr;
using std::make_shared;
/// <summary>
/// 物体的列表，可以判断是否与物体中任意一个物体相交，
/// 记录最近的t和相交信息
/// </summary>

class hittableList :public hittable {
public:
    hittableList() {}
    hittableList(shared_ptr<hittable> object) { add(object); }

    void clear() { objects.clear(); }
    void add(shared_ptr<hittable> object) { objects.push_back(object); }

    virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const;
    virtual bool bounding_box(double t0, double t1, aabb& output_box) const;
    auto getObjects() {
        return this->objects;
    }

    std::vector<shared_ptr<hittable>> objects;
};
/// <summary>
/// 寻找最近的交点，记录相交信息
/// </summary>
/// <param name="r">ray</param>
/// <param name="t_min">tmin</param>
/// <param name="t_max">tmax</param>
/// <param name="rec">hit_record结构体</param>
/// <returns>是否有交点</returns>
bool hittableList::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    hit_record temp_rec;
    bool hit_anything = false;
    auto closest_so_far = t_max;

    for (const auto& object : objects) {
        if (object->hit(r, t_min, closest_so_far, temp_rec)) {
            hit_anything = true;
            closest_so_far = temp_rec.t;
            rec = temp_rec;
        }
    }

    return hit_anything;
}


bool hittableList::bounding_box(double t0, double t1, aabb& output_box) const {
    if (objects.empty()) return false;

    aabb temp_box;
    bool first_box = true;

    for (const auto& object : objects) {
        if (!object->bounding_box(t0, t1, temp_box)) return false;
        output_box = first_box ? temp_box : surrounding_box(output_box, temp_box);
        first_box = false;
    }

    return true;
}