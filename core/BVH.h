#pragma once
#include "HittableList.h"
#include "Ray.h"
#include <algorithm>


class bvh_node : public hittable {
public:
    bvh_node();

    bvh_node(hittableList& list, double time0, double time1)
        : bvh_node(list.objects, 0, list.objects.size(), time0, time1)
    {}

    bvh_node(
        std::vector<std::shared_ptr<hittable>>& objects,
        size_t start, size_t end, double time0, double time1);

    virtual bool hit(const ray& r, double tmin, double tmax, hit_record& rec) const;
    virtual bool bounding_box(double t0, double t1, aabb& output_box) const;

public:
    shared_ptr<hittable> left;
    shared_ptr<hittable> right;
    aabb box;
};

inline bool box_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b, int axis) {
    aabb box_a;
    aabb box_b;

    if (!a->bounding_box(0, 0, box_a) || !b->bounding_box(0, 0, box_b))
        std::cerr << "No bounding box in bvh_node constructor.\n";

    return box_a.min().e[axis] < box_b.min().e[axis];
}


inline bool box_x_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
    return box_compare(a, b, 0);
}

inline bool box_y_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
    return box_compare(a, b, 1);
}

inline bool box_z_compare(const shared_ptr<hittable> a, const shared_ptr<hittable> b) {
    return box_compare(a, b, 2);
}

bvh_node::bvh_node(std::vector<std::shared_ptr<hittable>>& objects,size_t start, size_t end, double time0, double time1) {
    //随机选择一个轴作为划分节点的依据
    int axis = random_int(0, 2);
    //根据选择的轴，选择对应的比较函数作为排序的依据
    auto comparator = (axis == 0)? box_x_compare:(axis == 1)?box_y_compare:box_z_compare;
    //计算物体的范围
    size_t object_span = end - start;
    //叶子节点是实际的hittable物体
    //如果只有一个物体 左右节点都指向该物体
    if (object_span == 1) {
        left = right = objects[start];
    }
    //如果有两个物体，根据比较函数的结果，选择将物体分别赋值给左右子节点
    else if (object_span == 2) {
        if (comparator(objects[start], objects[start + 1])) {
            left = objects[start];
            right = objects[start + 1];
        }
        else {
            left = objects[start + 1];
            right = objects[start];
        }
    }
    else { //排序找到中间位置 递归执行
        std::sort(objects.begin() + start, objects.begin() + end, comparator);

        auto mid = start + object_span / 2;
        left = make_shared<bvh_node>(objects, start, mid, time0, time1);
        right = make_shared<bvh_node>(objects, mid, end, time0, time1);
    }

    aabb box_left, box_right;

    if (!left->bounding_box(time0, time1, box_left)||!right->bounding_box(time0, time1, box_right)
        )
        std::cerr << "No bounding box in bvh_node constructor.\n";

    box = surrounding_box(box_left, box_right);
}



bool bvh_node::hit(const ray& r, double t_min, double t_max, hit_record& rec) const {
    if (!box.hit(r, t_min, t_max))
        return false;

    bool hit_left = left->hit(r, t_min, t_max, rec);
    bool hit_right = right->hit(r, t_min, hit_left ? rec.t : t_max, rec);

    return hit_left || hit_right;
}

bool bvh_node::bounding_box(double t0, double t1, aabb& output_box) const {
    output_box = box;
    return true;
}