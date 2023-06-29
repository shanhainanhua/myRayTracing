#pragma once
#include "utils.h"
#include "PerLin.h"
class texture {
public:
    virtual vec3 value(double u, double v, const vec3& p) const = 0;
};

class constant_texture : public texture {
public:
    constant_texture() {}
    constant_texture(vec3 c) : color(c) {}

    virtual vec3 value(double u, double v, const vec3& p) const {
        return color;
    }

public:
    vec3 color;
};

class checker_texture : public texture {
public:
    checker_texture() {}
    checker_texture(shared_ptr<texture> t0, shared_ptr<texture> t1) : even(t0), odd(t1) {}

    virtual vec3 value(double u, double v, const vec3& p) const {
        auto sines = sin(10 * p.x()) * sin(10 * p.y()) * sin(10 * p.z());
        if (sines < 0)
            return odd->value(u, v, p);
        else
            return even->value(u, v, p);
    }

public:
    shared_ptr<texture> odd;
    shared_ptr<texture> even;
};

class noise_texture : public texture {
public:
    noise_texture() {}
    noise_texture(double sc) : scale(sc) {}
    //柏林插值的输出结果有可能是负数, 这些负数在伽马校正时经过开平方跟`sqrt()`会变成NaN。我们将输出结果映射到0与1之间。
    //扰动函数通常是间接使用的, 在程序生成纹理这方面的"hello world"是一个类似大理石的纹理。基本思路是让颜色与sine函数的值成比例, 并使用扰动函数去调整相位(平移了sin(x)中的x), 使得带状条纹起伏波荡。修正我们直接使用扰动turb或者噪声noise给颜色赋值的方法， 我们会得到一个类似大理石的纹理 
    virtual vec3 value(double u, double v, const vec3& p) const {
        return vec3(1, 1, 1) * 0.5 * (1 + sin(scale * p.z() + 10* noise.turb(p)));
    }

public:
    perlin noise;
    double scale;
};