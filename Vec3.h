#pragma once
#include <iostream>
#include "utils.h"
class vec3 {
public:
    vec3() : e{ 0,0,0 } {}
    vec3(double e0, double e1, double e2) : e{ e0, e1, e2 } {}

    double x() const { return e[0]; }
    double y() const { return e[1]; }
    double z() const { return e[2]; }

    vec3 operator-() const { return vec3(-e[0], -e[1], -e[2]); }
    double operator[](int i) const { return e[i]; }
    double& operator[](int i) { return e[i]; }

    vec3& operator+=(const vec3& v) {
        e[0] += v.e[0];
        e[1] += v.e[1];
        e[2] += v.e[2];
        return *this;
    }

    vec3& operator*=(const double t) {
        e[0] *= t;
        e[1] *= t;
        e[2] *= t;
        return *this;
    }

    vec3& operator/=(const double t) {
        return *this *= 1 / t;
    }

    double length() const {
        return sqrt(length_squared());
    }

    double length_squared() const {
        return e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    }
    /// <summary>
    /// һ�����ؿ��Է��������ߣ�Ȼ�����ȡƽ��
    /// �ۼ���ɫ�����Բ��������
    /// </summary>
    /// <param name="image"></param>
    /// <param name="i">x</param>
    /// <param name="j">y</param>
    /// <param name="samples_per_pixel">��������</param>
    void write_color(cv::Mat& image, int j, int i,int samples_per_pixel) const {
        auto scale = 1.0 / samples_per_pixel;
        //����gamma=2������٤���ע��Ҫ��Ϊ1/2���ݣ��൱�ڿ�����
        auto r = sqrt(scale * e[0]);
        auto g = sqrt(scale * e[1]);
        auto b = sqrt(scale * e[2]);
        r = static_cast<int>(256 * clamp(r, 0.0, 0.999));
        g = static_cast<int>(256 * clamp(g, 0.0, 0.999));
        b = static_cast<int>(256 * clamp(b, 0.0, 0.999));
        image.at<cv::Vec3b>(j, i) = cv::Vec3b(b,g,r); // ��������ֵ��ע�� BGR ˳��
    }
    inline static vec3 random() {
        return vec3(random_double(), random_double(), random_double());
    }

    inline static vec3 random(double min, double max) {
        return vec3(random_double(min, max), random_double(min, max), random_double(min, max));
    }

public:
    double e[3];
};




inline std::ostream& operator<<(std::ostream& out, const vec3& v) {
    return out << v.e[0] << ' ' << v.e[1] << ' ' << v.e[2];
}

inline vec3 operator+(const vec3& u, const vec3& v) {
    return vec3(u.e[0] + v.e[0], u.e[1] + v.e[1], u.e[2] + v.e[2]);
}

inline vec3 operator-(const vec3& u, const vec3& v) {
    return vec3(u.e[0] - v.e[0], u.e[1] - v.e[1], u.e[2] - v.e[2]);
}

inline vec3 operator*(const vec3& u, const vec3& v) {
    return vec3(u.e[0] * v.e[0], u.e[1] * v.e[1], u.e[2] * v.e[2]);
}

inline vec3 operator*(double t, const vec3& v) {
    return vec3(t * v.e[0], t * v.e[1], t * v.e[2]);
}

inline vec3 operator*(const vec3& v, double t) {
    return t * v;
}

inline vec3 operator/(vec3 v, double t) {
    return (1 / t) * v;
}

inline double dot(const vec3& u, const vec3& v) {
    return u.e[0] * v.e[0]
        + u.e[1] * v.e[1]
        + u.e[2] * v.e[2];
}

inline vec3 cross(const vec3& u, const vec3& v) {
    return vec3(u.e[1] * v.e[2] - u.e[2] * v.e[1],
        u.e[2] * v.e[0] - u.e[0] * v.e[2],
        u.e[0] * v.e[1] - u.e[1] * v.e[0]);
}

inline vec3 unit_vector(vec3 v) {
    return v / v.length();
}

inline vec3 reflect(const vec3& v, const vec3& n) {
    return v - 2 * dot(v, n) * n;
}

/// <summary>
/// ���������ڵ�����㡣���ǻ������򵥵�����:�񶨷�(rejection method)������, ��һ��xyzȡֵ��ΧΪ-1��+1�ĵ�λ��������ѡȡһ�������, �����������������������ֱ���õ�������
/// </summary>
/// <returns></returns>
vec3 random_in_unit_sphere() {
    while (true) {
        auto p = vec3::random(-1, 1);
        if (p.length_squared() >= 1) continue;
        return p;
    }
}
/// <summary>
/// ֱ�Ӵ�����㿪ʼѡȡһ������ķ���, Ȼ�����ж��Ƿ��ڷ��������ڵ��Ǹ�������ʹ��lambertian������ģ��ǰ, ���ڵĹ���׷�������д󲿷�ʹ�õĶ���������������־���ֱ�ӵõ���������p+n
/// </summary>
/// <param name="normal"></param>
/// <returns></returns>

vec3 random_in_hemisphere(const vec3& normal) {
    vec3 in_unit_sphere = random_in_unit_sphere();
    if (dot(in_unit_sphere, normal) > 0.0) // In the same hemisphere as the normal
        return in_unit_sphere;
    else
        return -in_unit_sphere;
}

/// <summary>
/// ������lambertianɢ���Ĺ��߾��뷨��ȽϽ��ĸ��ʻ����, ���Ƿֲ��ɻ���Ӿ��⡣������Ϊ����ѡȡ���ǵ�λ�����ϵĵ㡣���ǿ���ͨ���ڵ�λ����ѡȡһ�������, Ȼ���䵥λ������øõ�
/// </summary>
/// <returns></returns>
vec3 random_unit_vector() {
    auto a = random_double(0, 2 * pi);
    auto z = random_double(-1, 1);
    auto r = sqrt(1 - z * z);
    return vec3(r * cos(a), r * sin(a), z);
}

/// <summary>
/// ��һ����λСԲ�̷�������
/// </summary>
/// <returns></returns>
vec3 random_in_unit_disk() {
    while (true) {
        auto p = vec3(random_double(-1, 1), random_double(-1, 1), 0);
        if (p.length_squared() >= 1) continue;
        return p;
    }
}


/// <summary>
/// ������������ ���Բ��Ϊ��ֱ��ƽ����������֮��
/// </summary>
/// <param name="uv"></param>
/// <param name="n"></param>
/// <param name="etai_over_etat"></param>
/// <returns></returns>
vec3 refract(const vec3& uv, const vec3& n, double etai_over_etat) {
    auto cos_theta = dot(-uv, n);
    vec3 r_out_parallel = etai_over_etat * (uv + cos_theta * n);
    vec3 r_out_perp = -sqrt(1.0 - r_out_parallel.length_squared()) * n;
    return r_out_parallel + r_out_perp;
}

