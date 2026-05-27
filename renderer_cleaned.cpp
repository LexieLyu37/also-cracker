#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>

struct Vector3 {
    double x, y, z;

    Vector3() : x(0), y(0), z(0) {}
    Vector3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}

    Vector3 operator+(const Vector3& other) const {
        return Vector3(x + other.x, y + other.y, z + other.z);
    }

    Vector3 operator-(const Vector3& other) const {
        return Vector3(x - other.x, y - other.y, z - other.z);
    }

    Vector3 operator*(double s) const {
        return Vector3(x * s, y * s, z * s);
    }

    Vector3 operator/(double s) const {
        return Vector3(x / s, y / s, z / s);
    }

    Vector3 operator*(const Vector3& other) const {
        return Vector3(x * other.x, y * other.y, z * other.z);
    }

    double dot(const Vector3& other) const {
        return x * other.x + y * other.y + z * other.z;
    }

    Vector3 cross(const Vector3& other) const {
        return Vector3(
            y * other.z - z * other.y,
            z * other.x - x * other.z,
            x * other.y - y * other.x
        );
    }

    double lengthSquared() const {
        return x * x + y * y + z * z;
    }

    double length() const {
        return std::sqrt(lengthSquared());
    }

    Vector3 normalized() const {
        double len = length();
        if (len < 1e-12) return *this;
        return *this / len;
    }
};

struct Material {
    Vector3 albedo;
    Vector3 emission;
};

struct Triangle {
    Vector3 v0, v1, v2;
    int materialIdx;
};

struct HitInfo {
    bool hit;
    double t;
    int triIdx;
    Vector3 point;
    Vector3 normal;
};

struct Camera {
    Vector3 pos;
    Vector3 lookAt;
    Vector3 up;
    double fov;
    int width;
    int height;

    Vector3 getRayDir(double u, double v) const {
        double aspect = (double)width / (double)height;
        double theta = fov * 3.14159265358979323846 / 180.0;
        double halfHeight = std::tan(theta * 0.5);
        double halfWidth = aspect * halfHeight;

        Vector3 w = (pos - lookAt).normalized();
        Vector3 uVec = up.cross(w).normalized();
        Vector3 vVec = w.cross(uVec);

        double screenX = (2.0 * u - 1.0) * halfWidth;
        double screenY = (1.0 - 2.0 * v) * halfHeight;

        return (uVec * screenX + vVec * screenY - w).normalized();
    }
};

enum class RenderMode {
    Flat,
    Normal,
    DirectLighting
};

// Ray-triangle intersection using the Moller-Trumbore algorithm
bool intersectTriangle(const Vector3& origin,
                       const Vector3& dir,
                       const Vector3& v0,
                       const Vector3& v1,
                       const Vector3& v2,
                       double& t) {
    const double EPS = 1e-8;

    Vector3 edge1 = v1 - v0;
    Vector3 edge2 = v2 - v0;
    Vector3 h = dir.cross(edge2);
    double a = edge1.dot(h);

    if (std::fabs(a) < EPS) return false;

    double f = 1.0 / a;
    Vector3 s = origin - v0;
    double u = f * s.dot(h);
    if (u < 0.0 || u > 1.0) return false;

    Vector3 q = s.cross(edge1);
    double v = f * dir.dot(q);
    if (v < 0.0 || u + v > 1.0) return false;

    t = f * edge2.dot(q);
    return t > EPS;
}

// Compute the geometric normal of a triangle
Vector3 triangleNormal(const Triangle& tri) {
    Vector3 e1 = tri.v1 - tri.v0;
    Vector3 e2 = tri.v2 - tri.v0;
    return e1.cross(e2).normalized();
}

// Find the closest triangle hit by a ray
HitInfo intersectScene(const Vector3& origin,
                       const Vector3& dir,
                       const std::vector<Triangle>& triangles) {
    HitInfo result;
    result.hit = false;
    result.t = std::numeric_limits<double>::max();
    result.triIdx = -1;

    for (size_t i = 0; i < triangles.size(); ++i) {
        double t;
        if (intersectTriangle(origin, dir,
                              triangles[i].v0,
                              triangles[i].v1,
                              triangles[i].v2,
                              t)) {
            if (t > 1e-6 && t < result.t) {
                result.hit = true;
                result.t = t;
                result.triIdx = (int)i;
            }
        }
    }

    if (result.hit) {
        const Triangle& tri = triangles[result.triIdx];
        result.point = origin + dir * result.t;
        result.normal = triangleNormal(tri);
    }

    return result;
}

// Check whether the path from a surface point to the light is blocked
bool occluded(const Vector3& from,
              const Vector3& to,
              const std::vector<Triangle>& triangles,
              int ignoreTriIdx) {
    Vector3 dir = to - from;
    double dist = dir.length();
    dir = dir / dist;

    for (size_t i = 0; i < triangles.size(); ++i) {
        if ((int)i == ignoreTriIdx) continue;

        double t;
        if (intersectTriangle(from, dir,
                              triangles[i].v0,
                              triangles[i].v1,
                              triangles[i].v2,
                              t)) {
            if (t > 1e-6 && t < dist - 1e-6) {
                return true;
            }
        }
    }

    return false;
}

// Return the base material colour without lighting
Vector3 shadeFlat(const HitInfo& hit,
                  const std::vector<Triangle>& triangles,
                  const std::vector<Material>& materials) {
    const Triangle& tri = triangles[hit.triIdx];
    const Material& mat = materials[tri.materialIdx];
    return mat.albedo;
}

// Visualise the surface normal as an RGB colour
Vector3 shadeNormal(const HitInfo& hit) {
    return (hit.normal + Vector3(1.0, 1.0, 1.0)) * 0.5;
}

// Basic direct lighting with a point light, shadow test, and distance attenuation
Vector3 shadeDirectLighting(const HitInfo& hit,
                            const std::vector<Triangle>& triangles,
                            const std::vector<Material>& materials) {
    const Triangle& tri = triangles[hit.triIdx];
    const Material& mat = materials[tri.materialIdx];

    Vector3 lightPos(2.0, 3.0, 2.0);
    Vector3 lightVector = lightPos - hit.point;
    double distanceSquared = lightVector.lengthSquared();
    Vector3 toLight = lightVector.normalized();

    Vector3 shadowOrigin = hit.point + hit.normal * 1e-4;
    if (occluded(shadowOrigin, lightPos, triangles, hit.triIdx)) {
        return mat.albedo * 0.1;
    }

    double diffuseStrength = std::max(0.0, hit.normal.dot(toLight));

    double attenuation = 1.0 / (1.0 + 0.2 * distanceSquared);

    Vector3 ambient(0.1, 0.1, 0.1);
    Vector3 diffuse = mat.albedo * (diffuseStrength * attenuation * 3.0);

    return ambient + diffuse;
}

// Dispatch to the appropriate shading function for the current render mode
Vector3 shadePixel(RenderMode mode,
                   const HitInfo& hit,
                   const std::vector<Triangle>& triangles,
                   const std::vector<Material>& materials) {
    if (mode == RenderMode::Flat) {
        return shadeFlat(hit, triangles, materials);
    } else if (mode == RenderMode::Normal) {
        return shadeNormal(hit);
    } else {
        return shadeDirectLighting(hit, triangles, materials);
    }
}

// Convert a colour channel from [0,1] to 8-bit
uint8_t toByte(double x) {
    x = std::max(0.0, std::min(1.0, x));
    return (uint8_t)(x * 255.0 + 0.5);
}

// Write the RGB image buffer to a binary PPM file
void writePPM(const std::string& filename,
              const std::vector<uint8_t>& image,
              int width,
              int height) {
    std::ofstream out(filename, std::ios::binary);
    if (!out) {
        std::cerr << "Failed to open " << filename << "\n";
        return;
    }

    out << "P6\n" << width << " " << height << "\n255\n";
    out.write(reinterpret_cast<const char*>(image.data()), image.size());
    out.close();
}

// Build a small test scene with three coloured triangles
void buildTestScene(std::vector<Material>& materials,
                    std::vector<Triangle>& triangles) {
    materials.clear();
    triangles.clear();

    int matRed = (int)materials.size();
    materials.push_back({Vector3(1.0, 0.2, 0.2), Vector3(0.0, 0.0, 0.0)});

    int matGreen = (int)materials.size();
    materials.push_back({Vector3(0.2, 1.0, 0.2), Vector3(0.0, 0.0, 0.0)});

    int matBlue = (int)materials.size();
    materials.push_back({Vector3(0.2, 0.2, 1.0), Vector3(0.0, 0.0, 0.0)});

    triangles.push_back({
        Vector3(-1.2, 0.0, -0.5),
        Vector3(1.2, 0.0, -0.5),
        Vector3(0.0, 1.5, -0.5),
        matRed
    });

    triangles.push_back({
        Vector3(-1.5, -0.5, -1.5),
        Vector3(-0.2, 1.0, -1.0),
        Vector3(-1.0, 0.2, 0.8),
        matGreen
    });

    triangles.push_back({
        Vector3(0.4, -0.3, -1.2),
        Vector3(1.5, 0.8, -0.8),
        Vector3(0.8, 0.3, 0.7),
        matBlue
    });
}

// Choose an output filename based on the current render mode
std::string outputFilename(RenderMode mode) {
    if (mode == RenderMode::Flat) {
        return "flat.ppm";
    } else if (mode == RenderMode::Normal) {
        return "normal.ppm";
    } else {
        return "direct.ppm";
    }
}

int main() {
    // Render settings
    const int width = 400;
    const int height = 400;
    RenderMode mode = RenderMode::DirectLighting;

    // Camera setup
    Camera cam;
    cam.pos = Vector3(0.0, 1.0, 3.5);
    cam.lookAt = Vector3(0.0, 0.8, 0.0);
    cam.up = Vector3(0.0, 1.0, 0.0);
    cam.fov = 60.0;
    cam.width = width;
    cam.height = height;

    // Scene data
    std::vector<Material> materials;
    std::vector<Triangle> triangles;
    buildTestScene(materials, triangles);

    // Output image buffer
    std::vector<uint8_t> image(width * height * 3, 0);

    // Main render loop
    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double u = (x + 0.5) / (double)width;
            double v = (y + 0.5) / (double)height;

            Vector3 dir = cam.getRayDir(u, v);
            HitInfo hit = intersectScene(cam.pos, dir, triangles);

            Vector3 color(0.0, 0.0, 0.0);

            if (hit.hit) {
                color = shadePixel(mode, hit, triangles, materials);
            }

            int idx = (y * width + x) * 3;
            image[idx + 0] = toByte(color.x);
            image[idx + 1] = toByte(color.y);
            image[idx + 2] = toByte(color.z);
        }
    }

    // Save result
    std::string filename = outputFilename(mode);
    writePPM(filename, image, width, height);
    std::cout << "Saved " << filename << "\n";

    return 0;
}