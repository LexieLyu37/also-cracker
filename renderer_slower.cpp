#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <limits>
#include <vector>
#include <algorithm>
#include <string>
#include <random>

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

    Vector3& operator+=(const Vector3& other) {
        x += other.x;
        y += other.y;
        z += other.z;
        return *this;
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

struct LightTri {
    int triIdx;
    Vector3 emission;
    double area;
    Vector3 normal;
};

enum class RenderMode {
    Flat,
    Normal,
    DirectLightingNEE
};

const double SHADOW_BIAS = 1e-4;
const double AMBIENT_STRENGTH = 0.08;
const int LIGHT_SAMPLES = 8;

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

Vector3 triangleNormal(const Triangle& tri) {
    Vector3 e1 = tri.v1 - tri.v0;
    Vector3 e2 = tri.v2 - tri.v0;
    return e1.cross(e2).normalized();
}

double triangleArea(const Triangle& tri) {
    Vector3 e1 = tri.v1 - tri.v0;
    Vector3 e2 = tri.v2 - tri.v0;
    return 0.5 * e1.cross(e2).length();
}

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

Vector3 randomPointOnTriangle(const Triangle& tri, double u1, double u2) {
    double su1 = std::sqrt(u1);
    double a = 1.0 - su1;
    double b = su1 * (1.0 - u2);
    double c = su1 * u2;
    return tri.v0 * a + tri.v1 * b + tri.v2 * c;
}

std::vector<LightTri> collectLightTriangles(const std::vector<Triangle>& triangles,
                                            const std::vector<Material>& materials) {
    std::vector<LightTri> lightTris;

    for (size_t i = 0; i < triangles.size(); ++i) {
        const Triangle& tri = triangles[i];
        const Material& mat = materials[tri.materialIdx];

        if (mat.emission.x > 0.0 || mat.emission.y > 0.0 || mat.emission.z > 0.0) {
            LightTri lightTri;
            lightTri.triIdx = (int)i;
            lightTri.emission = mat.emission;
            lightTri.area = triangleArea(tri);
            lightTri.normal = triangleNormal(tri);
            lightTris.push_back(lightTri);
        }
    }

    return lightTris;
}

Vector3 shadeFlat(const HitInfo& hit,
                  const std::vector<Triangle>& triangles,
                  const std::vector<Material>& materials) {
    const Triangle& tri = triangles[hit.triIdx];
    const Material& mat = materials[tri.materialIdx];
    return mat.albedo;
}

Vector3 shadeNormal(const HitInfo& hit) {
    return (hit.normal + Vector3(1.0, 1.0, 1.0)) * 0.5;
}

Vector3 shadeDirectLightingNEE(const HitInfo& hit,
                               const std::vector<Triangle>& triangles,
                               const std::vector<Material>& materials,
                               const std::vector<LightTri>& lightTris,
                               std::mt19937& rng,
                               std::uniform_real_distribution<double>& dist) {
    const Triangle& hitTri = triangles[hit.triIdx];
    const Material& hitMat = materials[hitTri.materialIdx];

    if (hitMat.emission.x > 0.0 || hitMat.emission.y > 0.0 || hitMat.emission.z > 0.0) {
        return hitMat.emission;
    }

    Vector3 ambient = hitMat.albedo * AMBIENT_STRENGTH;
    Vector3 totalDirect(0.0, 0.0, 0.0);

    for (const LightTri& lightTri : lightTris) {
        const Triangle& lightGeom = triangles[lightTri.triIdx];
        Vector3 lightAccum(0.0, 0.0, 0.0);

        for (int s = 0; s < LIGHT_SAMPLES; ++s) {
            double u1 = dist(rng);
            double u2 = dist(rng);

            Vector3 lightPoint = randomPointOnTriangle(lightGeom, u1, u2);
            Vector3 toLightVec = lightPoint - hit.point;
            double dist2 = toLightVec.lengthSquared();
            double distLen = std::sqrt(dist2);
            Vector3 toLight = toLightVec / distLen;

            double cosHit = std::max(0.0, hit.normal.dot(toLight));
            if (cosHit <= 0.0) continue;

            Vector3 toSurface = toLight * -1.0;
            double cosLight = std::max(0.0, lightTri.normal.dot(toSurface));
            if (cosLight <= 0.0) continue;

            Vector3 shadowOrigin = hit.point + hit.normal * SHADOW_BIAS;
            if (occluded(shadowOrigin, lightPoint, triangles, lightTri.triIdx)) {
                continue;
            }

            double G = (cosHit * cosLight) / dist2;
            Vector3 brdf = hitMat.albedo / 3.14159265358979323846;
            Vector3 contribution = brdf * lightTri.emission * (G * lightTri.area);

            lightAccum += contribution;
        }

        totalDirect += lightAccum / (double)LIGHT_SAMPLES;
    }

    return ambient + totalDirect;
}

Vector3 shadePixel(RenderMode mode,
                   const HitInfo& hit,
                   const std::vector<Triangle>& triangles,
                   const std::vector<Material>& materials,
                   const std::vector<LightTri>& lightTris,
                   std::mt19937& rng,
                   std::uniform_real_distribution<double>& dist) {
    if (mode == RenderMode::Flat) {
        return shadeFlat(hit, triangles, materials);
    } else if (mode == RenderMode::Normal) {
        return shadeNormal(hit);
    } else {
        return shadeDirectLightingNEE(hit, triangles, materials, lightTris, rng, dist);
    }
}

uint8_t toByte(double x) {
    x = std::max(0.0, std::min(1.0, x));
    x = std::pow(x, 1.0 / 2.2);
    return (uint8_t)(x * 255.0 + 0.5);
}

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

void addRectangle(std::vector<Triangle>& triangles,
                  int materialIdx,
                  const Vector3& a,
                  const Vector3& b,
                  const Vector3& c,
                  const Vector3& d) {
    triangles.push_back({a, b, c, materialIdx});
    triangles.push_back({a, c, d, materialIdx});
}

void addBox(std::vector<Triangle>& triangles,
            int materialIdx,
            const Vector3& minCorner,
            const Vector3& maxCorner) {
    double x0 = minCorner.x;
    double y0 = minCorner.y;
    double z0 = minCorner.z;

    double x1 = maxCorner.x;
    double y1 = maxCorner.y;
    double z1 = maxCorner.z;

    addRectangle(
        triangles, materialIdx,
        Vector3(x0, y0, z1),
        Vector3(x1, y0, z1),
        Vector3(x1, y1, z1),
        Vector3(x0, y1, z1)
    );

    addRectangle(
        triangles, materialIdx,
        Vector3(x1, y0, z0),
        Vector3(x0, y0, z0),
        Vector3(x0, y1, z0),
        Vector3(x1, y1, z0)
    );

    addRectangle(
        triangles, materialIdx,
        Vector3(x0, y0, z0),
        Vector3(x0, y0, z1),
        Vector3(x0, y1, z1),
        Vector3(x0, y1, z0)
    );

    addRectangle(
        triangles, materialIdx,
        Vector3(x1, y0, z1),
        Vector3(x1, y0, z0),
        Vector3(x1, y1, z0),
        Vector3(x1, y1, z1)
    );

    addRectangle(
        triangles, materialIdx,
        Vector3(x0, y1, z1),
        Vector3(x1, y1, z1),
        Vector3(x1, y1, z0),
        Vector3(x0, y1, z0)
    );

    addRectangle(
        triangles, materialIdx,
        Vector3(x0, y0, z0),
        Vector3(x1, y0, z0),
        Vector3(x1, y0, z1),
        Vector3(x0, y0, z1)
    );
}

void buildTestScene(std::vector<Material>& materials,
                    std::vector<Triangle>& triangles) {
    materials.clear();
    triangles.clear();

    int matWhite = (int)materials.size();
    materials.push_back({Vector3(0.85, 0.85, 0.85), Vector3(0.0, 0.0, 0.0)});

    int matRed = (int)materials.size();
    materials.push_back({Vector3(0.85, 0.25, 0.25), Vector3(0.0, 0.0, 0.0)});

    int matGreen = (int)materials.size();
    materials.push_back({Vector3(0.25, 0.85, 0.25), Vector3(0.0, 0.0, 0.0)});

    int matLight = (int)materials.size();
    materials.push_back({Vector3(0.0, 0.0, 0.0), Vector3(11.0, 11.0, 11.0)});

    addRectangle(
        triangles, matWhite,
        Vector3(-2.0, 0.0, -2.0),
        Vector3( 2.0, 0.0, -2.0),
        Vector3( 2.0, 0.0,  2.0),
        Vector3(-2.0, 0.0,  2.0)
    );

    addRectangle(
        triangles, matWhite,
        Vector3(-2.0, 2.0, -2.0),
        Vector3( 2.0, 2.0, -2.0),
        Vector3( 2.0, 2.0,  2.0),
        Vector3(-2.0, 2.0,  2.0)
    );

    addRectangle(
        triangles, matWhite,
        Vector3(-2.0, 0.0, -2.0),
        Vector3( 2.0, 0.0, -2.0),
        Vector3( 2.0, 2.0, -2.0),
        Vector3(-2.0, 2.0, -2.0)
    );

    addRectangle(
        triangles, matRed,
        Vector3(-2.0, 0.0, -2.0),
        Vector3(-2.0, 2.0, -2.0),
        Vector3(-2.0, 2.0,  2.0),
        Vector3(-2.0, 0.0,  2.0)
    );

    addRectangle(
        triangles, matGreen,
        Vector3(2.0, 0.0, -2.0),
        Vector3(2.0, 2.0, -2.0),
        Vector3(2.0, 2.0,  2.0),
        Vector3(2.0, 0.0,  2.0)
    );

    addRectangle(
        triangles, matLight,
        Vector3(-1.1, 1.99, -1.1),
        Vector3( 1.1, 1.99, -1.1),
        Vector3( 1.1, 1.99,  1.1),
        Vector3(-1.1, 1.99,  1.1)
    );

    addBox(
        triangles,
        matWhite,
        Vector3(-0.5, 0.0, -0.35),
        Vector3(0.5, 0.85, 0.35)
    );
}

std::string outputFilename(RenderMode mode) {
    if (mode == RenderMode::Flat) {
        return "flat.ppm";
    } else if (mode == RenderMode::Normal) {
        return "normal.ppm";
    } else {
        return "direct_nee_usable.ppm";
    }
}

int main() {
    const int width = 400;
    const int height = 400;
    RenderMode mode = RenderMode::DirectLightingNEE;

    Camera cam;
    cam.pos = Vector3(0.0, 1.0, 2.8);
    cam.lookAt = Vector3(0.0, 0.75, 0.0);
    cam.up = Vector3(0.0, 1.0, 0.0);
    cam.fov = 55.0;
    cam.width = width;
    cam.height = height;

    std::vector<Material> materials;
    std::vector<Triangle> triangles;
    buildTestScene(materials, triangles);

    std::vector<LightTri> lightTris = collectLightTriangles(triangles, materials);

    std::mt19937 rng(1234);
    std::uniform_real_distribution<double> dist(0.0, 1.0);

    std::vector<uint8_t> image(width * height * 3, 0);

    for (int y = 0; y < height; ++y) {
        for (int x = 0; x < width; ++x) {
            double u = (x + 0.5) / (double)width;
            double v = (y + 0.5) / (double)height;

            Vector3 dir = cam.getRayDir(u, v);
            HitInfo hit = intersectScene(cam.pos, dir, triangles);

            Vector3 color(0.0, 0.0, 0.0);

            if (hit.hit) {
                color = shadePixel(mode, hit, triangles, materials, lightTris, rng, dist);
            }

            int idx = (y * width + x) * 3;
            image[idx + 0] = toByte(color.x);
            image[idx + 1] = toByte(color.y);
            image[idx + 2] = toByte(color.z);
        }
    }

    std::string filename = outputFilename(mode);
    writePPM(filename, image, width, height);
    std::cout << "Saved " << filename << "\n";

    return 0;
}