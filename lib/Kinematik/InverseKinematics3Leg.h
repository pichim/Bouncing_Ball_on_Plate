#ifndef INVERSE_KINEMATICS_3LEG_H
#define INVERSE_KINEMATICS_3LEG_H

#include <array>
#include <string>

class InverseKinematics3Leg
{
public:
    struct Geometry
    {
        float r0 = 61.001;         // radius of motor axes
        float r1 = 72.5;           // servo horn length
        float r2 = 101.5;          // rod length
        float rp = 215.0 / 2.0;    // platform radius

        // Motor azimuth angles [rad]
        std::array<float, 3> TH = {
            0.0,
            2.0 * 3.14159265358979323846 / 3.0,
            4.0 * 3.14159265358979323846 / 3.0
        };
    };

    struct Input
    {
        float roll  = 0.0;   // [rad]
        float pitch = 0.0;   // [rad]
        float h     = 98.1; // [same length unit as geometry]
    };

    struct Result
    {
        bool success = false;
        std::string errorMessage;

        // Hidden correction
        float dx = 0.0;
        float dy = 0.0;
        float yaw = 0.0;     // [rad]

        // Platform center after correction
        std::array<float, 3> center = {0.0, 0.0, 0.0};

        // Final chosen motor angles
        std::array<float, 3> alphaRad = {0.0, 0.0, 0.0};
        std::array<float, 3> alphaDeg = {0.0, 0.0, 0.0};

        // Two analytical candidates per leg [deg], wrapped to [0, 360)
        std::array<std::array<float, 2>, 3> alphaCandidatesDeg = {{
            {{0.0, 0.0}},
            {{0.0, 0.0}},
            {{0.0, 0.0}}
        }};

        // Checks
        std::array<float, 3> planeResidual = {0.0, 0.0, 0.0};
        std::array<float, 3> rodError      = {0.0, 0.0, 0.0};
        std::array<bool,   3> hornOutward   = {false, false, false};

        float maxAbsPlaneResidual = 0.0;
        float maxAbsRodError      = 0.0;
        bool allOutward            = false;
    };

    InverseKinematics3Leg();
    explicit InverseKinematics3Leg(const Geometry& geometry);

    void setGeometry(const Geometry& geometry);
    const Geometry& getGeometry() const;

    static float deg2rad(float deg);

    Result compute(const Input& input) const;

private:
    struct Vec3
    {
        float x = 0.0;
        float y = 0.0;
        float z = 0.0;
    };

    struct Mat3
    {
        float m[3][3] = {{0.0}};
    };

    Geometry geometry_;

    static constexpr float kPi  = 3.14159265358979323846;
    static constexpr float kTol = 1e-12;

    // Math helpers
    static float rad2deg(float rad);
    static float wrapDeg360(float deg);
    static float clamp(float value, float minValue, float maxValue);

    static Vec3 add(const Vec3& a, const Vec3& b);
    static Vec3 sub(const Vec3& a, const Vec3& b);
    static Vec3 scale(const Vec3& v, float s);
    static float norm(const Vec3& v);

    static Vec3 matVecMul(const Mat3& R, const Vec3& v);
    static Mat3 Rx(float phi);
    static Mat3 Ry(float phi);
    static Mat3 Rz(float phi);

    static std::array<Vec3, 3> buildM0(const Geometry& g);
    static std::array<Vec3, 3> buildB0(const Geometry& g);
    static std::array<Vec3, 3> buildP0(const Geometry& g);
};

#endif // INVERSE_KINEMATICS_3LEG_H