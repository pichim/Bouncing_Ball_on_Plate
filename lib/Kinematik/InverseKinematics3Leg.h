#ifndef INVERSE_KINEMATICS_3LEG_H
#define INVERSE_KINEMATICS_3LEG_H

#include <array>
#include <string>

class InverseKinematics3Leg
{
public:
    struct Geometry
    {
        double r0 = 61.001;   // radius of motor axes
        double r1 = 72.5;     // servo horn length
        double r2 = 72.5;     // rod length
        double rp = 100.0;    // platform radius

        // Motor azimuth angles [rad]
        std::array<double, 3> TH = {
            0.0,
            2.0 * 3.14159265358979323846 / 3.0,
            4.0 * 3.14159265358979323846 / 3.0
        };
    };

    struct Input
    {
        double roll  = 0.0;   // [rad]
        double pitch = 0.0;   // [rad]
        double h     = 100.0; // [same length unit as geometry]
    };

    struct Result
    {
        bool success = false;
        std::string errorMessage;

        // Hidden correction
        double dx = 0.0;
        double dy = 0.0;
        double yaw = 0.0;     // [rad]

        // Platform center after correction
        std::array<double, 3> center = {0.0, 0.0, 0.0};

        // Final chosen motor angles
        std::array<double, 3> alphaRad = {0.0, 0.0, 0.0};
        std::array<double, 3> alphaDeg = {0.0, 0.0, 0.0};

        // Two analytical candidates per leg [deg], wrapped to [0, 360)
        std::array<std::array<double, 2>, 3> alphaCandidatesDeg = {{
            {{0.0, 0.0}},
            {{0.0, 0.0}},
            {{0.0, 0.0}}
        }};

        // Checks
        std::array<double, 3> planeResidual = {0.0, 0.0, 0.0};
        std::array<double, 3> rodError      = {0.0, 0.0, 0.0};
        std::array<bool,   3> hornOutward   = {false, false, false};

        double maxAbsPlaneResidual = 0.0;
        double maxAbsRodError      = 0.0;
        bool allOutward            = false;
    };

    explicit InverseKinematics3Leg(const Geometry& geometry = Geometry{});

    void setGeometry(const Geometry& geometry);
    const Geometry& getGeometry() const;

    Result compute(const Input& input) const;

private:
    struct Vec3
    {
        double x = 0.0;
        double y = 0.0;
        double z = 0.0;
    };

    struct Mat3
    {
        double m[3][3] = {{0.0}};
    };

    Geometry geometry_;

    static constexpr double kPi  = 3.14159265358979323846;
    static constexpr double kTol = 1e-12;

    // Math helpers
    static double rad2deg(double rad);
    static double wrapDeg360(double deg);
    static double clamp(double value, double minValue, double maxValue);

    static Vec3 add(const Vec3& a, const Vec3& b);
    static Vec3 sub(const Vec3& a, const Vec3& b);
    static Vec3 scale(const Vec3& v, double s);
    static double norm(const Vec3& v);

    static Vec3 matVecMul(const Mat3& R, const Vec3& v);
    static Mat3 Rx(double phi);
    static Mat3 Ry(double phi);
    static Mat3 Rz(double phi);

    static std::array<Vec3, 3> buildM0(const Geometry& g);
    static std::array<Vec3, 3> buildB0(const Geometry& g);
    static std::array<Vec3, 3> buildP0(const Geometry& g);
};

#endif // INVERSE_KINEMATICS_3LEG_H