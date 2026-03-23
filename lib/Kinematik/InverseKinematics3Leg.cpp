#include "InverseKinematics3Leg.h"

#include <algorithm>
#include <cmath>
#include <limits>

InverseKinematics3Leg::InverseKinematics3Leg(const Geometry& geometry)
    : geometry_(geometry)
{
}

void InverseKinematics3Leg::setGeometry(const Geometry& geometry)
{
    geometry_ = geometry;
}

const InverseKinematics3Leg::Geometry& InverseKinematics3Leg::getGeometry() const
{
    return geometry_;
}

InverseKinematics3Leg::Result InverseKinematics3Leg::compute(const Input& input) const
{
    Result result;

    const Geometry& g = geometry_;

    const auto P0 = buildP0(g);

    // =========================================================================
    // STEP 1: TILT PLATFORM
    // P_tilt = Ry(pitch) * Rx(roll) * P0
    // =========================================================================
    const Mat3 Rx_roll  = Rx(input.roll);
    const Mat3 Ry_pitch = Ry(input.pitch);

    std::array<Vec3, 3> P_tilt{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        const Vec3 tmp = matVecMul(Rx_roll, P0[i]);
        P_tilt[i] = matVecMul(Ry_pitch, tmp);
    }

    // =========================================================================
    // STEP 2: ANALYTICAL PLATFORM LOCKING
    // =========================================================================
    std::array<double, 3> a = {0.0, 0.0, 0.0};
    std::array<double, 3> b = {0.0, 0.0, 0.0};

    for (std::size_t i = 0; i < 3; ++i)
    {
        const double x_i = P_tilt[i].x;
        const double y_i = P_tilt[i].y;
        const double th  = g.TH[i];

        a[i] =  x_i * std::cos(th) + y_i * std::sin(th);
        b[i] = -x_i * std::sin(th) + y_i * std::cos(th);
    }

    const double A = a[0] + a[1] + a[2];
    const double B = b[0] + b[1] + b[2];

    result.yaw = std::atan2(-B, A);

    std::array<double, 3> f = {0.0, 0.0, 0.0};
    for (std::size_t i = 0; i < 3; ++i)
    {
        f[i] = a[i] * std::sin(result.yaw) + b[i] * std::cos(result.yaw);
    }

    result.dy = -f[0];
    result.dx = (f[1] - f[2]) / std::sqrt(3.0);

    const Mat3 Rz_yaw = Rz(result.yaw);

    std::array<Vec3, 3> Pcorr{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        const Vec3 rotated = matVecMul(Rz_yaw, P_tilt[i]);
        Pcorr[i] = add(rotated, Vec3{result.dx, result.dy, input.h});
    }

    // Platform center C = mean(Pcorr)
    Vec3 C{};
    for (const auto& p : Pcorr)
    {
        C = add(C, p);
    }
    C = scale(C, 1.0 / 3.0);

    result.center = {C.x, C.y, C.z};

    // =========================================================================
    // STEP 3: 2D IK FOR EACH LEG
    // =========================================================================
    for (std::size_t i = 0; i < 3; ++i)
    {
        // 3.1 Corrected platform joint in local leg frame
        const Vec3 P_local = matVecMul(Rz(-g.TH[i]), Pcorr[i]);
        result.planeResidual[i] = P_local.y;

        // 3.2 Relative target to motor axis
        const double u   = P_local.x - g.r0;
        const double w   = P_local.z;
        const double rho = std::hypot(u, w);

        // Reachability check
        if (rho < std::abs(g.r1 - g.r2) - kTol || rho > (g.r1 + g.r2) + kTol)
        {
            result.success = false;
            result.errorMessage = "Leg " + std::to_string(i + 1) +
                                  " is not reachable for the requested pose.";
            return result;
        }

        // Safety against division by zero / numerically singular case
        if (rho < kTol)
        {
            result.success = false;
            result.errorMessage = "Leg " + std::to_string(i + 1) +
                                  " has rho too close to zero.";
            return result;
        }

        // 3.3 Two analytical branches
        const double c = clamp((g.r1 * g.r1 + rho * rho - g.r2 * g.r2) /
                               (2.0 * g.r1 * rho), -1.0, 1.0);

        const double beta  = std::atan2(u, -w);
        const double gamma = std::acos(c);

        const std::array<double, 2> cand = {beta + gamma, beta - gamma};

        std::array<Vec3, 2> Hcand{};
        std::array<double, 2> candDegWrap = {0.0, 0.0};
        std::array<bool, 2> isHornOutward = {false, false};

        for (std::size_t k = 0; k < 2; ++k)
        {
            Hcand[k] = Vec3{
                g.r0 + g.r1 * std::sin(cand[k]),
                0.0,
                -g.r1 * std::cos(cand[k])
            };

            candDegWrap[k] = wrapDeg360(rad2deg(cand[k]));
            isHornOutward[k] = (Hcand[k].x >= g.r0 - kTol);
        }

        result.alphaCandidatesDeg[i][0] = candDegWrap[0];
        result.alphaCandidatesDeg[i][1] = candDegWrap[1];

        // 3.4 Keep only outward-horn branches
        int chosen = -1;

        if (isHornOutward[0] && isHornOutward[1])
        {
            chosen = (candDegWrap[0] <= candDegWrap[1]) ? 0 : 1;
        }
        else if (isHornOutward[0])
        {
            chosen = 0;
        }
        else if (isHornOutward[1])
        {
            chosen = 1;
        }
        else
        {
            result.success = false;
            result.errorMessage = "Leg " + std::to_string(i + 1) +
                                  " has no outward-horn solution for this pose.";
            return result;
        }

        result.alphaRad[i] = cand[chosen];
        result.alphaDeg[i] = candDegWrap[chosen];
        result.hornOutward[i] = isHornOutward[chosen];

        const Vec3 H_local = Hcand[chosen];
        result.rodError[i] = norm(sub(P_local, H_local)) - g.r2;
    }

    result.maxAbsPlaneResidual = std::max({
        std::abs(result.planeResidual[0]),
        std::abs(result.planeResidual[1]),
        std::abs(result.planeResidual[2])
    });

    result.maxAbsRodError = std::max({
        std::abs(result.rodError[0]),
        std::abs(result.rodError[1]),
        std::abs(result.rodError[2])
    });

    result.allOutward = result.hornOutward[0] &&
                        result.hornOutward[1] &&
                        result.hornOutward[2];

    result.success = true;
    result.errorMessage.clear();
    return result;
}

// ============================================================================
// PRIVATE HELPERS
// ============================================================================

double InverseKinematics3Leg::rad2deg(double rad)
{
    return rad * 180.0 / kPi;
}

double InverseKinematics3Leg::wrapDeg360(double deg)
{
    double wrapped = std::fmod(deg, 360.0);
    if (wrapped < 0.0)
    {
        wrapped += 360.0;
    }
    return wrapped;
}

double InverseKinematics3Leg::clamp(double value, double minValue, double maxValue)
{
    return std::max(minValue, std::min(value, maxValue));
}

InverseKinematics3Leg::Vec3 InverseKinematics3Leg::add(const Vec3& a, const Vec3& b)
{
    return Vec3{a.x + b.x, a.y + b.y, a.z + b.z};
}

InverseKinematics3Leg::Vec3 InverseKinematics3Leg::sub(const Vec3& a, const Vec3& b)
{
    return Vec3{a.x - b.x, a.y - b.y, a.z - b.z};
}

InverseKinematics3Leg::Vec3 InverseKinematics3Leg::scale(const Vec3& v, double s)
{
    return Vec3{v.x * s, v.y * s, v.z * s};
}

double InverseKinematics3Leg::norm(const Vec3& v)
{
    return std::sqrt(v.x * v.x + v.y * v.y + v.z * v.z);
}

InverseKinematics3Leg::Vec3 InverseKinematics3Leg::matVecMul(const Mat3& R, const Vec3& v)
{
    return Vec3{
        R.m[0][0] * v.x + R.m[0][1] * v.y + R.m[0][2] * v.z,
        R.m[1][0] * v.x + R.m[1][1] * v.y + R.m[1][2] * v.z,
        R.m[2][0] * v.x + R.m[2][1] * v.y + R.m[2][2] * v.z
    };
}

InverseKinematics3Leg::Mat3 InverseKinematics3Leg::Rx(double phi)
{
    const double c = std::cos(phi);
    const double s = std::sin(phi);

    Mat3 R{};
    R.m[0][0] = 1.0; R.m[0][1] = 0.0; R.m[0][2] = 0.0;
    R.m[1][0] = 0.0; R.m[1][1] = c;   R.m[1][2] = -s;
    R.m[2][0] = 0.0; R.m[2][1] = s;   R.m[2][2] = c;
    return R;
}

InverseKinematics3Leg::Mat3 InverseKinematics3Leg::Ry(double phi)
{
    const double c = std::cos(phi);
    const double s = std::sin(phi);

    Mat3 R{};
    R.m[0][0] = c;   R.m[0][1] = 0.0; R.m[0][2] = s;
    R.m[1][0] = 0.0; R.m[1][1] = 1.0; R.m[1][2] = 0.0;
    R.m[2][0] = -s;  R.m[2][1] = 0.0; R.m[2][2] = c;
    return R;
}

InverseKinematics3Leg::Mat3 InverseKinematics3Leg::Rz(double phi)
{
    const double c = std::cos(phi);
    const double s = std::sin(phi);

    Mat3 R{};
    R.m[0][0] = c;   R.m[0][1] = -s;  R.m[0][2] = 0.0;
    R.m[1][0] = s;   R.m[1][1] = c;   R.m[1][2] = 0.0;
    R.m[2][0] = 0.0; R.m[2][1] = 0.0; R.m[2][2] = 1.0;
    return R;
}

std::array<InverseKinematics3Leg::Vec3, 3>
InverseKinematics3Leg::buildM0(const Geometry& g)
{
    std::array<Vec3, 3> M0{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        M0[i] = Vec3{
            g.r0 * std::cos(g.TH[i]),
            g.r0 * std::sin(g.TH[i]),
            0.0
        };
    }
    return M0;
}

std::array<InverseKinematics3Leg::Vec3, 3>
InverseKinematics3Leg::buildB0(const Geometry& g)
{
    std::array<Vec3, 3> B0{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        B0[i] = Vec3{
            g.r0 * std::cos(g.TH[i]),
            g.r0 * std::sin(g.TH[i]),
            -g.r1
        };
    }
    return B0;
}

std::array<InverseKinematics3Leg::Vec3, 3>
InverseKinematics3Leg::buildP0(const Geometry& g)
{
    std::array<Vec3, 3> P0{};
    for (std::size_t i = 0; i < 3; ++i)
    {
        P0[i] = Vec3{
            g.rp * std::cos(g.TH[i]),
            g.rp * std::sin(g.TH[i]),
            0.0
        };
    }
    return P0;
}