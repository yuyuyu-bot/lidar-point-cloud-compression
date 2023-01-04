#ifndef RANGE_IMAGE_PROJECTION
#define RANGE_IMAGE_PROJECTION

#include <opencv2/core/simd_intrinsics.hpp>

#include "image.hpp"
#include "point_cloud.hpp"

namespace LPCC {
namespace RangeImageProjection {

template <typename RangeType, int FLIP = 0>
class Projector {
public:
    struct Parameters {
        Parameters(
            const RangeType range_max_ = 100,
            const float horizontal_degree_resolution_ = 0.18f,
            const int vertical_division_ = 64,
            const float vertical_degree_ = 30.f,
            const float vertical_degree_offset_ = 25.f)
        : range_max(range_max_),
          horizontal_degree_resolution(horizontal_degree_resolution_),
          vertical_division(vertical_division_),
          vertical_degree(vertical_degree_),
          vertical_degree_offset(vertical_degree_offset_)
        {}

        RangeType range_max;
        float     horizontal_degree_resolution;
        int       vertical_division;
        float     vertical_degree;
        float     vertical_degree_offset;
    };

    Projector(const Parameters& params = Parameters{})
    : range_max(params.range_max),
      horizontal_degree_resolution(params.horizontal_degree_resolution),
      vertical_division(params.vertical_division),
      vertical_degree(params.vertical_degree),
      vertical_degree_offset(params.vertical_degree_offset),
      range_scale(static_cast<float>(std::numeric_limits<RangeType>::max()) / params.range_max),
      vertical_degree_resolution(params.vertical_degree / static_cast<float>(params.vertical_division)),
      width(static_cast<std::size_t>(horizontal_degree / params.horizontal_degree_resolution)),
      height(static_cast<std::size_t>(params.vertical_division)),
      range_image(width, height),
      yaw_image(width, height),
      pitch_image(width, height) {}

    auto& range() { return range_image; }
    const auto& range() const { return range_image; }
    auto& yaw() { return yaw_image; }
    const auto& yaw() const { return yaw_image; }
    auto& pitch() { return pitch_image; }
    const auto& pitch() const { return pitch_image; }

    void from_point_cloud(const PointCloud::PointCloud& cloud, Image::Image<RangeType>& range_out) {
        constexpr auto simd_width = sizeof(cv::v_float32x4) / sizeof(float);
        const auto v_zeros                        = cv::v_setzero_f32();
        const auto v_range_scale                  = cv::v_setall_f32(range_scale);
        const auto v_horizontal_degree_offset     = cv::v_setall_f32(horizontal_degree_offset);
        const auto v_horizontal_degree_resolution = cv::v_setall_f32(horizontal_degree_resolution);
        const auto v_vertical_degree_offset       = cv::v_setall_f32(vertical_degree_offset);
        const auto v_vertical_degree_resolution   = cv::v_setall_f32(vertical_degree_resolution);
        const auto v_width_m_1                    = cv::v_setall_f32(static_cast<float>(width - 1));
        const auto v_height_m_1                   = cv::v_setall_f32(static_cast<float>(height - 1));
        const auto v_range_t_min                  = cv::v_setall_f32(std::numeric_limits<RangeType>::min());
        const auto v_range_t_max                  = cv::v_setall_f32(std::numeric_limits<RangeType>::max());

        const auto clamp = [](const cv::v_float32x4& v_v, const cv::v_float32x4& v_min, const cv::v_float32x4& v_max) {
            return cv::v_max(v_min, cv::v_min(v_v, v_max));
        };

        std::size_t idx = 0;
#ifdef CV_SIMD128
        for (; idx + simd_width < cloud.num_elems(); idx += simd_width) {
            cv::v_float32x4 v_point_x;
            cv::v_float32x4 v_point_y;
            cv::v_float32x4 v_point_z;
            cv::v_float32x4 v_point_r;
            cv::v_load_deinterleave(reinterpret_cast<const float*>(cloud.data.data() + idx), v_point_x, v_point_y, v_point_z, v_point_r);

            // Calculate some parameters for spherical coord.
            const auto v_distance     = cv::v_sqrt(v_point_x * v_point_x + v_point_y * v_point_y);
            const auto v_range        = cv::v_sqrt(v_point_x * v_point_x + v_point_y * v_point_y + v_point_z * v_point_z);
            const auto v_range_scaled = clamp(v_range * v_range_scale, v_range_t_min, v_range_t_max);
            const auto v_yaw          = atan2_degree_simd(v_point_y, v_point_x);
            const auto v_pitch        = atan2_degree_simd(v_point_z, v_distance);

            const auto v_x = clamp((v_yaw + v_horizontal_degree_offset) / v_horizontal_degree_resolution, v_zeros, v_width_m_1);
            auto v_y = clamp((v_pitch + v_vertical_degree_offset) / v_vertical_degree_resolution, v_zeros, v_height_m_1);
            if constexpr (FLIP != 0) {
                v_y = v_height_m_1 - v_y;
            }

#pragma unroll simd_width
            for (std::size_t i = 0; i < simd_width; i++) {
                const auto x = static_cast<std::size_t>(v_x.val[i]);
                const auto y = static_cast<std::size_t>(v_y.val[i]);
                range_out.at(x, y) = static_cast<RangeType>(v_range_scaled.val[i]);
                range_image.at(x,  y) = v_range.val[i];
                yaw_image.at(x, y) = v_yaw.val[i];
                pitch_image.at(x, y) = v_pitch.val[i];
            }
        }
#endif // CV_SIMD128
        for (; idx < cloud.num_elems(); idx++) {
            const auto point_x = cloud.data[idx].x;
            const auto point_y = cloud.data[idx].y;
            const auto point_z = cloud.data[idx].z;

            // Calculate some parameters for spherical coord.
            const auto distance = std::sqrt(point_x * point_x + point_y * point_y);
            const auto range    = std::sqrt(point_x * point_x + point_y * point_y + point_z * point_z);
            const auto yaw      = atan2_degree(point_y, point_x);
            const auto pitch    = atan2_degree(point_z, distance);

            const auto x = std::min(
                static_cast<std::size_t>((yaw + horizontal_degree_offset) / horizontal_degree_resolution),
                width - 1);
            const auto y = std::min(
                static_cast<std::size_t>((pitch + vertical_degree_offset) / vertical_degree_resolution),
                height - 1);

            const auto Y = (FLIP == 0) ? static_cast<std::size_t>(y) : height - 1 - y;
            range_out.at(x, Y) = static_cast<RangeType>(
                std::clamp<float>(
                    range * range_scale,
                    std::numeric_limits<RangeType>::min(),
                    std::numeric_limits<RangeType>::max()));
            range_image.at(x, Y) = range;
            yaw_image.at(x, Y) = yaw;
            pitch_image.at(x, Y) = pitch;
        }
    }

    void to_point_cloud(const Image::Image<RangeType>& range_in, PointCloud::PointCloud& cloud) {
        for (std::size_t y = 0; y < height; y++) {
            for (std::size_t x = 0; x < width; x++) {
                const auto range = static_cast<float>(range_in.at(x, y)) / range_scale;
                if (range <= 0.f) {
                    continue;
                }

                const auto [point_x, point_y, point_z] = calculate_3d_coord(range, x, y);
                cloud.append(PointCloud::PointXYZReflection{.x=point_x, .y=point_y, .z=point_z, .reflection=range});
            }
        }
    }

    void to_point_cloud(const Image::Image<RangeType>& range_in, const Image::Image<int>& label_image, PointCloud::PointCloud& cloud) {
        for (std::size_t y = 0; y < height; y++) {
            for (std::size_t x = 0; x < width; x++) {
                const auto range = static_cast<float>(range_in.at(x, y)) / range_scale;
                if (range <= 0.f) {
                    continue;
                }

                const auto&& [point_x, point_y, point_z] = calculate_3d_coord(range, x, y);
                cloud.append(PointCloud::PointXYZReflection{
                    .x=point_x, .y=point_y, .z=point_z, .reflection=static_cast<float>(label_image.at(x, y))});
            }
        }
    }

private:
#ifdef CV_SIMD128
    cv::v_float32x4 atan2_degree_simd(const cv::v_float32x4& v_y, const cv::v_float32x4 v_x) {
        const auto v_abs_x = cv::v_abs(v_x);
        const auto v_abs_y = cv::v_abs(v_y);
        const auto a = cv::v_min(v_abs_x, v_abs_y) / cv::v_max(v_abs_x, v_abs_y);
        const auto s = a * a;
        auto r = ((cv::v_setall_f32(-0.0464964749f) * s +
                   cv::v_setall_f32(0.15931422f)) * s -
                   cv::v_setall_f32(0.327622764f)
                 ) * s * a + a;
        r = cv::v_select(v_abs_x < v_abs_y, cv::v_setall_f32(1.57079637f) - r, r);
        r = cv::v_select(v_x < cv::v_setzero_f32(), cv::v_setall_f32(std::numbers::pi_v<float>) - r, r);
        r = cv::v_select(v_y < cv::v_setzero_f32(), cv::v_setall_f32(-1.f) * r, r);
        r = r * cv::v_setall_f32(180.f) / cv::v_setall_f32(std::numbers::pi_v<float>);
        return r;
    }
#endif // CV_SIMD128

    std::tuple<float, float, float> calculate_3d_coord(const float range, const std::size_t x, const std::size_t y) {
        const auto Y = (FLIP == 0) ? y : height - 1 - y;

        const auto yaw      = (static_cast<float>(x) + 0.5f) * horizontal_degree_resolution - horizontal_degree_offset;
        const auto pitch    = (static_cast<float>(Y) + 0.5f) * vertical_degree_resolution - vertical_degree_offset;
        const auto distance = range * std::cos(degree_to_radian(pitch));
        const auto point_z  = range * std::sin(degree_to_radian(pitch));
        const auto point_y  = distance * std::sin(degree_to_radian(yaw));
        const auto point_x  = distance * std::cos(degree_to_radian(yaw));

        return { point_x, point_y, point_z };
    }

public:
    static constexpr auto horizontal_degree_offset = 180.f;
    static constexpr auto horizontal_degree = 360.f;

    const RangeType   range_max;
    const float       horizontal_degree_resolution;
    const int         vertical_division;
    const float       vertical_degree;
    const float       vertical_degree_offset;
    const float       range_scale;
    const float       vertical_degree_resolution;
    const std::size_t width;
    const std::size_t height;

private:
    Image::Image<float> range_image;
    Image::Image<float> yaw_image;
    Image::Image<float> pitch_image;
};

} // namespace RangeImageProjection
} // LPCC

#endif // RANGE_IMAGE_PROJECTION
