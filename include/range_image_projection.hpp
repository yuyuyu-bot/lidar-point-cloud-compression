#ifndef RANGE_IMAGE_PROJECTION
#define RANGE_IMAGE_PROJECTION

#include "image.hpp"
#include "point_cloud.hpp"

namespace {
namespace LPCC {
namespace RangeImageProjection {

template <typename RangeType>
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

    static constexpr auto horizontal_degree_offset = 180.f;
    static constexpr auto horizontal_degree = 360.f;

    const RangeType   range_max;
    const float       horizontal_degree_resolution;
    const float       vertical_division;
    const float       vertical_degree;
    const float       vertical_degree_offset;
    const float       range_scale;
    const float       vertical_degree_resolution;
    const std::size_t width;
    const std::size_t height;

public:
    Projector(const Parameters& params = Parameters{})
    : range_max(params.range_max),
      horizontal_degree_resolution(params.horizontal_degree_resolution),
      vertical_division(params.vertical_division),
      vertical_degree(params.vertical_degree),
      vertical_degree_offset(params.vertical_degree_offset),
      range_scale(static_cast<float>(std::numeric_limits<RangeType>::max()) / params.range_max),
      vertical_degree_resolution(params.vertical_degree / params.vertical_division),
      width(static_cast<int>(horizontal_degree / params.horizontal_degree_resolution)),
      height(static_cast<int>(params.vertical_division)),
      range_image(width, height),
      yaw_image(width, height),
      pitch_image(width, height) {}

    auto& range() { return range_image; }
    const auto& range() const { return range_image; }
    auto& yaw() { return yaw_image; }
    const auto& yaw() const { return yaw_image; }
    auto& pitch() { return pitch_image; }
    const auto& pitch() const { return pitch_image; }

    template <int FLIP = 0>
    void from_point_cloud(const PointCloud::PointCloud& cloud, Image::Image<RangeType>& range_out) {
        for (std::size_t idx = 0; idx < cloud.num_elems(); idx++) {
            const auto point_x = cloud.data[idx].x;
            const auto point_y = cloud.data[idx].y;
            const auto point_z = cloud.data[idx].z;

            // Calculate some parameters for spherical coord.
            const auto distance = std::sqrt(point_x * point_x + point_y * point_y);
            const auto range    = std::sqrt(point_x * point_x + point_y * point_y + point_z * point_z);
            const auto yaw      = atan2_degree(point_y, point_x);
            const auto pitch    = atan2_degree(point_z, distance);

            const auto x = std::clamp<int>(
                (yaw + horizontal_degree_offset) / horizontal_degree_resolution,
                0,
                width - 1);
            const auto y = std::clamp<int>(
                (pitch + vertical_degree_offset) / vertical_degree_resolution,
                0,
                height - 1);

            const int Y = (FLIP == 0) ? y : height - 1 - y;
            range_out.at(x, Y) = static_cast<RangeType>(
                std::clamp<RangeType>(range * range_scale, std::numeric_limits<RangeType>::min(), std::numeric_limits<RangeType>::max()));
            range_image.at(x,  Y) = range;
            yaw_image.at(x, Y) = yaw;
            pitch_image.at(x, Y) = pitch;
        }
    }

    template <int FLIP = 0>
    void to_point_cloud(const Image::Image<RangeType>& range_in, PointCloud::PointCloud& cloud) {
        for (std::size_t y = 0; y < height; y++) {
            for (std::size_t x = 0; x < width; x++) {
                const auto range = static_cast<float>(range_in.at(x, y)) / range_scale;
                if (range <= 0.f) {
                    continue;
                }

                const auto [point_x, point_y, point_z] = calculate_3d_coord<FLIP>(range, x, y);
                cloud.append(PointCloud::PointXYZReflection{.x=point_x, .y=point_y, .z=point_z, .reflection=range});
            }
        }
    }

    template <int FLIP = 0>
    void to_point_cloud(const Image::Image<RangeType>& range_in, const Image::Image<int>& label_image, PointCloud::PointCloud& cloud) {
        for (std::size_t y = 0; y < height; y++) {
            for (std::size_t x = 0; x < width; x++) {
                const auto range = static_cast<float>(range_in.at(x, y)) / range_scale;
                if (range <= 0.f) {
                    continue;
                }

                const auto&& [point_x, point_y, point_z] = calculate_3d_coord<FLIP>(range, x, y);
                cloud.append(PointCloud::PointXYZReflection{
                    .x=point_x, .y=point_y, .z=point_z, .reflection=static_cast<float>(label_image.at(x, y))});
            }
        }
    }

private:
    template <int FLIP>
    std::tuple<float, float, float> calculate_3d_coord(const float range, const std::size_t x, const std::size_t y) {
        const int Y = (FLIP == 0) ? y : height - 1 - y;

        const auto yaw      = (x + 0.5f) * horizontal_degree_resolution - horizontal_degree_offset;
        const auto pitch    = (Y + 0.5f) * vertical_degree_resolution - vertical_degree_offset;
        const auto distance = range * std::cos(degree_to_radian(pitch));
        const auto point_z  = range * std::sin(degree_to_radian(pitch));
        const auto point_y  = distance * std::sin(degree_to_radian(yaw));
        const auto point_x  = distance * std::cos(degree_to_radian(yaw));

        return { point_x, point_y, point_z };
    }

private:
    Image::Image<float> range_image;
    Image::Image<float> yaw_image;
    Image::Image<float> pitch_image;
};


} // namespace RangeImageProjection
} // LPCC
} // anonymous namespace

#endif // RANGE_IMAGE_PROJECTION
