#include <cmath>
#include <numbers>
#include <queue>
#include <iostream>
#include <utility>

#include "image.hpp"
#include "math_utils.hpp"
#include "n_neighbor.hpp"
#include "point_cloud.hpp"
#include "range_image_projection.hpp"

using namespace LPCC;

void benchmark(const std::string& filename);

constexpr auto NOT_LABELED = -1;
constexpr auto GROUND_LABEL = 0;
constexpr auto GROUND_DEGREE_THRESHOLD = 5.f;
constexpr auto ONE_OBJECT_THRESHOLD = 10.f;
constexpr auto NEIGHBOR_N = 4;

template <int FLIP = 0>
void ground_labeling(const Image::Image<std::uint16_t>& range_image, Image::Image<int>& label_image,
                     const float vertical_degree_resolution, const float vertical_degree_offset) {
    const auto width = range_image.width();
    const auto height = range_image.height();
    constexpr auto INVALID_ANGLE = std::numeric_limits<float>::lowest();

    // Generate angle image
    Image::Image<float> angle_image(width, height);
    std::fill_n(angle_image.data(), angle_image.length(), INVALID_ANGLE); // Initialize angles.
    const auto ys = (FLIP == 0) ? 0 : height - 1;
    const auto ye = (FLIP == 0) ? height - 2 : 1;
    const auto update_op = (FLIP == 0) ? [](const std::size_t& v) { return v + 1; } : [](const std::size_t& v) { return v - 1; };
    for (std::size_t y = ys; y != ye; y = update_op(y)) {
        for (std::size_t x = 0; x < width; x++) {
            const auto point_y  = range_image.at(x, y);
            const auto point_y1 = range_image.at(x, update_op(y));
            if (point_y == 0 || point_y1 == 0) {
                continue;
            }
            const auto pitch_y  = degree_to_radian(static_cast<float>(y)            * vertical_degree_resolution - vertical_degree_offset);
            const auto pitch_y1 = degree_to_radian(static_cast<float>(update_op(y)) * vertical_degree_resolution - vertical_degree_offset);

            const float del_z = std::abs(point_y1 * std::sin(pitch_y1) - point_y * std::sin(pitch_y));
            const float del_x = std::abs(point_y1 * std::cos(pitch_y1) - point_y * std::cos(pitch_y));
            angle_image.at(x, y) = atan2_degree(del_z, del_x);
        }
    }

    // Initialize label.
    std::fill_n(label_image.data(), label_image.length(), NOT_LABELED);

    const auto label_ground_bfs =
        [&range_image, &angle_image, &label_image, width, height](const std::size_t x, const std::size_t y) {
            std::queue<std::pair<std::size_t, std::size_t>> queue;
            queue.push(std::make_pair(x, y));

            Image::Image<bool> pushed(width, height);
            pushed.at(x, y) = true;

            while (!queue.empty()) {
                const auto [cx, cy] = queue.front();
                queue.pop();

                // Label as ground.
                label_image.at(cx, cy) = GROUND_LABEL;

                const auto cangle = angle_image.at(cx, cy);
                for (std::size_t nidx = 0; nidx < NNeighbor::len<NEIGHBOR_N>(); nidx++) {
                    const auto s_nx = static_cast<int>(cx) + NNeighbor::X<NEIGHBOR_N>()[nidx];
                    const auto s_ny = static_cast<int>(cy) + NNeighbor::Y<NEIGHBOR_N>()[nidx];
                    const auto nx = static_cast<std::size_t>(s_nx);
                    const auto ny = static_cast<std::size_t>(s_ny);
                    if (s_nx < 0 || s_nx >= static_cast<int>(width) || s_ny < 0 || s_ny >= static_cast<int>(height) ||
                        range_image.at(nx, ny) == 0) {
                        continue;
                    }

                    const auto nangle = angle_image.at(nx, ny);
                    if (nangle >= -180.f && std::abs(cangle - nangle) < GROUND_DEGREE_THRESHOLD &&
                        label_image.at(nx, ny) == NOT_LABELED && !pushed.at(nx, ny)) {
                        queue.push(std::make_pair(nx, ny));
                        pushed.at(nx, ny) = true;
                    }
                }
            }
        };

    for (std::size_t x = 0; x < width; x++) {
        std::size_t y = ys;
        while (range_image.at(x, y) == 0 && y != ye) {
            y = update_op(y);
        }
        if (y != ye) {
            label_ground_bfs(x, y);
        }
    }
}

void range_image_labeling(const Image::Image<std::uint16_t>& range_image, Image::Image<int>& label_image,
                          const float yaw_precision = 0.18f, const float pitch_precision = 0.45f) {
    const auto width = range_image.width();
    const auto height = range_image.height();

    const auto sin_x = std::sin(yaw_precision * std::numbers::pi_v<float> / 180.f);
    const auto cos_x = std::cos(yaw_precision * std::numbers::pi_v<float> / 180.f);
    const auto sin_y = std::sin(pitch_precision * std::numbers::pi_v<float> / 180.f);
    const auto cos_y = std::cos(pitch_precision * std::numbers::pi_v<float> / 180.f);

    const auto label_component_bfs =
        [&range_image, &label_image, width, height, sin_x, cos_x, sin_y, cos_y]
        (const std::size_t x, const std::size_t y, const int label) {
            std::queue<std::pair<std::size_t, std::size_t>> queue;
            queue.push(std::make_pair(x, y));

            Image::Image<bool> pushed(width, height);
            pushed.at(x, y) = true;

            while (!queue.empty()) {
                const auto [cx, cy] = queue.front();
                queue.pop();

                // Label as ground.
                label_image.at(cx, cy) = label;

                const auto crange = range_image.at(cx, cy);
                for (std::size_t nidx = 0; nidx < NNeighbor::len<NEIGHBOR_N>(); nidx++) {
                    const auto s_nx = static_cast<int>(cx) + NNeighbor::X<NEIGHBOR_N>()[nidx];
                    const auto s_ny = static_cast<int>(cy) + NNeighbor::Y<NEIGHBOR_N>()[nidx];
                    const auto nx = static_cast<std::size_t>(s_nx);
                    const auto ny = static_cast<std::size_t>(s_ny);
                    if (s_nx < 0 || s_nx >= static_cast<int>(width) || s_ny < 0 || s_ny >= static_cast<int>(height)) {
                        continue;
                    }

                    const auto nrange = range_image.at(nx, ny);
                    if (nrange == 0) {
                        continue;
                    }
                    const auto d1 = std::max(crange, nrange);
                    const auto d2 = std::min(crange, nrange);

                    auto degree = 0.f;
                    if (nidx % 2 == 0) {
                        degree = atan2_degree(d2 * sin_x, d1 - d2 * cos_x);
                    }
                    else {
                        degree = atan2_degree(d2 * sin_y, d1 - d2 * cos_y);
                    }

                    if (degree > ONE_OBJECT_THRESHOLD && label_image.at(nx, ny) == NOT_LABELED && !pushed.at(nx, ny)) {
                        queue.push(std::make_pair(nx, ny));
                        pushed.at(nx, ny) = true;
                    }
                }
            }
        };

    int label = 1;
    for (std::size_t y = 0; y < height; y++) {
        for (std::size_t x = 0; x < width; x++) {
            if (label_image.at(x, y) == NOT_LABELED && range_image.at(x, y) != 0) {
                label_component_bfs(x, y, label);
                label++;
            }
        }
    }
}

int main(int argc, char** argv) {
    if (argc != 2) {
        std::cerr << "[Usage] ./clustering_based_pcc <filename>" << std::endl;
    }

    const auto filename = std::string{argv[1]};
    benchmark(filename);

    constexpr auto FLIP = 1;
    const PointCloud::PointCloud cloud(filename);

    RangeImageProjection::Projector<std::uint16_t> range_projector;
    const auto width = range_projector.width;
    const auto height = range_projector.height;
    Image::Image<std::uint16_t> range_image(width, height);
    range_projector.from_point_cloud<FLIP>(cloud, range_image);
    range_image.save("dump_range.png");

    Image::Image<int> label_image(width, height);
    ground_labeling<FLIP>(range_image, label_image, range_projector.vertical_degree_resolution,
                          range_projector.vertical_degree_offset);
    range_image_labeling(range_image, label_image, range_projector.horizontal_degree_resolution,
                         range_projector.vertical_degree_resolution);

    PointCloud::PointCloud cloud_out(cloud.num_elems());
    range_projector.to_point_cloud<FLIP>(range_projector.range(), label_image, cloud_out);
    cloud_out.save("cloud_out.bin");

    Image::Image<std::uint8_t, 3> color_label(width, height);
    labels_to_color(label_image, color_label);
    color_label.save("label.png");

    return 0;
}
