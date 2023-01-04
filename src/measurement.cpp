#include <iostream>
#include <opencv2/highgui.hpp>

#include "math_utils.hpp"
#include "point_cloud.hpp"
#include "range_image_projection.hpp"
#include "timer.hpp"

using namespace LPCC;

template <typename RangeType>
void encode_decode_measurement(const PointCloud::PointCloud& cloud,
                               const std::string& format,
                               const std::vector<int>& params = std::vector<int>(),
                               const int num_itr = 10) {
    const auto size_of_cloud = [](const PointCloud::PointCloud& _cloud) {
        return _cloud.num_elems() * sizeof(float) * 3;
    };

    Timer timer;
    RangeImageProjection::Projector<RangeType> range_projector;
    Image::Image<RangeType> range_image(range_projector.width, range_projector.height);
    std::vector<std::uint8_t> encoded;
    Image::Image<RangeType> decoded(range_image.width(), range_image.height());
    PointCloud::PointCloud decoded_cloud;

    std::int64_t duration_encode_sum = 0ll;
    std::int64_t duration_decode_sum = 0ll;

    for (int itr = 0; itr <= num_itr; itr++) {
        decoded_cloud.reset(cloud.num_elems());

        // Encode
        timer.tick();
        range_projector.from_point_cloud(cloud, range_image);
        encoded = range_image.encode(format, params);
        if (itr > 0) {
            duration_encode_sum += timer.tock();
        }

        // Decode
        timer.tick();

        decoded.decode(encoded);
        range_projector.to_point_cloud(decoded, decoded_cloud);
        if (itr > 0) {
            duration_decode_sum += timer.tock();
        }
    }

    std::cout << format << " :" << std::endl;
    std::cout << "\tDurations :" << std::endl;
    std::cout << "\t\tEncode           : " << duration_encode_sum / num_itr << " [usec]" << std::endl;
    std::cout << "\t\tDecode           : " << duration_decode_sum / num_itr << " [usec]" << std::endl;
    std::cout << "\tStats :" << std::endl;
    std::cout << "\t\tOriginal size    : " << size_of_cloud(cloud) << " [bytes]" << std::endl;
    std::cout << "\t\tEncoded size     : " << static_cast<float>(encoded.size()) << " [bytes]" << std::endl;
    std::cout << "\t\tCompression rate : " << static_cast<float>(encoded.size()) / static_cast<float>(size_of_cloud(cloud)) * 100 << " [%]" << std::endl;
    std::cout << "\t\tDecoded size     : " << size_of_cloud(decoded_cloud) << " [bytes]" << std::endl;
    std::cout << "\t\tOriginal points  : " << cloud.num_elems() << std::endl;
    std::cout << "\t\tDecoded points   : " << decoded_cloud.num_elems() << std::endl;
    decoded_cloud.save("decoded_cloud" + format + ".bin");
}

void benchmark(const std::string& filename) {
    const PointCloud::PointCloud cloud(filename);

    encode_decode_measurement<std::uint16_t>(cloud, ".png", std::vector<int>{ cv::IMWRITE_PNG_COMPRESSION, 1 });
    encode_decode_measurement<std::uint16_t>(cloud, ".tiff");
    encode_decode_measurement<std::uint16_t>(cloud, ".jp2", std::vector<int>{ cv::IMWRITE_JPEG2000_COMPRESSION_X1000, 1000 });
    encode_decode_measurement<std::uint8_t>(cloud, ".jpg", std::vector<int>{ cv::IMWRITE_JPEG_QUALITY, 95 });
}
