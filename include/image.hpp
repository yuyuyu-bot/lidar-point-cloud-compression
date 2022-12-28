#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <random>
#include <string>

namespace {
namespace LPCC {
namespace Image {


template <typename ElemType>
class Image {
public:
    Image(const std::size_t width, const std::size_t height, const ElemType* const data = nullptr) {
        create(width, height, data);
    }

    Image(const std::string& filename) {
        static_assert(std::is_same_v<ElemType, std::uint16_t>, "Image loading supports only std::uint16_t.");
        const cv::Mat1w image(filename, cv::IMREAD_UNCHANGED);
        create(image.cols, image.rows, image.data);
    }

    void create(const std::size_t width, const std::size_t height, const ElemType* const data = nullptr) {
        width_ = width;
        height_ = height;
        step_ = width;
        len_ = width * height;

        data_.reset(new ElemType[len_]);
        if (data != nullptr) {
            std::copy_n(data, len_, data_.get());
        }
        else {
            std::fill_n(data_.get(), len_, ElemType{0});
        }
    }

    template <typename RetType = std::size_t> auto width() const { return static_cast<RetType>(width_); }
    template <typename RetType = std::size_t> auto height() const { return static_cast<RetType>(height_); }
    template <typename RetType = std::size_t> auto length() const { return static_cast<RetType>(len_); }

    ElemType& at(const std::size_t x, const std::size_t y) {
        return data_[step_ * y + x];
    }

    ElemType at(const std::size_t x, const std::size_t y) const {
        return data_[step_ * y + x];
    }

    auto data() {
        return data_.get();
    }

    auto data() const {
        return data_.get();
    }

    auto encode(const std::string& format, const std::vector<int>& params = std::vector<int>()) {
        static_assert(std::is_same_v<ElemType, std::uint8_t> || std::is_same_v<ElemType, std::uint16_t>);

        cv::Mat image;
        if constexpr (std::is_same_v<ElemType, std::uint8_t>) {
            image = cv::Mat(height_, width_, CV_8UC1, data_.get());
        } else if constexpr (std::is_same_v<ElemType, std::uint16_t>) {
            image = cv::Mat(height_, width_, CV_16UC1, data_.get());
        }

        std::vector<std::uint8_t> buffer;
        cv::imencode(format, image, buffer, params);
        return buffer;
    }

    void decode(const std::vector<std::uint8_t>& buffer) {
        const cv::Mat image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
        std::copy_n(image.ptr<ElemType>(), image.rows * image.cols, data_.get());
    }

private:
    std::unique_ptr<ElemType[]> data_;
    std::size_t width_;
    std::size_t height_;
    std::size_t step_;
    std::size_t len_;
};

struct RGB {
    RGB(const std::uint8_t R = 0, const std::uint8_t G = 0, const std::uint8_t B = 0) : r(R), g(G), b(B) {}
    std::uint8_t r;
    std::uint8_t g;
    std::uint8_t b;
};

template <typename ElemType, typename SaveType = ElemType>
void save_raw(const std::string& filename, const Image<ElemType>& image,
              const std::function<SaveType(ElemType)>& element_op = [](const ElemType& v){ return v; }) {
    std::ofstream ofs(filename);
    if (!ofs) {
        std::cerr << "Failed to open " << filename << std::endl;
        return;
    }

    for (std::size_t y = 0; y < image.height(); y++) {
        for (std::size_t x = 0; x < image.width; x++) {
            ofs << element_op(image.at(x, y));
        }
    }
};

template <typename ElemType, typename SaveType = ElemType>
void save_image(const std::string& filename, const Image<ElemType>& image,
                const float offset = 0.f, const float scale = 1.f) {
    static_assert(std::is_same_v<ElemType, RGB> ||
                  std::is_same_v<ElemType, std::uint16_t> ||
                  std::is_same_v<ElemType, float>);

    if constexpr (std::is_same_v<ElemType, RGB>) {
        const cv::Mat cv_img(image.height(), image.width(), CV_8UC3, image.data());
        cv::imwrite(filename, cv_img);
    } else if constexpr (std::is_same_v<ElemType, std::uint16_t>) {
        const cv::Mat cv_img(image.height(), image.width(), CV_16UC1, image.data());
        cv::imwrite(filename, cv_img);
    } else if constexpr (std::is_same_v<ElemType, float>) {
        cv::Mat1w cv_img(image.height(), image.width());
        for (std::size_t y = 0; y < image.height(); y++) {
            for (std::size_t x = 0; x < image.width(); x++) {
                const auto value   = (image.at(x, y) + offset) * scale;
                const auto clamped = std::clamp<std::uint16_t>(
                    value, std::numeric_limits<std::uint16_t>::min(), std::numeric_limits<std::uint16_t>::max());
                cv_img.at<std::uint16_t>(y, x) = clamped;
            }
        }
        cv::imwrite(filename, cv_img);
    }
};

void labels_to_color(const Image<int>& label_image, Image<RGB>& color_image) {
    if (label_image.width() != color_image.width() || label_image.height() != color_image.height()) {
        std::cerr << "Color image is not allocated correctly." << std::endl;
        return;
    }

    const auto width = label_image.width();
    const auto height = label_image.height();

    int num_labels = -1;
    for (std::size_t y = 0; y < height; y++) {
        for (std::size_t x = 0; x < width; x++) {
            num_labels = std::max(num_labels, label_image.at(x, y));
        }
    }

    std::mt19937 engine(42);
    const auto colors = std::make_unique<RGB[]>(num_labels);
    for (int idx = 0; idx < num_labels; idx++) {
        colors[idx].r = engine() % std::numeric_limits<std::uint8_t>::max();
        colors[idx].g = engine() % std::numeric_limits<std::uint8_t>::max();
        colors[idx].b = engine() % std::numeric_limits<std::uint8_t>::max();
    }

    for (std::size_t y = 0; y < height; y++) {
        for (std::size_t x = 0; x < width; x++) {
            if (label_image.at(x, y) < 0) {
                continue;
            }
            color_image.at(x, y) = colors[label_image.at(x, y)];
        }
    }
}

} // namespace Image
} // namespace LPCC
} // anonymous namespace

#endif // IMAGE_HPP
