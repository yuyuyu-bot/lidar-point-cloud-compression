#ifndef IMAGE_HPP
#define IMAGE_HPP

#include <fstream>
#include <functional>
#include <iostream>
#include <memory>
#include <opencv2/highgui.hpp>
#include <random>
#include <string>

namespace LPCC {
namespace Image {

template <typename ElemType, std::size_t Channels = 1>
class Image {
public:
    Image(const std::size_t width, const std::size_t height, const ElemType* const data = nullptr) {
        create(width, height, data);
    }

    Image(const std::string& filename) {
        static_assert(std::is_same_v<ElemType, std::uint16_t>, "Image loading supports only std::uint16_t.");
        const cv::Mat1w image = cv::imread(filename, cv::IMREAD_UNCHANGED);
        create(image.cols, image.rows, image.data);
    }

    Image(const std::vector<std::uint8_t>& buffer) {
        const cv::Mat image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
        create(image.cols, image.rows, image.data);
    }

    void create(const std::size_t width, const std::size_t height, const ElemType* const data = nullptr) {
        width_ = width;
        height_ = height;
        step_ = width * Channels;
        len_ = width * height * Channels;

        data_.reset(new ElemType[len_]);
        if (data != nullptr) {
            std::copy_n(data, len_, data_.get());
        }
        else {
            std::fill_n(data_.get(), len_, ElemType{0});
        }
    }

    auto width() const { return width_; }
    auto height() const { return height_; }
    auto length() const { return len_; }

    ElemType& at(const std::size_t x, const std::size_t y, const std::size_t ch = 0) {
        return data_[step_ * y + x * Channels + ch];
    }

    ElemType at(const std::size_t x, const std::size_t y, const std::size_t ch = 0) const {
        return data_[step_ * y + x * Channels + ch];
    }

    auto data() { return data_.get(); }
    auto data() const { return data_.get(); }

    auto encode(const std::string& format, const std::vector<int>& params = std::vector<int>()) const {
        static_assert(Channels == 1);
        static_assert(std::is_same_v<ElemType, std::uint8_t> || std::is_same_v<ElemType, std::uint16_t> ||
                      std::is_same_v<ElemType, float>);

        const auto rows = static_cast<int>(this->height());
        const auto cols = static_cast<int>(this->width());

        cv::Mat image;
        if constexpr (std::is_same_v<ElemType, std::uint8_t>) {
            image = cv::Mat(rows, cols, CV_8UC1, this->data());
        } else if constexpr (std::is_same_v<ElemType, std::uint16_t>) {
            image = cv::Mat(rows, cols, CV_16UC1, this->data());
        } else if constexpr (std::is_same_v<ElemType, float>) {
            image = cv::Mat(rows, cols, CV_8UC4, this->data());
        }

        std::vector<std::uint8_t> buffer;
        cv::imencode(format, image, buffer, params);
        return buffer;
    }

    void decode(const std::vector<std::uint8_t>& buffer) {
        const cv::Mat image = cv::imdecode(buffer, cv::IMREAD_UNCHANGED);
        std::copy_n(image.ptr<ElemType>(), image.rows * image.cols * image.channels(), this->data());
    }

    void save(const std::string& filename, const float offset = 0.f, const float scale = 1.f) {
        const auto rows = static_cast<int>(this->height());
        const auto cols = static_cast<int>(this->width());
        const auto scaling = [offset, scale](const cv::Mat& img, const ElemType min, const ElemType max) {
            cv::Mat ret = (img + static_cast<double>(offset)) * static_cast<double>(scale);
            return cv::max(min, cv::min(max, ret));
        };

        cv::Mat img;
        if constexpr (std::is_same_v<ElemType, std::uint8_t>) {
            img = cv::Mat(rows, cols, CV_8UC(Channels), this->data());
            img = scaling(img, std::numeric_limits<std::uint8_t>::min(), std::numeric_limits<std::uint8_t>::max());
            img.convertTo(img, CV_8U);
        } else if constexpr (std::is_same_v<ElemType, std::uint16_t>) {
            img = cv::Mat(rows, cols, CV_16UC(Channels), this->data());
            img = scaling(img, std::numeric_limits<std::uint16_t>::min(), std::numeric_limits<std::uint16_t>::max());
            img.convertTo(img, CV_16U);
        } else if constexpr (std::is_same_v<ElemType, float>) {
            img = cv::Mat(rows, cols, CV_32FC(Channels), this->data());
            img = scaling(img, std::numeric_limits<std::uint16_t>::min(), std::numeric_limits<std::uint16_t>::max());
            img.convertTo(img, CV_16U);
        }
        else {
            std::cerr << "Not Implemented." << std::endl;
            return;
        }

        cv::imwrite(filename, img);
    }

private:
    std::unique_ptr<ElemType[]> data_;
    std::size_t width_;
    std::size_t height_;
    std::size_t step_;
    std::size_t len_;
};

inline void labels_to_color(const Image<int>& label_image, Image<std::uint8_t, 3>& color_image) {
    if (label_image.width() != color_image.width() || label_image.height() != color_image.height()) {
        std::cerr << "Color image is not allocated correctly." << std::endl;
        return;
    }

    const auto width = label_image.width();
    const auto height = label_image.height();

    std::size_t num_labels = 0;
    for (std::size_t y = 0; y < height; y++) {
        for (std::size_t x = 0; x < width; x++) {
            if (label_image.at(x, y) < 0) {
                continue;
            }
            num_labels = std::max(num_labels, static_cast<std::size_t>(label_image.at(x, y)));
        }
    }

    std::mt19937 engine(42);
    const auto colors = std::make_unique<std::uint8_t[]>(num_labels * 3);
    for (std::size_t idx = 0; idx < num_labels; idx++) {
        colors[idx * 3 + 0] = engine() % std::numeric_limits<std::uint8_t>::max();
        colors[idx * 3 + 1] = engine() % std::numeric_limits<std::uint8_t>::max();
        colors[idx * 3 + 2] = engine() % std::numeric_limits<std::uint8_t>::max();
    }

    for (std::size_t y = 0; y < height; y++) {
        for (std::size_t x = 0; x < width; x++) {
            if (label_image.at(x, y) < 0) {
                continue;
            }
            color_image.at(x, y, 0) = colors[static_cast<std::size_t>(label_image.at(x, y)) * 3 + 0];
            color_image.at(x, y, 1) = colors[static_cast<std::size_t>(label_image.at(x, y)) * 3 + 1];
            color_image.at(x, y, 2) = colors[static_cast<std::size_t>(label_image.at(x, y)) * 3 + 2];
        }
    }
}

} // namespace Image
} // namespace LPCC

#endif // IMAGE_HPP
