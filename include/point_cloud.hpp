#ifndef POINT_CLOUD_HPP
#define POINT_CLOUD_HPP

#include <algorithm>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

namespace {
namespace LPCC {
namespace PointCloud {

struct PointXYZReflection {
    using ElemType = float;
    static constexpr auto Size = sizeof(ElemType) * 4;
    ElemType x;
    ElemType y;
    ElemType z;
    ElemType reflection;
};

class PointCloud {
public:
    using PointType = PointXYZReflection;
    std::vector<PointType> data;

public:
    PointCloud() : data() {}
    PointCloud(const std::string& filename) {
        load(filename);
    }
    PointCloud(const std::size_t len) {
        data.clear();
        data.reserve(len);
    }

    void append(const PointType& point) {
        data.push_back(point);
    }

    void reset(const std::size_t len = 0) {
        data.clear();
        data.reserve(len);
    }

    void load(const std::string& filename) {
        std::ifstream ifs(filename, std::ios::binary);
        if (!ifs) {
            std::cerr << "Error: PointCloud failed to open " << filename << "." << std::endl;
            std::exit(EXIT_FAILURE);
        }

        reset();
        while (!ifs.eof()) {
            PointType buff;
            ifs.read(reinterpret_cast<char*>(&buff), PointType::Size);
            data.push_back(buff);
        }
    }

    void save(const std::string& filename) const {
        std::ofstream ofs(filename, std::ios::binary);
        ofs.write(reinterpret_cast<const char*>(data.data()), data.size() * PointType::Size);
    }

    const auto num_elems() const { return data.size(); }
};

} // namespace PointCloud
} // namespace LPCC
} // anonymous namespace

#endif // POINT_CLOUD_HPP
