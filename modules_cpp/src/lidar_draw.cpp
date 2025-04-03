#include <pybind11/pybind11.h>
#include <pybind11/numpy.h>
#include <pybind11/stl.h>
#include <opencv2/opencv.hpp>
#include <vector>
#include <cmath>

namespace py = pybind11;

struct MapInfo {
    int originX;
    int originY;
    double csize;
};

std::vector<int> hex_to_rgb(const std::string &hex) {
    int r, g, b;
    if (sscanf(hex.c_str(), "#%02x%02x%02x", &r, &g, &b) != 3) {
        throw std::runtime_error("Invalid hex color string.");
    }
    return {r, g, b};
}

void draw_lidar_on_global_map(py::array_t<uint8_t> img_disp, 
                                  const std::vector<std::vector<double>> &urg_data, 
                                  double rx, double ry, double ra, 
                                  const MapInfo &mapInfo, 
                                  double start_angle, double end_angle, double step_angle, 
                                  const std::string &color_hex) {

    // NumPy配列をcv::Matに変換
    py::buffer_info buf_info = img_disp.request();
    uint8_t *ptr = static_cast<uint8_t *>(buf_info.ptr);
    cv::Mat img_disp_mat(buf_info.shape[0], buf_info.shape[1], CV_8UC3, ptr);

    std::vector<int> color = hex_to_rgb(color_hex);
    int height = img_disp_mat.rows;
    int width = img_disp_mat.cols;

    // LiDAR変換用にcos, sin のリストを作る
    int n_angles = static_cast<int>((end_angle - start_angle) / step_angle) + 1;
    std::vector<double> cs(n_angles), sn(n_angles);
    for (int i = 0; i < n_angles; ++i) {
        double angle = (i * step_angle + start_angle) * M_PI / 180.0 + ra;
        cs[i] = cos(angle);
        sn[i] = sin(angle);
    }

    std::vector<double> d_values;
    for (const auto& data : urg_data) {
        d_values.push_back(data[1]);
    }

    try {
        std::vector<int> ix, iy;
        for (size_t i = 0; i < urg_data.size(); ++i) {
            double d = d_values[i];
            double x = d * cs[i % n_angles] / 1000.0 + rx; // X座標変換
            double y = d * sn[i % n_angles] / 1000.0 + ry; // Y座標変換
            int ix_val = static_cast<int>(x / mapInfo.csize + mapInfo.originX);
            int iy_val = static_cast<int>(-y / mapInfo.csize + mapInfo.originY);
            // 範囲内の座標をフィルタリング
            if (ix_val >= 0 && ix_val < width && iy_val >= 0 && iy_val < height) {
                ix.push_back(ix_val);
                iy.push_back(iy_val);
            }
        }

        // 描画
        for (size_t i = 0; i < ix.size(); ++i) {
            cv::circle(img_disp_mat, cv::Point(ix[i], iy[i]), 5, cv::Scalar(color[0], color[1], color[2]), -1);
        }
    } catch (const std::exception &e) {
        py::print("Error in processing lidar data: ", e.what());
    }

    //return img_disp_mat;
}

PYBIND11_MODULE(lidar_draw, m) {
    py::class_<MapInfo>(m, "MapInfo")
        .def(py::init<>())
        .def_readwrite("originX", &MapInfo::originX)
        .def_readwrite("originY", &MapInfo::originY)
        .def_readwrite("csize", &MapInfo::csize);

    m.def("draw_lidar_on_global_map", &draw_lidar_on_global_map, 
          "Draw lidar data on a global map.", 
          py::arg("img_disp"), py::arg("urg_data"), py::arg("rx"), py::arg("ry"), py::arg("ra"), 
          py::arg("mapInfo"), py::arg("start_angle"), py::arg("end_angle"), py::arg("step_angle"), py::arg("color_hex"));
}