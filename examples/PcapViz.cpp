#include <algorithm>
#include <iostream>
#include <memory>
#include <random>

#include "ouster/impl/build.h"
#include "ouster/point_viz.h"
#include <fstream>
#include <iomanip>
#include <iostream>
#include <vector>

#include "helpers.h"
#include "ouster/impl/build.h"
#include "ouster/client.h"
#include "ouster/lidar_scan.h"
#include "ouster/types.h"

using namespace ouster;

//! [docs-stag-x-image-form]
img_t<double> get_x_in_image_form(const LidarScan& scan, bool destaggered,
                                  const sensor::sensor_info& info) {
    // For convenience, save w and h to variables
    const size_t w = info.format.columns_per_frame;
    const size_t h = info.format.pixels_per_column;

    // Get the XYZ in ouster::Points (n x 3 Eigen array) form
    XYZLut lut = make_xyz_lut(info);
    auto cloud = cartesian(scan.field(sensor::ChanField::RANGE), lut);

    // Access x and reshape as needed
    // Note that the values in cloud.col(0) are ordered
    auto x = Eigen::Map<const img_t<double>>(cloud.col(0).data(), h, w);
    auto x_destaggered = destagger<double>(x, info.format.pixel_shift_by_row);

    // Apply destagger if desired
    if (!destaggered) return x;
    return x_destaggered;
}
//! [docs-etag-x-image-form]
//

int main(int argc, char* argv[]) {
    if (argc != 3) {
        std::cerr << "Version: " << ouster::SDK_VERSION_FULL << " ("
                  << ouster::BUILD_SYSTEM << ")"
                  << "\n\nUsage: representation_example <pcap_file> <json_file>"
                  << std::endl;
        return argc == 1 ? EXIT_SUCCESS : EXIT_FAILURE;
    }

    // std::random boilerplate
    std::random_device rd;
    std::default_random_engine re(rd());
    std::uniform_real_distribution<float> dis(-20.0, 20.0);
    std::uniform_real_distribution<float> dis2(0.0, 1.0);

    const std::string pcap_file = argv[1];
    const std::string json_file = argv[2];

    auto handle = sensor_utils::replay_initialize(pcap_file);
    auto info = sensor::metadata_from_json(json_file);

    size_t w = info.format.columns_per_frame;
    size_t h = info.format.pixels_per_column;

    auto scan = LidarScan(w, h, info.format.udp_profile_lidar);

    std::cerr << "Reading in scan from pcap..." << std::endl;
    get_complete_scan(handle, scan, info);

    // 1. Getting XYZ
    std::cerr << "1. Calculating 3d Points... " << std::endl;
    //! [doc-stag-cpp-xyz]
    XYZLut lut = make_xyz_lut(info);
    auto range = scan.field(sensor::ChanField::RANGE);
    auto rawPoints = cartesian(range, lut);
    const size_t cloud_size = rawPoints.size();

    std::vector<float> points(3 * cloud_size);
    for (int i = 1; i <= static_cast<int>(cloud_size); i++) {
        int ind = (i-1)*3; 
        rawPoints(i,0) = points[ind]; 
        rawPoints(i,1) = points[ind+1]; 
        rawPoints(i,2) = points[ind+2]; 
    }

    // initialize visualizer and add keyboard/mouse callbacks
    ouster::viz::PointViz viz("Viz example");
    ouster::viz::add_default_controls(viz);

    // register point cloud with the visualizer
    auto cloud = std::make_shared<ouster::viz::Cloud>(cloud_size);
    viz.add(cloud);

    // update visualizer cloud object
    cloud->set_xyz(points.data());
    //cloud->set_key(colors.data());

    // send updates to be rendered. This method is thread-safe
    viz.update();

    // run rendering loop. Will return when the window is closed
    std::cout << "Running rendering loop: press ESC to exit" << std::endl;
    viz.run();
    std::cout << "Window closed, exiting" << std::endl;

    return EXIT_SUCCESS;
}
