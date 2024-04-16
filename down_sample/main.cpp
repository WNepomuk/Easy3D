/********************************************************************
 * Copyright (C) 2015 Liangliang Nan <liangliang.nan@gmail.com>
 * https://3d.bk.tudelft.nl/liangliang/
 *
 * This file is part of Easy3D. If it is useful in your research/work,
 * I would be grateful if you show your appreciation by citing it:
 * ------------------------------------------------------------------
 *      Liangliang Nan.
 *      Easy3D: a lightweight, easy-to-use, and efficient C++ library
 *      for processing and rendering 3D data.
 *      Journal of Open Source Software, 6(64), 3255, 2021.
 * ------------------------------------------------------------------
 *
 * Easy3D is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License Version 3
 * as published by the Free Software Foundation.
 *
 * Easy3D is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program. If not, see <http://www.gnu.org/licenses/>.
 ********************************************************************/

#include <easy3d/algo/point_cloud_ransac.h>
#include <easy3d/util/initializer.h>
#include <easy3d/algo/point_cloud_normals.h>
#include <easy3d/fileio/point_cloud_io.h>
#include <easy3d/algo/point_cloud_simplification.h>
#include <easy3d/util/logging.h>


using namespace easy3d;

// This example shows how to
//		- extract planes from a point cloud using RANSAC.


int main(int argc, char **argv) {

    logging::initialize();

    // Parse command line arguments
    if (argc < 8) {
        std::cout << "To few arguments, expected 7, got " << argc - 1 << std::endl;
    }

    if (argc > 8) {
        std::cout << "To many arguments, expected 7, got " << argc - 1 << std::endl;
    }

    int min_support = std::stoi(argv[1]);
    float dist_threshold = std::stof(argv[2]);
    float bitmap_resolution = std::stof(argv[3]);
    float normal_threshold = std::stof(argv[4]);
    float overlook_probability = std::stof(argv[5]);

    const std::string file = argv[6];
    const std::string out_file = argv[7];

    //const std::string file = "/Users/nepomukwolf/dev/tum/PC2LCA/Easy3D/resources/data/polyhedron.bin";
    //const std::string out_file = "/Users/nepomukwolf/dev/tum/PC2LCA/data/tests.vg";
    //const std::string file = "/Users/nepomukwolf/dev/tum/PC2LCA/data/cleaned_0502_scaled_subsampled_500000.las";
    PointCloud* cloud = PointCloudIO::load(file);


    if (!cloud)
        return 1;

    // Estimate point cloud normals
    PointCloudNormals::estimate(cloud);


    auto normals = cloud->get_vertex_property<vec3>("v:normal");

    if (!normals) {
        std::cerr << "Plane extraction using RANSAC requires normal information but it is not available" << std::endl;
        return 1;
    }

    PrimitivesRansac algo;
    algo.add_primitive_type(PrimitivesRansac::PLANE);

    // you can try different parameters of RANSAC (usually you don't need to tune them)
    //const int num = algo.detect(cloud, 1000, 0.001f, 0.02f, 0.8f, 0.001f);
    const int num = algo.detect(cloud, min_support, dist_threshold, bitmap_resolution, normal_threshold, overlook_probability);

    if (num > 0) {
        std::cout << num << " primitives extracted" << std::endl;
    }

    // Export the point cloud as bvg
    PointCloudIO::save(out_file, cloud);
    std::cout << "File saved to: " << out_file << std::endl;
    return 0;
}
