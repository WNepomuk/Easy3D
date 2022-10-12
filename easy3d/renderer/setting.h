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

#ifndef EASY3D_RENDERER_SETTING_H
#define EASY3D_RENDERER_SETTING_H


#include <easy3d/core/types.h>


namespace easy3d {

    class ClippingPlane;

    /**
     * \brief Default parameters used for initializing rendering and UI
     * \namespace easy3d::setting
     */
    namespace setting {

        /**
         * Enable initializing settings for Easy3D on program start up. This feature is disabled by default.
         * By calling this function with a valid file name, the rendering parameters will be loaded from the
         * setting file on program startup.
		 * @param setting_file A string specifying the name of setting file.
		 *		- If \p setting_file is "default": create a setting file with a title in the form "AppName.ini" next to
		 *        the executable file.
		 *      - If \p setting_file is another non-empty string: a setting file with the same name will be created and 
		 *		  setting parameters will be written to this file.
         */
        void initialize(const std::string &setting_file = "default");

        /// Save the setting (i.e., rendering parameters) to a file.
        bool save(const std::string &filename);

        /// Load the setting (i.e., rendering parameters) from a file.
        bool load(const std::string &filename);

        /// background color of the viewer
        extern vec4 background_color;
        /// highlight: color for highlighted/selected primitives
        extern vec4 highlight_color;
        /// lighting
        extern vec4 light_position;  // light position defined in camera coordinate system
        /// material
        extern vec4 material_ambient;
        extern vec4 material_specular;
        extern float material_shininess;    // specular power

        /// effect
        extern float effect_ssao_radius;
        extern float effect_ssao_intensity;
        extern float effect_ssao_bias;
        extern float effect_ssao_sharpness;
        extern bool effect_edl_enabled;
        extern bool effect_transparency_enabled;
        extern bool effect_shadow_enabled;
        extern float effect_shadow_light_distance;
        extern float effect_shadow_softness;
        extern float effect_shadow_darkness;

        /// points drawable
        extern bool points_drawable_two_side_lighting;
        extern bool points_drawable_distinct_backside_color;
        extern vec4 points_drawable_backside_color;
        // lines drawable
        extern bool lines_drawable_two_side_lighting;
        extern bool lines_drawable_distinct_backside_color;
        extern vec4 lines_drawable_backside_color;
        // triangles drawable
        extern bool triangles_drawable_two_side_lighting;
        extern bool triangles_drawable_distinct_backside_color;
        extern vec4 triangles_drawable_backside_color;

        /// point cloud
        extern bool point_cloud_vertices_visible;
        extern vec4 point_cloud_vertices_color;
        extern bool point_cloud_vertices_impostors;
        extern float point_cloud_vertices_size;

        /// surface mesh - surface
        extern bool surface_mesh_faces_phong_shading;
        extern bool surface_mesh_faces_visible;
        extern vec4 surface_mesh_faces_color;
        extern float surface_mesh_faces_opacity;
        /// surface mesh - vertices
        extern bool surface_mesh_vertices_visible;
        extern vec4 surface_mesh_vertices_color;
        extern bool surface_mesh_vertices_imposters;
        extern float surface_mesh_vertices_size;
        /// surface mesh - edges
        extern bool surface_mesh_edges_visible;
        extern vec4 surface_mesh_edges_color;
        extern bool surface_mesh_edges_imposters;
        extern float surface_mesh_edges_size;
        /// surface mesh - borders
        extern bool surface_mesh_borders_visible;
        extern vec4 surface_mesh_borders_color;
        extern bool surface_mesh_borders_imposters;
        extern float surface_mesh_borders_size;

        /// graph: vertices
        extern bool graph_vertices_visible;
        extern vec4 graph_vertices_color;
        extern bool graph_vertices_imposters;
        extern float graph_vertices_size;
        /// graph: edges
        extern bool graph_edges_visible;
        extern vec4 graph_edges_color;
        extern bool graph_edges_imposters;
        extern float graph_edges_size;

        /// polyhedral mesh - surface
        extern bool poly_mesh_faces_visible;
        extern vec4 poly_mesh_faces_color;
        /// polyhedral mesh - vertices
        extern bool poly_mesh_vertices_visible;
        extern vec4 poly_mesh_vertices_color;
        extern bool poly_mesh_vertices_imposters;
        extern float poly_mesh_vertices_size;
        /// polyhedral mesh - edges
        extern bool poly_mesh_edges_visible;
        extern vec4 poly_mesh_edges_color;
        extern bool poly_mesh_edges_imposters;
        extern float poly_mesh_edges_size;

        /// clipping plane
        extern ClippingPlane *clipping_plane;
        extern vec4 clipping_plane_color;


    } // namespace setting

} // namespace easy3d


#endif // EASY3D_RENDERER_SETTING_H
