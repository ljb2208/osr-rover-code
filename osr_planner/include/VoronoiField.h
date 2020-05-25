#ifndef VORONOI_FIELD_H
#define VORONOI_FIELD_H

#include <iostream>
#include <vector>
#include <boost/polygon/voronoi.hpp>
#include <boost/polygon/polygon.hpp>

#include "Settings.h"

using namespace std;
using namespace boost::polygon;

namespace OsrPlanner {
    
    typedef double coordinate_type;
    typedef point_data<coordinate_type> point_type;
    typedef segment_data<coordinate_type> segment_type;
    typedef rectangle_data<coordinate_type> rect_type;
    typedef voronoi_builder<int> VB;
    typedef voronoi_diagram<coordinate_type> VD;
    typedef VD::cell_type cell_type;
    typedef VD::cell_type::source_index_type source_index_type;
    typedef VD::cell_type::source_category_type source_category_type;
    typedef VD::edge_type edge_type;
    typedef VD::cell_container_type cell_container_type;
    typedef VD::cell_container_type vertex_container_type;
    typedef VD::edge_container_type edge_container_type;
    typedef VD::const_cell_iterator const_cell_iterator;
    typedef VD::const_vertex_iterator const_vertex_iterator;
    typedef VD::const_edge_iterator const_edge_iterator;
//###################################################
//                                            VECTOR2
//###################################################
/// A class describing a simple 2D vector
    class VoronoiCell {
        public:
            VoronoiCell(float x, float y, int type) {
                this->x = x;
                this->y = y;
                this->type = type;
            };

            float x;
            float y;
            int type;
    };


    class VoronoiField {
        public:
            VoronoiField() {};

            void setSettings(Settings* settings);
            void setMap(vector<vector<bool>> map, int width, int height);

            

        private:
            

            vector<vector<bool>> map;
            vector<vector<VoronoiCell>> vMap;
            int width;
            int height;
            Settings* settings;

            void buildVoronoiDiagram();
            void constructBRect();
            void updateBRect(const point_type& point);
            void clipInfiniteEdge(const edge_type& edge, vector<point_type>* clippedEdge);
            point_type retrievePoint(const cell_type& cell); 
            segment_type retrieveSegment(const cell_type& cell);
            void sampleCurvedEdge(const edge_type& edge, std::vector<point_type>* sampledEdge);

            vector<point_type> pointData;
            vector<segment_type> segmentData;
            rect_type bRect;
            bool bRectInitialized;
            point_type shift;

            void visualize();


    };
}
#endif // VORONOI_FIELD_H
