#include "VoronoiField.h"
#include <iostream>

using namespace OsrPlanner;
using namespace boost::polygon;

void VoronoiField::setSettings(Settings* settings)
{
    this->settings = settings;
}

void VoronoiField::setMap(vector<vector<bool>> map, int width, int height)
{
    this->map = map;
    this->width = width;
    this->height = height;

    buildVoronoiDiagram();
}

void VoronoiField::buildVoronoiDiagram()
{
    pointData.clear();
    segmentData.clear();

    for (int x=0; x < width; x++)
    {
        vMap.push_back(vector<VoronoiCell>());
        for (int y=0; y < height; y++)
        {
            if (map[x][y])
            {
                VoronoiCell vc = VoronoiCell(x, y, 1);
                vMap[x].push_back(vc);
                point_type pt = point_type(x, y);
                pointData.push_back(pt);
                updateBRect(pt);
            }
            else
            {
                VoronoiCell vc = VoronoiCell(x, y, 0);
                vMap[x].push_back(vc);
            }
            
        }
    }

    constructBRect();

    voronoi_diagram<double> vd;
    construct_voronoi(pointData.begin(), pointData.end(), &vd);

    cout << "Num cells: " << vd.num_cells() << "\n";
    cout << "Num Edges: " << vd.num_edges() << "\n";
    cout << "Num Verticies: " << vd.num_vertices() << "\n";

    int result = 0;

    vector<point_type> samples;

    // iterate cells
    for (const_edge_iterator it = vd.edges().begin(); it != vd.edges().end(); ++it)
    {
        if (!it->is_primary())
            continue;

        if (!it->is_finite()) {
            clipInfiniteEdge(*it, &samples);
        } else {
            point_type vertex0(it->vertex0()->x(), it->vertex0()->y());
            samples.push_back(vertex0);
            point_type vertex1(it->vertex1()->x(), it->vertex1()->y());
            samples.push_back(vertex1);

            if (it->is_curved()) {
                cout << "Curved edge \n";                
        }
      }
    }

    for (size_t i=0; i < samples.size(); i+=2)
    {
        point_type pt1 = samples[i];
        point_type pt2 = samples[i+1];
        
        float minX = min(pt1.x(), pt2.x());
        float maxX = max(pt1.x(), pt2.x());

        float minY = min(pt1.y(), pt2.y());
        float maxY = max(pt1.y(), pt2.y());

        minX = max(0.0f, minX);
        maxX = min((float)width - 1, maxX);

        minY = max(0.0f, minY);
        maxY = min((float)height - 1, maxY);

        for (int x=minX; x <= maxX; x++)
        {
            for (int y=minY; y <= maxY; y++)
            {
                // VoronoiCell c = VoronoiCell(x, y, 2);
                // vMap[x][y] = c;
            }
        }
    }

    visualize();
    
}

void VoronoiField::constructBRect() {
    double side = (std::max)(xh(bRect) - xl(bRect), yh(bRect) - yl(bRect));
    center(shift, bRect);
    set_points(bRect, shift, shift);
    bloat(bRect, side * 1.2);
  }

void VoronoiField::updateBRect(const point_type& point)
{
    if (bRectInitialized)
        encompass(bRect, point);
    else
    {
        set_points(bRect, point, point);
        bRectInitialized = true;
    }
}

void VoronoiField::clipInfiniteEdge(const edge_type& edge, vector<point_type>* clippedEdge)
{
    const cell_type& cell1 = *edge.cell();
    const cell_type& cell2 = *edge.twin()->cell();
    point_type origin, direction;

    // Infinite edges could not be created by two segment sites.
    if (cell1.contains_point() && cell2.contains_point()) {
      point_type p1 = retrievePoint(cell1);
      point_type p2 = retrievePoint(cell2);
      origin.x((p1.x() + p2.x()) * 0.5);
      origin.y((p1.y() + p2.y()) * 0.5);
      direction.x(p1.y() - p2.y());
      direction.y(p2.x() - p1.x());
    } else {
      origin = cell1.contains_segment() ?
          retrievePoint(cell2) :
          retrievePoint(cell1);
      segment_type segment = cell1.contains_segment() ?
          retrieveSegment(cell1) :
          retrieveSegment(cell2);
      coordinate_type dx = high(segment).x() - low(segment).x();
      coordinate_type dy = high(segment).y() - low(segment).y();
      if ((low(segment) == origin) ^ cell1.contains_point()) {
        direction.x(dy);
        direction.y(-dx);
      } else {
        direction.x(-dy);
        direction.y(dx);
      }
    }
    coordinate_type side = xh(bRect) - xl(bRect);
    coordinate_type koef =
        side / (std::max)(fabs(direction.x()), fabs(direction.y()));
    if (edge.vertex0() == NULL) {
      clippedEdge->push_back(point_type(
          origin.x() - direction.x() * koef,
          origin.y() - direction.y() * koef));
    } else {
      clippedEdge->push_back(
          point_type(edge.vertex0()->x(), edge.vertex0()->y()));
    }
    if (edge.vertex1() == NULL) {
      clippedEdge->push_back(point_type(
          origin.x() + direction.x() * koef,
          origin.y() + direction.y() * koef));
    } else {
      clippedEdge->push_back(
          point_type(edge.vertex1()->x(), edge.vertex1()->y()));
    }
}

point_type VoronoiField::retrievePoint(const cell_type& cell) 
{
    source_index_type index = cell.source_index();
    source_category_type category = cell.source_category();
    if (category == SOURCE_CATEGORY_SINGLE_POINT) {
      return pointData[index];
    }
    index -= pointData.size();
    if (category == SOURCE_CATEGORY_SEGMENT_START_POINT) {
      return low(segmentData[index]);
    } else {
      return high(segmentData[index]);
    }
 }

 segment_type VoronoiField::retrieveSegment(const cell_type& cell) 
 {
    source_index_type index = cell.source_index() - pointData.size();
    return segmentData[index];
}

void VoronoiField::sampleCurvedEdge(const edge_type& edge, std::vector<point_type>* sampledEdge) 
{
    coordinate_type max_dist = 1E-3 * (xh(bRect) - xl(bRect));
    point_type point = edge.cell()->contains_point() ?
        retrievePoint(*edge.cell()) :
        retrievePoint(*edge.twin()->cell());
    segment_type segment = edge.cell()->contains_point() ?
        retrieveSegment(*edge.twin()->cell()) :
        retrieveSegment(*edge.cell());    
  }

  void VoronoiField::visualize() {
    // write pgm files

    FILE* F = fopen("vfield.pgm", "w");
    if (!F) {
        std::cerr << "could not open 'result.pgm' for writing!\n";
        return;
    }
    fprintf(F, "P6\n");
    fprintf(F, "%d %d 255\n", width, height);

    for(int y = height; y >=0; y--){      
        for(int x = 0; x<width; x++){	
            unsigned char c = 0;
            // if (vMap[x][y].type == 2) {
            //     fputc( , F );
            //     fputc( 0, F );
            //     fputc( 0, F );
            // }
            if (vMap[x][y].type == 1)
            {
                fputc( 0, F );
                fputc( 0, F );
                fputc( 0, F );
            }
            else
            {
                fputc( 80, F );
                fputc( 80, F );
                fputc( 80, F );
            }
            
        }
    }
    fclose(F);
}
