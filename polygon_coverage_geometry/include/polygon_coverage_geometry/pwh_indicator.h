#ifndef POLYGON_COVERAGE_GEOMETRY_PWH_INDICATOR_H_
#define POLYGON_COVERAGE_GEOMETRY_PWH_INDICATOR_H_


#include "polygon_coverage_geometry/cgal_definitions.h"

namespace polygon_coverage_planning {


// This class is used to indicate the vertex or edge in the polygon with holes data structure.
class PWHIndicator {
public:
    PWHIndicator(const PolygonWithHoles * pwh_ptr, size_t polygon_index, size_t vertex_index) : pwh_ptr_(pwh_ptr),
                                                                                                polygon_index_(polygon_index), vertex_index_(vertex_index) {}

    PWHIndicator next() const {
        const PolygonWithHoles & pwh_ = *pwh_ptr_;
        auto next_indicator = *this;
        size_t max_vertex_size = 0;
        if(polygon_index_ == 0) {
            max_vertex_size = pwh_.outer_boundary().size() - 1;
        } else if(polygon_index_ <= pwh_.holes().size()) {
            max_vertex_size = pwh_.holes()[polygon_index_-1].size() - 1;
        } else {
            throw std::out_of_range("polygon_index");
        }
        if(next_indicator.vertex_index_ >= max_vertex_size) {
            next_indicator.vertex_index_ = 0;
        } else {
            next_indicator.vertex_index_++;
        }
        return next_indicator;
    }

    PWHIndicator prev() const {
        const PolygonWithHoles & pwh_ = *pwh_ptr_;
        auto pre_indicator = *this;
        size_t max_vertex_size = 0;
        if(polygon_index_ == 0) {
            max_vertex_size = pwh_.outer_boundary().size() - 1;
        } else if(polygon_index_ <= pwh_.holes().size()) {
            max_vertex_size = pwh_.holes()[polygon_index_-1].size() - 1;
        } else {
            throw std::out_of_range("polygon_index");
        }
        if(pre_indicator.vertex_index_ <= 0) {
            pre_indicator.vertex_index_ = max_vertex_size;
        } else {
            pre_indicator.vertex_index_--;
        }
        return pre_indicator;
    }

    const Polygon_2 & getPolygon() const {
        const PolygonWithHoles & pwh_ = *pwh_ptr_;
        if(polygon_index_ == 0) {
            return pwh_.outer_boundary();
        } else if(polygon_index_ <= pwh_.holes().size()){
            return pwh_.holes()[polygon_index_ - 1];
        } else {
            throw std::out_of_range("polygon_index");
        }
    }

    VertexConstCirculator getVertexCirculator() const {
        return getPolygon().vertices_circulator() + vertex_index_;
    }

    EdgeConstCirculator getEdgeCirculator() const {
        return getPolygon().edges_circulator() + vertex_index_;
    }


    Point_2 getVertex() const {
        return *(getVertexCirculator());
    }

    Segment_2 getEdge() const {
        return *(getEdgeCirculator());
    }

    bool isInSamePolygon(const PWHIndicator & other) const {
        return other.polygon_index_ == polygon_index_;
    }

    bool isNext(const PWHIndicator & other) const {
        if(!isInSamePolygon(other)) {
            return false;
        }
        return other.getVertexCirculator() - getVertexCirculator() == 1;
    }

    bool isPrev(const PWHIndicator & other) const {
        if(!isInSamePolygon(other)) {
            return false;
        }
        return getVertexCirculator() - other.getVertexCirculator() == 1;
    }

    bool operator==(PWHIndicator & other) {
      return pwh_ptr_ == other.pwh_ptr_ && polygon_index_ == other.polygon_index_ && vertex_index_ == other.vertex_index_;
    }

private:
    const PolygonWithHoles * pwh_ptr_;
    size_t polygon_index_;
    size_t vertex_index_;
};




} // namespace polygon_coverage_planning




#endif  // POLYGON_COVERAGE_GEOMETRY_PWH_INDICATOR_H_
