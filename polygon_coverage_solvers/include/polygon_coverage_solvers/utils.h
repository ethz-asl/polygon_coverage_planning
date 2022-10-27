

#ifndef POLYGON_COVERAGE_SOLVERS_UTILS_H_
#define POLYGON_COVERAGE_SOLVERS_UTILS_H_

#include <string>
#include <algorithm>
#include <cctype>

namespace polygon_coverage_planning {

// trim from start (in place)
static inline void ltrim(std::string &s) {
    s.erase(s.begin(), std::find_if(s.begin(), s.end(), [](unsigned char ch) {
        return !std::isspace(ch);
    }));
}

// trim from end (in place)
static inline void rtrim(std::string &s) {
    s.erase(std::find_if(s.rbegin(), s.rend(), [](unsigned char ch) {
        return !std::isspace(ch);
    }).base(), s.end());
}

// trim from both ends (in place)
static inline void trim(std::string &s) {
    ltrim(s);
    rtrim(s);
}

}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_SOLVERS_UTILS_H_
