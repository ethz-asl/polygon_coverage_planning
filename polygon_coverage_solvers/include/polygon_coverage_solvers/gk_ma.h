#ifndef POLYGON_COVERAGE_SOLVERS_GK_MA_H_
#define POLYGON_COVERAGE_SOLVERS_GK_MA_H_

#include <string>
#include <vector>

#include <mono/metadata/object.h>

// Interfaces with the GK MA GTSP solver.
namespace polygon_coverage_planning {
namespace gk_ma {
struct Task {
  Task(const std::vector<std::vector<int>>& m,
       const std::vector<std::vector<int>>& clusters)
      : m(m), clusters(clusters) {}
  bool mIsSymmetric() const;
  bool mIsSquare() const;
  std::vector<std::vector<int>> m;
  std::vector<std::vector<int>> clusters;
};

// References GkMa.exe. Singleton, because it may only be referenced once during
// runtime.
// https://stackoverflow.com/questions/1008019/c-singleton-design-pattern
class GkMa {
 public:
  inline static GkMa& getInstance() {
    static GkMa instance;
    return instance;
  }
  GkMa(GkMa const&) = delete;
  void operator=(GkMa const&) = delete;

  void setSolver(const std::string& file, bool binary);
  void setSolver(const Task& task);
  bool solve();
  inline std::vector<int> getSolution() const { return solution_; }

 private:
  GkMa();
  ~GkMa();

  MonoArray* vectorOfVectorToMonoArray(
      const std::vector<std::vector<int>>& in) const;

  MonoDomain* domain_;
  MonoObject* solver_;
  MonoClass* solver_class_;

  std::vector<int> solution_;
};
}  // namespace gk_ma
}  // namespace polygon_coverage_planning

#endif  // POLYGON_COVERAGE_SOLVERS_GK_MA_H_
