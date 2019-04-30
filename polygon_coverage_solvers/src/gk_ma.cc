#include "polygon_coverage_solvers/gk_ma.h"

#include <mono/jit/jit.h>
#include <mono/metadata/assembly.h>

#include <glog/logging.h>
#include <ros/package.h>

namespace mav_coverage_planning {
namespace gk_ma {

const std::string kFile = "GkMa.exe";
const std::string kPackageName = "polygon_coverage_solvers";
const std::string kPackagePath = ros::package::getPath(kPackageName);
const std::string kCatkinPath = kPackagePath.substr(0, kPackagePath.find("/src/"));
const std::string kLibraryPath = kCatkinPath + "/devel/lib";
const std::string kExecutablePath = kLibraryPath + "/" + kFile;

bool Task::mIsSquare() const {
  for (size_t i = 0; i < m.size(); ++i) {
    if (m[i].size() != m.size()) {
      return false;
    }
  }
  return true;
}

bool Task::mIsSymmetric() const {
  if (!mIsSquare()) {
    return false;
  }
  for (size_t i = 0; i < m.size(); ++i) {
    for (size_t j = 0; j < m[i].size(); ++j) {
      if (m[i][j] != m[j][i]) {
        return false;
      }
    }
  }
  return true;
}

GkMa::GkMa() {
  domain_ = mono_jit_init(kFile.c_str());
  MonoAssembly* assembly =
      mono_domain_assembly_open(domain_, kExecutablePath.c_str());
  CHECK(assembly);

  // Load OurSolver class.
  MonoImage* image = mono_assembly_get_image(assembly);
  solver_class_ = mono_class_from_name(image, "GkMa", "OurSolver");
  CHECK(solver_class_) << "Cannot find OurSolver in assembly "
                       << mono_image_get_filename(image);
  solver_ = mono_object_new(domain_, solver_class_);  // Allocate memory.
}

GkMa::~GkMa() { mono_jit_cleanup(domain_); }

void GkMa::setSolver(const std::string& file, bool binary) {
  void* args[2];
  args[0] = mono_string_new(domain_, file.c_str());
  args[1] = &binary;

  // Find constructor method.
  void* iter = NULL;
  MonoMethod* ctor = NULL;
  while ((ctor = mono_class_get_methods(solver_class_, &iter))) {
    if (strcmp(mono_method_get_name(ctor), ".ctor") == 0) {
      // Check if the ctor takes two arguments.
      MonoMethodSignature* sig = mono_method_signature(ctor);
      if (mono_signature_get_param_count(sig) == 2) {
        break;
      }
    }
  }

  mono_runtime_invoke(ctor, solver_, args, NULL);
}

void GkMa::setSolver(const Task& task) {
  void* args[3];
  args[0] = vectorOfVectorToMonoArray(task.m);
  args[1] = vectorOfVectorToMonoArray(task.clusters);
  bool is_symmetric = task.mIsSymmetric();
  args[2] = &is_symmetric;

  // Find constructor method.
  void* iter = NULL;
  MonoMethod* ctor = NULL;
  while ((ctor = mono_class_get_methods(solver_class_, &iter))) {
    if (strcmp(mono_method_get_name(ctor), ".ctor") == 0) {
      // Check if the ctor takes two arguments.
      MonoMethodSignature* sig = mono_method_signature(ctor);
      if (mono_signature_get_param_count(sig) == 3) {
        break;
      }
    }
  }

  mono_runtime_invoke(ctor, solver_, args, NULL);
}

MonoArray* GkMa::vectorOfVectorToMonoArray(
    const std::vector<std::vector<int>>& in) const {
  MonoArray* result =
      mono_array_new(domain_, mono_get_array_class(), in.size());
  for (size_t i = 0; i < in.size(); ++i) {
    MonoArray* row =
        mono_array_new(domain_, mono_get_int32_class(), in[i].size());
    for (size_t j = 0; j < in[i].size(); ++j) {
      mono_array_set(row, int, j, in[i][j]);
    }
    mono_array_set(result, MonoArray*, i, row);
  }
  return result;
}

bool GkMa::solve() {
  // TODO(rikba): Check if solver ctor was called.
  if (!solver_) {
    LOG(ERROR) << "Solver not set.";
    return false;
  }

  // Find methods SolutionAtIndex(int index) and Solve().
  void* iter = NULL;
  MonoMethod* m = NULL;
  MonoMethod* solve = NULL;
  MonoMethod* solution_at_index = NULL;
  while ((m = mono_class_get_methods(solver_class_, &iter))) {
    if (strcmp(mono_method_get_name(m), "Solve") == 0) {
      solve = m;
    } else if (strcmp(mono_method_get_name(m), "SolutionAtIndex") == 0) {
      solution_at_index = m;
    }
  }
  if (solve == NULL || solution_at_index == NULL) {
    LOG_IF(ERROR, solve == NULL) << "Method Solve() not found.";
    LOG_IF(ERROR, solution_at_index == NULL)
        << "Method SolutionAtIndex(int index) not found.";
    return false;
  }

  // Solve()
  mono_runtime_invoke(solve, solver_, NULL, NULL);

  // Solution().
  MonoProperty* prop =
      mono_class_get_property_from_name(solver_class_, "SolutionLength");
  MonoMethod* solution_length = mono_property_get_get_method(prop);
  if (solution_length == NULL) {
    LOG(ERROR) << "Getter SolutionLength() not found.";
    return false;
  }
  MonoObject* result =
      mono_runtime_invoke(solution_length, solver_, NULL, NULL);
  int num_nodes = *(int*)mono_object_unbox(result);

  solution_.resize(num_nodes);
  for (int i = 0; i < num_nodes; ++i) {
    void* args[1];
    args[0] = &i;
    MonoObject* node =
        mono_runtime_invoke(solution_at_index, solver_, args, NULL);
    solution_[i] = *(int*)mono_object_unbox(node);
  }

  return true;
}

}  // namespace gk_ma
}  // namespace mav_coverage_planning
