#include "ontologenius_query/VariableStorage.h"

namespace ontologenius_query
{

  variables_t VariableStorage::operator[](const std::string& name)
  {
    if(variables_.find(name) == variables_.end())
    {
      variables_t var;
      var.setted = false;
      var.type = none_type;
      variables_[name] = var;
    }

    return variables_[name];
  }

  void VariableStorage::set(const std::string& name, type_e& type, const std::vector<std::string>& values)
  {
    variables_t var;
    var.values = values;
    var.type = type;
    var.setted = true;
    variables_[name] = var;
  }

} // namespace ontologenius_query
