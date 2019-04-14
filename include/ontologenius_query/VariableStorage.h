#ifndef ONTOLOGENIUS_QUERY_VARIABLESTORAGE_H
#define ONTOLOGENIUS_QUERY_VARIABLESTORAGE_H

#include <string>
#include <vector>
#include <map>

namespace ontologenius_query
{

enum type_e
{
  class_type,
  indiv_type,
  data_type,
  object_property_type,
  data_property_type,
  none_type
};

struct variables_t
{
  std::vector<std::string> values;
  type_e type;
  bool setted;
};

class VariableStorage
{
public:
  variables_t operator[](const std::string& name);
  void set(const std::string& name, const type_e& type, const std::vector<std::string>& values);

private:
  std::map<std::string, variables_t> variables_;
};

} // namespace ontologenius_query

#endif // ONTOLOGENIUS_QUERY_VARIABLESTORAGE_H
