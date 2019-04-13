#ifndef ONTOLOGENIUS_QUERY_QUERYANALYZER_H
#define ONTOLOGENIUS_QUERY_QUERYANALYZER_H

#include <string>
#include <vector>

#include "ontologenius_query/VariableStorage.h"

#include "ontoloGenius/utility/OntologiesManipulator.h"
#include "ontoloGenius/utility/OntologyManipulator.h"

namespace ontologenius_query
{

class QueryAnalyzer
{
public:
  QueryAnalyzer(OntologiesManipulator* onto, std::string& ns);

  std::vector<std::string> run(std::string& query);
  std::string getError();

private:
   OntologyManipulator* onto_;
   VariableStorage variables_;

   std::vector<std::string> split(const std::string& str, const std::string& delim);
};

} // namespace ontologenius_query

#endif // ONTOLOGENIUS_QUERY_QUERYANALYZER_H
