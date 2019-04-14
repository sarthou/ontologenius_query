#ifndef ONTOLOGENIUS_QUERY_QUERYANALYZER_H
#define ONTOLOGENIUS_QUERY_QUERYANALYZER_H

#include <string>
#include <vector>

#include "ontologenius_query/VariableStorage.h"

#include "ontoloGenius/utility/OntologiesManipulator.h"
#include "ontoloGenius/utility/OntologyManipulator.h"

namespace ontologenius_query
{

struct resource_t
{
  std::string name;
  bool variable;
};

struct triplet_t
{
  resource_t subject;
  resource_t predicat;
  resource_t object;
};

class QueryAnalyzer
{
public:
  QueryAnalyzer(OntologiesManipulator* onto, std::string& ns);
  QueryAnalyzer(OntologyManipulator* onto);

  std::vector<std::string> run(std::string& query);
  std::string getError();

private:
   OntologyManipulator* onto_;
   VariableStorage variables_;
   std::string error_;

   std::vector<std::string> solveSubQuery(const std::string& subquery);
   triplet_t getTriplet(const std::string& subquery);
   resource_t getResource(const std::string& resource);

   std::vector<std::string> split(const std::string& str, const std::string& delim);
   std::vector<std::string> vectorUnion(const std::vector<std::string>& vect1, const std::vector<std::string>& vect2);
   void removeUselessSpace(std::string& text);
};

} // namespace ontologenius_query

#endif // ONTOLOGENIUS_QUERY_QUERYANALYZER_H
