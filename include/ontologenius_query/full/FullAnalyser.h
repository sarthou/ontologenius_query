#ifndef ONTOLOGENIUSQUERY_FULLANALYSER_H
#define ONTOLOGENIUSQUERY_FULLANALYSER_H

#include "ontologenius/OntologiesManipulator.h"
#include "ontologenius/OntologyManipulator.h"

#include "ontologenius_query/ressource.h"

namespace ontologenius_query
{

class FullAnalyser
{
public:
  FullAnalyser(OntologiesManipulator* onto, std::string& ns);
  FullAnalyser(OntologyManipulator* onto);

  std::vector<std::map<std::string, std::string>> run(const std::string& query);

  std::string getError() { return error_; }

private:
   OntologyManipulator* onto_;
   std::string error_;

   std::vector<std::map<std::string, std::string>> resolve(std::vector<triplet_t> query, std::map<std::string, std::string> accu = {});
   void resolveSubQuery(triplet_t triplet, const std::map<std::string, std::string>& accu, std::string& var_name, std::vector<std::string>& values);

   std::vector<std::string> getOn(const triplet_t& triplet, const std::string& selector = "");
   std::vector<std::string> getFrom(const triplet_t& triplet, const std::string& selector = "");
   std::vector<std::string> getUp(const triplet_t& triplet, const std::string& selector = "");
   std::vector<std::string> getType(const triplet_t& triplet, const std::string& selector = "");
   std::vector<std::string> find(const triplet_t& triplet, const std::string& selector = "");
   std::vector<std::string> getName(const triplet_t& triplet, const std::string& selector = "");

   triplet_t getTriplet(const std::string& subquery);
   resource_t getResource(const std::string& resource);
   std::string toString(triplet_t triplet);

   std::vector<std::string> split(const std::string& str, const std::string& delim);
   void removeUselessSpace(std::string& text);
};

} // namespace ontologenius_query

#endif // ONTOLOGENIUSQUERY_FULLANALYSER_H
