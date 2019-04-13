#include "ontologenius_query/QueryAnalyzer.h"

namespace ontologenius_query
{

  QueryAnalyzer::QueryAnalyzer(OntologiesManipulator* onto, std::string& ns)
  {
    onto->add(ns);
    onto_ = onto->get(ns);
  }

  std::vector<std::string> QueryAnalyzer::run(std::string& query)
  {
    if(onto_ == nullptr)
      return std::vector<std::string>();
      
    std::vector<std::string> sub_queries = split(query, ",");

    return sub_queries;
  }

  std::string QueryAnalyzer::getError()
  {
    return "";
  }

  std::vector<std::string> QueryAnalyzer::split(const std::string& str, const std::string& delim)
  {
    std::vector<std::string> tokens;
    size_t prev = 0, pos = 0;
    do
    {
      pos = str.find(delim, prev);
      if (pos == std::string::npos)
        pos = str.length();

      std::string token = str.substr(prev, pos-prev);

      if (!token.empty())
        tokens.push_back(token);
      prev = pos + delim.length();
    }
    while ((pos < str.length()) && (prev < str.length()));

    return tokens;
  }

} // namespace ontologenius_query
