#include "ontologenius_query/QueryAnalyzer.h"

namespace ontologenius_query
{

  QueryAnalyzer::QueryAnalyzer(OntologiesManipulator* onto, std::string& ns)
  {
    /*onto->add(ns);
    onto_ = onto->get(ns);*/
    error_ = "";
  }

  QueryAnalyzer::QueryAnalyzer(OntologyManipulator* onto)
  {
    onto_ = onto;
    error_ = "";
  }

  std::vector<std::string> QueryAnalyzer::run(std::string& query)
  {
    /*if(onto_ == nullptr)
    {
      error_ = "can not create an ontology manipulator";
      return std::vector<std::string>();
    }*/

    std::vector<std::string> sub_queries = split(query, ",");

    std::vector<std::string> res;
    for(size_t i = 0; i < sub_queries.size(); i++)
    {
      if(error_ == "")
        res = solveSubQuery(sub_queries[i]);
    }

    return res;
  }

  std::string QueryAnalyzer::getError()
  {
    return error_;
  }

  std::vector<std::string> QueryAnalyzer::solveSubQuery(const std::string& subquery)
  {
    std::vector<std::string> res;
    triplet_t triplet = getTriplet(subquery);

    if(error_ == "")
    {
      std::cout << "subject :" << triplet.subject.name << ":" << triplet.subject.variable << std::endl;
      std::cout << "predicat :" << triplet.predicat.name << ":" << triplet.predicat.variable << std::endl;
      std::cout << "object :" << triplet.object.name << ":" << triplet.object.variable << std::endl;
    }

    return res;
  }

  triplet_t QueryAnalyzer::getTriplet(const std::string& subquery)
  {
    std::vector<std::string> resources = split(subquery, " ");

    size_t cpt = 0;
    triplet_t res;
    for(const auto& x : resources)
    {
      if(x != "")
      {
        resource_t resource = getResource(x);
        switch (cpt)
        {
          case 0: res.subject = resource; break;
          case 1: res.predicat = resource; break;
          case 2: res.object = resource; break;
          default: error_ = "invalid subqury format in : " + subquery;
        }
        cpt++;
      }
    }

    return res;
  }

  resource_t QueryAnalyzer::getResource(const std::string& resource)
  {
    std::string text = resource;
    removeUselessSpace(text);

    resource_t res;

    if(text[0] == '?')
    {
      res.variable = true;
      text = text.substr(1);
    }
    else
      res.variable = false;
    res.name = text;

    return res;
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

  std::vector<std::string> QueryAnalyzer::vectorUnion(const std::vector<std::string>& vect1, const std::vector<std::string>& vect2)
  {
    std::vector<std::string> res;
    for(const auto& x : vect1)
    {
      if(std::find(vect2.begin(), vect2.end(), x) != vect2.end())
        res.push_back(x);
    }
    return res;
  }

  void QueryAnalyzer::removeUselessSpace(std::string& text)
  {
    while((text[0] == ' ') && (text.size() != 0))
      text.erase(0,1);

    while((text[text.size() - 1] == ' ') && (text.size() != 0))
      text.erase(text.size() - 1,1);
  }

} // namespace ontologenius_query
