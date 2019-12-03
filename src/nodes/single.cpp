#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "ontologenius_query/OntologeniusQueryService.h"
#include "ontologenius_query/OntologeniusQueryFullService.h"

#include "ontologenius_query/QueryAnalyzer.h"
#include "ontologenius_query/full/FullAnalyser.h"

#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator* onto_;

bool queryService(ontologenius_query::OntologeniusQueryService::Request& req,
                  ontologenius_query::OntologeniusQueryService::Response& res)
{
  ontologenius_query::QueryAnalyzer analyzer(onto_);
  std::vector<std::string> results = analyzer.run(req.query);

  res.values = results;
  res.error = analyzer.getError();

  return true;
}

bool fullQueryService(ontologenius_query::OntologeniusQueryFullService::Request& req,
                      ontologenius_query::OntologeniusQueryFullService::Response& res)
{
  ontologenius_query::FullAnalyser analyzer(onto_);
  std::vector<std::map<std::string, std::string>> results = analyzer.run(req.query);

  for(auto result : results)
  {
    ontologenius_query::OntologeniusQueryResponse tmp;
    for(auto r : result)
    {
      tmp.names.push_back(r.first);
      tmp.values.push_back(r.second);
    }
    res.results.push_back(tmp);
  }

  res.error = analyzer.getError();

  return true;
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_query");
  ros::NodeHandle n;

  OntologyManipulator onto(&n);
  onto_ = &onto;

  ros::ServiceServer service = n.advertiseService("ontologenius_query/query", queryService);
  ros::ServiceServer full_service = n.advertiseService("ontologenius_query/fullQuery", fullQueryService);

  ros::spin();

  return 0;
}
