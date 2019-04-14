#include <iostream>
#include <string>
#include <vector>

#include "ros/ros.h"

#include "ontologenius_query/OntologeniusQueryService.h"
#include "ontologenius_query/QueryAnalyzer.h"

#include "ontoloGenius/utility/OntologyManipulator.h"

OntologyManipulator* onto_;

bool queryService(ontologenius_query::OntologeniusQueryService::Request& req,
                  ontologenius_query::OntologeniusQueryService::Response& res)
{
  std::cout << "request : " << req.query << " on " << req.ns << std::endl;

  ontologenius_query::QueryAnalyzer analyzer(onto_);
  std::vector<std::string> results = analyzer.run(req.query);

  for(auto x : results)
    std::cout << x << std::endl;

  res.values = results;
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

  ros::spin();

  return 0;
}
