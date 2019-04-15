#include <ros/ros.h>
#include <ros/package.h>

#include <gtest/gtest.h>

#include "ontoloGenius/utility/OntologyManipulator.h"

#include "ontologenius_query/OntologeniusQueryService.h"

ros::NodeHandle* n_;

bool call(std::string query, std::vector<std::string>& res)
{
  ros::ServiceClient client = n_->serviceClient<ontologenius_query::OntologeniusQueryService>("ontologenius_query/query");
  ontologenius_query::OntologeniusQueryService srv;
  srv.request.query = query;
  if(!client.call(srv))
    return false;
  else
  {
    if(srv.response.error != "")
      return false;
    else
    {
      res = srv.response.values;
      return true;
    }
  }
}

TEST(dynamic_tests, insert)
{
  bool res_bool = true;
  std::vector<std::string> res;
  ros::Rate wait(1);

  EXPECT_TRUE(call("?obj isA box", res));
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "mini_box") != res.end()) &&
              (find(res.begin(), res.end(), "little_box") != res.end()) &&
              (find(res.begin(), res.end(), "big_box") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_FALSE(call("?obj isA box blue", res));

  EXPECT_TRUE(call("?obj isA cube, ?obj isA red", res));
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "redCube") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(call("?obj isA box, ?indiv isUnder ?obj", res));
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "little_box") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(call("?obj isA object, ?indiv isUnder ?obj", res));
  res_bool = ((res.size() == 3) &&
              (find(res.begin(), res.end(), "blueCube") != res.end()) &&
              (find(res.begin(), res.end(), "little_box") != res.end()) &&
              (find(res.begin(), res.end(), "redCube") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(call("blueCube isOn ?cube", res));
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "redCube") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(call("new_box isA box", res));
  wait.sleep();
  EXPECT_TRUE(call("?obj isA box", res));
  res_bool = ((res.size() == 4) &&
              (find(res.begin(), res.end(), "mini_box") != res.end()) &&
              (find(res.begin(), res.end(), "little_box") != res.end()) &&
              (find(res.begin(), res.end(), "new_box") != res.end()) &&
              (find(res.begin(), res.end(), "big_box") != res.end()));
  EXPECT_TRUE(res_bool);

  EXPECT_TRUE(call("blueCube hasColor blue_color", res));
  wait.sleep();
  EXPECT_TRUE(call("?obj hasColor blue_color", res));
  res_bool = ((res.size() == 1) &&
              (find(res.begin(), res.end(), "blueCube") != res.end()));
  EXPECT_TRUE(res_bool);
}

int main(int argc, char** argv)
{
  ros::init(argc, argv, "ontologenius_query_tester");

  ros::NodeHandle n;
  n_ = &n;
  OntologyManipulator onto(&n);

  onto.close();

  ros::Rate wait(1);
  wait.sleep();

  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();

  return 0;
}
