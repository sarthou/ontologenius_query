#ifndef ONTOLOGENIUSQUERY_RESSOURCE_H
#define ONTOLOGENIUSQUERY_RESSOURCE_H

namespace ontologenius_query
{

  struct resource_t
  {
    std::string name;
    bool variable;
    bool regex;
  };

  struct triplet_t
  {
    resource_t subject;
    resource_t predicat;
    resource_t object;
  };

} // namespace ontologenius_query

#endif // ONTOLOGENIUSQUERY_RESSOURCE_H
