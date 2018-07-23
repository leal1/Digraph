// Digraph.hpp
//
// VertexInfo and EdgeInfo Constraints:
// The constraint is that the template arguments vertexInfo and EdgeInfo
// cannot be void. Also the template arguments vertexInfo and EdgeInfo must
// be able to be printed out if using program in main
//
// ICS 46 Winter 2018
// Project #5: Rock and Roll Stops the Traffic
//
// This header file declares a class template called Digraph, which is
// intended to implement a generic directed graph.  The implementation
// uses the adjacency lists technique, so each vertex stores a linked
// list of its outgoing edges.
//
// Along with the Digraph class template is a class DigraphException
// and a couple of utility structs that aren't generally useful outside
// of this header file.
//
// In general, directed graphs are all the same, except in the sense
// that they store different kinds of information about each vertex and
// about each edge; these two types are the type parameters to the
// Digraph class template.

#ifndef DIGRAPH_HPP
#define DIGRAPH_HPP

#include <functional>
#include <list>
#include <map>
#include <utility>
#include <vector>
#include <iterator>
#include <algorithm>
#include <string>
#include <limits>
#include <queue>



// DigraphExceptions are thrown from some of the member functions in the
// Digraph class template, so that exception is declared here, so it
// will be available to any code that includes this header file.

class DigraphException
{
public:
    DigraphException(const std::string& reason);

    std::string reason() const;

private:
    std::string reason_;
};


inline DigraphException::DigraphException(const std::string& reason)
    : reason_{reason}
{
}


inline std::string DigraphException::reason() const
{
    return reason_;
}



// A DigraphEdge lists a "from vertex" (the number of the vertex from which
// the edge points), a "to vertex" (the number of the vertex to which the
// edge points), and an EdgeInfo object.  Because different kinds of Digraphs
// store different kinds of edge information, DigraphEdge is a struct template.

template <typename EdgeInfo>
struct DigraphEdge
{
    int fromVertex;
    int toVertex;
    EdgeInfo einfo;
};



// A DigraphVertex includes two things: a VertexInfo object and a list of
// its outgoing edges.  Because different kinds of Digraphs store different
// kinds of vertex and edge information, DigraphVertex is a struct template.

template <typename VertexInfo, typename EdgeInfo>
struct DigraphVertex
{
    VertexInfo vinfo;
    std::list<DigraphEdge<EdgeInfo>> edges;
};



// Digraph is a class template that represents a directed graph implemented
// using adjacency lists.  It takes two type parameters:
//
// * VertexInfo, which specifies the kind of object stored for each vertex
// * EdgeInfo, which specifies the kind of object stored for each edge
//
// You'll need to implement the member functions declared here; each has a
// comment detailing how it is intended to work.
//
// Each vertex in a Digraph is identified uniquely by a "vertex number".
// Vertex numbers are not necessarily sequential and they are not necessarily
// zero- or one-based.

template <typename VertexInfo, typename EdgeInfo>
class Digraph
{
public:
    // The default constructor initializes a new, empty Digraph so that
    // contains no vertices and no edges.
    Digraph();

    // The copy constructor initializes a new Digraph to be a deep copy
    // of another one (i.e., any change to the copy will not affect the
    // original).
    Digraph(const Digraph& d);

    // The move constructor initializes a new Digraph from an expiring one.
    Digraph(Digraph&& d) noexcept;

    // The destructor deallocates any memory associated with the Digraph.
    ~Digraph() noexcept;

    // The assignment operator assigns the contents of the given Digraph
    // into "this" Digraph, with "this" Digraph becoming a separate, deep
    // copy of the contents of the given one (i.e., any change made to
    // "this" Digraph afterward will not affect the other).
    Digraph& operator=(const Digraph& d);

    // The move assignment operator assigns the contents of an expiring
    // Digraph into "this" Digraph.
    Digraph& operator=(Digraph&& d) noexcept;

    // vertices() returns a std::vector containing the vertex numbers of
    // every vertex in this Digraph.
    std::vector<int> vertices() const;

    // edges() returns a std::vector of std::pairs, in which each pair
    // contains the "from" and "to" vertex numbers of an edge in this
    // Digraph.  All edges are included in the std::vector.
    std::vector<std::pair<int, int>> edges() const;

    // This overload of edges() returns a std::vector of std::pairs, in
    // which each pair contains the "from" and "to" vertex numbers of an
    // edge in this Digraph.  Only edges outgoing from the given vertex
    // number are included in the std::vector.  If the given vertex does
    // not exist, a DigraphException is thrown instead.
    std::vector<std::pair<int, int>> edges(int vertex) const;

    // vertexInfo() returns the VertexInfo object belonging to the vertex
    // with the given vertex number.  If that vertex does not exist, a
    // DigraphException is thrown instead.
    VertexInfo vertexInfo(int vertex) const;

    // edgeInfo() returns the EdgeInfo object belonging to the edge
    // with the given "from" and "to" vertex numbers.  If either of those
    // vertices does not exist *or* if the edge does not exist, a
    // DigraphException is thrown instead.
    EdgeInfo edgeInfo(int fromVertex, int toVertex) const;

    // addVertex() adds a vertex to the Digraph with the given vertex
    // number and VertexInfo object.  If there is already a vertex in
    // the graph with the given vertex number, a DigraphException is
    // thrown instead.
    void addVertex(int vertex, const VertexInfo& vinfo);

    // addEdge() adds an edge to the Digraph pointing from the given
    // "from" vertex number to the given "to" vertex number, and
    // associates with the given EdgeInfo object with it.  If one
    // of the vertices does not exist *or* if the same edge is already
    // present in the graph, a DigraphException is thrown instead.
    void addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo);

    // removeVertex() removes the vertex (and all of its incoming
    // and outgoing edges) with the given vertex number from the
    // Digraph.  If the vertex does not exist already, a DigraphException
    // is thrown instead.
    void removeVertex(int vertex);

    // removeEdge() removes the edge pointing from the given "from"
    // vertex number to the given "to" vertex number from the Digraph.
    // If either of these vertices does not exist *or* if the edge
    // is not already present in the graph, a DigraphException is
    // thrown instead.
    void removeEdge(int fromVertex, int toVertex);

    // vertexCount() returns the number of vertices in the graph.
    int vertexCount() const noexcept;

    // edgeCount() returns the total number of edges in the graph,
    // counting edges outgoing from all vertices.
    int edgeCount() const noexcept;

    // This overload of edgeCount() returns the number of edges in
    // the graph that are outgoing from the given vertex number.
    // If the given vertex does not exist, a DigraphException is
    // thrown instead.
    int edgeCount(int vertex) const;

    // isStronglyConnected() returns true if the Digraph is strongly
    // connected (i.e., every vertex is reachable from every other),
    // false otherwise.
    bool isStronglyConnected() const;

    // findShortestPaths() takes a start vertex number and a function
    // that takes an EdgeInfo object and determines an edge weight.
    // It uses Dijkstra's Shortest Path Algorithm to determine the
    // shortest paths from the start vertex to every other vertex
    // in the graph.  The result is returned as a std::map<int, int>
    // where the keys are vertex numbers and the value associated
    // with each key k is the precedessor of that vertex chosen by
    // the algorithm.  For any vertex without a predecessor (e.g.,
    // a vertex that was never reached, or the start vertex itself),
    // the value is simply a copy of the key.
    std::map<int, int> findShortestPaths(
        int startVertex,
        std::function<double(const EdgeInfo&)> edgeWeightFunc) const;


private:
    // Add whatever member variables you think you need here.  One
    // possibility is a std::map where the keys are vertex numbers
    // and the values are DigraphVertex<VertexInfo, EdgeInfo> objects.

    std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> Map;

    void DFTr( std::map<int, DigraphVertex<VertexInfo, EdgeInfo>> Map,
              std::map<int,bool>&visited,
                int vertex, int & visitCount) const;



    // You can also feel free to add any additional member functions
    // you'd like (public or private), so long as you don't remove or
    // change the signatures of the ones that already exist.
};



// You'll need to implement the member functions below.  There's enough
// code in place to make them compile, but they'll all need to do the
// correct thing instead.

template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph()
{
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(const Digraph& d)
{
  Map = d.Map;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::Digraph(Digraph&& d) noexcept
{
  std::swap(Map, d.Map);
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>::~Digraph() noexcept
{
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(const Digraph& d)
{
  Map = d.Map;
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
Digraph<VertexInfo, EdgeInfo>& Digraph<VertexInfo, EdgeInfo>::operator=(Digraph&& d) noexcept
{
  std::swap(Map,d.Map);
    return *this;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<int> Digraph<VertexInfo, EdgeInfo>::vertices() const
{
  std::vector<int> vertices;
  for(auto element: Map)
  {
    vertices.push_back(element.first);
  }
  return vertices;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges() const
{
    std::vector<std::pair<int, int>> edge_vec;
    for(auto element : Map)
    {
      for(auto edges : element.second.edges)
      {
        edge_vec.push_back(std::make_pair(edges.fromVertex, edges.toVertex));
      }
    }
    return edge_vec;
}


template <typename VertexInfo, typename EdgeInfo>
std::vector<std::pair<int, int>> Digraph<VertexInfo, EdgeInfo>::edges(int vertex) const
{
  if(Map.find(vertex) == Map.end())
  {
    throw DigraphException("Vertex Does Not Exist");
  }
  std::vector<std::pair<int, int>> edge_vec;
  for(auto element: Map)
  {
    for(auto edge : element.second.edges)
    {
      if(edge.fromVertex == vertex)
      {
        edge_vec.push_back(std::make_pair(edge.fromVertex, edge.toVertex));
      }
    }
  }

  return edge_vec;
}


template <typename VertexInfo, typename EdgeInfo>
VertexInfo Digraph<VertexInfo, EdgeInfo>::vertexInfo(int vertex) const
{
    if(Map.find(vertex) == Map.end())
    {
      throw DigraphException("Vertex Does Not Exist");
    }
    return ((Map.find(vertex))->second).vinfo;
}


template <typename VertexInfo, typename EdgeInfo>
EdgeInfo Digraph<VertexInfo, EdgeInfo>::edgeInfo(int fromVertex, int toVertex) const
{
    if(Map.find(fromVertex) == Map.end() || Map.find(toVertex) == Map.end())
    {
      throw DigraphException("fromVertex or toVertex Does Not Exist");
    }
    for( auto element : Map)
    {
      for(auto edge : element.second.edges)
      {
        if(edge.fromVertex == fromVertex && edge.toVertex == toVertex)
        {
          return edge.einfo;
        }
      }
    }
    throw DigraphException("Edge Does Not Exist");
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addVertex(int vertex, const VertexInfo& vinfo)
{
  if(Map.find(vertex) != Map.end())
  {
    throw DigraphException("Vertex already exists");
  }

  DigraphVertex<VertexInfo, EdgeInfo> temp = {};
  temp.vinfo = vinfo;
  Map[vertex] = temp;
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::addEdge(int fromVertex, int toVertex, const EdgeInfo& einfo)
{
  if(Map.find(fromVertex) == Map.end() || Map.find(toVertex) == Map.end())
  {
    throw DigraphException("fromVertex or toVertex Does Not Exist");
  }
  for( auto element : Map)
  {
    for(auto edge : element.second.edges)
    {
      if(edge.fromVertex == fromVertex && edge.toVertex == toVertex)
      {
        throw DigraphException("Edge Already Exists");
      }
    }
  }
  DigraphEdge<EdgeInfo> new_edge = {fromVertex, toVertex, einfo};
  Map[fromVertex].edges.insert(Map[fromVertex].edges.end(), new_edge);
}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeVertex(int vertex)
{
  if(Map.find(vertex) == Map.end())
  {
    throw DigraphException("Vertex Does Not Exist");
  }
  Map.erase(vertex);
  typename std::list<DigraphEdge<EdgeInfo>>::iterator iter;
  for(auto element : Map)
  {
    typename std::list<DigraphEdge<EdgeInfo>>::iterator iter = element.second.edges.begin();
    while(iter != element.second.edges.end())
    {
      if((*iter).toVertex == vertex)
      {
        element.second.edges.erase(iter++);
      }
      else { iter++;}
    }

  }

}


template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::removeEdge(int fromVertex, int toVertex)
{
  if(Map.find(fromVertex) == Map.end() || Map.find(toVertex) == Map.end())
  {
    throw DigraphException("fromVertex or toVertex Does Not Exist");
  }
  typename std::list<DigraphEdge<EdgeInfo>>::iterator iter = Map[fromVertex].edges.begin();
  while(iter != Map[fromVertex].edges.end())
  {
    if((*iter).toVertex == toVertex)
    {
      Map[fromVertex].edges.erase(iter++);
      return;
    }
    else{iter++;}
  }
  throw DigraphException("Edge Does Not Exists");

}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::vertexCount() const noexcept
{
  int count = 0;
  for (auto element : Map)
  {
    count++;
  }
  return count;
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount() const noexcept
{
  int count = 0;
  for(auto element : Map)
  {
    count += element.second.edges.size();
  }
  return count;
}


template <typename VertexInfo, typename EdgeInfo>
int Digraph<VertexInfo, EdgeInfo>::edgeCount(int vertex) const
{
  int count = 0;
    if(Map.find(vertex) == Map.end())
    {
      throw DigraphException("Vertex doesn't exist");
    }
    for (auto element : Map)
    {
      for(auto edge : element.second.edges)
      {
        if(edge.fromVertex == vertex)
        {
          count ++;
        }
      }
    }
    return count;
}


template <typename VertexInfo, typename EdgeInfo>
bool Digraph<VertexInfo, EdgeInfo>::isStronglyConnected() const
{
    if(vertexCount() == 0) {return true;}
    for(auto element : Map)
    {
      int visitCount = 0;
      std::map<int, bool>  visited;
      for (auto vertex : vertices())
      {
        visited[vertex] = false;
      }
      DFTr(Map, visited, element.first, visitCount);
      if(visitCount != vertexCount())
      {
        return false;
      }
    }
    return true;

}

template <typename VertexInfo, typename EdgeInfo>
void Digraph<VertexInfo, EdgeInfo>::DFTr(  std::map<int, DigraphVertex<VertexInfo,
                  EdgeInfo>> Map,  std::map<int, bool>& visited,
                  int vertex, int & visitCount) const
{
  visitCount++;
  visited[vertex] = true;

  for(auto edge : Map[vertex].edges)
  {
    if(visited[edge.toVertex] == false)
    {
      DFTr(Map, visited, edge.toVertex, visitCount);
    }
  }


}


template <typename VertexInfo, typename EdgeInfo>
std::map<int, int> Digraph<VertexInfo, EdgeInfo>::findShortestPaths(
    int startVertex,
    std::function<double(const EdgeInfo&)> edgeWeightFunc) const
{
  typedef std::pair<int,int> pqPair;
  std::map<int, bool> bool_known;
  std::map<int, int> prev;
  std::map<int, int> shortest_known;

  for (auto vertex : vertices())
  {
    if( vertex == startVertex) {prev[vertex] = vertex;}

    shortest_known[vertex] = vertex == startVertex ? 0 : std::numeric_limits<int>::max();
    bool_known[vertex] = false;
  }
  std::priority_queue < pqPair, std::vector<pqPair>,
                      std::greater<pqPair> > pq;
  pq.push(std::make_pair(0,startVertex ));
  while(pq.empty() == false)
  {
    int curr_vertex = pq.top().second;
    pq.pop();
    if(bool_known[curr_vertex] == false)
    {
      bool_known[curr_vertex] = true;
      for( auto edge :edges(curr_vertex))
      {
        if(shortest_known[edge.second] >
                  shortest_known[curr_vertex] + edgeWeightFunc(edgeInfo(edge.first, edge.second)))
        {
          shortest_known[edge.second] =
                    shortest_known[curr_vertex] + edgeWeightFunc(edgeInfo(edge.first, edge.second));
          prev[edge.second] = curr_vertex;
          pq.push(std::make_pair(shortest_known[edge.second], edge.second));
        }
      }

    }
  }
  for( auto element : bool_known)
  {
    if(element.second == false)
    {
      prev[element.first] = element.first;
    }
  }
  return prev;
}


#endif // DIGRAPH_HPP
