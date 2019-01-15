/*
  Copyright (c) 2015, Damian Barczynski <daan.net@wp.eu>
  Following tool is licensed under the terms and conditions of the ISC license.
  For more information visit https://opensource.org/licenses/ISC.
*/
#ifndef __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#define __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
#include <math.h>
#include <iostream>
#include <vector>
#include <functional>
#include <set>

namespace AStar
{
  struct Vec2i
  {
    int x, y;
    Vec2i(){};
    Vec2i (int x_, int y_) {
      x = x_;
      y = y_;
    };
    bool operator == (const Vec2i& coordinates_);
  };

  using uint = unsigned int;
  using HeuristicFunction = std::function<uint(Vec2i, Vec2i)>;
  using CoordinateList = std::vector<Vec2i>;

  struct Node
  {
    Node(){};
    uint G, H;
    Vec2i coordinates;
    Node *parent;

    Node(Vec2i coord_, Node *parent_ = nullptr);
    uint getScore();
  };

  using NodeSet = std::set<Node*>;

  class Generator
  {
    Node* findNodeOnList(NodeSet& nodes_, Vec2i coordinates_);
    void releaseNodes(NodeSet& nodes_);

  public:
    Generator();
    void setWorldSize(Vec2i worldSize_);
    void setDiagonalMovement(bool enable_);
    void setHeuristic(HeuristicFunction heuristic_);
    CoordinateList findPath(Vec2i source_, Vec2i target_);
    void addCollision(Vec2i coordinates_);
    void removeCollision(Vec2i coordinates_);
    void clearCollisions();
    bool detectCollision(Vec2i coordinates_);
    void addCollisionList(const std::vector<Vec2i>& collisions);
    void drawWorld();
    void setSource(Vec2i source_);
    void setTarget(Vec2i target_);
    Vec2i getSource() {return source;};
    Vec2i getTarget() {return target;};

  private:
    HeuristicFunction heuristic;
    CoordinateList direction, walls;
    Vec2i worldSize;
    uint directions;
    Vec2i source, target;
  };

  class Heuristic
  {
    static Vec2i getDelta(Vec2i source_, Vec2i target_);

  public:
    static uint manhattan(Vec2i source_, Vec2i target_);
    static uint euclidean(Vec2i source_, Vec2i target_);
    static uint octagonal(Vec2i source_, Vec2i target_);
  };
}

#endif // __ASTAR_HPP_8F637DB91972F6C878D41D63F7E7214F__
