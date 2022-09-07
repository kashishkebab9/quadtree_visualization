//
//  Quadtree.h
//  QuadTree
//
//  Created by Kashish Garg on 9/3/22.
//

#ifndef Quadtree_h
#define Quadtree_h
#endif /* Quadtree_h */

#include <__nullptr>
#include <cstddef>
#include <iostream>
#include <string>
#include <vector>
#include <algorithm>
using namespace ci;
using namespace ci::app;
using namespace std;


struct Vector2 {
  float x;
  float y;
};

struct Box {
  Vector2 position;
  float width;
  float height;
};

struct PointElement {
  Vector2 position;
};


class QuadtreeNode {
  private:
    Box box;
    int depth;
    bool subdivided;
    std::vector<PointElement> elements;
    bool obstacle;

    QuadtreeNode* ne;
    QuadtreeNode* nw;
    QuadtreeNode* se;
    QuadtreeNode* sw;
    QuadtreeNode* parent_node;
    
    int f_cost;
    int g_cost;
    int h_cost;
    
    // the direction from which this node spawned out of it's parent
    enum ChildDirection {
      NE = 0,
      NW = 1,
      SE = 2,
      SW = 4,
      root = 5
    };
    ChildDirection direction_from_parent;
    
    

  public:
    
    friend class Quadtree;
    friend class QuadTreeApp;

    QuadtreeNode(int depth_, Box box_, ChildDirection child_direction_, QuadtreeNode* parent_node_) {
      this->box = box_;
      this->depth = depth_;
      this->direction_from_parent = child_direction_;
      this->parent_node = parent_node_;
      subdivided = false;
      obstacle = false;

      ne = NULL;
      nw = NULL;
      se = NULL;
      sw = NULL;
    }
    
    bool CheckIfObstacle() {
        if(this->obstacle) return true;
        else {
            return false;
        }
    }


    void Subdivide() {
      std::cout << "Subdividing this node!" << std::endl;

      //we want to manage the current memory of the elements in our vector and place them into the correct pointers
      // swapping N and S for the directions fixes our issue for some reason.
      Box ne_box;
      ne_box.position.x = this->box.position.x + this->box.width/4;
      ne_box.position.y = this->box.position.y - this->box.height/4;
      ne_box.width = this->box.width/2;
      ne_box.height = this->box.height/2;
      this->ne = new QuadtreeNode(this->depth+1, ne_box, NE, this);

      Box nw_box;
      nw_box.position.x = this->box.position.x - this->box.width/4;
      nw_box.position.y = this->box.position.y - this->box.height/4;
      nw_box.width = this->box.width/2;
      nw_box.height = this->box.height/2;
      this->nw = new QuadtreeNode(this->depth + 1, nw_box, NW, this);

      Box se_box;
      se_box.position.x = this->box.position.x + this->box.width/4;
      se_box.position.y = this->box.position.y + this->box.height/4;
      se_box.width = this->box.width/2;
      se_box.height = this->box.height/2;
      this->se = new QuadtreeNode(this->depth + 1, se_box, SE, this);

      Box sw_box;
      sw_box.position.x = this->box.position.x - this->box.width/4;
      sw_box.position.y = this->box.position.y + this->box.height/4;
      sw_box.width = this->box.width/2;
      sw_box.height = this->box.height/2;
      this->sw = new QuadtreeNode(this->depth + 1, sw_box, SW, this);

      for (auto n: this->elements) {
        if(n.position.y > this->box.position.y) {
          if (n.position.x > this->box.position.x) {
            this->ne->elements.push_back(n);
          } else {
            this->nw->elements.push_back(n);
          }

        } else {
          if (n.position.x > this->box.position.x) {
            this->se->elements.push_back(n);
          } else {
            this->sw->elements.push_back(n);
          }
        }


      }
      //make sure to empty out the elements of this quadtree
      this->elements.clear();
      this->subdivided = true;
    }
    



};

class Quadtree {

  private:

    QuadtreeNode* root;
    int el_limit;
    int depth_limit;
    int obstacle_limit;
    std::vector<Point> elements;

  public:
    enum TraverseDirection {
      NORTH,
      NORTHEAST,
      EAST,
      SOUTHEAST,
      SOUTH,
      SOUTHWEST,
      WEST,
      NORTHWEST
    };
    
    Quadtree(QuadtreeNode* root_) {
      this->root = root_;
      el_limit = 3;
      depth_limit = 6;
      obstacle_limit = 1;

    }

    void Insert(PointElement point) {
      QuadtreeNode* iter = root;

      float p_x = point.position.x;
      float p_y = point.position.y;
      

      while (iter->subdivided) {

        if (p_y < iter->box.position.y) {
          if(p_x > iter->box.position.x) {
            iter = iter->ne;
          } else {
            iter = iter->nw;
          }
        } else {
          if(p_x > iter->box.position.x) {
            iter = iter->se;
          } else {
            iter = iter->sw;
          }
        }

      }

      if (iter->depth == this->depth_limit) {
        iter->elements.push_back(point);
        if (iter->elements.size() >= this->obstacle_limit) {
            iter->obstacle = true;
        }


      } else {
        iter->elements.push_back(point);
        if(!(iter->elements.size() < this->el_limit)) {
          iter->Subdivide();
        }

      }
    }
    
   // finding neighbors for all directions:
    QuadtreeNode* WalkNorth(QuadtreeNode* node) {
        
        if (node == NULL || node->parent_node == NULL) {
            return NULL;
        }
        if (node->direction_from_parent == QuadtreeNode::SE) {
            return node->parent_node->ne;
        } else if(node->direction_from_parent == QuadtreeNode::SW) {
            return node->parent_node->nw;
        }else if(node->direction_from_parent == QuadtreeNode::NW) {
            QuadtreeNode* north_parent = WalkNorth(node->parent_node);
            if(north_parent == NULL) {
                return NULL;
            }
            
            if (!north_parent->subdivided) {
                return north_parent;
            } else {
                return north_parent->sw;
            }
        }else if(node->direction_from_parent == QuadtreeNode::NE) {
            QuadtreeNode* north_parent = WalkNorth(node->parent_node);
            if(north_parent == NULL) {
                return NULL;
            }
            if (!north_parent->subdivided) {
                return north_parent;
            } else {
                return north_parent->se;
            }
        }
    }
    
        QuadtreeNode* WalkEast(QuadtreeNode* node) {
        
        if (node == NULL || node->parent_node == NULL) {
            return NULL;
        }
        if (node->direction_from_parent == QuadtreeNode::NW) {
            return node->parent_node->ne;
        } else if(node->direction_from_parent == QuadtreeNode::SW) {
            return node->parent_node->se;
        }else if(node->direction_from_parent == QuadtreeNode::NE) {
            QuadtreeNode* east_parent = WalkEast(node->parent_node);
            if(east_parent == NULL) {
                return NULL;
            }
            
            if (!east_parent->subdivided) {
                return east_parent;
            } else {
                return east_parent->nw;
            }
        }else if(node->direction_from_parent == QuadtreeNode::SE) {
            QuadtreeNode* east_parent = WalkEast(node->parent_node);
            if(east_parent == NULL) {
                return NULL;
            }
            if (!east_parent->subdivided) {
                return east_parent;
            } else {
                return east_parent->sw;
            }
        }
    }

    
    QuadtreeNode* WalkWest(QuadtreeNode* node) {
        
        if (node == NULL || node->parent_node == NULL) {
            return NULL;
        }
        if (node->direction_from_parent == QuadtreeNode::NE) {
            return node->parent_node->nw;
        } else if(node->direction_from_parent == QuadtreeNode::SE) {
            return node->parent_node->sw;
        }else if(node->direction_from_parent == QuadtreeNode::NW) {
            QuadtreeNode* left_parent = WalkWest(node->parent_node);
            if(left_parent == NULL) {
                return NULL;
            }
            
            if (!left_parent->subdivided) {
                return left_parent;
            } else {
                return left_parent->ne;
            }
        }else if(node->direction_from_parent == QuadtreeNode::SW) {
            QuadtreeNode* left_parent = WalkWest(node->parent_node);
            if(left_parent == NULL) {
                return NULL;
            }
            if (!left_parent->subdivided) {
                return left_parent;
            } else {
                return left_parent->se;
            }
        }
    }
    
        QuadtreeNode* WalkSouth(QuadtreeNode* node) {
        
        if (node == NULL || node->parent_node == NULL) {
            return NULL;
        }
        if (node->direction_from_parent == QuadtreeNode::NE) {
            return node->parent_node->se;
        } else if(node->direction_from_parent == QuadtreeNode::NW) {
            return node->parent_node->sw;
        }else if(node->direction_from_parent == QuadtreeNode::SW) {
            QuadtreeNode* south_parent = WalkSouth(node->parent_node);
            if(south_parent == NULL) {
                return NULL;
            }
            
            if (!south_parent->subdivided) {
                return south_parent;
            } else {
                return south_parent->nw;
            }
        }else if(node->direction_from_parent == QuadtreeNode::SE) {
            QuadtreeNode* south_parent = WalkSouth(node->parent_node);
            if(south_parent == NULL) {
                return NULL;
            }
            if (!south_parent->subdivided) {
                return south_parent;
            } else {
                return south_parent->ne;
            }
        }
    }

    

    std::vector<QuadtreeNode*> SmallerNodes(QuadtreeNode* neighbor, TraverseDirection direction) {
        std::vector<QuadtreeNode*> candidates;
        std::vector<QuadtreeNode*> neighbors;
        if (neighbor != NULL) {
            candidates.insert(candidates.begin(), neighbor);
        }
        
        while (candidates.size() > 0) {
            if(!candidates[0]->subdivided) {
                neighbors.insert(neighbors.end(), candidates[0]);
            } else {
                
                switch(direction) {
                    case(NORTH):
                        candidates.insert(candidates.end(), candidates[0]->se);
                        candidates.insert(candidates.end(), candidates[0]->sw);
                        break;
                        
                    case(EAST):
                        candidates.insert(candidates.end(), candidates[0]->nw);
                        candidates.insert(candidates.end(), candidates[0]->sw);
                        break;

                    case(SOUTH):
                        candidates.insert(candidates.end(), candidates[0]->ne);
                        candidates.insert(candidates.end(), candidates[0]->nw);
                        break;
                        
                    case(WEST):
                        candidates.insert(candidates.end(), candidates[0]->ne);
                        candidates.insert(candidates.end(), candidates[0]->se);
                        break;
                        
                        

                        
                }
            }
            candidates.erase(candidates.begin());
        }
        return neighbors;
        
    }
    
    std::vector<QuadtreeNode*> WalkNorthEast(QuadtreeNode* node, std::vector<QuadtreeNode*> north_neighbors, std::vector<QuadtreeNode*> east_neighbors) {
        
        std::vector<QuadtreeNode*> intersection;
        std::vector<QuadtreeNode*> north_neighbors_east_neighbor;
        std::vector<QuadtreeNode*> east_neighbors_north_neighbor;
        for (auto dir_node : north_neighbors) {
            QuadtreeNode* north_neighbor_east_neighbor = WalkEast(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(north_neighbor_east_neighbor, EAST);
            for (auto tmp: temp) {
                north_neighbors_east_neighbor.push_back(tmp);
            }
        }
        for (auto dir_node : east_neighbors) {
            QuadtreeNode* east_neighbor_north_neighbor = WalkNorth(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(east_neighbor_north_neighbor, NORTH);
            for (auto tmp: temp) {
                east_neighbors_north_neighbor.push_back(tmp);
            }
        }
        std::sort(north_neighbors_east_neighbor.begin(), north_neighbors_east_neighbor.end());
        std::sort(east_neighbors_north_neighbor.begin(), east_neighbors_north_neighbor.end());
        
        std::set_intersection(north_neighbors_east_neighbor.begin(), north_neighbors_east_neighbor.end(),
                                east_neighbors_north_neighbor.begin(), east_neighbors_north_neighbor.end(),
                              back_inserter(intersection));
        return intersection;
    }
    
    std::vector<QuadtreeNode*> WalkSouthEast(QuadtreeNode* node, std::vector<QuadtreeNode*> south_neighbors, std::vector<QuadtreeNode*> east_neighbors) {
        
        std::vector<QuadtreeNode*> intersection;
        std::vector<QuadtreeNode*> south_neighbors_east_neighbor;
        std::vector<QuadtreeNode*> east_neighbors_south_neighbor;
        for (auto dir_node : south_neighbors) {
            QuadtreeNode* south_neighbor_east_neighbor = WalkEast(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(south_neighbor_east_neighbor, EAST);
            for (auto tmp: temp) {
                south_neighbors_east_neighbor.push_back(tmp);
            }
        }
        for (auto dir_node : east_neighbors) {
            QuadtreeNode* east_neighbor_south_neighbor = WalkSouth(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(east_neighbor_south_neighbor, SOUTH);
            for (auto tmp: temp) {
                east_neighbors_south_neighbor.push_back(tmp);
            }
        }
        std::sort(south_neighbors_east_neighbor.begin(), south_neighbors_east_neighbor.end());
        std::sort(east_neighbors_south_neighbor.begin(), east_neighbors_south_neighbor.end());
        
        std::set_intersection(south_neighbors_east_neighbor.begin(), south_neighbors_east_neighbor.end(),
                                east_neighbors_south_neighbor.begin(), east_neighbors_south_neighbor.end(),
                              back_inserter(intersection));
        return intersection;
    }


    std::vector<QuadtreeNode*> WalkNorthWest(QuadtreeNode* node, std::vector<QuadtreeNode*> north_neighbors, std::vector<QuadtreeNode*> west_neighbors) {
        
        std::vector<QuadtreeNode*> intersection;
        std::vector<QuadtreeNode*> north_neighbors_west_neighbor;
        std::vector<QuadtreeNode*> west_neighbors_north_neighbor;
        for (auto dir_node : north_neighbors) {
            QuadtreeNode* north_neighbor_west_neighbor = WalkWest(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(north_neighbor_west_neighbor, WEST);
            for (auto tmp: temp) {
                north_neighbors_west_neighbor.push_back(tmp);
            }
        }
        for (auto dir_node : west_neighbors) {
            QuadtreeNode* west_neighbor_north_neighbor = WalkNorth(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(west_neighbor_north_neighbor, NORTH);
            for (auto tmp: temp) {
                west_neighbors_north_neighbor.push_back(tmp);
            }
        }
        std::sort(north_neighbors_west_neighbor.begin(), north_neighbors_west_neighbor.end());
        std::sort(west_neighbors_north_neighbor.begin(), west_neighbors_north_neighbor.end());
        
        std::set_intersection(north_neighbors_west_neighbor.begin(), north_neighbors_west_neighbor.end(),
                                west_neighbors_north_neighbor.begin(), west_neighbors_north_neighbor.end(),
                              back_inserter(intersection));
        return intersection;
    }
    
    std::vector<QuadtreeNode*> WalkSouthWest(QuadtreeNode* node, std::vector<QuadtreeNode*> south_neighbors, std::vector<QuadtreeNode*> west_neighbors) {
        
        std::vector<QuadtreeNode*> intersection;
        std::vector<QuadtreeNode*> south_neighbors_west_neighbor;
        std::vector<QuadtreeNode*> west_neighbors_south_neighbor;
        for (auto dir_node : south_neighbors) {
            QuadtreeNode* south_neighbor_west_neighbor = WalkWest(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(south_neighbor_west_neighbor, WEST);
            for (auto tmp: temp) {
                south_neighbors_west_neighbor.push_back(tmp);
            }
        }
        for (auto dir_node : west_neighbors) {
            QuadtreeNode* west_neighbor_south_neighbor = WalkSouth(dir_node);
            std::vector<QuadtreeNode*> temp = SmallerNodes(west_neighbor_south_neighbor, SOUTH);
            for (auto tmp: temp) {
                west_neighbors_south_neighbor.push_back(tmp);
            }
        }
        std::sort(south_neighbors_west_neighbor.begin(), south_neighbors_west_neighbor.end());
        std::sort(west_neighbors_south_neighbor.begin(), west_neighbors_south_neighbor.end());
        
        std::set_intersection(south_neighbors_west_neighbor.begin(), south_neighbors_west_neighbor.end(),
                                west_neighbors_south_neighbor.begin(), west_neighbors_south_neighbor.end(),
                              back_inserter(intersection));
        return intersection;
    }

};
