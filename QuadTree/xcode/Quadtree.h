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
#include <cmath>
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
    QuadtreeNode* astar_parent;
    
    int g_cost;      //dist from it to start
    int h_cost;      //dist from it to goal
    
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
    int f_cost;      //g_cost + h_cost

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
      astar_parent = NULL;
    }
    
    bool CheckIfObstacle() {
        if(this->obstacle) return true;
        else {
            return false;
        }
    }

    void SetAStarParent(QuadtreeNode* node) {
        this->astar_parent = node;
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
    
    QuadtreeNode* start_node{NULL};
    QuadtreeNode* goal_node{NULL};
    bool start_selected{false};
    bool goal_selected{false};
    bool acost_succeeded;
    
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
    
    std::vector<QuadtreeNode*> GetAllNeighbors(QuadtreeNode* iter) {
        QuadtreeNode* west_neighbor = WalkWest(iter);
        std::vector<QuadtreeNode*> west_neighbors = SmallerNodes(west_neighbor, Quadtree::WEST);
        
        QuadtreeNode* north_neighbor = WalkNorth(iter);
        std::vector<QuadtreeNode*> north_neighbors = SmallerNodes(north_neighbor, Quadtree::NORTH);
        
        QuadtreeNode* south_neighbor = WalkSouth(iter);
        std::vector<QuadtreeNode*> south_neighbors = SmallerNodes(south_neighbor, Quadtree::SOUTH);
        
        QuadtreeNode* east_neighbor = WalkEast(iter);
        std::vector<QuadtreeNode*> east_neighbors = SmallerNodes(east_neighbor, Quadtree::EAST);
        
        std::vector<QuadtreeNode*> north_east_neighbors;
        north_east_neighbors = WalkNorthEast(iter, north_neighbors, east_neighbors);
        
        std::vector<QuadtreeNode*> south_east_neighbors;
        south_east_neighbors = WalkSouthEast(iter, south_neighbors, east_neighbors);
        
        std::vector<QuadtreeNode*> north_west_neighbors;
        north_west_neighbors = WalkNorthWest(iter, north_neighbors, west_neighbors);
        
        std::vector<QuadtreeNode*> south_west_neighbors;
        south_west_neighbors = WalkSouthWest(iter, south_neighbors, west_neighbors);
        
        std::vector<QuadtreeNode*> all_neighbors;
        all_neighbors.insert(all_neighbors.begin(), north_neighbors.begin(), north_neighbors.end());
        all_neighbors.insert(all_neighbors.end(), north_east_neighbors.begin(), north_east_neighbors.end());
        all_neighbors.insert(all_neighbors.end(), east_neighbors.begin(), east_neighbors.end());
        all_neighbors.insert(all_neighbors.end(), south_east_neighbors.begin(), south_east_neighbors.end());
        all_neighbors.insert(all_neighbors.end(), south_neighbors.begin(), south_neighbors.end());
        all_neighbors.insert(all_neighbors.end(), south_west_neighbors.begin(), south_west_neighbors.end());
        all_neighbors.insert(all_neighbors.end(), west_neighbors.begin(), west_neighbors.end());
        all_neighbors.insert(all_neighbors.end(), north_west_neighbors.begin(), north_west_neighbors.end());
        return all_neighbors;
    }

    void CalculateFCost(QuadtreeNode* node) {
        if(this->start_selected && this->goal_selected) {
             //need to grab the xy position of the goal and the xy position of the current node
            std::cout << "Goal x: " << this->goal_node->box.position.x << std::endl;
            std::cout << "Node x: " << node->box.position.x << std::endl;
            
            std::cout << "Goal y: " << this->goal_node->box.position.y << std::endl;
            std::cout << "Node y: " << node->box.position.y << std::endl;
            node->h_cost = sqrt( pow(this->goal_node->box.position.x - node->box.position.x, 2) + pow(this->goal_node->box.position.y - node->box.position.y, 2) );
            node->g_cost = sqrt(pow(this->start_node->box.position.x - node->box.position.x, 2) + pow(this->start_node->box.position.y - node->box.position.y, 2) );
            
            if (node->g_cost < 0) {
                node->g_cost = 0;
            }
            if (node->h_cost < 0) {
                node->h_cost = 0;
            }
            
            node->f_cost = node->g_cost + node->h_cost;
            std::cout << "g_cost: " << node->g_cost <<  std::endl;
            std::cout << "h_cost: " << node->h_cost <<  std::endl;
            std::cout << "f_cost: " << node->f_cost <<  std::endl;
           
        }
 
        
    }
    
    QuadtreeNode* AStar() {
        std::cout << "Starting Astar" << std::endl;
        std::vector<QuadtreeNode*> output;
        
        std::vector<QuadtreeNode*> open_set; //the set of nodes to be evaluated
        std::vector<QuadtreeNode*> closed_set; // the set of nodes evaluated
        QuadtreeNode* current;
        open_set.push_back(this->start_node);
        int current_i = 0;
        
        //while goal is not in open_set
        while (!std::count(open_set.begin(), open_set.end(), this->goal_node)) {
            int min_fcost = INT_MAX;
            for (int i =0; i < open_set.size(); i++) {
                CalculateFCost(open_set[i]);
                if((open_set[i])->f_cost < min_fcost) {
                    //update f_cost
                    min_fcost = (open_set[i])->f_cost;
                    //set the current iterator to lowest fcost node
                    current = open_set[i];
                }
            }
            
            output.push_back(current);
            current_i++;
            
            //move current node to closed set
            closed_set.push_back(current);
            //open_set.erase(open_set.begin()+ i);
            open_set.erase(std::remove(open_set.begin(), open_set.end(), current), open_set.end());
            
            gl::pushModelMatrix();
            gl::color(Color(0, 0, 1));
            
            cinder::Rectf rect(current->box.position.x - current->box.width/2, current->box.position.y - current->box.height/2, current->box.position.x + current->box.width/2, current->box.position.y + current->box.height/2);
            gl::drawStrokedRect(rect);
            gl::popModelMatrix();
            
            if (current == this->goal_node) {
                return output[output.size()-1];
            }
            std::vector<QuadtreeNode*> current_neighbors =GetAllNeighbors(current);
            if(!current_neighbors.empty()) {
                for (int j = 0; j < current_neighbors.size(); j++) {
                    if (current_neighbors[j]->CheckIfObstacle() || std::count(closed_set.begin(), closed_set.end(), current_neighbors[j])) {
                        continue;
                    }
                    if (!std::count(open_set.begin(), open_set.end(), current_neighbors[j])) {
                        CalculateFCost(current_neighbors[j]);
                        (current_neighbors[j])->SetAStarParent(current);
                        open_set.push_back(current_neighbors[j]);
                    }
                }
            }
            
        }
        return output[output.size()-1];
        
    }
    
    
    
    
    
};
