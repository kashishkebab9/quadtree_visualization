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

    QuadtreeNode* ne;
    QuadtreeNode* nw;
    QuadtreeNode* se;
    QuadtreeNode* sw;

  public:
    friend class Quadtree;
    friend class QuadTreeApp;


    QuadtreeNode(int depth_, Box box_) {
      this->box = box_;
      this->depth = depth_;
      subdivided = false;
      std::cout << "QuadtreeNode has been created at depth: " << depth_ << std::endl;

      ne = NULL;
      nw = NULL;
      se = NULL;
      sw = NULL;
    }
    


    void Subdivide() {
      std::cout << "Subdividing this node!" << std::endl;

      //we want to manage the current memory of the elements in our vector and place them into the correct pointers
      Box ne_box;
      ne_box.position.x = this->box.position.x + this->box.width/4;
      ne_box.position.y = this->box.position.y + this->box.height/4;
      ne_box.width = this->box.width/2;
      ne_box.height = this->box.height/2;
      this->ne = new QuadtreeNode(this->depth+1, ne_box);

      Box nw_box;
      nw_box.position.x = this->box.position.x - this->box.width/4;
      nw_box.position.y = this->box.position.y + this->box.height/4;
      nw_box.width = this->box.width/2;
      nw_box.height = this->box.height/2;


      this->nw = new QuadtreeNode(this->depth + 1, nw_box);

      Box se_box;
      se_box.position.x = this->box.position.x + this->box.width/4;
      se_box.position.y = this->box.position.y - this->box.height/4;
      se_box.width = this->box.width/2;
      se_box.height = this->box.height/2;

      this->se = new QuadtreeNode(this->depth + 1, se_box);

      Box sw_box;
      sw_box.position.x = this->box.position.x - this->box.width/4;
      sw_box.position.y = this->box.position.y - this->box.height/4;
      sw_box.width = this->box.width/2;
      sw_box.height = this->box.height/2;

      this->sw = new QuadtreeNode(this->depth + 1, sw_box);

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
    std::vector<Point> elements;

  public:
    Quadtree(QuadtreeNode* root_) {
      this->root = root_;
      el_limit = 4;
      depth_limit = 7;
    }

    void Insert(PointElement point) {
      QuadtreeNode* iter = root;

      float p_x = point.position.x;
      float p_y = point.position.y;
      

      while (iter->subdivided) {

        if (p_y > iter->box.position.y) {
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
        std::cout << "At Depth Limit, appending to current list" << std::endl;
        iter->elements.push_back(point);


      } else {
        iter->elements.push_back(point);
        std::cout << "Not at Depth Limit!" << std::endl;

        if(!(iter->elements.size() < this->el_limit)) {
          iter->Subdivide();
        }

      }






    }

};
