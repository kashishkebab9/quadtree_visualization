#include "cinder/app/App.h"
#include "cinder/app/RendererGl.h"
#include "cinder/gl/gl.h"
#include "Quadtree.h"

using namespace ci;
using namespace ci::app;
using namespace std;

class QuadTreeApp : public App {
  private:
    Box box;
    QuadtreeNode* root;
    Quadtree* qt;
    std::vector<float> posx_vector;
    std::vector<float> posy_vector;
    float start_x;
    float start_y;
    
    bool start_selected{false};
    
    float end_x;
    float end_y;
    
    bool end_selected{false};
    float selected_x;
    float selected_y;
    bool left_selected{false};
    
    
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
    void mouseDrag( MouseEvent event ) override;
    void update() override;
	void draw() override;
    void drawNode(QuadtreeNode* iter) ;
    void drawNeighbors(QuadtreeNode* node);
    void drawStartNode(QuadtreeNode* iter);
    void drawEndNode(QuadtreeNode* iter);
    QuadtreeNode* GetStartNode(QuadtreeNode* iter);
    QuadtreeNode* GetEndNode(QuadtreeNode* iter);
    
};

void QuadTreeApp::setup()
{
    setWindowSize(1000, 1000);
    this->box.position.x = getWindowCenter().x;
    this->box.position.y = getWindowCenter().y;
    this->box.width = getWindowWidth();
    this->box.height = getWindowHeight();
    root = new QuadtreeNode(0, box, QuadtreeNode::ChildDirection::root, nullptr);
    qt = new Quadtree(root);
    root->Subdivide();
    root->ne->Subdivide();
    root->nw->Subdivide();
    root->se->Subdivide();
    root->sw->Subdivide();
}

void QuadTreeApp::mouseDown( MouseEvent event )
{
   if (event.isShiftDown() ) {
       this->start_x = getMousePos().x - getWindowPosX();
       this->start_y = getMousePos().y -  getWindowPosY();
       qt->start_selected = true;
      
   }
    
   if (event.isControlDown() ) {
       this->end_x = getMousePos().x - getWindowPosX();
       this->end_y = getMousePos().y -  getWindowPosY();
       qt->goal_selected = true;
   }
    
    if (event.isAltDown()) {
       this->selected_x = getMousePos().x - getWindowPosX();
       this->selected_y = getMousePos().y -  getWindowPosY();
       this->left_selected = true;
    }
                
    if (!event.isShiftDown() && !event.isControlDown() && !event.isAltDown()) {
        auto posx = getMousePos().x - getWindowPosX();
        auto posy = getMousePos().y -  getWindowPosY();
        std::cout << "posx: " << posx << std::endl;
        std::cout << "posy: " << posy << std::endl;
        posx_vector.insert( posx_vector.end(), posx);
        posy_vector.insert(posy_vector.end(), posy);
        PointElement point;
        point.position.x = posx;
        point.position.y = posy;
        qt->Insert(point);
    }
}

void QuadTreeApp::mouseDrag(MouseEvent event) {
    
    if (!event.isShiftDown() && !event.isControlDown()) {
        auto posx = getMousePos().x - getWindowPosX();
        auto posy = getMousePos().y -  getWindowPosY();
        std::cout << "posx: " << posx << std::endl;
        std::cout << "posy: " << posy << std::endl;
        posx_vector.push_back(posx);
        posy_vector.push_back(posy);
        PointElement point;
        point.position.x = posx;
        point.position.y = posy;
        qt->Insert(point);
        
    }
    
}

void QuadTreeApp::update()
{
}

void QuadTreeApp::draw()
{
    //Draw Nodes/Rectangles
	gl::clear( Color( 1, 1, 1 ) );
    QuadtreeNode* iter;
    iter = root;
    drawNode(iter);
    
    //Draw all points on the screen
    for (int i= 0;  i < posx_vector.size(); i++) {
        vec2 position(posx_vector[i], posy_vector[i]);
        gl::pushModelMatrix();
        gl::color(Color(0, 0, 0));
        gl::drawSolidCircle(position, 2);
        gl::popModelMatrix();
    }
    
    // Draw start/end point
    if (qt->start_selected) {
        vec2 position(this->start_x, this->start_y);
        gl::pushModelMatrix();
        gl::color(Color(0, 1, 0));
        gl::drawSolidCircle(position, 4);
        gl::popModelMatrix();
        qt->start_node = root;
        qt->start_node = GetStartNode(qt->start_node);
        drawStartNode(qt->start_node);
    }
    
    if (qt->goal_selected) {
        vec2 position(this->end_x, this->end_y);
        gl::pushModelMatrix();
        gl::color(Color(1, 0, 0));
        gl::drawSolidCircle(position, 4);
        gl::popModelMatrix();
        qt->goal_node = root;
        qt->goal_node = GetEndNode(qt->goal_node);
        drawEndNode(qt->goal_node);
    }
    

    if (qt->goal_selected && qt->start_selected) {
        QuadtreeNode* iter = qt->AStar();
            
            gl::color(Color(0, 0, 0));
            gl::lineWidth(7);
            gl::drawLine(vec2(end_x, end_y), vec2(iter->box.position.x, iter->box.position.y));
            
           do {
               if(iter->astar_parent != NULL && iter != NULL) {
                gl::color(Color(0, 0, 0));
                gl::lineWidth(20);
                gl::drawLine(vec2(iter->box.position.x, iter->box.position.y), vec2(iter->astar_parent->box.position.x, iter->astar_parent->box.position.y));
                iter = iter->astar_parent;
               }
           } while(iter->astar_parent != qt->start_node);
            
            if(iter->astar_parent != NULL) {
                gl::color(Color(0, 0, 0));
                gl::lineWidth(20);
                gl::drawLine(vec2(iter->box.position.x, iter->box.position.y), vec2(start_x, start_y));
            }
    }
    
}

void QuadTreeApp::drawNode(QuadtreeNode* iter) {
    gl::pushModelMatrix();
    gl::color(Color(0, 0, 0));
    cinder::Rectf rect(iter->box.position.x - iter->box.width/2, iter->box.position.y + iter->box.height/2, iter->box.position.x + iter->box.width/2, iter->box.position.y - iter->box.height/2);
    gl::drawStrokedRect(rect, 1);
    gl::popModelMatrix();
    
    gl::pushModelMatrix();
    gl::color(Color(0, 0, 0));
    vec2 position_node(iter->box.position.x, iter->box.position.y);
    gl::drawStrokedCircle(position_node, 2);
    gl::popModelMatrix();
    
    if(iter->CheckIfObstacle()) {
        
        gl::pushModelMatrix();
        gl::color(Color(0, 0, 0));
        cinder::Rectf rect(iter->box.position.x - iter->box.width/2, iter->box.position.y + iter->box.height/2, iter->box.position.x + iter->box.width/2, iter->box.position.y - iter->box.height/2);
        gl::drawSolidRect(rect);
        gl::popModelMatrix();
        
    }
    
    if(iter->ne != NULL) drawNode(iter->ne);
    if(iter->nw != NULL) drawNode(iter->nw);
    if(iter->se != NULL) drawNode(iter->se);
    if(iter->sw != NULL) drawNode(iter->sw);
    
}

void QuadTreeApp::drawNeighbors(QuadtreeNode* iter) {
    while (iter->subdivided) {
        if(selected_y < iter->box.position.y) {
            if(selected_x > iter->box.position.x) {
                iter = iter->ne;
            } else {
                iter = iter->nw;
            }
        } else {
            if(selected_x > iter->box.position.x) {
                iter = iter->se;
            } else {
                iter = iter->sw;
            }
        }
    }
    
    
    std::vector<QuadtreeNode*> all_neighbors = qt->GetAllNeighbors(iter);
    

    for (auto node: all_neighbors) {
        if (node != NULL) {
            if(!node->CheckIfObstacle()) {
                
                
                gl::pushModelMatrix();
                gl::color(Color(0, 0, 1));
                cinder::Rectf rect(node->box.position.x - node->box.width/2, node->box.position.y - node->box.height/2, node->box.position.x + node->box.width/2, node->box.position.y + node->box.height/2);
                gl::drawStrokedRect(rect, 1);
                gl::popModelMatrix();
                
                
                
            }
        }
    }
    
}

QuadtreeNode* QuadTreeApp::GetStartNode(QuadtreeNode* iter) {
    
    if(iter->subdivided) {
        
        while (iter->subdivided) {
            if (start_y < iter->box.position.y) {
                if(start_x > iter->box.position.x) {
                    iter = iter->ne;
                } else {
                    iter = iter->nw;
                }
            } else {
                if(start_x > iter->box.position.x) {
                    iter = iter->se;
                } else {
                    iter = iter->sw;
                }
            }
        }
    }
    
    return iter;
    
}

void QuadTreeApp::drawStartNode(QuadtreeNode* iter) {
    //we need to first find the node we have our start point in:
    
    //now iter is a leaf node
    gl::pushModelMatrix();
    gl::color(Color(0, 1, 0));
    cinder::Rectf rect(iter->box.position.x - iter->box.width/2, iter->box.position.y - iter->box.height/2, iter->box.position.x + iter->box.width/2, iter->box.position.y + iter->box.height/2);
    gl::drawStrokedRect(rect, 1);
    gl::popModelMatrix();
   
}

QuadtreeNode* QuadTreeApp::GetEndNode(QuadtreeNode* iter) {
    
    //we need to first find the node we have our start point in:
    if(iter->subdivided) {
        
        while (iter->subdivided) {
            if (end_y < iter->box.position.y) {
                if(end_x > iter->box.position.x) {
                    iter = iter->ne;
                } else {
                    iter = iter->nw;
                }
            } else {
                if(end_x > iter->box.position.x) {
                    iter = iter->se;
                } else {
                    iter = iter->sw;
                }
            }
        }
    }
    return iter;
}

void QuadTreeApp::drawEndNode(QuadtreeNode* iter) {
    
    //now iter is a leaf node
    gl::pushModelMatrix();
    gl::color(Color(1, 0, 0));
    cinder::Rectf rect(iter->box.position.x - iter->box.width/2, iter->box.position.y - iter->box.height/2, iter->box.position.x + iter->box.width/2, iter->box.position.y + iter->box.height/2);
    gl::drawStrokedRect(rect, 1);
    gl::popModelMatrix();
   
}

CINDER_APP( QuadTreeApp, RendererGl )
