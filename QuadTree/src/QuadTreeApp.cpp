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
       this->start_selected = true;
   }
    
   if (event.isControlDown() ) {
       this->end_x = getMousePos().x - getWindowPosX();
       this->end_y = getMousePos().y -  getWindowPosY();
       this->end_selected = true;
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
    if (this->start_selected) {
        vec2 position(this->start_x, this->start_y);
        gl::pushModelMatrix();
        gl::color(Color(0, 1, 0));
        gl::drawSolidCircle(position, 4);
        gl::popModelMatrix();
        QuadtreeNode* start = root;
        std::cout << start->direction_from_parent << std::endl;
        drawStartNode(start);
    }
    
    if (this->end_selected) {
        vec2 position(this->end_x, this->end_y);
        gl::pushModelMatrix();
        gl::color(Color(1, 0, 0));
        gl::drawSolidCircle(position, 4);
        gl::popModelMatrix();
        QuadtreeNode* end = root;
        drawEndNode(end);
    }
    
    if(this->left_selected) {
        vec2 position(this->selected_x, this->selected_y);
        gl::pushModelMatrix();
        gl::color(Color(0, 0, 1));
        gl::drawSolidCircle(position, 4);
        gl::popModelMatrix();
        iter = root;
        drawNeighbors(iter);
        
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
    
    QuadtreeNode* west_neighbor = qt->WalkWest(iter);
    std::vector<QuadtreeNode*> west_neighbors = qt->SmallerNodes(west_neighbor, Quadtree::WEST);
    
    QuadtreeNode* north_neighbor = qt->WalkNorth(iter);
    std::vector<QuadtreeNode*> north_neighbors = qt->SmallerNodes(north_neighbor, Quadtree::NORTH);
    
    QuadtreeNode* south_neighbor = qt->WalkSouth(iter);
    std::vector<QuadtreeNode*> south_neighbors = qt->SmallerNodes(south_neighbor, Quadtree::SOUTH);
    
    QuadtreeNode* east_neighbor = qt->WalkEast(iter);
    std::vector<QuadtreeNode*> east_neighbors = qt->SmallerNodes(east_neighbor, Quadtree::EAST);
    
    std::vector<QuadtreeNode*> north_east_neighbors;
    north_east_neighbors = qt->WalkNorthEast(iter, north_neighbors, east_neighbors);
    
    std::vector<QuadtreeNode*> south_east_neighbors;
    south_east_neighbors = qt->WalkSouthEast(iter, south_neighbors, east_neighbors);
    
    std::vector<QuadtreeNode*> north_west_neighbors;
    north_west_neighbors = qt->WalkNorthWest(iter, north_neighbors, west_neighbors);
    
    std::vector<QuadtreeNode*> south_west_neighbors;
    south_west_neighbors = qt->WalkSouthWest(iter, south_neighbors, west_neighbors);
    
    std::vector<QuadtreeNode*> all_neighbors;
    all_neighbors.insert(all_neighbors.begin(), north_neighbors.begin(), north_neighbors.end());
    all_neighbors.insert(all_neighbors.end(), north_east_neighbors.begin(), north_east_neighbors.end());
    all_neighbors.insert(all_neighbors.end(), east_neighbors.begin(), east_neighbors.end());
    all_neighbors.insert(all_neighbors.end(), south_east_neighbors.begin(), south_east_neighbors.end());
    all_neighbors.insert(all_neighbors.end(), south_neighbors.begin(), south_neighbors.end());
    all_neighbors.insert(all_neighbors.end(), south_west_neighbors.begin(), south_west_neighbors.end());
    all_neighbors.insert(all_neighbors.end(), west_neighbors.begin(), west_neighbors.end());
    all_neighbors.insert(all_neighbors.end(), north_west_neighbors.begin(), north_west_neighbors.end());
    
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

void QuadTreeApp::drawStartNode(QuadtreeNode* iter) {
    //we need to first find the node we have our start point in:
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
    } else {
        return;
    }
    
    //now iter is a leaf node
    gl::pushModelMatrix();
    gl::color(Color(0, 1, 0));
    cinder::Rectf rect(iter->box.position.x - iter->box.width/2, iter->box.position.y - iter->box.height/2, iter->box.position.x + iter->box.width/2, iter->box.position.y + iter->box.height/2);
    gl::drawStrokedRect(rect, 1);
    gl::popModelMatrix();
   
}

void QuadTreeApp::drawEndNode(QuadtreeNode* iter) {
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
    } else {
        return;
    }
    
    //now iter is a leaf node
    gl::pushModelMatrix();
    gl::color(Color(1, 0, 0));
    cinder::Rectf rect(iter->box.position.x - iter->box.width/2, iter->box.position.y - iter->box.height/2, iter->box.position.x + iter->box.width/2, iter->box.position.y + iter->box.height/2);
    gl::drawStrokedRect(rect, 1);
    gl::popModelMatrix();
   
}

CINDER_APP( QuadTreeApp, RendererGl )
