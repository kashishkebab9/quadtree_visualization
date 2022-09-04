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
    
  public:
	void setup() override;
	void mouseDown( MouseEvent event ) override;
    void mouseDrag( MouseEvent event ) override;
	void update() override;
	void draw() override;
    void iterate(QuadtreeNode* iter) ;
};

    

void QuadTreeApp::setup()
{
    this->box.position.x = getWindowCenter().x;
    this->box.position.y = getWindowCenter().y;
    this->box.width = getWindowWidth();
    this->box.height = getWindowHeight();
    root = new QuadtreeNode(0, box);
    qt = new Quadtree(root);
}

void QuadTreeApp::mouseDown( MouseEvent event )
{
    
    
}

void QuadTreeApp::mouseDrag(MouseEvent event) {
    
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

void QuadTreeApp::update()
{
}

void QuadTreeApp::draw()
{
	gl::clear( Color( 0, 0, 0 ) );
    QuadtreeNode* iter;
    iter = root;
    iterate(iter);
    
    for (int i= 0;  i < posx_vector.size(); i++) {
        vec2 position(posx_vector[i], posy_vector[i]);
        gl::pushModelMatrix();
        gl::color(Color(1, 1, 1));
        gl::drawSolidCircle(position, 2);
        gl::popModelMatrix();
        
    }
    
}

void QuadTreeApp::iterate(QuadtreeNode* iter) {
    gl::pushModelMatrix();
    gl::color(Color(1, 1, 1));
    cinder::Rectf rect(iter->box.position.x - iter->box.width/2, iter->box.position.y - iter->box.height/2, iter->box.position.x + iter->box.width/2, iter->box.position.y + iter->box.height/2);
    gl::drawStrokedRect(rect, 1);
    gl::popModelMatrix();
    
    if(iter->ne != NULL) iterate(iter->ne);
    if(iter->nw != NULL) iterate(iter->nw);
    if(iter->se != NULL) iterate(iter->se);
    if(iter->sw != NULL) iterate(iter->sw);
    
}


CINDER_APP( QuadTreeApp, RendererGl )
