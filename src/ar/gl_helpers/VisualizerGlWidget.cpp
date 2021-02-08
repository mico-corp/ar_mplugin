//--//---------------------------------------------------------------------------------------------------------------------
//  AR MICO plugin
//---------------------------------------------------------------------------------------------------------------------
//  Copyright 2020 Pablo Ramon Soria (a.k.a. Bardo91) pabramsor@gmail.com
//---------------------------------------------------------------------------------------------------------------------
//  Permission is hereby granted, free of charge, to any person obtaining a copy of this software
//  and associated documentation files (the "Software"), to deal in the Software without restriction,
//  including without limitation the rights to use, copy, modify, merge, publish, distribute,
//  sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is
//  furnished to do so, subject to the following conditions:
//
//  The above copyright notice and this permission notice shall be included in all copies or substantial
//  portions of the Software.
//
//  THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING
//  BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND
//  NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES
//  OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
//  CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
//---------------------------------------------------------------------------------------------------------------------



#include <mico/ar/gl_helpers/VisualizerGlWidget.h>

namespace mico{

    VisualizerGlWidget::VisualizerGlWidget(QWidget *_parent){
        this->setFocusPolicy(Qt::StrongFocus);
        this->setMinimumWidth(200);
        this->setMinimumHeight(200);

        scene_.addAxis(Eigen::Matrix4f::Identity());    /// origin
    }
    
    VisualizerGlWidget::~VisualizerGlWidget(){

    }


    void VisualizerGlWidget::addPoint(mico::Scene3d::Point _p){
        
    }

    void VisualizerGlWidget::updatePose(Eigen::Matrix4f _pose){
        pose_ = _pose;
    }

    void VisualizerGlWidget::addLine(mico::Scene3d::Point _p1, mico::Scene3d::Point _p2){
        scene_.addLine(_p1, _p2);
    }

    void VisualizerGlWidget::clearAll(){

    }

    void VisualizerGlWidget::updateBackgroundImage(const cv::Mat &_image) {
        unsigned int texture;
        glGenTextures(1, &texture);
        glBindTexture(GL_TEXTURE_2D, texture);
        // set the texture wrapping/filtering options (on the currently bound texture object)
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_REPEAT);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        // load and generate the texture
        if (_image.rows) {
            glTexImage2D(GL_TEXTURE_2D, 0, GL_RGB, _image.cols, _image.rows, 0, GL_RGB, GL_UNSIGNED_BYTE, _image.data);
            glGenerateMipmap(GL_TEXTURE_2D);
        };
    }

    void VisualizerGlWidget::initializeGL(){
        connect(context(), &QOpenGLContext::aboutToBeDestroyed, this, &VisualizerGlWidget::cleanup);

        initializeOpenGLFunctions();
        scene_.init(false);

        glTimer_ = new QTimer(this);
        connect(glTimer_, SIGNAL(timeout()), this, SLOT(update()));
        glTimer_->start(100);
    }
    
    void VisualizerGlWidget::paintGL(){
        drawBackground();

        scene_.moveCamera(pose_);

        scene_.displayAll();
        
    }
    
    void VisualizerGlWidget::resizeGL(int _width, int _height){
        // Compute aspect ratio of the new window
        if (_height == 0)
            _height = 1; // To prevent divide by 0

        scene_.resizeGL(_width, _height);
    }
    

    void VisualizerGlWidget::keyPressEvent(QKeyEvent *event) {
    
    }

    void VisualizerGlWidget::keyReleaseEvent(QKeyEvent *event) {
        
    }
    void VisualizerGlWidget::mousePressEvent(QMouseEvent *event){
    }
    
    void VisualizerGlWidget::mouseReleaseEvent(QMouseEvent *event){
    }

    void VisualizerGlWidget::mouseMoveEvent(QMouseEvent *event){
        
    }

    void VisualizerGlWidget::wheelEvent(QWheelEvent *event){
       
    }

        
    void VisualizerGlWidget::cleanup() {
        makeCurrent();
        
        doneCurrent();
    }


    void VisualizerGlWidget::drawBackground() {

        if (bgTex_ != 0) {
            // Save previous matrix information
            glMatrixMode(GL_PROJECTION);
            glPushMatrix();
            glLoadIdentity();
            gluOrtho2D(0.0, 1.0, 1.0, 0.0);
            glMatrixMode(GL_MODELVIEW);
            glPushMatrix();
            glLoadIdentity();

            // Draw background
            glBindTexture(GL_TEXTURE_2D, bgTex_);
            glColor3f(1.0, 1.0, 1.0);
            glBegin(GL_QUADS);
            glTexCoord2f(0.0, 1.0); glTexCoord2f(0.05, 0.05);
            glTexCoord2f(1.0, 1.0); glTexCoord2f(0.3, 0.05);
            glTexCoord2f(1.0, 0.0); glTexCoord2f(0.3, 0.15);
            glTexCoord2f(0.0, 0.0); glTexCoord2f(0.05, 0.15);
            glEnd();

            // Recover previous matrix infromation.
            glMatrixMode(GL_PROJECTION);
            glPopMatrix();
            glMatrixMode(GL_MODELVIEW);
            glPopMatrix();

        }
    }
}
