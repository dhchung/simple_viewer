#ifndef MYGLWIDGET_H
#define MYGLWIDGET_H

#include "utils.h"

// Important: the widget should be promoted to this class in qtcreator
// see mainwindow.ui

class MyGlWidget : public QOpenGLWidget, public QOpenGLExtraFunctions{
Q_OBJECT

public:
    explicit MyGlWidget(QWidget *parent = nullptr);
    ~MyGlWidget();

public slots:
    void SlotDrawAxis(int arg);
    void SlotPointSize(double arg);
    void SlotCloudNow(std::string data_path);

private:
    QOpenGLShaderProgram * m_program;
    GLuint m_model_loc;
    GLuint m_view_loc;
    GLuint m_projection_loc;
    GLuint m_color_loc;
    GLuint m_color_bool;
    GLuint m_color_alpha;

    unsigned int VBO, VAO;
    int gl_width;
    int gl_height;
    bool draw_axis;
    double point_size;

    QPointF mouse_last_pose;
    bool mouse_left_pressed;
    bool mouse_middle_pressed;
    bool mouse_right_pressed;    

    void initializeGL() override;
    void paintGL() override;
    void resizeGL(int w, int h) override;
    void mousePressEvent(QMouseEvent *event) override;
    void mouseReleaseEvent(QMouseEvent *event) override;
    void mouseMoveEvent(QMouseEvent *event) override;
    void wheelEvent(QWheelEvent * event) override;
    void drawPoints();
    void drawAxis();

    void paintStuffs();

    CameraPosition cam_pose;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_now;
};

#endif // MYGLWIDGET_H
