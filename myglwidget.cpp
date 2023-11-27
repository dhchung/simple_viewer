#include "myglwidget.h"

MyGlWidget::MyGlWidget(QWidget *parent) : QOpenGLWidget(parent) {
    draw_axis = false;
    point_size = 1.0;
    cloud_now.reset(new pcl::PointCloud<pcl::PointXYZI>());

    mouse_left_pressed = false;
    mouse_middle_pressed = false;
    mouse_right_pressed = false;

}

MyGlWidget::~MyGlWidget() {

}

void MyGlWidget::SlotDrawAxis(int arg) {
    draw_axis = arg == 1 ? true : false;
    paintStuffs();
}

void MyGlWidget::SlotPointSize(double arg) {
    point_size = arg;
    paintStuffs();
}

void MyGlWidget::SlotCloudNow(std::string data_path) {
    cloud_now->clear();

    // Load Point Cloud
    // Depends on the cloud format
    // For this example, the binary file is written as:
    // x, y, z, intensity, time, reflectivity, ambient, range

    std::ifstream file(data_path, std::ios::in | std::ios::binary);
    while(file) {
        pcl::PointXYZI pcl_pt;

        float x, y, z, intensity;
        uint32_t time, range;
        uint16_t reflectivity, ambient;

        file.read((char*)&x, sizeof(float));
        file.read((char*)&y, sizeof(float));
        file.read((char*)&z, sizeof(float));
        file.read((char*)&intensity, sizeof(float));
        file.read((char*)&time, sizeof(uint32_t));
        file.read((char*)&reflectivity, sizeof(uint16_t));
        file.read((char*)&ambient, sizeof(uint16_t));
        file.read((char*)&range, sizeof(uint32_t));


        pcl_pt.x = x;
        pcl_pt.y = y;
        pcl_pt.z = z;
        pcl_pt.intensity = intensity;

        cloud_now->push_back(pcl_pt);
    }
    if(cloud_now->empty()) {
        return;
    } else {
        paintStuffs();
    }
}


void MyGlWidget::initializeGL() {
    // Initialization
    initializeOpenGLFunctions();
    // Color of the background: r, g, b, alpha
    glClearColor(130.0 / 255.0, 130.0 / 255.0, 130.0 / 255.0, 1.0f);
    // Enable depth test
    glEnable(GL_DEPTH_TEST);
    // Enable blending
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    // Set the program and link it to the shaders
    m_program = new QOpenGLShaderProgram(this);
    m_program->addShaderFromSourceFile(QOpenGLShader::Vertex, "shader/vertex_shader.vs");
    m_program->addShaderFromSourceFile(QOpenGLShader::Fragment, "shader/fragment_shader.fs");
    m_program->link();
    m_model_loc = m_program->uniformLocation("model");
    m_view_loc = m_program->uniformLocation("view");
    m_projection_loc = m_program->uniformLocation("projection");
    m_color_loc = m_program->uniformLocation("input_color");
    m_color_bool = m_program->uniformLocation("color_bool");
    m_color_alpha = m_program->uniformLocation("color_alpha");
}

void MyGlWidget::paintGL() {
    // Clear all color and depth buffers from the previous frame
    glClear(GL_COLOR_BUFFER_BIT | GL_DEPTH_BUFFER_BIT);
    glClearColor(130.0 / 255.0, 130.0 / 255.0, 130.0 / 255.0, 1.0f); //r, g, b, alpha

    if(draw_axis) {
        drawAxis();
    }
    drawPoints();
}

void MyGlWidget::resizeGL(int w, int h) {
    // Window resize callback

    gl_width = w;
    gl_height = h;
    glViewport(0, 0, w, h);
}

void MyGlWidget::mousePressEvent(QMouseEvent *event) {

    this->setFocus();
    if(event->button() == Qt::LeftButton) {
        mouse_left_pressed = true;
    }
    if(event->button() == Qt::MidButton) {
        mouse_middle_pressed = true;
    }
    if(event->button() == Qt::RightButton) {
        mouse_right_pressed = true;
    }
    mouse_last_pose = event->pos();
}

void MyGlWidget::mouseReleaseEvent(QMouseEvent *event) {

    if(event->button() == Qt::LeftButton) {
        mouse_left_pressed = false;
    }
    if(event->button() == Qt::MidButton) {
        mouse_middle_pressed = false;
    }
    if(event->button() == Qt::RightButton) {
        mouse_right_pressed = false;
    }
}

void MyGlWidget::mouseMoveEvent(QMouseEvent *event) {

    float delta_x = event->x() - mouse_last_pose.x();
    float delta_y = event->y() - mouse_last_pose.y();
    if(mouse_left_pressed && !mouse_right_pressed) {
        // Camera movement with fixed center position

        cam_pose.mouse_movement_update(delta_x, delta_y);
        paintStuffs();
    }
    if(mouse_middle_pressed) {
        // Camera translation

        cam_pose.mouse_scroll_movement_update(delta_x, delta_y);
        paintStuffs();
    }
    if(mouse_right_pressed && !mouse_left_pressed) {
        // Center movement with fixed camera position

        cam_pose.mouse_right_movement_update(delta_x, delta_y);
        paintStuffs();
    }

    mouse_last_pose = event->pos();
}

void MyGlWidget::wheelEvent(QWheelEvent *event) {
    // Zoom in zoom out

    float delta_zoom = event->angleDelta().y();
    cam_pose.mouse_scroll_update(delta_zoom);
    paintStuffs();
}

void MyGlWidget::drawPoints() {
    if(cloud_now->empty()) {
        return;
    }

    // Set all points in to Vertex vector
    std::vector<Vertex> points(cloud_now->size());
    for(int i = 0; i < cloud_now->size(); ++i) {
        pcl::PointXYZI & pt = cloud_now->points[i];
        points[i].x = pt.x;
        points[i].y = pt.y;
        points[i].z = pt.z;

        float color_ratio = pt.intensity/3000.0f;
        if(color_ratio > 1.0) {
            color_ratio = 1.0;
        }
        points[i].r = 1.0;
        points[i].g = color_ratio;
        points[i].b = 0.0;
    }

    // Model transformation matrix
    QMatrix4x4 model;
    model.setToIdentity();

    // Camera location
    QMatrix4x4 view;
    view = cam_pose.view;

    // Camera model
    QMatrix4x4 projection;
    projection.perspective(90.0f, float(gl_width) / float(gl_height), 0.01f, 1000.0f); // field of view, ratio, near, far

    // Set the shader parameters
    m_program->bind();
    m_program->setUniformValue(m_model_loc, model);
    m_program->setUniformValue(m_view_loc, view);
    m_program->setUniformValue(m_projection_loc, projection);
    m_program->setUniformValue(m_color_bool, 0);
    m_program->setUniformValue(m_color_alpha, 1.0f);

    // Generate the vertex array object & vertex buffer object
    // If points doesn't change, these doesn't have to be updated every rendering process
    glGenVertexArrays(1, &VAO);
    glGenBuffers(1, &VBO);

    glBindVertexArray(VAO);
    glBindBuffer(GL_ARRAY_BUFFER, VBO);

    glBufferData(GL_ARRAY_BUFFER,
                 points.size() * sizeof(Vertex),
                 &points.front(),
                 GL_STATIC_DRAW);
    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)0);
    glEnableVertexAttribArray(0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6 * sizeof(float), (void *)(3 * sizeof(float)));
    glEnableVertexAttribArray(1);
    glPointSize(point_size);
    glDrawArrays(GL_POINTS, 0, cloud_now->size());

    glDeleteVertexArrays(1, &VAO);
    glDeleteBuffers(1, &VBO);
    m_program->release();
}

void MyGlWidget::drawAxis() {

    // Draw axis of the position where the camera is looking at (center)

    m_program->bind();
    m_program->setUniformValue(m_color_bool, 0);
    m_program->setUniformValue(m_color_alpha, 1.0f);

    float cx = cam_pose.center.x();
    float cy = cam_pose.center.y();
    float cz = cam_pose.center.z();

    float line_length = 1.0f;
    float line_width = 3.5f;

    // x, y, z, r, g, b
    float vertices[]{
        cx + 0.0f,              cy + 0.0f,               cz + 0.0f,                   1.0f, 0.0f, 0.0f,//v0
        cx + line_length*1.0f,  cy + 0.0f,               cz + 0.0f,                   1.0f, 0.0f, 0.0f,//vx
        cx + 0.0f,              cy + 0.0f,               cz + 0.0f,                   0.0f, 1.0f, 0.0f,//v0
        cx + 0.0f,              cy + line_length*1.0f,   cz + 0.0f,                   0.0f, 1.0f, 0.0f,//vy
        cx + 0.0f,              cy + 0.0f,               cz + 0.0f,                   0.0f, 0.0f, 1.0f,//v0
        cx + 0.0f,              cy + 0.0f,               cz + line_length*1.0f,       0.0f, 0.0f, 1.0f//vz
    };

    unsigned int VAO_axis;
    unsigned int VBO_axis;

    glGenVertexArrays(1, &VAO_axis);
    glGenBuffers(1, &VBO_axis);

    glBindVertexArray(VAO_axis);
    glBindBuffer(GL_ARRAY_BUFFER, VBO_axis);
    glBufferData(GL_ARRAY_BUFFER, sizeof(vertices), vertices, GL_STATIC_DRAW);

    glVertexAttribPointer(0, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)0);
    glVertexAttribPointer(1, 3, GL_FLOAT, GL_FALSE, 6*sizeof(float), (void*)(3*sizeof(float)));
    glEnableVertexAttribArray(0);
    glEnableVertexAttribArray(1);


    QMatrix4x4 model;
    QMatrix4x4 view;
    QMatrix4x4 projection;
    projection.perspective(90.0f, float(gl_width) / float(gl_height), 0.01f, 1000.0f); // field of view, ratio, near, far

    view = cam_pose.view;
    model.setToIdentity();

    m_program->bind();
    m_program->setUniformValue(m_model_loc, model);
    m_program->setUniformValue(m_view_loc, view);
    m_program->setUniformValue(m_projection_loc, projection);

    glLineWidth(line_width);
    glDrawArrays(GL_LINES, 0, 6);

    glDeleteVertexArrays(1, &VAO_axis);
    glDeleteBuffers(1, &VBO_axis);
    m_program->release();
}


void MyGlWidget::paintStuffs() {
    paintGL();
    update();
}