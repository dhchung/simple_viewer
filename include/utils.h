#ifndef UTILS_H
#define UTILS_H

#include <QDialog>
#include <QOpenGLWidget>
#include <QOpenGLExtraFunctions>
#include <QMatrix4x4>
#include <QOpenGLShaderProgram>
#include <QQuaternion>
#include <QMouseEvent>
#include <QWheelEvent>
#include <QKeyEvent>
#include <QResizeEvent>
#include <QObject>
#include <QFileDialog>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

#include <dirent.h>
#include <fstream>

struct Vertex{
    float x;
    float y;
    float z;
    float r;
    float g;
    float b;

    Vertex() {
        x = 0.0f;
        y = 0.0f;
        z = 0.0f;
        r = 0.0f;
        g = 0.0f;
        b = 0;
    }

};

struct CameraPosition{
    float mouse_sensitivity = 0.1;
    float scroll_sensitivity = 0.001;
    float scroll_move_sensitivity = 0.01;

    QVector3D up = QVector3D(0.0, 0.0, 1.0);
    QVector3D center = QVector3D(0.0, 0.0, 0.0);
    float distance = 10.0f;
    float yaw = 45.0f;
    float pitch = 30.0f;

    float x = center.x() + distance * cos(pitch * M_PI/180.0) * cos(yaw * M_PI/180.0);
    float y = center.y() + distance * cos(pitch * M_PI/180.0) * sin(yaw * M_PI/180.0);
    float z = center.z() + distance * sin(pitch * M_PI/180.0);

    QVector3D position = QVector3D(x, y ,z);
    QMatrix4x4 view;

    CameraPosition(){
        resetCamera();
        update_camera_matrix();
    }

    void resetCamera() {
        up = QVector3D(0.0, 0.0, 1.0);
        center = QVector3D(0.0, 0.0, 0.0);
        distance = 10.0f;
        yaw = 45.0f;
        pitch = 30.0f;

        x = center.x() + distance * cos(pitch * M_PI/180.0) * cos(yaw * M_PI/180.0);
        y = center.y() + distance * cos(pitch * M_PI/180.0) * sin(yaw * M_PI/180.0);
        z = center.z() + distance * sin(pitch * M_PI/180.0);

        position = QVector3D(x, y ,z);
        update_camera_matrix();
    }

    void mouse_movement_update(float dx, float dy){
        yaw -= dx * mouse_sensitivity;
        pitch += dy * mouse_sensitivity;
        if(abs(pitch) > 89.99) {
            pitch = pitch/abs(pitch) * 89.99;
        }
        update_camera_matrix();
    }

    void mouse_right_movement_update(float dx, float dy){
        yaw -= dx * mouse_sensitivity;
        pitch += dy * mouse_sensitivity;
        if(abs(pitch) > 89.99) {
            pitch = pitch/abs(pitch) * 89.99;
        }
        update_camera_matrix_center_move();
    }


    void mouse_scroll_movement_update(float dx, float dy) {
        QVector3D front = center - position;
        front.normalize();

        QVector3D right = QVector3D::crossProduct(front, up);
        right.normalize();

        QVector3D cam_up = QVector3D::crossProduct(right, front);
        cam_up.normalize();

        center -= distance / 10.0 * scroll_move_sensitivity * right * dx -
                distance / 10.0 * scroll_move_sensitivity * cam_up * dy;


        update_camera_matrix();
    }

    void mouse_scroll_update(float dscroll) {
        distance -= scroll_sensitivity * dscroll;
        if(distance < 0.1){
            distance = 0.1;
        }
        update_camera_matrix();
    }

    void update_camera_matrix(){
        x = center.x() + distance * cos(pitch * M_PI/180.0) * cos(yaw * M_PI/180.0);
        y = center.y() + distance * cos(pitch * M_PI/180.0) * sin(yaw * M_PI/180.0);
        z = center.z() + distance * sin(pitch * M_PI/180.0);
        position = QVector3D(x, y ,z);
        view.setToIdentity();
        view.lookAt(position, center, up);
    }

    void update_camera_matrix_center_move(){
        center.setX(x - distance * cos(pitch * M_PI/180.0) * cos(yaw * M_PI/180.0));
        center.setY(y - distance * cos(pitch * M_PI/180.0) * sin(yaw * M_PI/180.0));
        center.setZ(z - distance * sin(pitch * M_PI/180.0));
        view.setToIdentity();
        view.lookAt(position, center, up);
    }

    void print_camera_status() {
        printf("x, y, z: %f, %f, %f\n", x, y, z);
        printf("center: %f, %f, %f\n", center.x(), center.y(), center.z());
        printf("yaw, pitch: %f, %f\n", yaw, pitch);
        printf("distance: %f\n", distance);
    }
};



#endif