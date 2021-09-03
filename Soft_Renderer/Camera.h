#pragma once
#include"tool/geometry.h"

extern Vec3f up;
class Camera {
public:
	Camera(Vec3f e, Vec3f org);
	~Camera();
	Vec3f eye;
	//以下为相机坐标系原点和三个轴
	Vec3f origin;
	Vec3f c_x;
	Vec3f c_y;
	Vec3f c_z;

};

void update(Camera& camera);
void handle(Camera& camera);