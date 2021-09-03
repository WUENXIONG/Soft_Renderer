#pragma once
#include"tool/geometry.h"

extern Vec3f up;
class Camera {
public:
	Camera(Vec3f e, Vec3f org);
	~Camera();
	Vec3f eye;
	//����Ϊ�������ϵԭ���������
	Vec3f origin;
	Vec3f c_x;
	Vec3f c_y;
	Vec3f c_z;

};

void update(Camera& camera);
void handle(Camera& camera);